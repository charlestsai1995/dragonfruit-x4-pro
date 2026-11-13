#include "input.h"
#include "dragonball.h"
#include "layout.h"
#include "touch_config.h"
#include "palm_config.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG = "input";

/* Pen events flow input_poll() (touch task) -> CPU task, which injects them
 * into Palm OS via the EvtEnqueuePenPoint ROM trap. We do NOT drive the raw
 * pen-IRQ digitizer path (db_spi_set_touch): on the IIIx ROM that re-enters
 * the AMXHardwareEZ pen handler and fatal-errors ("Pen down int. invalid"). */
/* Deep enough to hold a whole Graffiti stroke's worth of sampled points if the
 * CPU task briefly falls behind draining them (it normally drains far faster
 * than the 100 Hz sampling). */
#define PEN_QUEUE_LEN 64
static QueueHandle_t pen_queue = NULL;

#define KEY_QUEUE_LEN 8
static QueueHandle_t key_queue = NULL;

static void pen_push(int16_t x, int16_t y, bool pen_up)
{
    if (!pen_queue)
        return;
    input_pen_event_t ev = { x, y, pen_up };
    xQueueSend(pen_queue, &ev, 0);   /* drop if full rather than block */
}

bool input_pop_pen_event(input_pen_event_t *out)
{
    if (!pen_queue)
        return false;
    return xQueueReceive(pen_queue, out, 0) == pdTRUE;
}

/* Injection API — enqueue from any task; the CPU task drains. */
void input_inject_pen(int16_t x, int16_t y, bool pen_up)
{
    pen_push(x, y, pen_up);
}

void input_inject_key(uint16_t ascii, uint16_t keycode, uint16_t modifiers)
{
    if (!key_queue)
        return;
    input_key_event_t ev = { ascii, keycode, modifiers };
    xQueueSend(key_queue, &ev, 0);   /* drop if full rather than block */
}

bool input_pop_key(input_key_event_t *out)
{
    if (!key_queue)
        return false;
    return xQueueReceive(key_queue, out, 0) == pdTRUE;
}

/* Latched once per awake Power-button press; drained by the CPU task. */
static volatile bool power_key_pending = false;

bool input_pop_power_key(void)
{
    if (!power_key_pending)
        return false;
    power_key_pending = false;
    return true;
}

/* Backup/restore request, latched once per >=1s hold of the UP/DOWN side button
 * (see handle_hold_button below); drained by the CPU task. */
static volatile input_br_req_t br_req_pending = INPUT_BR_NONE;

int input_pop_br_request(void)
{
    input_br_req_t r = br_req_pending;
    if (r != INPUT_BR_NONE)
        br_req_pending = INPUT_BR_NONE;
    return r;
}

/* GT911 I2C */
#define GT911_ADDR          0x14
#define GT911_ADDR_ALT      0x5D
#define GT911_CMD_REG       0x8040
#define GT911_CFG_REG       0x8047
#define GT911_STATUS_REG    0x814E
#define GT911_TOUCH_REG     0x8150
#define GT911_PRODUCT_REG   0x8140
#define GT911_FWVER_REG     0x8144
#define GT911_RESOL_REG     0x8146

#if CONFIG_IDF_TARGET_ESP32S3
/* Xteink X4 Pro pins */
#define PIN_SDA         39
#define PIN_SCL         38
#define PIN_INT         10
#define PIN_TOUCH_PWR   2
#define PIN_TOUCH_RST   4
#define PIN_BTN_L       0    /* Up button   -> Page Up (vchrPageUp) / >=1s hold = Backup  */
#define PIN_BTN_M       3    /* Power btn   -> Home (vchrLaunch) / >=1s hold = Sleep/Wake */
#define PIN_BTN_R       7    /* Down button -> Page Down (vchrPageDown) / >=1s hold = Restore */
#define I2C_BUS_SPEED   100000
#define CW2017_ADDR     0x63
#define CW2017_REG_VCELL 0x02
#define CW2017_REG_MODE  0x08
#else
/* M5 Paper pins */
#define PIN_SDA         21
#define PIN_SCL         22
#define PIN_INT         36
#define PIN_BTN_L       37
#define PIN_BTN_M       38
#define PIN_BTN_R       39
#define I2C_BUS_SPEED   400000
#endif

bool input_button_held(int which)
{
    return !gpio_get_level(which ? PIN_BTN_R : PIN_BTN_L);
}

/* Panel<->Palm mapping lives in components/layout (shared with display.c, the
 * single source of truth). The only fixed relation we still own here is the
 * GT911 raw -> native IT8951 panel-pixel step below (a hardware constant,
 * independent of how the device is held); layout_panel_to_palm() then turns a
 * native panel pixel into Palm coords for whatever layout is active. */

static uint8_t gt911_addr = GT911_ADDR;
static bool gt911_found = false;
static i2c_master_bus_handle_t gt911_bus;   /* I2C0 master bus */
static i2c_master_dev_handle_t gt911_dev;   /* GT911 device (7-bit addr bound at probe) */
#if CONFIG_IDF_TARGET_ESP32S3
static i2c_master_dev_handle_t cw2017_dev;  /* CW2017 battery fuel gauge (0x63) */
#endif
static volatile int s_battery_mv = 4000;
static bool last_pen_down = false;
static int16_t last_px = -1, last_py = -1;   /* last streamed stroke point */
static bool last_btn_l = false;
static bool last_btn_m = false;
static bool last_btn_r = false;
/* Long-hold timing for the UP/DOWN side buttons (backup/restore trigger) plus
 * the deferred-tap release that emits a single key event on short tap. */
static TickType_t l_press_tick = 0, r_press_tick = 0;
static bool l_hold_fired = false, r_hold_fired = false;
static uint32_t poll_diag_count = 0;

int input_get_battery_mv(void)
{
    return s_battery_mv;
}

#if CONFIG_IDF_TARGET_ESP32S3
static void cw2017_poll_battery(void)
{
    if (!cw2017_dev) return;
    uint8_t reg = CW2017_REG_VCELL;
    uint8_t buf[2] = {0};
    if (i2c_master_transmit_receive(cw2017_dev, &reg, 1, buf, 2, 50) == ESP_OK) {
        uint16_t raw14 = (((uint16_t)buf[0] << 8) | buf[1]) & 0x3FFFu;
        int mv = (int)((raw14 * 5u + 8u) >> 4u);
        if (mv >= 2500 && mv <= 4500) {
            s_battery_mv = mv;
        }
    }
}

static void cw2017_init_gauge(void)
{
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = CW2017_ADDR,
        .scl_speed_hz    = I2C_BUS_SPEED,
    };
    if (i2c_master_bus_add_device(gt911_bus, &dc, &cw2017_dev) != ESP_OK) {
        cw2017_dev = NULL;
        return;
    }
    uint8_t reg = CW2017_REG_MODE;
    uint8_t mode = 0;
    if (i2c_master_transmit_receive(cw2017_dev, &reg, 1, &mode, 1, 50) != ESP_OK) {
        ESP_LOGW(TAG, "CW2017 not responding at 0x%02X", CW2017_ADDR);
        i2c_master_bus_rm_device(cw2017_dev);
        cw2017_dev = NULL;
        return;
    }
    if (mode != 0x00) {
        uint8_t wake_cmd[2] = { CW2017_REG_MODE, 0x00 };
        i2c_master_transmit(cw2017_dev, wake_cmd, sizeof(wake_cmd), 50);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    cw2017_poll_battery();
    ESP_LOGI(TAG, "CW2017 fuel gauge ready (%d mV)", s_battery_mv);
}
#endif

/* --- GT911 I2C helpers --- */

/* Bind the GT911 device handle to a 7-bit address. The panel answers at 0x14 or
 * 0x5D (probed at init); re-creating the device handle is how the i2c_master API
 * retargets the address. */
static esp_err_t gt911_use_addr(uint8_t addr)
{
    if (gt911_dev) { i2c_master_bus_rm_device(gt911_dev); gt911_dev = NULL; }
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = I2C_BUS_SPEED,
    };
    gt911_addr = addr;
    return i2c_master_bus_add_device(gt911_bus, &dc, &gt911_dev);
}

/* Reads/writes are a big-endian 16-bit register address followed by data. The
 * 100 ms timeout matches the legacy i2c_master_cmd_begin budget. */
static esp_err_t gt911_read(uint16_t reg, uint8_t *buf, size_t len)
{
    uint8_t r[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    return i2c_master_transmit_receive(gt911_dev, r, sizeof(r), buf, len, 100);
}

static esp_err_t gt911_write_byte(uint16_t reg, uint8_t val)
{
    uint8_t w[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val };
    return i2c_master_transmit(gt911_dev, w, sizeof(w), 100);
}

/* Write a multi-byte block in one transaction (used to push the whole 186-byte
 * GT911 config block at 0x8047). The GT911 applies an updated config only on a
 * single contiguous write ending with the checksum + fresh-flag bytes, so the
 * register address and payload go out as one transmit. */
static esp_err_t gt911_write_block(uint16_t reg, const uint8_t *buf, size_t len)
{
    uint8_t w[2 + GT911_CFG_LEN];   /* the 186-byte config block is the largest write */
    if (len > GT911_CFG_LEN) return ESP_ERR_INVALID_SIZE;
    w[0] = (uint8_t)(reg >> 8);
    w[1] = (uint8_t)(reg & 0xFF);
    memcpy(w + 2, buf, len);
    return i2c_master_transmit(gt911_dev, w, 2 + len, 100);
}

/* digitizer.cfg lives at the SD root (i.e. M5_SD_MOUNT_POINT "/digitizer.cfg").
 * Kept as a literal so input.c need not depend on m5_sd just for the macro. */
#define DIGITIZER_CFG_PATH "/sdcard/digitizer.cfg"

/* Logging sink for the (pure) digitizer.cfg parser — routes per-key
 * applied/skipped messages to the serial log. */
static void gt911_cfg_log(void *ctx, const char *msg)
{
    (void)ctx;
    ESP_LOGW(TAG, "digitizer.cfg: %s", msg);
}

/* Read the GT911's current sensitivity config, then either CREATE digitizer.cfg
 * from those current values (when it doesn't exist) or APPLY the file's values
 * to the chip (when it does). Runs once at the end of input_init(): the chip is
 * probed + awake and, since no task has started yet, the display's shared SPI
 * bus is free for the SD mount. A missing card or unreadable chip just leaves
 * the panel at its factory config. */
static void gt911_sync_sensitivity(void)
{
    uint8_t block[GT911_CFG_LEN];
    if (gt911_read(GT911_CFG_BASE, block, GT911_CFG_LEN) != ESP_OK) {
        ESP_LOGW(TAG, "GT911 config read failed; leaving sensitivity at default");
        return;
    }
    gt911_sens_t cur;
    gt911_sens_from_block(block, &cur);

    static char text[1024];
    size_t len = 0;
    config_load_result_t r = config_read_file(DIGITIZER_CFG_PATH, text, sizeof(text), &len);

    if (r == CONFIG_NO_FILE) {
        /* First boot with this card: persist the current values so the user has
         * a documented file to edit. Do NOT reprogram the chip. */
        int n = gt911_sens_serialize(&cur, text, sizeof(text));
        if (n < 0) { ESP_LOGE(TAG, "digitizer.cfg serialize overflow"); return; }
        if (config_write_file(DIGITIZER_CFG_PATH, text, (size_t)n) == CONFIG_OK)
            ESP_LOGI(TAG, "created digitizer.cfg from current values "
                     "(touch=%u leave=%u noise=%u shake=%u filter=%u large=%u)",
                     cur.touch_level, cur.leave_level, cur.noise_reduction,
                     cur.shake_count, cur.filter, cur.large_touch);
        return;
    }
    if (r != CONFIG_OK)
        return;   /* no SD / read error — keep the factory config */

    gt911_sens_t want = cur;
    gt911_sens_parse(text, &want, gt911_cfg_log, NULL);
    if (memcmp(&want, &cur, sizeof want) == 0) {
        ESP_LOGI(TAG, "digitizer.cfg matches current config; no change");
        return;
    }

    /* Overlay the requested knobs onto the freshly-read block, recompute the
     * GT911 checksum, and set the config-fresh flag so the chip reloads. The
     * config-version byte (block[0]) is deliberately left as-read so the chip
     * treats this as a volatile session update re-applied every boot. */
    gt911_sens_to_block(&want, block);
    block[GT911_CFG_CKSUM] = gt911_cfg_checksum(block);
    block[GT911_CFG_FRESH] = 0x01;
    if (gt911_write_block(GT911_CFG_BASE, block, GT911_CFG_LEN) == ESP_OK)
        ESP_LOGI(TAG, "applied digitizer.cfg "
                 "(touch=%u leave=%u noise=%u shake=%u filter=%u large=%u)",
                 want.touch_level, want.leave_level, want.noise_reduction,
                 want.shake_count, want.filter, want.large_touch);
    else
        ESP_LOGE(TAG, "GT911 config write failed");
}

/* --- Public API --- */

esp_err_t input_init(void)
{
#if CONFIG_IDF_TARGET_ESP32S3
    /* Xteink X4 Pro: Enable active-low touch power rail (GPIO 2) and run
     * GT911 reset sequence on RST (GPIO 4) + INT (GPIO 10) for 0x5D/0x14. */
    gpio_hold_dis(PIN_TOUCH_PWR);
    gpio_set_direction(PIN_TOUCH_PWR, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_TOUCH_PWR, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_hold_dis(PIN_TOUCH_RST);
    gpio_hold_dis(PIN_INT);
    gpio_set_direction(PIN_TOUCH_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_INT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_INT, 0);
    gpio_set_level(PIN_TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
#endif

    i2c_master_bus_config_t bus_conf = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_conf, &gt911_bus), TAG, "I2C bus init failed");

    /* M5EPD library waits 100ms after Wire.begin() before first GT911 access */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* GT911 INT pin */
    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << PIN_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&int_conf);

    /* Hardware buttons (active-low) */
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << PIN_BTN_L) | (1ULL << PIN_BTN_M) | (1ULL << PIN_BTN_R),
        .mode = GPIO_MODE_INPUT,
#if CONFIG_IDF_TARGET_ESP32S3
        .pull_up_en = GPIO_PULLUP_ENABLE,
#else
        .pull_up_en = GPIO_PULLUP_DISABLE,
#endif
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);

#if CONFIG_IDF_TARGET_ESP32S3
    cw2017_init_gauge();
#endif

    /* Probe GT911 at primary address, then alternate */
    uint8_t product_id[4];
    if (gt911_use_addr(GT911_ADDR_ALT) == ESP_OK &&
        gt911_read(GT911_PRODUCT_REG, product_id, 4) == ESP_OK) {
        gt911_found = true;
    } else if (gt911_use_addr(GT911_ADDR) == ESP_OK &&
               gt911_read(GT911_PRODUCT_REG, product_id, 4) == ESP_OK) {
        gt911_found = true;
    }

    if (gt911_found) {
        ESP_LOGI(TAG, "GT911 at 0x%02X  product=%.4s", gt911_addr, product_id);

        /* Read firmware version */
        uint8_t fw_ver[2] = {0};
        gt911_read(GT911_FWVER_REG, fw_ver, 2);
        ESP_LOGI(TAG, "  FW version: %d", fw_ver[0] | (fw_ver[1] << 8));

        /* Read active resolution */
        uint8_t res[4] = {0};
        gt911_read(GT911_RESOL_REG, res, 4);
        ESP_LOGI(TAG, "  Resolution: %dx%d",
                 res[0] | (res[1] << 8), res[2] | (res[3] << 8));

        /* Read config header */
        uint8_t cfg[7] = {0};
        gt911_read(GT911_CFG_REG, cfg, 7);
        ESP_LOGI(TAG, "  Config: ver=%d x=%d y=%d touches=%d mod_sw1=0x%02X",
                 cfg[0],
                 cfg[1] | (cfg[2] << 8),
                 cfg[3] | (cfg[4] << 8),
                 cfg[5] & 0x0F,
                 cfg[6]);

        /* Ensure GT911 is in active coordinate-reading mode (not asleep) */
        gt911_write_byte(GT911_CMD_REG, 0x00);
        vTaskDelay(pdMS_TO_TICKS(50));

        /* Clear status register */
        gt911_write_byte(GT911_STATUS_REG, 0x00);

        /* Create or apply the SD sensitivity config (digitizer.cfg) now that the
         * chip is probed + awake and the shared display SPI bus is still free. */
        gt911_sync_sensitivity();
    } else {
        ESP_LOGW(TAG, "GT911 not found");
    }

    pen_queue = xQueueCreate(PEN_QUEUE_LEN, sizeof(input_pen_event_t));
    if (!pen_queue)
        ESP_LOGE(TAG, "pen queue alloc failed");

    key_queue = xQueueCreate(KEY_QUEUE_LEN, sizeof(input_key_event_t));
    if (!key_queue)
        ESP_LOGE(TAG, "key queue alloc failed");

    ESP_LOGI(TAG, "Input ready (touch=%s)", gt911_found ? "yes" : "no");
    return ESP_OK;
}

#define BR_HOLD_MS      1000     /* hold >= this -> backup/restore request */
#define VCHR_PAGE_UP    0x000B   /* vchrPageUp (chrPageUp) */
#define VCHR_PAGE_DOWN  0x000C   /* vchrPageDown (chrPageDown) */
#define VCHR_LAUNCH     0x0108   /* vchrLaunch (Home / Applications) */
#define CMD_KEY_MASK    0x0008   /* commandKeyMask */

/* Deferred-tap + long-hold handling for one UP/DOWN side button. A short tap
 * (< BR_HOLD_MS) enqueues a Palm OS virtual key event on release via
 * input_inject_key(), while a >= BR_HOLD_MS hold latches a backup/restore
 * request instead and suppresses the tap. Mirrors the middle-button hold. */
static void handle_hold_button(bool pressed, bool *last, TickType_t *press_tick,
                               bool *hold_fired, uint16_t vchr,
                               input_br_req_t req)
{
    TickType_t now = xTaskGetTickCount();
    if (pressed && !*last) {                  /* press edge: start timing, defer the key */
        *press_tick = now;
        *hold_fired = false;
    } else if (pressed && *last) {            /* held */
        if (!*hold_fired &&
            (now - *press_tick) >= pdMS_TO_TICKS(BR_HOLD_MS)) {
            br_req_pending = req;
            *hold_fired = true;
            ESP_LOGI(TAG, "%s (hold) requested",
                     req == INPUT_BR_BACKUP ? "BACKUP" : "RESTORE");
        }
    } else if (!pressed && *last) {           /* release edge */
        if (!*hold_fired) {                   /* short tap -> enqueue Palm OS key */
            input_inject_key(vchr, 0, CMD_KEY_MASK);
            ESP_LOGI(TAG, "BTN tap -> key 0x%04X", vchr);
        }
    }
    *last = pressed;
}

void input_poll(void)
{
    /* --- Touch --- */
    if (gt911_found) {
        uint8_t status = 0;
        esp_err_t err = gt911_read(GT911_STATUS_REG, &status, 1);

        /* Periodic diagnostic logging (every ~5 s at the 100 Hz poll rate) */
        if (poll_diag_count++ % 500 == 0) {
#if CONFIG_IDF_TARGET_ESP32S3
            cw2017_poll_battery();
#endif
            ESP_LOGI(TAG, "poll: err=%d status=0x%02X int=%d",
                     err, status, gpio_get_level(PIN_INT));
        }

        if (err == ESP_OK && (status & 0x80)) {
            uint8_t num_fingers = status & 0x0F;

            if (num_fingers > 0) {
                uint8_t data[8];
                if (gt911_read(GT911_TOUCH_REG, data, 8) == ESP_OK) {
                    /* Step 1 — GT911 raw -> native IT8951 panel coords. This is
                     * a FIXED hardware relation (independent of how the device
                     * is held). GT911 reports X=0-539 (short), Y=0-959 (long);
                     * the panel is addressed landscape (eink_x=0-959 along the
                     * long edge, eink_y=0-539). The GT911 short axis (gt_x) runs
                     * opposite the panel Y axis, so invert it. */
                    uint16_t gt_x = ((uint16_t)data[1] << 8) | data[0];
                    uint16_t gt_y = ((uint16_t)data[3] << 8) | data[2];

                    int eink_x = (int)gt_y;
                    int eink_y = (layout_panel_h() - 1) - (int)gt_x;

                    /* Step 2 — native panel pixel -> Palm coords via the active
                     * layout (the same transform display.c renders through, so
                     * a tap lands on whatever was drawn there, for any rotation/
                     * placement). Returns false when the touch is in dead space
                     * (no region) — treated like a pen-up so a stroke that
                     * leaves a region ends cleanly. palm_y<160 = screen,
                     * 160..219 = silkscreen; both inject via the same pen path
                     * and the ROM dispatches. */
                    int palm_x, palm_y;
                    if (layout_panel_to_palm(eink_x, eink_y, &palm_x, &palm_y)) {
                        /* Stream the stroke: pen-down on the press transition,
                         * then a fresh point whenever the position changes.
                         * Graffiti recognition needs the whole path — the ROM
                         * collector samples the live pen point until pen-up. A
                         * still tap produces no moves, so taps and silkscreen
                         * buttons behave exactly as before (single down + up). */
                        if (!last_pen_down) {
                            pen_push((int16_t)palm_x, (int16_t)palm_y, false);
                            ESP_LOGI(TAG, "pen DOWN gt=%d,%d eink=%d,%d palm=%d,%d",
                                     gt_x, gt_y, eink_x, eink_y, palm_x, palm_y);
                            last_pen_down = true;
                        } else if (palm_x != last_px || palm_y != last_py) {
                            pen_push((int16_t)palm_x, (int16_t)palm_y, false);
                        }
                        last_px = (int16_t)palm_x;
                        last_py = (int16_t)palm_y;
                    } else if (last_pen_down) {
                        pen_push(-1, -1, true);
                        last_pen_down = false;
                        last_px = last_py = -1;
                    }
                }
            } else {
                if (last_pen_down) {
                    pen_push(-1, -1, true);
                    last_pen_down = false;
                    last_px = last_py = -1;
                }
            }

            gt911_write_byte(GT911_STATUS_REG, 0x00);
        }
    }

    /* --- Hardware buttons (active low). UP/DOWN (Left/Right) tap = Palm page
     * up/down (vchrPageUp/vchrPageDown via EvtEnqueueKey); a >= 1s hold latches
     * a backup (UP) / restore (DOWN) request that the CPU task confirms with an
     * on-panel countdown (see main.c).
     * MIDDLE is the POWER / Home button: a short tap (< 1s) enqueues
     * vchrLaunch (Home / Applications Launcher), while a hold of >= 1s while
     * awake injects vchrHardPower so the running OS sleeps. Waking from sleep is
     * handled by the ESP32 light-sleep GPIO wake + a >=1s re-hold check in the
     * CPU task (main.c), which then pulses the matrix POWER key — while asleep
     * this input task isn't running, so no asleep branch is needed. */
    bool btn_l = !gpio_get_level(PIN_BTN_L);
    bool btn_m = !gpio_get_level(PIN_BTN_M);
    bool btn_r = !gpio_get_level(PIN_BTN_R);

    handle_hold_button(btn_l, &last_btn_l, &l_press_tick, &l_hold_fired,
                       VCHR_PAGE_UP, INPUT_BR_BACKUP);

    /* Middle button: short tap (< 1s) = Home (vchrLaunch); awake long-hold
     * (>= 1s) = Power -> sleep. Both are delivered via EvtEnqueueKey (the
     * running event loop ignores the matrix->event path for power). */
    #define MID_POWER_HOLD_MS 1000
    static TickType_t mid_press_tick = 0;
    static bool mid_power_fired = false;   /* this press already produced a POWER action */
    if (btn_m && !last_btn_m) {                 /* press edge */
        mid_press_tick = xTaskGetTickCount();
        mid_power_fired = false;
    } else if (btn_m && last_btn_m) {           /* held */
        if (!mid_power_fired &&
            (xTaskGetTickCount() - mid_press_tick) >= pdMS_TO_TICKS(MID_POWER_HOLD_MS)) {
            power_key_pending = true;
            mid_power_fired = true;
            ESP_LOGI(TAG, "POWER (hold) -> vchrHardPower");
        }
    } else if (!btn_m && last_btn_m) {          /* release edge */
        if (!mid_power_fired) {
            input_inject_key(VCHR_LAUNCH, 0, CMD_KEY_MASK);
            ESP_LOGI(TAG, "POWER (tap) -> vchrLaunch");
        }
    }
    last_btn_m = btn_m;

    handle_hold_button(btn_r, &last_btn_r, &r_press_tick, &r_hold_fired,
                       VCHR_PAGE_DOWN, INPUT_BR_RESTORE);
}
