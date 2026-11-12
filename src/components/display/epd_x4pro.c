/* epd_x4pro.c — E-Paper display driver for Xteink X4 Pro (800x480).
 *
 * Implements the it8951_* driver interface expected by display.c on top of the
 * Xteink X4 Pro 4.3" 800x480 e-paper panel. Supports all three controller
 * variants shipped across X4 Pro / X4 batches via runtime auto-detection:
 *   - UC8279 (LUT versions 0x02, 0x03, 0x67, 0x68, 0x69)
 *   - UC8179 (LUT version 0x01)
 *   - SSD1677 (fallback)
 *
 * Pinout (Xteink X4 Pro):
 *   SCLK = GPIO 12, MOSI = GPIO 11, CS = GPIO 13,
 *   DC   = GPIO 18, RST  = GPIO 14, BUSY = GPIO 6
 */

#include "it8951.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "epd_x4pro";

#define PIN_EPD_SCLK    12
#define PIN_EPD_MOSI    11
#define PIN_EPD_CS      13
#define PIN_EPD_DC      18
#define PIN_EPD_RST     14
#define PIN_EPD_BUSY    6

#define EPD_WIDTH             800
#define EPD_HEIGHT            480
#define EPD_WIDTH_BYTES       (EPD_WIDTH / 8)                  /* 100 bytes/row */
#define EPD_BUFFER_SIZE       (EPD_WIDTH_BYTES * EPD_HEIGHT)   /* 48,000 bytes  */
#define UC_ADDRESSED_HEIGHT   600
#define UC8279_GATE_OFFSET    120
#define DMA_CHUNK_BYTES       4096

typedef enum {
    PANEL_UC8279 = 0,
    PANEL_UC8179,
    PANEL_SSD1677,
} x4pro_panel_t;

/* UC8279 / UC8179 commands */
#define UC_CMD_PANEL_SETTING        0x00
#define UC_CMD_POWER_OFF            0x02
#define UC_CMD_POWER_OFF_SEQUENCE   0x03
#define UC_CMD_POWER_ON             0x04
#define UC_CMD_BOOSTER_SOFT_START   0x06
#define UC_CMD_DEEP_SLEEP           0x07
#define UC_CMD_DTM1                 0x10
#define UC_CMD_DISPLAY_REFRESH      0x12
#define UC_CMD_DTM2                 0x13
#define UC_CMD_PLL                  0x30
#define UC_CMD_VCOM_DATA_INTERVAL   0x50
#define UC_CMD_RESOLUTION           0x61
#define UC_CMD_GATE_SOURCE_START    0x65
#define UC_CMD_VER                  0x70
#define UC_CMD_FLG                  0x71
#define UC_CMD_PARTIAL_WINDOW       0x90
#define UC_CMD_PARTIAL_IN           0x91
#define UC_CMD_PARTIAL_OUT          0x92
#define UC_CMD_RMTP                 0xA2
#define UC_CMD_CCSET                0xE0
#define UC_CMD_GATE_SCAN            0xE1
#define UC_CMD_POWER_SAVE           0xE3
#define UC_CMD_TSSET                0xE5

/* SSD1677 commands */
#define SSD_CMD_DRIVER_OUTPUT_CTRL  0x01
#define SSD_CMD_BOOSTER_SOFT_START  0x0C
#define SSD_CMD_DEEP_SLEEP          0x10
#define SSD_CMD_DATA_ENTRY_MODE     0x11
#define SSD_CMD_SOFT_RESET          0x12
#define SSD_CMD_TEMP_SENSOR_CTRL    0x18
#define SSD_CMD_WRITE_TEMP          0x1A
#define SSD_CMD_MASTER_ACTIVATION   0x20
#define SSD_CMD_DISPLAY_UPDATE_1    0x21
#define SSD_CMD_DISPLAY_UPDATE_2    0x22
#define SSD_CMD_WRITE_RAM_BW        0x24
#define SSD_CMD_WRITE_RAM_RED       0x26
#define SSD_CMD_BORDER_WAVEFORM     0x3C
#define SSD_CMD_SET_RAM_X_RANGE     0x44
#define SSD_CMD_SET_RAM_Y_RANGE     0x45
#define SSD_CMD_AUTO_WRITE_BW_RAM   0x46
#define SSD_CMD_AUTO_WRITE_RED_RAM  0x47
#define SSD_CMD_SET_RAM_X_COUNTER   0x4E
#define SSD_CMD_SET_RAM_Y_COUNTER   0x4F

static spi_device_handle_t s_spi;
static x4pro_panel_t       s_panel = PANEL_UC8279;
static uint8_t             s_lut_ver = 0x68;
static uint8_t            *s_fb_bw;        /* 800x480 1bpp framebuffer (1=white, 0=black) */
static uint8_t            *s_dma_buf;      /* 4 KB internal DMA staging buffer */
static bool                s_screen_on;
static bool                s_old_plane_valid;
static bool                s_need_full_clear = true;

/* 4x4 Bayer ordered dither thresholds (0..15) for mapping 4bpp grays (1..14)
 * into the 1bpp e-paper framebuffer while keeping 0 solid black and 15 solid white. */
static const uint8_t s_bayer4x4[4][4] = {
    {  0,  8,  2, 10 },
    { 12,  4, 14,  6 },
    {  3, 11,  1,  9 },
    { 15,  7, 13,  5 },
};

/* ------------------------------------------------------------------ *
 *  Bit-banged 3-wire SPI panel probe (runs once before SPI bus init)
 * ------------------------------------------------------------------ */

typedef struct {
    uint8_t flg;
    uint8_t ver[5];
} probe_sample_t;

static void probe_clk_delay(void)
{
    esp_rom_delay_us(1);
}

static void probe_write_byte(uint8_t val)
{
    for (int bit = 0; bit < 8; bit++) {
        gpio_set_level(PIN_EPD_MOSI, (val & 0x80) ? 1 : 0);
        probe_clk_delay();
        gpio_set_level(PIN_EPD_SCLK, 1);
        probe_clk_delay();
        gpio_set_level(PIN_EPD_SCLK, 0);
        val <<= 1;
    }
}

static uint8_t probe_read_byte(void)
{
    uint8_t val = 0;
    for (int bit = 0; bit < 8; bit++) {
        probe_clk_delay();
        val = (uint8_t)((val << 1) | (gpio_get_level(PIN_EPD_MOSI) ? 1 : 0));
        gpio_set_level(PIN_EPD_SCLK, 1);
        probe_clk_delay();
        gpio_set_level(PIN_EPD_SCLK, 0);
    }
    return val;
}

static void probe_cmd_read(uint8_t cmd, uint8_t *out, size_t len)
{
    gpio_set_direction(PIN_EPD_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_EPD_DC, 0);
    gpio_set_level(PIN_EPD_CS, 0);
    probe_clk_delay();
    probe_write_byte(cmd);
    gpio_set_level(PIN_EPD_DC, 1);
    gpio_set_direction(PIN_EPD_MOSI, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_EPD_MOSI, GPIO_PULLUP_ONLY);
    probe_clk_delay();
    for (size_t i = 0; i < len; i++) {
        out[i] = probe_read_byte();
    }
    gpio_set_level(PIN_EPD_CS, 1);
    gpio_set_direction(PIN_EPD_MOSI, GPIO_MODE_OUTPUT);
}

static void probe_read_pass(uint32_t reset_low_ms, probe_sample_t *sample)
{
    gpio_hold_dis(PIN_EPD_RST);
    gpio_set_direction(PIN_EPD_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_EPD_CS, 1);
    gpio_set_direction(PIN_EPD_SCLK, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_EPD_SCLK, 0);
    gpio_set_direction(PIN_EPD_DC, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_EPD_DC, 0);
    gpio_set_direction(PIN_EPD_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_EPD_BUSY, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_EPD_RST, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(PIN_EPD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(reset_low_ms));
    gpio_set_level(PIN_EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(30));

    probe_cmd_read(UC_CMD_FLG, &sample->flg, 1);
    probe_cmd_read(UC_CMD_VER, sample->ver, sizeof(sample->ver));
}

static bool bytes_uniform(const uint8_t *b, size_t n)
{
    for (size_t i = 1; i < n; i++) {
        if (b[i] != b[0]) return false;
    }
    return true;
}

static bool sample_structured(const probe_sample_t *s)
{
    bool driven_idle = (s->flg != 0x00 && s->flg != 0xFF && (s->flg & 0x01) != 0);
    return driven_idle && !bytes_uniform(s->ver, sizeof(s->ver));
}

static void detect_x4pro_panel(void)
{
    probe_sample_t pass1 = {0}, pass2 = {0};
    probe_read_pass(1, &pass1);
    if (!sample_structured(&pass1)) {
        vTaskDelay(pdMS_TO_TICKS(2));
        probe_read_pass(50, &pass1);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    probe_read_pass(sample_structured(&pass1) ? 50 : 1, &pass2);

    ESP_LOGI(TAG, "panel probe: VER1=%02X %02X %02X %02X %02X FLG1=%02X | VER2=%02X %02X %02X %02X %02X FLG2=%02X",
             pass1.ver[0], pass1.ver[1], pass1.ver[2], pass1.ver[3], pass1.ver[4], pass1.flg,
             pass2.ver[0], pass2.ver[1], pass2.ver[2], pass2.ver[3], pass2.ver[4], pass2.flg);

    bool same_ver = (memcmp(pass1.ver, pass2.ver, sizeof(pass1.ver)) == 0);
    bool uc_confirmed = (same_ver && sample_structured(&pass1) && sample_structured(&pass2));
    uint8_t lut_id = pass2.ver[2];

    if (uc_confirmed) {
        if (lut_id == 0x01) {
            s_panel = PANEL_UC8179;
            s_lut_ver = lut_id;
            ESP_LOGI(TAG, "detected panel: UC8179 (LUT=0x%02X)", s_lut_ver);
            return;
        }
        if (lut_id == 0x02 || lut_id == 0x03 || lut_id == 0x67 || lut_id == 0x68 || lut_id == 0x69) {
            s_panel = PANEL_UC8279;
            s_lut_ver = lut_id;
            ESP_LOGI(TAG, "detected panel: UC8279 (LUT=0x%02X)", s_lut_ver);
            return;
        }
    }

    /* Check if BUSY is LOW after reset (UC8279/UC8179 BUSY is active-low -> HIGH when idle,
     * whereas SSD1677 BUSY is active-high -> LOW when idle). */
    int busy_idle = gpio_get_level(PIN_EPD_BUSY);
    if (busy_idle == 0) {
        s_panel = PANEL_SSD1677;
        ESP_LOGI(TAG, "detected panel: SSD1677 (BUSY idle low)");
    } else {
        s_panel = PANEL_UC8279;
        s_lut_ver = 0x68;
        ESP_LOGI(TAG, "fallback panel: UC8279 (BUSY idle high)");
    }
}

/* ------------------------------------------------------------------ *
 *  Low-level SPI bus helpers
 * ------------------------------------------------------------------ */

static void spi_send_raw(const void *data, size_t len)
{
    if (len == 0) return;
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_spi, &t));
}

static void epd_cmd(uint8_t cmd)
{
    gpio_set_level(PIN_EPD_DC, 0);
    gpio_set_level(PIN_EPD_CS, 0);
    spi_send_raw(&cmd, 1);
    gpio_set_level(PIN_EPD_CS, 1);
}

static void epd_data_byte(uint8_t d)
{
    gpio_set_level(PIN_EPD_DC, 1);
    gpio_set_level(PIN_EPD_CS, 0);
    spi_send_raw(&d, 1);
    gpio_set_level(PIN_EPD_CS, 1);
}

static void epd_cmd_data(uint8_t cmd, const uint8_t *data, size_t len)
{
    gpio_set_level(PIN_EPD_CS, 0);
    gpio_set_level(PIN_EPD_DC, 0);
    spi_send_raw(&cmd, 1);
    if (len > 0 && data != NULL) {
        gpio_set_level(PIN_EPD_DC, 1);
        spi_send_raw(data, len);
    }
    gpio_set_level(PIN_EPD_CS, 1);
}

static void epd_reset(uint32_t low_ms, uint32_t settle_ms)
{
    gpio_hold_dis(PIN_EPD_RST);
    gpio_set_level(PIN_EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_EPD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(low_ms));
    gpio_set_level(PIN_EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10 + settle_ms));
}

/* UC8279 / UC8179 BUSY is active-LOW (0 = busy, 1 = ready) */
static bool uc_wait_ready(const char *op, uint32_t timeout_ms)
{
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(PIN_EPD_BUSY) == 0) {
        if ((esp_timer_get_time() - t0) > (int64_t)timeout_ms * 1000) {
            ESP_LOGW(TAG, "UC BUSY timeout on %s", op ? op : "?");
            return false;
        }
        vTaskDelay(1);
    }
    return true;
}

static bool uc_wait_busy(const char *op)
{
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(PIN_EPD_BUSY) == 1) {
        if ((esp_timer_get_time() - t0) > 100 * 1000) {
            break; /* BUSY pulse may already have completed for fast ops */
        }
        esp_rom_delay_us(250);
    }
    return uc_wait_ready(op, 15000);
}

/* SSD1677 BUSY is active-HIGH (1 = busy, 0 = ready) */
static bool ssd_wait_ready(const char *op, uint32_t timeout_ms)
{
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(PIN_EPD_BUSY) == 1) {
        if ((esp_timer_get_time() - t0) > (int64_t)timeout_ms * 1000) {
            ESP_LOGW(TAG, "SSD1677 BUSY timeout on %s", op ? op : "?");
            return false;
        }
        vTaskDelay(1);
    }
    return true;
}

/* ------------------------------------------------------------------ *
 *  UC8279 & UC8179 plane streaming + refresh
 * ------------------------------------------------------------------ */

static void uc_fill_plane(uint8_t cmd, uint8_t val)
{
    memset(s_dma_buf, val, DMA_CHUNK_BYTES);
    size_t rem = (size_t)EPD_WIDTH_BYTES * UC_ADDRESSED_HEIGHT; /* 60,000 bytes */
    epd_cmd(cmd);
    gpio_set_level(PIN_EPD_DC, 1);
    gpio_set_level(PIN_EPD_CS, 0);
    while (rem > 0) {
        size_t chunk = (rem > DMA_CHUNK_BYTES) ? DMA_CHUNK_BYTES : rem;
        spi_send_raw(s_dma_buf, chunk);
        rem -= chunk;
    }
    gpio_set_level(PIN_EPD_CS, 1);
}

static void uc8279_stream_plane(uint8_t cmd, const uint8_t *plane, bool invert)
{
    epd_cmd(cmd);
    gpio_set_level(PIN_EPD_DC, 1);
    gpio_set_level(PIN_EPD_CS, 0);

    /* 1. Gate offset: 120 rows of 0xFF (12,000 bytes) */
    memset(s_dma_buf, 0xFF, DMA_CHUNK_BYTES);
    size_t gate_rem = (size_t)EPD_WIDTH_BYTES * UC8279_GATE_OFFSET;
    while (gate_rem > 0) {
        size_t chunk = (gate_rem > DMA_CHUNK_BYTES) ? DMA_CHUNK_BYTES : gate_rem;
        spi_send_raw(s_dma_buf, chunk);
        gate_rem -= chunk;
    }

    /* 2. Active 480 rows (48,000 bytes) */
    size_t off = 0;
    while (off < EPD_BUFFER_SIZE) {
        size_t chunk = (EPD_BUFFER_SIZE - off > DMA_CHUNK_BYTES) ? DMA_CHUNK_BYTES : (EPD_BUFFER_SIZE - off);
        if (!invert) {
            memcpy(s_dma_buf, plane + off, chunk);
        } else {
            for (size_t i = 0; i < chunk; i++) {
                s_dma_buf[i] = (uint8_t)~plane[off + i];
            }
        }
        spi_send_raw(s_dma_buf, chunk);
        off += chunk;
    }

    gpio_set_level(PIN_EPD_CS, 1);
}

static void uc8179_stream_plane(uint8_t cmd, const uint8_t *plane, bool invert)
{
    epd_cmd(cmd);
    gpio_set_level(PIN_EPD_DC, 1);
    gpio_set_level(PIN_EPD_CS, 0);

    /* 1. Active 480 rows in reverse Y order (row 479 down to 0) */
    size_t staged = 0;
    for (int row = EPD_HEIGHT - 1; row >= 0; row--) {
        const uint8_t *src = plane + (size_t)row * EPD_WIDTH_BYTES;
        if (!invert) {
            memcpy(s_dma_buf + staged, src, EPD_WIDTH_BYTES);
        } else {
            for (size_t i = 0; i < EPD_WIDTH_BYTES; i++) {
                s_dma_buf[staged + i] = (uint8_t)~src[i];
            }
        }
        staged += EPD_WIDTH_BYTES;
        if (staged + EPD_WIDTH_BYTES > DMA_CHUNK_BYTES || row == 0) {
            spi_send_raw(s_dma_buf, staged);
            staged = 0;
        }
    }

    /* 2. Trailing 120 rows (rows 480..599) of 0xFF (12,000 bytes) */
    memset(s_dma_buf, 0xFF, DMA_CHUNK_BYTES);
    size_t pad_rem = (size_t)EPD_WIDTH_BYTES * (UC_ADDRESSED_HEIGHT - EPD_HEIGHT);
    while (pad_rem > 0) {
        size_t chunk = (pad_rem > DMA_CHUNK_BYTES) ? DMA_CHUNK_BYTES : pad_rem;
        spi_send_raw(s_dma_buf, chunk);
        pad_rem -= chunk;
    }

    gpio_set_level(PIN_EPD_CS, 1);
}

static void uc8279_init_controller(void)
{
    static const uint8_t panel[]     = { 0x37, 0x4D };
    static const uint8_t res[]       = { 0x03, 0x20, 0x02, 0x58 };
    static const uint8_t gs_start[]  = { 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t pwr_off[]   = { 0x20 };
    static const uint8_t pll[]       = { 0x0E };
    static const uint8_t gate_scan[] = { 0x02 };

    epd_reset(10, 50);
    epd_cmd_data(UC_CMD_PANEL_SETTING, panel, sizeof(panel));
    epd_cmd_data(UC_CMD_RESOLUTION, res, sizeof(res));
    epd_cmd_data(UC_CMD_GATE_SOURCE_START, gs_start, sizeof(gs_start));
    epd_cmd_data(UC_CMD_POWER_OFF_SEQUENCE, pwr_off, sizeof(pwr_off));
    epd_cmd_data(UC_CMD_PLL, pll, sizeof(pll));
    epd_cmd_data(UC_CMD_GATE_SCAN, gate_scan, sizeof(gate_scan));
    s_screen_on = false;
}

static void uc8179_init_controller(void)
{
    static const uint8_t panel[]     = { 0x3F, 0x0A };
    static const uint8_t res[]       = { 0x03, 0x20, 0x02, 0x58 };
    static const uint8_t gs_start[]  = { 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t pwr_off[]   = { 0x20 };
    static const uint8_t booster[]   = { 0x25, 0x25, 0x3C, 0x25 };
    static const uint8_t gate_scan[] = { 0x02 };
    static const uint8_t pwr_save[]  = { 0x22 };

    epd_reset(10, 50);
    epd_cmd_data(UC_CMD_PANEL_SETTING, panel, sizeof(panel));
    epd_cmd_data(UC_CMD_RESOLUTION, res, sizeof(res));
    epd_cmd_data(UC_CMD_GATE_SOURCE_START, gs_start, sizeof(gs_start));
    epd_cmd_data(UC_CMD_POWER_OFF_SEQUENCE, pwr_off, sizeof(pwr_off));
    epd_cmd_data(UC_CMD_BOOSTER_SOFT_START, booster, sizeof(booster));
    epd_cmd_data(UC_CMD_GATE_SCAN, gate_scan, sizeof(gate_scan));
    epd_cmd_data(UC_CMD_POWER_SAVE, pwr_save, sizeof(pwr_save));
    s_screen_on = false;
}

static esp_err_t uc8279_refresh(bool full_refresh)
{
    bool fast = (!full_refresh && s_old_plane_valid && !s_need_full_clear);
    uc8279_stream_plane(UC_CMD_DTM2, s_fb_bw, false);
    if (!fast) {
        uc_fill_plane(UC_CMD_DTM1, 0xFF);
    }

    epd_cmd(UC_CMD_VCOM_DATA_INTERVAL);
    epd_data_byte(fast ? 0xD7 : 0x97);
    epd_cmd(UC_CMD_CCSET);
    epd_data_byte(0x02);
    epd_cmd(UC_CMD_TSSET);
    epd_data_byte(fast ? 0x5A : 0x1E);
    if (fast) {
        epd_cmd(UC_CMD_POWER_OFF_SEQUENCE);
        epd_data_byte(0x20);
        epd_cmd(UC_CMD_GATE_SCAN);
        epd_data_byte(0x02);
    }

    if (!s_screen_on) {
        epd_cmd(UC_CMD_POWER_ON);
        if (!uc_wait_busy("8279_PON")) return ESP_ERR_TIMEOUT;
        s_screen_on = true;
    }

    if (fast) {
        static const uint8_t win[] = { 0x00, 0x00, 0x03, 0x1F, 0x00, 0x78, 0x02, 0x57, 0x01 };
        epd_cmd(UC_CMD_PARTIAL_IN);
        epd_cmd_data(UC_CMD_PARTIAL_WINDOW, win, sizeof(win));
    }

    static const uint8_t otp_panel[] = { 0x17, 0x4D };
    epd_cmd_data(UC_CMD_PANEL_SETTING, otp_panel, sizeof(otp_panel));
    epd_cmd(UC_CMD_DISPLAY_REFRESH);
    if (!uc_wait_busy("8279_DRF")) {
        if (fast) epd_cmd(UC_CMD_PARTIAL_OUT);
        return ESP_ERR_TIMEOUT;
    }
    if (fast) {
        epd_cmd(UC_CMD_PARTIAL_OUT);
    }

    uc8279_stream_plane(UC_CMD_DTM1, s_fb_bw, false);
    s_old_plane_valid = true;
    s_need_full_clear = false;
    return ESP_OK;
}

static esp_err_t uc8179_refresh(bool full_refresh)
{
    bool fast = (!full_refresh && s_old_plane_valid && !s_need_full_clear);
    uc8179_stream_plane(UC_CMD_DTM2, s_fb_bw, false);
    if (!fast) {
        uc_fill_plane(UC_CMD_DTM1, 0xFF);
    }

    static const uint8_t active_cdi[] = { 0x29, 0x07 };
    static const uint8_t idle_cdi[]   = { 0xA9, 0x07 };
    static const uint8_t panel[]      = { 0x1F, 0x0A };

    epd_cmd_data(UC_CMD_VCOM_DATA_INTERVAL, active_cdi, sizeof(active_cdi));
    epd_cmd(UC_CMD_CCSET);
    epd_data_byte(0x02);
    epd_cmd(UC_CMD_TSSET);
    epd_data_byte(fast ? 0x5A : 0x1E);
    epd_cmd_data(UC_CMD_PANEL_SETTING, panel, sizeof(panel));
    if (fast) {
        epd_cmd(UC_CMD_POWER_OFF_SEQUENCE);
        epd_data_byte(0x20);
        epd_cmd(UC_CMD_GATE_SCAN);
        epd_data_byte(0x02);
    }

    if (!s_screen_on) {
        epd_cmd(UC_CMD_POWER_ON);
        if (!uc_wait_busy("8179_PON")) return ESP_ERR_TIMEOUT;
        s_screen_on = true;
    }

    if (fast) epd_cmd(UC_CMD_PARTIAL_IN);
    epd_cmd(UC_CMD_DISPLAY_REFRESH);
    if (!uc_wait_busy("8179_DRF")) {
        if (fast) epd_cmd(UC_CMD_PARTIAL_OUT);
        return ESP_ERR_TIMEOUT;
    }
    if (fast) epd_cmd(UC_CMD_PARTIAL_OUT);

    epd_cmd_data(UC_CMD_VCOM_DATA_INTERVAL, idle_cdi, sizeof(idle_cdi));
    uc8179_stream_plane(UC_CMD_DTM1, s_fb_bw, false);
    s_old_plane_valid = true;
    s_need_full_clear = false;
    return ESP_OK;
}

/* ------------------------------------------------------------------ *
 *  SSD1677 init, plane streaming + refresh
 * ------------------------------------------------------------------ */

static void ssd1677_set_ram_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    y = EPD_HEIGHT - y - h;
    epd_cmd(SSD_CMD_DATA_ENTRY_MODE);
    epd_data_byte(0x01); /* X inc, Y dec */

    uint8_t xr[4] = { (uint8_t)(x & 0xFF), (uint8_t)(x >> 8),
                      (uint8_t)((x + w - 1) & 0xFF), (uint8_t)((x + w - 1) >> 8) };
    epd_cmd_data(SSD_CMD_SET_RAM_X_RANGE, xr, sizeof(xr));

    uint8_t yr[4] = { (uint8_t)((y + h - 1) & 0xFF), (uint8_t)((y + h - 1) >> 8),
                      (uint8_t)(y & 0xFF), (uint8_t)(y >> 8) };
    epd_cmd_data(SSD_CMD_SET_RAM_Y_RANGE, yr, sizeof(yr));

    uint8_t xc[2] = { (uint8_t)(x & 0xFF), (uint8_t)(x >> 8) };
    epd_cmd_data(SSD_CMD_SET_RAM_X_COUNTER, xc, sizeof(xc));

    uint8_t yc[2] = { (uint8_t)((y + h - 1) & 0xFF), (uint8_t)((y + h - 1) >> 8) };
    epd_cmd_data(SSD_CMD_SET_RAM_Y_COUNTER, yc, sizeof(yc));
}

static void ssd1677_write_ram(uint8_t cmd, const uint8_t *plane)
{
    epd_cmd(cmd);
    gpio_set_level(PIN_EPD_DC, 1);
    gpio_set_level(PIN_EPD_CS, 0);
    size_t off = 0;
    while (off < EPD_BUFFER_SIZE) {
        size_t chunk = (EPD_BUFFER_SIZE - off > DMA_CHUNK_BYTES) ? DMA_CHUNK_BYTES : (EPD_BUFFER_SIZE - off);
        memcpy(s_dma_buf, plane + off, chunk);
        spi_send_raw(s_dma_buf, chunk);
        off += chunk;
    }
    gpio_set_level(PIN_EPD_CS, 1);
}

static void ssd1677_init_controller(void)
{
    epd_reset(2, 20);
    epd_cmd(SSD_CMD_SOFT_RESET);
    ssd_wait_ready("SOFT_RESET", 5000);

    epd_cmd(SSD_CMD_TEMP_SENSOR_CTRL);
    epd_data_byte(0x80);

    static const uint8_t booster[] = { 0xAE, 0xC7, 0xC3, 0xC0, 0x40 };
    epd_cmd_data(SSD_CMD_BOOSTER_SOFT_START, booster, sizeof(booster));

    static const uint8_t drv_out[] = { (uint8_t)((EPD_HEIGHT - 1) & 0xFF),
                                       (uint8_t)((EPD_HEIGHT - 1) >> 8), 0x02 };
    epd_cmd_data(SSD_CMD_DRIVER_OUTPUT_CTRL, drv_out, sizeof(drv_out));

    epd_cmd(SSD_CMD_BORDER_WAVEFORM);
    epd_data_byte(0x01);

    ssd1677_set_ram_area(0, 0, EPD_WIDTH, EPD_HEIGHT);
    epd_cmd(SSD_CMD_AUTO_WRITE_BW_RAM);
    epd_data_byte(0xF7);
    ssd_wait_ready("AUTO_BW", 5000);

    epd_cmd(SSD_CMD_AUTO_WRITE_RED_RAM);
    epd_data_byte(0xF7);
    ssd_wait_ready("AUTO_RED", 5000);
    s_screen_on = false;
}

static esp_err_t ssd1677_refresh(bool full_refresh)
{
    bool fast = (!full_refresh && s_old_plane_valid && !s_need_full_clear);
    ssd1677_set_ram_area(0, 0, EPD_WIDTH, EPD_HEIGHT);
    ssd1677_write_ram(SSD_CMD_WRITE_RAM_BW, s_fb_bw);
    if (!fast) {
        ssd1677_write_ram(SSD_CMD_WRITE_RAM_RED, s_fb_bw);
    }

    epd_cmd(SSD_CMD_DISPLAY_UPDATE_1);
    epd_data_byte(fast ? 0x00 : 0x40);

    uint8_t disp_mode = 0;
    if (!s_screen_on) {
        s_screen_on = true;
        disp_mode |= 0xC0;
    }
    disp_mode |= fast ? 0x1C : 0x34;

    epd_cmd(SSD_CMD_DISPLAY_UPDATE_2);
    epd_data_byte(disp_mode);
    epd_cmd(SSD_CMD_MASTER_ACTIVATION);
    if (!ssd_wait_ready("SSD_REFRESH", 10000)) {
        s_screen_on = false;
        return ESP_ERR_TIMEOUT;
    }

    ssd1677_set_ram_area(0, 0, EPD_WIDTH, EPD_HEIGHT);
    ssd1677_write_ram(SSD_CMD_WRITE_RAM_RED, s_fb_bw);
    s_old_plane_valid = true;
    s_need_full_clear = false;
    return ESP_OK;
}

static esp_err_t epd_refresh(uint16_t mode)
{
    bool full = (mode == IT8951_MODE_INIT);
    switch (s_panel) {
        case PANEL_UC8179:  return uc8179_refresh(full);
        case PANEL_SSD1677: return ssd1677_refresh(full);
        case PANEL_UC8279:
        default:            return uc8279_refresh(full);
    }
}

/* ------------------------------------------------------------------ *
 *  Public it8951_* API used by display.c
 * ------------------------------------------------------------------ */

static uint16_t s_load_x, s_load_y, s_load_w, s_load_h;
static uint16_t s_load_cx, s_load_cy;

static inline void fb_set_pixel(int px, int py, bool white)
{
    if ((unsigned)px >= EPD_WIDTH || (unsigned)py >= EPD_HEIGHT) return;
    uint8_t *byte = &s_fb_bw[py * EPD_WIDTH_BYTES + (px >> 3)];
    uint8_t mask = (uint8_t)(0x80u >> (px & 7));
    if (white) *byte |= mask;
    else       *byte &= (uint8_t)~mask;
}

esp_err_t it8951_init(it8951_dev_info_t *out_info)
{
    /* 1. Probe panel type using 3-wire bit-bang before attaching hardware SPI */
    detect_x4pro_panel();

    /* 2. Configure control GPIOs */
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PIN_EPD_CS) | (1ULL << PIN_EPD_DC) | (1ULL << PIN_EPD_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out_cfg);
    gpio_set_level(PIN_EPD_CS, 1);
    gpio_set_level(PIN_EPD_DC, 1);
    gpio_set_level(PIN_EPD_RST, 1);

    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << PIN_EPD_BUSY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&in_cfg);

    /* 3. Initialize SPI2 bus at 20 MHz Mode 0 */
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_EPD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_EPD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DMA_CHUNK_BYTES + 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi));

    /* 4. Allocate 800x480 1bpp framebuffer in PSRAM and 4 KB DMA staging buffer */
    if (!s_fb_bw) {
        s_fb_bw = heap_caps_malloc(EPD_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_fb_bw) s_fb_bw = heap_caps_malloc(EPD_BUFFER_SIZE, MALLOC_CAP_8BIT);
    }
    if (!s_dma_buf) {
        s_dma_buf = heap_caps_malloc(DMA_CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!s_fb_bw || !s_dma_buf) {
        ESP_LOGE(TAG, "Failed to allocate display buffers");
        return ESP_ERR_NO_MEM;
    }
    memset(s_fb_bw, 0xFF, EPD_BUFFER_SIZE);

    /* 5. Initialize panel controller */
    switch (s_panel) {
        case PANEL_UC8179:  uc8179_init_controller();  break;
        case PANEL_SSD1677: ssd1677_init_controller(); break;
        case PANEL_UC8279:
        default:            uc8279_init_controller();  break;
    }
    s_old_plane_valid = false;
    s_need_full_clear = true;

    if (out_info) {
        memset(out_info, 0, sizeof(*out_info));
        out_info->panel_w = EPD_WIDTH;
        out_info->panel_h = EPD_HEIGHT;
        snprintf(out_info->fw_version, sizeof(out_info->fw_version), "X4Pro");
        snprintf(out_info->lut_version, sizeof(out_info->lut_version),
                 "%s-0x%02X",
                 s_panel == PANEL_UC8179 ? "UC8179" :
                 s_panel == PANEL_SSD1677 ? "SSD1677" : "UC8279",
                 s_lut_ver);
    }
    return ESP_OK;
}

void it8951_clear(uint16_t mode)
{
    if (!s_fb_bw) return;
    memset(s_fb_bw, 0xFF, EPD_BUFFER_SIZE);
    epd_refresh(mode);
}

void it8951_load_area_start(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    s_load_x = x;
    s_load_y = y;
    s_load_w = w;
    s_load_h = h;
    s_load_cx = 0;
    s_load_cy = 0;
}

void it8951_load_pixels(const uint8_t *data_4bpp, size_t len)
{
    if (!s_fb_bw || !data_4bpp) return;
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data_4bpp[i];
        for (int nib = 0; nib < 2; nib++) {
            if (s_load_cy < s_load_h && s_load_cx < s_load_w) {
                int px = (int)s_load_x + (int)s_load_cx;
                int py = (int)s_load_y + (int)s_load_cy;
                uint8_t g = (nib == 0) ? ((b >> 4) & 0xF) : (b & 0xF);
                bool white;
                if (g == 0x0)      white = false;
                else if (g == 0xF) white = true;
                else               white = (g > s_bayer4x4[py & 3][px & 3]);
                fb_set_pixel(px, py, white);
            }
            s_load_cx++;
            if (s_load_cx >= s_load_w) {
                s_load_cx = 0;
                s_load_cy++;
            }
        }
    }
}

void it8951_load_end(void)
{
}

void it8951_display_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t mode)
{
    (void)x; (void)y; (void)w; (void)h;
    epd_refresh(mode);
}

void it8951_wait_ready(void)
{
}

void it8951_fill_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t pix4)
{
    if (!s_fb_bw) return;
    uint8_t g = pix4 & 0xF;
    for (uint16_t ry = 0; ry < h; ry++) {
        int py = (int)y + (int)ry;
        if ((unsigned)py >= EPD_HEIGHT) continue;
        for (uint16_t rx = 0; rx < w; rx++) {
            int px = (int)x + (int)rx;
            if ((unsigned)px >= EPD_WIDTH) continue;
            bool white = (g == 0x0) ? false : (g == 0xF) ? true : (g > s_bayer4x4[py & 3][px & 3]);
            fb_set_pixel(px, py, white);
        }
    }
}

int64_t it8951_display_timed(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t mode)
{
    (void)x; (void)y; (void)w; (void)h;
    int64_t t0 = esp_timer_get_time();
    epd_refresh(mode);
    return esp_timer_get_time() - t0;
}
