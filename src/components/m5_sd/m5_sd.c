#include "m5_sd.h"

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/gpio.h"
#if CONFIG_IDF_TARGET_ESP32S3
#include "driver/sdmmc_host.h"
#else
#include "driver/sdspi_host.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "m5_sd";

#if CONFIG_IDF_TARGET_ESP32S3
/* Xteink X4 Pro: 1-bit SDMMC with active-low SD power switch on GPIO 5 */
#define PIN_SD_PWR        GPIO_NUM_5
#define PIN_SDMMC_CLK     GPIO_NUM_41
#define PIN_SDMMC_CMD     GPIO_NUM_42
#define PIN_SDMMC_D0      GPIO_NUM_40
#define SD_MOUNT_ATTEMPTS 4

static bool s_sd_power_initialized = false;

static void x4pro_sd_power(bool on)
{
    gpio_hold_dis(PIN_SD_PWR);
    gpio_set_direction(PIN_SD_PWR, GPIO_MODE_OUTPUT);
    /* Active-low power switch: LOW = ON, HIGH = OFF */
    gpio_set_level(PIN_SD_PWR, on ? 0 : 1);
}

esp_err_t m5_sd_mount(sdmmc_card_t **out_card)
{
    if (out_card) *out_card = NULL;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1;
    slot.clk = PIN_SDMMC_CLK;
    slot.cmd = PIN_SDMMC_CMD;
    slot.d0  = PIN_SDMMC_D0;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mcfg = {
        .format_if_mount_failed = false,
        .max_files              = 4,
        .allocation_unit_size   = 0,
    };

    if (!s_sd_power_initialized) {
        x4pro_sd_power(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        s_sd_power_initialized = true;
    }

    sdmmc_card_t *card = NULL;
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < SD_MOUNT_ATTEMPTS; attempt++) {
        err = esp_vfs_fat_sdmmc_mount(M5_SD_MOUNT_POINT, &host, &slot, &mcfg, &card);
        if (err == ESP_OK) break;
        ESP_LOGW(TAG, "SDMMC mount attempt %d/%d failed: %s",
                 attempt + 1, SD_MOUNT_ATTEMPTS, esp_err_to_name(err));
        x4pro_sd_power(false);
        vTaskDelay(pdMS_TO_TICKS(80));
        x4pro_sd_power(true);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    if (err != ESP_OK) return err;

    if (out_card) *out_card = card;
    return ESP_OK;
}

void m5_sd_unmount(sdmmc_card_t *card)
{
    if (card) esp_vfs_fat_sdcard_unmount(M5_SD_MOUNT_POINT, card);
}

#else
/* Same wiring as the display: the microSD shares SPI3_HOST with GPIO4 as CS,
 * already driven high (deasserted) by it8951.c at display init. */
#define SD_HOST    SPI3_HOST
#define SD_CS_GPIO GPIO_NUM_4

#define SD_MOUNT_ATTEMPTS 4
#define SD_RETRY_DELAY_MS 40

esp_err_t m5_sd_mount(sdmmc_card_t **out_card)
{
    if (out_card) *out_card = NULL;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_HOST;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = SD_CS_GPIO;
    slot.host_id = SD_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mcfg = {
        .format_if_mount_failed = false,
        .max_files              = 2,  /* ROM read / installer scan open at most one
                                         file at a time (the scan closes its DIR
                                         before reading any file), so 2 is ample */
        .allocation_unit_size   = 0,
    };

    sdmmc_card_t *card = NULL;
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < SD_MOUNT_ATTEMPTS; attempt++) {
        err = esp_vfs_fat_sdspi_mount(M5_SD_MOUNT_POINT, &host, &slot, &mcfg, &card);
        if (err == ESP_OK) break;
        ESP_LOGW(TAG, "SD mount attempt %d/%d failed: %s",
                 attempt + 1, SD_MOUNT_ATTEMPTS, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(SD_RETRY_DELAY_MS));
    }
    if (err != ESP_OK) return err;

    if (out_card) *out_card = card;
    return ESP_OK;
}

void m5_sd_unmount(sdmmc_card_t *card)
{
    esp_vfs_fat_sdcard_unmount(M5_SD_MOUNT_POINT, card);
}
#endif
