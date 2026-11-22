#include "wifi_ntp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "palm_config.h"

static const char *TAG = "wifi_ntp";

#define BIT_GOT_IP  (1 << 0)
#define BIT_FAIL    (1 << 1)
#define BIT_CANCEL  (1 << 2)
#define BIT_DONE    (1 << 3)

#define WIFI_CONNECT_TIMEOUT_MS  15000
#define SNTP_POLL_STEP_MS          500
#define SNTP_POLL_STEPS             30   /* 15s total */
#define MAX_WIFI_RETRIES             5

static wifi_ntp_config_t s_cfg;
static bool              s_configured = false;
static bool              s_infra_inited = false;
static esp_netif_t      *s_sta_netif = NULL;
static EventGroupHandle_t s_eg = NULL;

static volatile bool             s_active = false;
static volatile bool             s_synced = false;
static volatile bool             s_pending = false;
static volatile wifi_ntp_state_t s_state = WIFI_NTP_DISABLED;
static char                      s_err[64] = "";
static int                       s_retry_num = 0;

static void strip_inline_comment(char *s)
{
    if (!s) return;
    char *c = strpbrk(s, "#;");
    if (c) *c = '\0';
    size_t n = strlen(s);
    while (n && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r'))
        s[--n] = '\0';
}

bool wifi_ntp_parse_config(const char *cfg_text, wifi_ntp_config_t *out)
{
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    snprintf(out->tz, sizeof(out->tz), "UTC0");
    snprintf(out->ntp_server, sizeof(out->ntp_server), "pool.ntp.org");

    if (cfg_text) {
        config_get_value(cfg_text, "wifi.ssid", out->ssid, sizeof(out->ssid));
        config_get_value(cfg_text, "wifi.password", out->password, sizeof(out->password));

        char buf[64];
        if (config_get_value(cfg_text, "wifi.tz", buf, sizeof(buf))) {
            strip_inline_comment(buf);
            if (buf[0]) snprintf(out->tz, sizeof(out->tz), "%s", buf);
        }
        if (config_get_value(cfg_text, "wifi.ntp", buf, sizeof(buf))) {
            strip_inline_comment(buf);
            if (buf[0]) snprintf(out->ntp_server, sizeof(out->ntp_server), "%s", buf);
        }
    }

    setenv("TZ", out->tz, 1);
    tzset();
    return out->ssid[0] != '\0';
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)data;
        if (xEventGroupGetBits(s_eg) & BIT_CANCEL) {
            xEventGroupSetBits(s_eg, BIT_FAIL);
        } else if (s_retry_num < MAX_WIFI_RETRIES) {
            s_retry_num++;
            ESP_LOGW(TAG, "STA disconnected (reason=%d), retry %d/%d",
                     disc ? disc->reason : -1, s_retry_num, MAX_WIFI_RETRIES);
            esp_wifi_connect();
        } else {
            snprintf(s_err, sizeof(s_err), "Wi-Fi disconnect (reason %d)",
                     disc ? disc->reason : -1);
            xEventGroupSetBits(s_eg, BIT_FAIL);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Connected, IP=" IPSTR, IP2STR(&ev->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_eg, BIT_GOT_IP);
    }
}

static esp_err_t ensure_network_infra(void)
{
    if (s_infra_inited) return ESP_OK;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        snprintf(s_err, sizeof(s_err), "NVS init: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        snprintf(s_err, sizeof(s_err), "netif init: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        snprintf(s_err, sizeof(s_err), "event loop: %s", esp_err_to_name(err));
        return err;
    }

    if (!s_sta_netif)
        s_sta_netif = esp_netif_create_default_wifi_sta();
    if (!s_sta_netif) {
        snprintf(s_err, sizeof(s_err), "netif STA create failed");
        return ESP_FAIL;
    }

    s_infra_inited = true;
    return ESP_OK;
}

static void ntp_sync_task(void *param)
{
    (void)param;
    bool wifi_inited = false;
    bool wifi_started = false;
    esp_event_handler_instance_t wifi_inst = NULL;
    esp_event_handler_instance_t ip_inst = NULL;

    ESP_LOGI(TAG, "Syncing NTP via SSID '%s' (server=%s, tz=%s)",
             s_cfg.ssid, s_cfg.ntp_server, s_cfg.tz);

    if (ensure_network_infra() != ESP_OK) {
        s_state = WIFI_NTP_FAILED;
        goto cleanup;
    }

    ESP_LOGI(TAG, "Free internal heap before Wi-Fi: %u bytes",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    init_cfg.nvs_enable = 0;
    esp_err_t err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) {
        snprintf(s_err, sizeof(s_err), "wifi init: %s", esp_err_to_name(err));
        s_state = WIFI_NTP_FAILED;
        goto cleanup;
    }
    wifi_inited = true;

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL, &wifi_inst);
    if (err == ESP_OK)
        err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                  &wifi_event_handler, NULL, &ip_inst);
    if (err != ESP_OK) {
        snprintf(s_err, sizeof(s_err), "event reg: %s", esp_err_to_name(err));
        s_state = WIFI_NTP_FAILED;
        goto cleanup;
    }

    wifi_config_t wcfg;
    memset(&wcfg, 0, sizeof(wcfg));
    size_t slen = strlen(s_cfg.ssid);
    if (slen > sizeof(wcfg.sta.ssid)) slen = sizeof(wcfg.sta.ssid);
    memcpy(wcfg.sta.ssid, s_cfg.ssid, slen);
    size_t plen = strlen(s_cfg.password);
    if (plen > sizeof(wcfg.sta.password)) plen = sizeof(wcfg.sta.password);
    memcpy(wcfg.sta.password, s_cfg.password, plen);
    wcfg.sta.threshold.authmode = s_cfg.password[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    wcfg.sta.pmf_cfg.capable = true;
    wcfg.sta.pmf_cfg.required = false;

    s_retry_num = 0;
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    if (err == ESP_OK) err = esp_wifi_start();
    if (err != ESP_OK) {
        snprintf(s_err, sizeof(s_err), "wifi start: %s", esp_err_to_name(err));
        s_state = WIFI_NTP_FAILED;
        goto cleanup;
    }
    wifi_started = true;

    EventBits_t bits = xEventGroupWaitBits(
        s_eg, BIT_GOT_IP | BIT_FAIL | BIT_CANCEL,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & BIT_CANCEL) {
        snprintf(s_err, sizeof(s_err), "cancelled");
        s_state = WIFI_NTP_FAILED;
        goto cleanup;
    }
    if (!(bits & BIT_GOT_IP)) {
        if (!s_err[0]) snprintf(s_err, sizeof(s_err), "Wi-Fi connect timeout");
        s_state = WIFI_NTP_FAILED;
        ESP_LOGW(TAG, "%s", s_err);
        goto cleanup;
    }

    s_state = WIFI_NTP_SYNCING;
    esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(s_cfg.ntp_server);
    err = esp_netif_sntp_init(&sntp_cfg);
    if (err != ESP_OK) {
        snprintf(s_err, sizeof(s_err), "SNTP init: %s", esp_err_to_name(err));
        s_state = WIFI_NTP_FAILED;
        goto cleanup;
    }

    esp_err_t wait_err = ESP_ERR_TIMEOUT;
    bool cancelled = false;
    for (int i = 0; i < SNTP_POLL_STEPS; i++) {
        if (xEventGroupGetBits(s_eg) & BIT_CANCEL) {
            cancelled = true;
            break;
        }
        wait_err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(SNTP_POLL_STEP_MS));
        if (wait_err != ESP_ERR_TIMEOUT) break;
    }
    esp_netif_sntp_deinit();

    if (cancelled) {
        snprintf(s_err, sizeof(s_err), "cancelled");
        s_state = WIFI_NTP_FAILED;
    } else {
        time_t now = time(NULL);
        if (wait_err == ESP_OK && now > 1700000000L) {
            struct tm tm_now;
            char tbuf[32] = "";
            if (localtime_r(&now, &tm_now))
                strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S %Z", &tm_now);
            ESP_LOGI(TAG, "NTP synced: %s (unix=%lld)", tbuf, (long long)now);
            s_synced = true;
            __sync_synchronize();
            s_pending = true;
            s_state = WIFI_NTP_SYNCED;
            s_err[0] = '\0';
        } else {
            snprintf(s_err, sizeof(s_err), "SNTP wait: %s", esp_err_to_name(wait_err));
            s_state = WIFI_NTP_FAILED;
            ESP_LOGW(TAG, "%s", s_err);
        }
    }

cleanup:
    if (wifi_started) {
        esp_wifi_disconnect();
        esp_wifi_stop();
    }
    if (ip_inst)   esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_inst);
    if (wifi_inst) esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_inst);
    if (wifi_inited) esp_wifi_deinit();

    s_active = false;
    xEventGroupSetBits(s_eg, BIT_DONE);
    vTaskDelete(NULL);
}

bool wifi_ntp_start(const wifi_ntp_config_t *cfg)
{
    if (!cfg || !cfg->ssid[0]) {
        s_configured = false;
        s_state = WIFI_NTP_DISABLED;
        return false;
    }
    s_cfg = *cfg;
    s_configured = true;
    setenv("TZ", s_cfg.tz, 1);
    tzset();
    if (!s_eg) {
        s_eg = xEventGroupCreate();
        if (!s_eg) {
            snprintf(s_err, sizeof(s_err), "event group alloc failed");
            s_state = WIFI_NTP_FAILED;
            return false;
        }
    }
    return wifi_ntp_trigger();
}

bool wifi_ntp_trigger(void)
{
    if (!s_configured || !s_cfg.ssid[0]) {
        snprintf(s_err, sizeof(s_err), "not configured in /palm.cfg");
        return false;
    }
    if (!s_eg) {
        s_eg = xEventGroupCreate();
        if (!s_eg) return false;
    }
    if (s_active) return false;

    xEventGroupClearBits(s_eg, BIT_GOT_IP | BIT_FAIL | BIT_CANCEL | BIT_DONE);
    s_err[0] = '\0';
    s_pending = false;
    s_state = WIFI_NTP_CONNECTING;
    s_active = true;

    BaseType_t ok = xTaskCreatePinnedToCore(ntp_sync_task, "wifi_ntp", 5120,
                                            NULL, 2, NULL, 0);
    if (ok != pdPASS) {
        s_active = false;
        snprintf(s_err, sizeof(s_err), "task create failed");
        s_state = WIFI_NTP_FAILED;
        xEventGroupSetBits(s_eg, BIT_DONE);
        return false;
    }
    return true;
}

void wifi_ntp_cancel(uint32_t timeout_ms)
{
    if (!s_active || !s_eg) return;
    xEventGroupSetBits(s_eg, BIT_CANCEL);
    xEventGroupWaitBits(s_eg, BIT_DONE, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
}

bool wifi_ntp_is_active(void) { return s_active; }
bool wifi_ntp_is_synced(void) { return s_synced; }

bool wifi_ntp_take_pending(void)
{
    if (!s_pending) return false;
    __sync_synchronize();
    s_pending = false;
    return true;
}

wifi_ntp_state_t wifi_ntp_state(void) { return s_state; }
const char      *wifi_ntp_last_error(void) { return s_err; }

const char *wifi_ntp_state_str(wifi_ntp_state_t st)
{
    switch (st) {
    case WIFI_NTP_DISABLED:   return "disabled";
    case WIFI_NTP_IDLE:       return "idle";
    case WIFI_NTP_CONNECTING: return "connecting";
    case WIFI_NTP_SYNCING:    return "syncing";
    case WIFI_NTP_SYNCED:     return "synced";
    case WIFI_NTP_FAILED:     return "failed";
    default:                  return "unknown";
    }
}
