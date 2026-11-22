#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    WIFI_NTP_DISABLED = 0,
    WIFI_NTP_IDLE,
    WIFI_NTP_CONNECTING,
    WIFI_NTP_SYNCING,
    WIFI_NTP_SYNCED,
    WIFI_NTP_FAILED,
} wifi_ntp_state_t;

typedef struct {
    char ssid[33];
    char password[65];
    char tz[64];
    char ntp_server[64];
} wifi_ntp_config_t;

/* Parse wifi.ssid, wifi.password, wifi.tz (default "UTC0"), and wifi.ntp
 * (default "pool.ntp.org") from /palm.cfg text and apply TZ to the C runtime.
 * Returns true if wifi.ssid is non-empty. */
bool wifi_ntp_parse_config(const char *cfg_text, wifi_ntp_config_t *out);

/* Save the parsed Wi-Fi/NTP config and launch an immediate one-shot sync on
 * Core 0. Returns false if wifi.ssid is empty or the task fails to start. */
bool wifi_ntp_start(const wifi_ntp_config_t *cfg);

/* Trigger a fresh one-shot Wi-Fi + SNTP sync using the previously stored
 * config (e.g. from the `ntp sync` console command). */
bool wifi_ntp_trigger(void);

/* Request cancellation of an in-flight sync and wait up to `timeout_ms` for the
 * radio to shut down cleanly (used before ESP32 light sleep / power-off). */
void wifi_ntp_cancel(uint32_t timeout_ms);

bool wifi_ntp_is_active(void);
bool wifi_ntp_is_synced(void);

/* Consume the one-shot "NTP synced, ready to apply to guest Palm OS" flag. */
bool wifi_ntp_take_pending(void);

wifi_ntp_state_t wifi_ntp_state(void);
const char      *wifi_ntp_state_str(wifi_ntp_state_t st);
const char      *wifi_ntp_last_error(void);
