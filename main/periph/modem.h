#pragma once
#include "esp_err.h"
#include <stdbool.h>

/** Initialise UART and PWRKEY GPIO. Does NOT power on the modem. */
esp_err_t modem_init(void);
esp_err_t modem_deinit(void);

/** Pulse PWRKEY low for 700 ms to power on the EG915.
 *  Waits up to timeout_ms for the "RDY" URC before returning.
 *  Returns ESP_OK if "RDY" received, ESP_ERR_TIMEOUT otherwise. */
esp_err_t modem_power_on(uint32_t timeout_ms);

/** Pulse PWRKEY low for 700 ms to power off the EG915 (same pulse, modem toggles). */
esp_err_t modem_power_off(void);

/** Returns true if the modem has been powered on (UART init'd + RDY seen). */
bool modem_is_on(void);

/** Send an SMS in text mode. Blocks until +CMGS confirmation or timeout. */
esp_err_t modem_sms_send(const char *number, const char *text);

/** Read signal strength. rssi_dbm receives value in dBm (0 = unknown).
 *  detail_buf receives the raw +QCSQ line with RSRP/RSRQ/SINR if not NULL. */
esp_err_t modem_signal(int *rssi_dbm, char *detail_buf, size_t detail_len);

/** Enable or disable the integrated GNSS receiver. */
esp_err_t modem_gnss_enable(bool on);

/** Read current GNSS position. Returns formatted string in out_buf.
 *  Returns ESP_FAIL with "no fix yet" if GNSS has no fix. */
esp_err_t modem_gnss_read(char *out_buf, size_t out_len);

/** Start/stop periodic signal+GPS stream printed as "MS rssi=… rsrp=… gps=…" lines.
 *  interval_ms minimum 1000. Stream task uses the same UART mutex as manual commands. */
esp_err_t modem_stream_set(bool on, uint32_t interval_ms);

/** Send an AT command, wait up to timeout_ms, copy response into buf. */
esp_err_t modem_at(const char *cmd, char *resp_buf, size_t buf_len,
                   uint32_t timeout_ms);
