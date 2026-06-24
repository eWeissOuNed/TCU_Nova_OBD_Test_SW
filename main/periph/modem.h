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

/** Send an AT command, wait up to timeout_ms, copy response into buf. */
esp_err_t modem_at(const char *cmd, char *resp_buf, size_t buf_len,
                   uint32_t timeout_ms);
