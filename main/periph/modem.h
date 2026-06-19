#pragma once
#include "esp_err.h"

/** Initialise UART and EG915. Returns ESP_ERR_NOT_SUPPORTED until implemented. */
esp_err_t modem_init(void);
esp_err_t modem_deinit(void);

/** Send an AT command, wait up to timeout_ms, copy response into buf. */
esp_err_t modem_at(const char *cmd, char *resp_buf, size_t buf_len,
                   uint32_t timeout_ms);
