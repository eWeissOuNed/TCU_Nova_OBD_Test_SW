/* modem.c – STUB
 * TODO: implement Quectel EG915 AT command driver over UART1.
 * Pins: TX=CFG_MODEM_TX, RX=CFG_MODEM_RX (see config.h)
 * Reference: Quectel EG915U AT Commands Manual
 */
#include "modem.h"
#include "../config.h"
#include "esp_log.h"

static const char *TAG = "MODEM";

esp_err_t modem_init(void)
{
    ESP_LOGW(TAG, "EG915 modem not yet implemented (TODO)");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t modem_deinit(void)
{
    return ESP_OK;
}

esp_err_t modem_at(const char *cmd, char *resp_buf, size_t buf_len,
                   uint32_t timeout_ms)
{
    (void)cmd; (void)resp_buf; (void)buf_len; (void)timeout_ms;
    return ESP_ERR_NOT_SUPPORTED;
}
