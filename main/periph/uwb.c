/* uwb.c – STUB
 * TODO: implement DWM3000 driver (SPI Mode 0, up to 20 MHz).
 * Pins: CFG_SPI_HOST (shared bus), CS=CFG_UWB_CS, IRQ=CFG_UWB_IRQ (see config.h)
 * Reference: Qorvo DW3000 API (github.com/Qorvo/qorvo-uwb-core)
 */
#include "uwb.h"
#include "../config.h"
#include "esp_log.h"

static const char *TAG = "UWB";

esp_err_t uwb_init(void)
{
    ESP_LOGW(TAG, "DWM3000 not yet implemented (TODO)");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t uwb_deinit(void)
{
    return ESP_OK;
}

esp_err_t uwb_range(uint32_t *result_mm)
{
    (void)result_mm;
    return ESP_ERR_NOT_SUPPORTED;
}
