/* sdcard.c – STUB
 * TODO: implement with esp_vfs_fat_sdspi_mount() once wiring is confirmed.
 * Pins: CFG_SPI_HOST (shared bus), CS=CFG_SD_CS (see config.h)
 */
#include "sdcard.h"
#include "../config.h"
#include "esp_log.h"

static const char *TAG = "SDCARD";

esp_err_t sdcard_init(void)
{
    ESP_LOGW(TAG, "SD card not yet implemented (TODO)");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t sdcard_deinit(void)
{
    return ESP_OK;
}

esp_err_t sdcard_list(char *buf, size_t buf_len)
{
    (void)buf; (void)buf_len;
    return ESP_ERR_NOT_SUPPORTED;
}
