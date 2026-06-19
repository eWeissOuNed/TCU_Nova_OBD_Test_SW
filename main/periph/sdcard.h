#pragma once
#include "esp_err.h"

/** Mount SD card on SPI bus. Returns ESP_ERR_NOT_SUPPORTED until implemented. */
esp_err_t sdcard_init(void);
esp_err_t sdcard_deinit(void);

/** List root directory. buf receives newline-separated filenames. */
esp_err_t sdcard_list(char *buf, size_t buf_len);
