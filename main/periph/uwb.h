#pragma once
#include "esp_err.h"

/** Initialise DWM3000 on SPI bus. Returns ESP_ERR_NOT_SUPPORTED until implemented. */
esp_err_t uwb_init(void);
esp_err_t uwb_deinit(void);

/** Trigger a TWR ranging measurement. result_mm receives the distance. */
esp_err_t uwb_range(uint32_t *result_mm);
