#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

esp_err_t led_init(void);
esp_err_t led_set(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);
esp_err_t led_all_off(void);

/** Set LED from a normalised axis value (-1..+1).
 *  positive→red, negative→blue, near-zero→dim green */
esp_err_t led_set_axis(uint8_t idx, float value);

/** Called before deep sleep – clears LEDs and releases RMT. */
void      led_shutdown(void);
