#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * gpio_cmd – generic GPIO control for test commands
 *
 * Commands exposed via cmd_handler:
 *   GPIO SET <pin> HIGH|LOW         drive pin as output at a fixed level
 *   GPIO GET <pin>                  read pin level (configured as input)
 *   GPIO TOGGLE <pin> <freq_hz>     toggle pin via esp_timer (precise half-period)
 *   GPIO TOGGLE <pin> STOP          stop toggling one pin
 *   GPIO STOP ALL                   stop all active toggles
 *
 * Up to GPIO_CMD_MAX_TOGGLE pins may toggle simultaneously.
 */

#define GPIO_CMD_MAX_TOGGLE  4

/** Configure pin as output and drive HIGH (true) or LOW (false). */
esp_err_t gpio_cmd_set(int pin, bool high);

/** Configure pin as input and read its current level into *out. */
esp_err_t gpio_cmd_get(int pin, bool *out);

/**
 * Start toggling pin at freq_hz using a high-resolution esp_timer.
 * Replaces any existing toggle on the same pin.
 * freq_hz range: 1 – 50 000 Hz.
 */
esp_err_t gpio_cmd_toggle(int pin, uint32_t freq_hz);

/** Stop toggling a specific pin (drives it LOW after stopping). */
esp_err_t gpio_cmd_toggle_stop(int pin);

/** Stop all active toggles. Called from sleep path. */
void gpio_cmd_toggle_stop_all(void);
