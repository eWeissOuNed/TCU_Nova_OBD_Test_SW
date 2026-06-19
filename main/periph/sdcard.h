#pragma once
#include <stdbool.h>
#include "esp_err.h"

/** Drive enable pin low and mount FAT filesystem.
 *  SPI bus must already be initialised (spi_bus_initialize called in main). */
esp_err_t sdcard_init(void);

/** Unmount filesystem and release enable pin (high-Z → pull-up disables card). */
esp_err_t sdcard_deinit(void);

/** Returns true if the card is currently mounted. */
bool      sdcard_is_mounted(void);

/** Fill buf with a newline-separated directory listing of the root folder. */
esp_err_t sdcard_list(char *buf, size_t buf_len);

/** Append one line to /sdcard/TEST.TXT.
 *  Line format: [seq] msg\n   (seq auto-increments across calls) */
esp_err_t sdcard_write(const char *msg);

/** Read /sdcard/TEST.TXT into buf (up to buf_len-1 bytes, null-terminated). */
esp_err_t sdcard_read(char *buf, size_t buf_len);

/** Delete /sdcard/TEST.TXT. */
esp_err_t sdcard_wipe(void);

/** Manually drive the EN pin: true = low (card on), false = high-Z (card off). */
void      sdcard_set_enable(bool on);
