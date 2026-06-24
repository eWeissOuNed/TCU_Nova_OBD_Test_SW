#pragma once
#include <stdint.h>
#include "esp_err.h"

/**
 * UWB driver – Qorvo DW3000 (DWM3000 module)
 *
 * SPI:  Mode 0 (CPOL=0, CPHA=0), shared SPI2 bus, CS=CFG_UWB_CS
 * RST:  CFG_UWB_RST (active-low, released after init)
 * IRQ:  CFG_UWB_IRQ (used by ranging, not yet implemented)
 *
 * Commands:
 *   UWB INIT        – add SPI device, verify DEV_ID
 *   UWB READ ID     – read and print DEV_ID register (0x00)
 *   UWB RANGE       – TWR ranging (not yet implemented)
 */

/** Initialise DW3000: add SPI device, release RST, verify DEV_ID. */
esp_err_t uwb_init(void);

/** Release SPI device handle. */
esp_err_t uwb_deinit(void);

/**
 * Read DW3000 DEV_ID register (file 0x00, offset 0x00, 4 bytes).
 * Expected: 0xDECA0302 for DW3000 silicon.
 * Requires uwb_init() to have been called first.
 */
esp_err_t uwb_read_id(uint32_t *dev_id);

/** TWR ranging – not yet implemented, returns ESP_ERR_NOT_SUPPORTED. */
esp_err_t uwb_range(uint32_t *result_mm);
