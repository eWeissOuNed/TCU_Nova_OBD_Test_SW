#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float ax_g,  ay_g,  az_g;   /* acceleration  in g      */
    float gx_dps, gy_dps, gz_dps; /* angular rate  in °/s  */
} imu_data_t;

/** Initialise SPI bus and LSM6DSOTR. Call once after spi_bus_initialize(). */
esp_err_t imu_init(void);

/** Read latest acceleration. Blocks until data ready (≤ 10 ms). */
esp_err_t imu_read(imu_data_t *out);

/** Enable/disable continuous streaming to console (50 ms interval). */
void      imu_stream_set(bool enable);

/** FreeRTOS task function – pass to xTaskCreate. */
void      imu_task(void *arg);
