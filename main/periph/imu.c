#include "imu.h"
#include "../config.h"
#include "led.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "IMU";

#define LSM6_WHO_AM_I   0x0F
#define LSM6_CTRL1_XL   0x10
#define LSM6_CTRL2_G    0x11    /* gyroscope ODR / FS config */
#define LSM6_CTRL3_C    0x12
#define LSM6_OUTX_L_G   0x22    /* gyro  X low byte (X,Y,Z contiguous) */
#define LSM6_OUTX_L_A   0x28    /* accel X low byte (X,Y,Z contiguous) */
#define LSM6_SENS_MG    0.061f  /* mg/LSB    for ±2 g      (accel) */
#define LSM6_SENS_MDPS  8.75f   /* mdps/LSB  for ±250 °/s  (gyro)  */

static spi_device_handle_t s_spi = NULL;
static volatile bool       s_stream = false;

/* ── SPI helpers ─────────────────────────────────────────────────────────── */
static void lsm6_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg & 0x7F, val };
    spi_transaction_t t = { .length = 16, .tx_buffer = buf };
    spi_device_transmit(s_spi, &t);
}

static void lsm6_read_burst(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t tx[17] = {0};
    uint8_t rx[17] = {0};
    tx[0] = 0x80 | reg;
    spi_transaction_t t = {
        .length    = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(s_spi, &t);
    memcpy(buf, rx + 1, len);
}

/* ── Public API ──────────────────────────────────────────────────────────── */
esp_err_t imu_init(void)
{
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz  = CFG_IMU_CLOCK_HZ,
        .mode            = 3,
        .spics_io_num    = CFG_IMU_CS,
        .queue_size      = 1,
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans= 1,
    };
    esp_err_t ret = spi_bus_add_device(CFG_SPI_HOST, &dev_cfg, &s_spi);
    if (ret != ESP_OK) return ret;

    /* Wait for WHO_AM_I */
    for (int tries = 0; tries < 5; tries++) {
        uint8_t who;
        lsm6_read_burst(LSM6_WHO_AM_I, &who, 1);
        if (who == 0x6C) {
            lsm6_write(LSM6_CTRL2_G,  0x40);  /* 104 Hz, ±250 °/s */
            lsm6_write(LSM6_CTRL1_XL, 0x40);  /* 104 Hz, ±2 g     */
            lsm6_write(LSM6_CTRL3_C,  0x44);  /* BDU + IF_INC     */
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_LOGI(TAG, "LSM6DSOTR found");
            return ESP_OK;
        }
        ESP_LOGW(TAG, "WHO_AM_I=0x%02X, retry %d/5", who, tries + 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGE(TAG, "LSM6DSOTR not responding");
    return ESP_ERR_NOT_FOUND;
}

esp_err_t imu_read(imu_data_t *out)
{
    if (!s_spi) return ESP_ERR_INVALID_STATE;
    uint8_t raw[6];

    /* gyroscope – 6 bytes starting at OUTX_L_G */
    lsm6_read_burst(LSM6_OUTX_L_G, raw, 6);
    int16_t gx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t gy = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t gz = (int16_t)((raw[5] << 8) | raw[4]);
    out->gx_dps = gx * LSM6_SENS_MDPS / 1000.0f;
    out->gy_dps = gy * LSM6_SENS_MDPS / 1000.0f;
    out->gz_dps = gz * LSM6_SENS_MDPS / 1000.0f;

    /* accelerometer – 6 bytes starting at OUTX_L_A */
    lsm6_read_burst(LSM6_OUTX_L_A, raw, 6);
    int16_t ax = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ay = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t az = (int16_t)((raw[5] << 8) | raw[4]);
    out->ax_g = ax * LSM6_SENS_MG / 1000.0f;
    out->ay_g = ay * LSM6_SENS_MG / 1000.0f;
    out->az_g = az * LSM6_SENS_MG / 1000.0f;

    return ESP_OK;
}

void imu_stream_set(bool enable)
{
    s_stream = enable;
}

void imu_task(void *arg)
{
    while (1) {
        imu_data_t d;
        if (imu_read(&d) == ESP_OK) {
            if (s_stream) {
                printf("IMU  ax=%.3f g  ay=%.3f g  az=%.3f g"
                       "  gx=%.2f d/s  gy=%.2f d/s  gz=%.2f d/s\n",
                       d.ax_g, d.ay_g, d.az_g,
                       d.gx_dps, d.gy_dps, d.gz_dps);
                led_set_axis(0, d.ax_g);
                led_set_axis(1, d.ay_g);
                led_set_axis(2, d.az_g);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
