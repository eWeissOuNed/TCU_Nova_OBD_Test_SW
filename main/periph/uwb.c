/* uwb.c – DW3000 / DWM3000 minimal driver
 *
 * Implements SPI bring-up and DEV_ID verification.
 * TWR ranging is a TODO once SPI comms are confirmed working.
 *
 * SPI frame (short-address mode, 2-byte header):
 *   Byte 0: [bit7: 0=read/1=write] [bits6:1: 6-bit register file ID] [bit0: 0=short]
 *   Byte 1: [bit7: 0] [bits6:0: 7-bit sub-register offset]
 *   Bytes 2…N: data (MISO for reads)
 *
 * DEV_ID register: file=0x00, offset=0x00, 4 bytes, little-endian
 *   Expected: 0xDECA0302 (DW3000 production silicon)
 */
#include "uwb.h"
#include "../config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "UWB";

#define DW3000_DEV_ID_EXPECTED  0xDECA0302U

static spi_device_handle_t s_spi = NULL;

/* ── SPI helpers ─────────────────────────────────────────────────────────── */

/**
 * Short-address register read.
 *   file_id  : 6-bit register file address (0x00–0x3F)
 *   sub_reg  : 7-bit offset within the file  (0x00–0x7F)
 *   buf      : output buffer
 *   len      : number of bytes to read (max 16 per call)
 */
static esp_err_t dw3000_read(uint8_t file_id, uint8_t sub_reg,
                              uint8_t *buf, size_t len)
{
    if (!s_spi)             return ESP_ERR_INVALID_STATE;
    if (len == 0 || len > 16) return ESP_ERR_INVALID_ARG;

    uint8_t tx[18] = {0};
    uint8_t rx[18] = {0};

    /* Header: read=0, 6-bit file_id in bits[6:1], short-addr bit[0]=0 */
    tx[0] = (uint8_t)((file_id & 0x3F) << 1);
    /* Sub-register: bit[7]=0, offset in bits[6:0] */
    tx[1] = (uint8_t)(sub_reg & 0x7F);

    spi_transaction_t t = {
        .length    = (len + 2) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t ret = spi_device_transmit(s_spi, &t);
    if (ret != ESP_OK) return ret;

    /* DW3000 begins driving MISO during the second header byte (after it has
     * decoded the first), so data arrives one byte earlier than expected.  */
    memcpy(buf, rx + 1, len);
    return ESP_OK;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t uwb_init(void)
{
    if (s_spi) {
        ESP_LOGW(TAG, "already initialised");
        return ESP_OK;
    }

    /* Release RST pin if wired (CFG_UWB_RST = GPIO_NUM_NC = -1 means not connected) */
    const int rst_pin = (int)(CFG_UWB_RST);
    if (rst_pin >= 0) {
        /* pin_bit_mask assigned outside the initialiser so the compiler
         * cannot constant-fold the shift at compile time (avoids both
         * shift-count-negative and shift-count-overflow errors when
         * CFG_UWB_RST is GPIO_NUM_NC = -1 and the branch is dead code). */
        gpio_config_t rst_cfg = {
            .pin_bit_mask = 0,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        /* Use volatile to prevent the compiler constant-folding
         * (unsigned)(-1) at compile time when CFG_UWB_RST=GPIO_NUM_NC. */
        volatile unsigned vpin = (unsigned)rst_pin;
        rst_cfg.pin_bit_mask = 1ULL << vpin;
        gpio_config(&rst_cfg);
        gpio_set_level((gpio_num_t)rst_pin, 0);     /* assert reset */
        vTaskDelay(pdMS_TO_TICKS(2));
        gpio_set_level((gpio_num_t)rst_pin, 1);     /* release reset */
        vTaskDelay(pdMS_TO_TICKS(5));               /* wait for PLL lock */
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz  = 1000000,  /* 1 MHz for init – safe across all DW3000 states */
        .mode            = 0,        /* DW3000: CPOL=0, CPHA=0 */
        .spics_io_num    = CFG_UWB_CS,
        .queue_size      = 1,
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans= 2,
    };
    esp_err_t ret = spi_bus_add_device(CFG_SPI_HOST, &dev_cfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* DW3000 WAKEUP: hold CS low for ≥500 µs to exit INIT_RC/SLEEP state.
     * We do a dummy 0-byte transaction which asserts CS for the pre/post
     * trans time; then delay to guarantee the 500 µs minimum.              */
    {
        spi_transaction_t wake = {
            .length    = 0,
            .tx_buffer = NULL,
            .rx_buffer = NULL,
            .flags     = SPI_TRANS_USE_TXDATA,
        };
        spi_device_transmit(s_spi, &wake);  /* CS toggled, ignore result */
        vTaskDelay(pdMS_TO_TICKS(5));       /* wait for IDLE_RC / PLL    */
    }

    /* Verify device ID */
    uint32_t dev_id = 0;
    ret = uwb_read_id(&dev_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DEV_ID read failed: %s", esp_err_to_name(ret));
        spi_bus_remove_device(s_spi);
        s_spi = NULL;
        return ret;
    }

    if (dev_id == DW3000_DEV_ID_EXPECTED) {
        ESP_LOGI(TAG, "DW3000 found  DEV_ID=0x%08"PRIX32, dev_id);
    } else {
        ESP_LOGW(TAG, "DEV_ID=0x%08"PRIX32"  (expected 0x%08X) – wrong device or no module fitted",
                 dev_id, DW3000_DEV_ID_EXPECTED);
    }

    return ESP_OK;
}

esp_err_t uwb_deinit(void)
{
    if (s_spi) {
        spi_bus_remove_device(s_spi);
        s_spi = NULL;
    }
    return ESP_OK;
}

esp_err_t uwb_read_id(uint32_t *dev_id)
{
    if (!dev_id) return ESP_ERR_INVALID_ARG;
    if (!s_spi)  return ESP_ERR_INVALID_STATE;

    uint8_t buf[4] = {0};
    esp_err_t ret = dw3000_read(0x00, 0x00, buf, 4);
    if (ret != ESP_OK) return ret;

    /* DW3000 registers are little-endian */
    *dev_id = (uint32_t)buf[0]
            | ((uint32_t)buf[1] << 8)
            | ((uint32_t)buf[2] << 16)
            | ((uint32_t)buf[3] << 24);
    return ESP_OK;
}

esp_err_t uwb_range(uint32_t *result_mm)
{
    (void)result_mm;
    ESP_LOGW(TAG, "TWR ranging not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}
