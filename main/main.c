#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "led_strip.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include "driver/spi_master.h"

/* ── LED config ─────────────────────────────────────────────────────────── */
#define LED_GPIO        25
#define LED_COUNT       3
#define STEP_MS         333     /* chase step period → full cycle ~1 s */

/* white at 20% brightness: 20% of 255 ≈ 51 */
static const uint8_t COLOURS[LED_COUNT][3] = {
    {2, 2, 2},   /* LED 0 */
    {2, 2, 2},   /* LED 1 */
    {2, 2, 2},   /* LED 2 */
};

/* ── CAN config ─────────────────────────────────────────────────────────── */
#define CAN_TX_GPIO      GPIO_NUM_4
#define CAN_RX_GPIO      GPIO_NUM_5
#define CAN_BITRATE      500000   /* 500 kbit/s */
#define CAN_RX_QUEUE_LEN 16

/* ── IMU (LSM6DSOTR) SPI config ─────────────────────────────────────────── */
#define IMU_SPI_HOST    SPI2_HOST
#define IMU_PIN_SCK     GPIO_NUM_0
#define IMU_PIN_MOSI    GPIO_NUM_3
#define IMU_PIN_MISO    GPIO_NUM_2
#define IMU_PIN_CS      GPIO_NUM_10
#define IMU_CLOCK_HZ    (1 * 1000 * 1000)   /* 1 MHz */
#define IMU_READ_MS     100                  /* print interval */

/* LSM6DSO register map */
#define LSM6_WHO_AM_I   0x0F    /* expected: 0x6C */
#define LSM6_CTRL1_XL   0x10    /* accel control */
#define LSM6_CTRL3_C    0x12    /* IF_INC auto-increment enable */
#define LSM6_OUTX_L_A   0x28    /* accel X low (6 bytes: XL XH YL YH ZL ZH) */

/* Accel sensitivity for ±2 g full-scale: 0.061 mg/LSB */
#define LSM6_ACCEL_SENS_MG  0.061f

static const char *TAG_IMU = "IMU";
static const char *TAG_CAN = "CAN";
static twai_node_handle_t s_twai = NULL;

/* Flat queue item – safe to copy across ISR/task boundary */
typedef struct {
    twai_frame_header_t header;
    uint8_t             data[64];
} can_rx_item_t;

static QueueHandle_t s_rx_queue;

/* ── LSM6DSO SPI helpers ─────────────────────────────────────────────────── */
static void lsm6_write(spi_device_handle_t spi, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg & 0x7F, val };
    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = buf,
    };
    spi_device_transmit(spi, &t);
}

static void lsm6_read_burst(spi_device_handle_t spi, uint8_t reg,
                             uint8_t *buf, size_t len)
{
    /* Fixed max size: 1 cmd byte + up to 16 data bytes */
    uint8_t tx[17] = {0};
    uint8_t rx[17] = {0};
    tx[0] = 0x80 | reg;

    spi_transaction_t t = {
        .length    = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(spi, &t);
    memcpy(buf, rx + 1, len);
}

/* ── IMU task ────────────────────────────────────────────────────────────── */
static void imu_task(void *arg)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = IMU_PIN_MOSI,
        .miso_io_num   = IMU_PIN_MISO,
        .sclk_io_num   = IMU_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(IMU_SPI_HOST, &bus_cfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz  = IMU_CLOCK_HZ,
        .mode            = 3,          /* CPOL=1, CPHA=1 */
        .spics_io_num    = IMU_PIN_CS,
        .queue_size      = 1,
        .cs_ena_pretrans  = 1,
        .cs_ena_posttrans = 1,
    };
    spi_device_handle_t spi;
    ESP_ERROR_CHECK(spi_bus_add_device(IMU_SPI_HOST, &dev_cfg, &spi));

    /* Verify WHO_AM_I – retry until found */
    while (1) {
        uint8_t who;
        lsm6_read_burst(spi, LSM6_WHO_AM_I, &who, 1);
        ESP_LOGI(TAG_IMU, "WHO_AM_I = 0x%02X", who);
        if (who == 0x6C) break;
        ESP_LOGE(TAG_IMU, "WHO_AM_I mismatch, retrying...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG_IMU, "LSM6DSOTR found");

    lsm6_write(spi, LSM6_CTRL1_XL, 0x40);   /* accel 104 Hz, ±2 g */
    lsm6_write(spi, LSM6_CTRL3_C,  0x44);   /* BDU + IF_INC */
    vTaskDelay(pdMS_TO_TICKS(10));

    while (1) {
        uint8_t raw[6];
        lsm6_read_burst(spi, LSM6_OUTX_L_A, raw, 6);

        int16_t ax = (int16_t)((raw[1] << 8) | raw[0]);
        int16_t ay = (int16_t)((raw[3] << 8) | raw[2]);
        int16_t az = (int16_t)((raw[5] << 8) | raw[4]);

        float ax_g = ax * LSM6_ACCEL_SENS_MG / 1000.0f;
        float ay_g = ay * LSM6_ACCEL_SENS_MG / 1000.0f;
        float az_g = az * LSM6_ACCEL_SENS_MG / 1000.0f;

        printf("IMU  ax=%.3f g  ay=%.3f g  az=%.3f g\n", ax_g, ay_g, az_g);
        vTaskDelay(pdMS_TO_TICKS(IMU_READ_MS));
    }
}

/* ── TWAI RX callback (ISR context) ─────────────────────────────────────── */
static bool IRAM_ATTR on_rx_done(twai_node_handle_t handle,
                                  const twai_rx_done_event_data_t *edata,
                                  void *user_ctx)
{
    can_rx_item_t item = {0};
    twai_frame_t frame = {
        .buffer     = item.data,
        .buffer_len = sizeof(item.data),
    };

    if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) {
        return false;
    }

    item.header = frame.header;

    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(s_rx_queue, &item, &hp);
    return hp == pdTRUE;
}

/* ── LED chase task ──────────────────────────────────────────────────────── */
static void led_task(void *arg)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num         = LED_GPIO,
        .max_leds               = LED_COUNT,
        .led_model              = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out       = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src        = RMT_CLK_SRC_DEFAULT,
        .resolution_hz  = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    led_strip_handle_t strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip));
    led_strip_clear(strip);

    int active = 0;
    while (1) {
        led_strip_clear(strip);
        led_strip_set_pixel(strip, active,
                            COLOURS[active][0],
                            COLOURS[active][1],
                            COLOURS[active][2]);
        led_strip_refresh(strip);
        active = (active + 1) % LED_COUNT;
        vTaskDelay(pdMS_TO_TICKS(STEP_MS));
    }
}

/* ── CAN RX → serial task ────────────────────────────────────────────────── */
static void can_rx_task(void *arg)
{
    can_rx_item_t item;

    while (1) {
        if (xQueueReceive(s_rx_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        printf("%s 0x%0*"PRIx32" [%d]",
               item.header.ide ? "EXT" : "STD",
               item.header.ide ? 8     : 3,
               item.header.id,
               item.header.dlc);

        if (item.header.rtr) {
            printf("  (RTR)");
        } else {
            for (int i = 0; i < item.header.dlc; i++) {
                printf(" %02X", item.data[i]);
            }
        }
        printf("\n");
    }
}

/* ── app_main ────────────────────────────────────────────────────────────── */
void app_main(void)
{
    /* --- TWAI / CAN init --- */
    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx               = CAN_TX_GPIO,
        .io_cfg.rx               = CAN_RX_GPIO,
        .io_cfg.quanta_clk_out   = GPIO_NUM_NC,
        .io_cfg.bus_off_indicator = GPIO_NUM_NC,
        .bit_timing.bitrate      = CAN_BITRATE,
        .tx_queue_depth          = 1,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &s_twai));

    s_rx_queue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(can_rx_item_t));
    twai_event_callbacks_t cbs = { .on_rx_done = on_rx_done };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_twai, &cbs, NULL));

    ESP_ERROR_CHECK(twai_node_enable(s_twai));
    ESP_LOGI(TAG_CAN, "TWAI started – TX GPIO%d  RX GPIO%d  %d kbit/s",
             CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE / 1000);

    /* --- start tasks --- */
    xTaskCreate(led_task,    "led",    2048, NULL, 5, NULL);
    xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 5, NULL);
    xTaskCreate(imu_task,    "imu",    4096, NULL, 5, NULL);
}
