#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "led_strip.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"

/* ── LED config ─────────────────────────────────────────────────────────── */
#define LED_GPIO        25
#define LED_COUNT       3
#define LED_BRIGHTNESS  0.4f

/* ── CAN config ─────────────────────────────────────────────────────────── */
#define CAN_TX_GPIO      GPIO_NUM_4
#define CAN_RX_GPIO      GPIO_NUM_5
#define CAN_STB_GPIO     GPIO_NUM_26   /* pull-up → high=standby, low=normal */
#define CAN_BITRATE      500000
#define CAN_RX_QUEUE_LEN 16

/* ── IMU (LSM6DSOTR) SPI config ─────────────────────────────────────────── */
#define IMU_SPI_HOST     SPI2_HOST
#define IMU_PIN_SCK      GPIO_NUM_0
#define IMU_PIN_MOSI     GPIO_NUM_3
#define IMU_PIN_MISO     GPIO_NUM_2
#define IMU_PIN_CS       GPIO_NUM_10
#define IMU_CLOCK_HZ     (1 * 1000 * 1000)
#define IMU_READ_MS      50

/* LSM6DSO registers */
#define LSM6_WHO_AM_I    0x0F
#define LSM6_CTRL1_XL    0x10
#define LSM6_CTRL3_C     0x12
#define LSM6_OUTX_L_A    0x28

/* Sensitivity: ±2 g, 0.061 mg/LSB */
#define LSM6_ACCEL_SENS_MG  0.061f

static const char *TAG_IMU = "IMU";
static const char *TAG_CAN = "CAN";

static twai_node_handle_t  s_twai  = NULL;
static led_strip_handle_t  s_strip = NULL;
static int64_t             s_sleep_allowed_at = 0;  /* µs timestamp */
static volatile bool       s_going_to_sleep   = false;

typedef struct {
    twai_frame_header_t header;
    uint8_t             data[64];
} can_rx_item_t;

static QueueHandle_t s_rx_queue;

/* ── helpers ─────────────────────────────────────────────────────────────── */
static uint8_t clamp_u8(float v)
{
    if (v < 0.0f) return 0;
    if (v > 255.0f) return 255;
    return (uint8_t)v;
}

/* ── LSM6DSO SPI helpers ─────────────────────────────────────────────────── */
static void lsm6_write(spi_device_handle_t spi, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg & 0x7F, val };
    spi_transaction_t t = { .length = 16, .tx_buffer = buf };
    spi_device_transmit(spi, &t);
}

static void lsm6_read_burst(spi_device_handle_t spi, uint8_t reg,
                             uint8_t *buf, size_t len)
{
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
        .clock_speed_hz   = IMU_CLOCK_HZ,
        .mode             = 3,
        .spics_io_num     = IMU_PIN_CS,
        .queue_size       = 1,
        .cs_ena_pretrans  = 1,
        .cs_ena_posttrans = 1,
    };
    spi_device_handle_t spi;
    ESP_ERROR_CHECK(spi_bus_add_device(IMU_SPI_HOST, &dev_cfg, &spi));

    while (1) {
        uint8_t who;
        lsm6_read_burst(spi, LSM6_WHO_AM_I, &who, 1);
        if (who == 0x6C) break;
        ESP_LOGE(TAG_IMU, "WHO_AM_I = 0x%02X, retrying...", who);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG_IMU, "LSM6DSOTR found");

    lsm6_write(spi, LSM6_CTRL1_XL, 0x40);   /* 104 Hz, ±2 g */
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

        /* LED0=X  LED1=Y  LED2=Z
         * positive → red, negative → blue, near-zero → dim green */
        if (!s_going_to_sleep) {
            float axes[3] = { ax_g, ay_g, az_g };
            for (int i = 0; i < LED_COUNT; i++) {
                float v   = axes[i];
                float mag = fabsf(v);
                if (mag > 1.0f) mag = 1.0f;

                uint8_t r = (v > 0.0f) ? clamp_u8(mag * 255.0f * LED_BRIGHTNESS) : 0;
                uint8_t b = (v < 0.0f) ? clamp_u8(mag * 255.0f * LED_BRIGHTNESS) : 0;
                uint8_t g = clamp_u8((1.0f - mag) * 20.0f * LED_BRIGHTNESS);

                led_strip_set_pixel(s_strip, i, r, g, b);
            }
            led_strip_refresh(s_strip);
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_READ_MS));
    }
}

/* ── TWAI RX callback (ISR context) ─────────────────────────────────────── */
static bool IRAM_ATTR on_rx_done(twai_node_handle_t handle,
                                  const twai_rx_done_event_data_t *edata,
                                  void *user_ctx)
{
    can_rx_item_t item = {0};
    twai_frame_t frame = { .buffer = item.data, .buffer_len = sizeof(item.data) };

    if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) return false;

    item.header = frame.header;
    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(s_rx_queue, &item, &hp);
    return hp == pdTRUE;
}

/* ── CAN heartbeat TX task ───────────────────────────────────────────────── */
#define HEARTBEAT_ID    0x001
#define HEARTBEAT_MS    1000

static void can_hb_task(void *arg)
{
    uint32_t counter = 0;

    while (1) {
        uint8_t payload[4] = {
            (counter >> 24) & 0xFF,
            (counter >> 16) & 0xFF,
            (counter >>  8) & 0xFF,
            (counter      ) & 0xFF,
        };

        twai_frame_t frame = {
            .header.id  = HEARTBEAT_ID,
            .header.dlc = 4,
            .header.ide = 0,
            .header.rtr = 0,
            .buffer     = payload,
            .buffer_len = sizeof(payload),
        };

        esp_err_t err = twai_node_transmit(s_twai, &frame, pdMS_TO_TICKS(10));
        if (err == ESP_OK) {
            printf("HB  0x%03X [4] %08"PRIx32"\n", HEARTBEAT_ID, counter);
        } else {
            ESP_LOGW(TAG_CAN, "HB TX failed: %s", esp_err_to_name(err));
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_MS));
    }
}

/* Sleep command: ID=0x258, 8 bytes, last 3 = 0x50 0x50 0x50 */
#define SLEEP_CMD_ID    0x102   /* 258 decimal */
static const uint8_t SLEEP_CMD_DATA[8] = { 0x00,0x00,0x00,0x00,0x00,0x50,0x50,0x50 };

/* ── CAN RX → serial task ────────────────────────────────────────────────── */
static void can_rx_task(void *arg)
{
    can_rx_item_t item;
    while (1) {
        if (xQueueReceive(s_rx_queue, &item, portMAX_DELAY) != pdTRUE) continue;

        printf("%s 0x%0*"PRIx32" [%d]",
               item.header.ide ? "EXT" : "STD",
               item.header.ide ? 8     : 3,
               item.header.id,
               item.header.dlc);

        if (item.header.rtr) {
            printf("  (RTR)");
        } else {
            for (int i = 0; i < item.header.dlc; i++) printf(" %02X", item.data[i]);
        }
        printf("\n");

        /* Sleep command check */
        if (!item.header.rtr &&
            item.header.id  == SLEEP_CMD_ID &&
            item.header.dlc == 8 &&
            memcmp(item.data, SLEEP_CMD_DATA, 8) == 0 &&
            esp_timer_get_time() >= s_sleep_allowed_at)
        {
            ESP_LOGI(TAG_CAN, "Sleep command received – deep sleep in 500 ms");
            ESP_LOGI(TAG_CAN, "Wakeup: any CAN message (RXD low on GPIO%d)", CAN_RX_GPIO);

            /* Block IMU task from touching LEDs, then give it time to finish
             * its current iteration before we delete the strip handle. */
            s_going_to_sleep = true;
            vTaskDelay(pdMS_TO_TICKS(60));   /* > IMU_READ_MS (50 ms) */

            /* TJA1042BT enters standby when TXD stays recessive (high).
             * Any CAN dominant bit pulls RXD low → wakes ESP32-C5. */
            esp_sleep_enable_ext1_wakeup(1ULL << CAN_RX_GPIO,
                                         ESP_EXT1_WAKEUP_ANY_LOW);

            led_strip_clear(s_strip);
            led_strip_refresh(s_strip);
            vTaskDelay(pdMS_TO_TICKS(100));  /* let RMT finish */
            led_strip_del(s_strip);          /* release RMT, GPIO goes low-Z */
            vTaskDelay(pdMS_TO_TICKS(400));
            /* Release STB → pull-up takes it high → transceiver enters standby */
            gpio_set_direction(CAN_STB_GPIO, GPIO_MODE_INPUT);
            esp_deep_sleep_start();
        }
    }
}

/* ── app_main ────────────────────────────────────────────────────────────── */
void app_main(void)
{
    /* After GPIO wakeup, ignore sleep command for 5 s (bus may still carry it) */
    if (esp_sleep_get_wakeup_causes() & ESP_SLEEP_WAKEUP_EXT1) {
        s_sleep_allowed_at = esp_timer_get_time() + 5000000LL;
        ESP_LOGI(TAG_CAN, "Woke from CAN – sleep cmd blocked for 5 s");
    }

    /* --- CAN transceiver: drive STB low → normal mode --- */
    gpio_config_t stb_cfg = {
        .pin_bit_mask = (1ULL << CAN_STB_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&stb_cfg);
    gpio_set_level(CAN_STB_GPIO, 0);   /* normal mode */

    /* --- LED strip init --- */
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
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip));
    led_strip_clear(s_strip);

    /* --- TWAI / CAN init --- */
    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx                = CAN_TX_GPIO,
        .io_cfg.rx                = CAN_RX_GPIO,
        .io_cfg.quanta_clk_out    = GPIO_NUM_NC,
        .io_cfg.bus_off_indicator = GPIO_NUM_NC,
        .bit_timing.bitrate       = CAN_BITRATE,
        .tx_queue_depth           = 1,
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &s_twai));

    s_rx_queue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(can_rx_item_t));
    twai_event_callbacks_t cbs = { .on_rx_done = on_rx_done };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_twai, &cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(s_twai));
    ESP_LOGI(TAG_CAN, "TWAI started – TX GPIO%d  RX GPIO%d  %d kbit/s",
             CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE / 1000);

    /* --- start tasks --- */
    xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 5, NULL);
    xTaskCreate(can_hb_task, "can_hb", 2048, NULL, 5, NULL);
    xTaskCreate(imu_task,    "imu",    4096, NULL, 5, NULL);

}
