#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "led_strip.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"

/* ── LED config ─────────────────────────────────────────────────────────── */
#define LED_GPIO        25
#define LED_COUNT       3
#define STEP_MS         333     /* chase step period → full cycle ~1 s */

/* white at 20% brightness: 20% of 255 ≈ 51 */
static const uint8_t COLOURS[LED_COUNT][3] = {
    {51, 51, 51},   /* LED 0 */
    {51, 51, 51},   /* LED 1 */
    {51, 51, 51},   /* LED 2 */
};

/* ── CAN config ─────────────────────────────────────────────────────────── */
#define CAN_TX_GPIO     GPIO_NUM_4
#define CAN_RX_GPIO     GPIO_NUM_5
#define CAN_BITRATE     500000  /* 500 kbit/s – change to 250000 / 125000 etc. */
#define CAN_RX_QUEUE_LEN 16

static const char *TAG = "CAN";
static twai_node_handle_t s_twai = NULL;

/* Flat queue item – no pointers, safe to copy across ISR/task boundary */
typedef struct {
    twai_frame_header_t header;
    uint8_t             data[64];   /* covers classic CAN (8 B) and CAN FD (64 B) */
} can_rx_item_t;

static QueueHandle_t s_rx_queue;

/* ── TWAI RX callback (ISR context) ─────────────────────────────────────── */
static bool IRAM_ATTR on_rx_done(twai_node_handle_t handle,
                                  const twai_rx_done_event_data_t *edata,
                                  void *user_ctx)
{
    /* edata is empty – pull the frame from the hardware buffer */
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

/* ── LED task ────────────────────────────────────────────────────────────── */
static void led_task(void *arg)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num            = LED_GPIO,
        .max_leds                  = LED_COUNT,
        .led_model                 = LED_MODEL_WS2812,
        .color_component_format    = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out          = false,
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

        /* Format:
         *   STD 0x1AB [3] 01 02 03
         *   EXT 0x1FFFF123 [8] 01 02 03 04 05 06 07 08  (RTR)
         */
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
        .io_cfg.quanta_clk_out   = GPIO_NUM_NC,  /* not used */
        .io_cfg.bus_off_indicator = GPIO_NUM_NC,  /* not used */
        .bit_timing.bitrate      = CAN_BITRATE,
        .tx_queue_depth          = 1,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &s_twai));

    /* Register RX callback before enabling */
    s_rx_queue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(can_rx_item_t));
    twai_event_callbacks_t cbs = { .on_rx_done = on_rx_done };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_twai, &cbs, NULL));

    ESP_ERROR_CHECK(twai_node_enable(s_twai));
    ESP_LOGI(TAG, "TWAI started – TX GPIO%d  RX GPIO%d  %d kbit/s",
             CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE / 1000);

    /* --- start tasks --- */
    xTaskCreate(led_task,    "led",    2048, NULL, 5, NULL);
    xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 5, NULL);
}
