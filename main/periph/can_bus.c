#include "can_bus.h"
#include "../config.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "CAN";

static twai_node_handle_t s_twai = NULL;
static volatile bool      s_rx_stream = false;

/* ── RX queue ────────────────────────────────────────────────────────────── */
#define RX_QUEUE_LEN 16
typedef struct { twai_frame_header_t hdr; uint8_t data[64]; } rx_item_t;
static QueueHandle_t s_rx_q = NULL;

static bool IRAM_ATTR on_rx_done(twai_node_handle_t handle,
                                  const twai_rx_done_event_data_t *edata,
                                  void *ctx)
{
    rx_item_t item = {0};
    twai_frame_t frame = { .buffer = item.data, .buffer_len = sizeof(item.data) };
    if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) return false;
    item.hdr = frame.header;
    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(s_rx_q, &item, &hp);
    return hp == pdTRUE;
}

/* ── Sleep command ───────────────────────────────────────────────────────── */
static const uint8_t SLEEP_DATA[8] = CFG_CAN_SLEEP_DATA;

bool can_is_sleep_cmd(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    return id == CFG_CAN_SLEEP_ID && dlc == 8 &&
           memcmp(data, SLEEP_DATA, 8) == 0;
}

/* ── Public API ──────────────────────────────────────────────────────────── */
esp_err_t can_init(void)
{
    /* STB pin: output low → normal mode */
    gpio_config_t stb = {
        .pin_bit_mask = 1ULL << CFG_CAN_STB_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&stb);
    gpio_set_level(CFG_CAN_STB_GPIO, 0);

    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx                = CFG_CAN_TX_GPIO,
        .io_cfg.rx                = CFG_CAN_RX_GPIO,
        .io_cfg.quanta_clk_out    = GPIO_NUM_NC,
        .io_cfg.bus_off_indicator = GPIO_NUM_NC,
        .bit_timing.bitrate       = CFG_CAN_BITRATE,
        .tx_queue_depth           = 4,
    };
    esp_err_t ret = twai_new_node_onchip(&node_cfg, &s_twai);
    if (ret != ESP_OK) return ret;

    s_rx_q = xQueueCreate(RX_QUEUE_LEN, sizeof(rx_item_t));
    twai_event_callbacks_t cbs = { .on_rx_done = on_rx_done };
    twai_node_register_event_callbacks(s_twai, &cbs, NULL);
    ret = twai_node_enable(s_twai);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "init OK – TX=GPIO%d RX=GPIO%d %"PRIu32" kbit/s",
                 CFG_CAN_TX_GPIO, CFG_CAN_RX_GPIO, (uint32_t)(CFG_CAN_BITRATE / 1000));
    }
    return ret;
}

esp_err_t can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (!s_twai) return ESP_ERR_INVALID_STATE;
    twai_frame_t frame = {
        .header.id  = id,
        .header.dlc = len,
        .header.ide = 0,
        .header.rtr = 0,
        .buffer     = (uint8_t *)data,
        .buffer_len = len,
    };
    return twai_node_transmit(s_twai, &frame, pdMS_TO_TICKS(20));
}

void can_rx_stream_set(bool enable)
{
    s_rx_stream = enable;
}

void can_enter_standby(void)
{
    gpio_set_direction(CFG_CAN_STB_GPIO, GPIO_MODE_INPUT);
    /* pull-up → high → TJA1042BT standby */
}

/* ── Tasks ───────────────────────────────────────────────────────────────── */
void can_rx_task(void *arg)
{
    rx_item_t item;
    while (1) {
        if (xQueueReceive(s_rx_q, &item, portMAX_DELAY) != pdTRUE) continue;

        if (s_rx_stream) {
            printf("%s 0x%0*"PRIx32" [%d]",
                   item.hdr.ide ? "EXT" : "STD",
                   item.hdr.ide ? 8 : 3,
                   item.hdr.id, item.hdr.dlc);
            if (item.hdr.rtr) {
                printf(" (RTR)");
            } else {
                for (int i = 0; i < item.hdr.dlc; i++)
                    printf(" %02X", item.data[i]);
            }
            printf("\n");
        }

        /* Forward to state machine via extern hook (defined in main.c) */
        extern void on_can_rx(uint32_t id, uint8_t dlc, const uint8_t *data);
        if (!item.hdr.rtr)
            on_can_rx(item.hdr.id, item.hdr.dlc, item.data);
    }
}

void can_hb_task(void *arg)
{
    uint32_t counter = 0;
    while (1) {
        uint8_t payload[4] = {
            (counter >> 24) & 0xFF, (counter >> 16) & 0xFF,
            (counter >>  8) & 0xFF, (counter      ) & 0xFF,
        };
        if (can_send(CFG_CAN_HB_ID, payload, 4) == ESP_OK) {
            printf("HB  0x%03"PRIx32" [4] %08"PRIx32"\n",
                   (uint32_t)CFG_CAN_HB_ID, counter);
        }
        counter++;
        vTaskDelay(pdMS_TO_TICKS(CFG_CAN_HB_MS));
    }
}
