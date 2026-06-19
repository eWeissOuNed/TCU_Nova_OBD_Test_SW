#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "config.h"
#include "state_machine.h"
#include "cmd_handler.h"
#include "periph/led.h"
#include "periph/imu.h"
#include "periph/can_bus.h"
#include "periph/sdcard.h"
#include "periph/uwb.h"
#include "periph/modem.h"

static const char *TAG = "MAIN";

/* Grace period after GPIO wakeup: ignore sleep commands for 5 s */
static int64_t s_sleep_allowed_at = 0;

/* ── CAN RX hook (called from can_rx_task) ───────────────────────────────── */
void on_can_rx(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    if (can_is_sleep_cmd(id, dlc, data)) {
        if (esp_timer_get_time() >= s_sleep_allowed_at) {
            ESP_LOGI(TAG, "CAN sleep command received");
            sm_set(STATE_SLEEP);
        } else {
            ESP_LOGI(TAG, "CAN sleep command ignored (grace period)");
        }
    }
}

/* ── app_main ────────────────────────────────────────────────────────────── */
void app_main(void)
{
    /* ── Wakeup cause ── */
    if (esp_sleep_get_wakeup_causes() & ESP_SLEEP_WAKEUP_EXT1) {
        s_sleep_allowed_at = esp_timer_get_time() + 5000000LL; /* 5 s */
        ESP_LOGI(TAG, "Woke from CAN – sleep cmd blocked for 5 s");
    }

    /* ── State machine ── */
    sm_init(STATE_IDLE);

    /* ── Shared SPI bus (IMU + SD + UWB share SPI2) ── */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = CFG_SPI_MOSI,
        .miso_io_num   = CFG_SPI_MISO,
        .sclk_io_num   = CFG_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(CFG_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* ── Peripherals ── */
    ESP_ERROR_CHECK(led_init());
    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(can_init());

    /* Stubbed peripherals – initialise when ready */
    sdcard_init();   /* returns ESP_ERR_NOT_SUPPORTED until implemented */
    uwb_init();
    modem_init();

    /* ── Tasks ── */
    xTaskCreate(can_rx_task,      "can_rx",  4096, NULL, 5, NULL);
    xTaskCreate(can_hb_task,      "can_hb",  2048, NULL, 4, NULL);
    xTaskCreate(imu_task,         "imu",     4096, NULL, 4, NULL);
    cmd_handler_start();  /* command prompt task, priority 3 */
}
