/* modem.c – Quectel EG915N-EU driver
 *
 * Power control:
 *   PWRKEY (GPIO1, open-drain): pulse LOW for ≥600 ms to toggle power on/off.
 *   The modem signals readiness by sending the "RDY" URC over UART.
 *
 * UART:
 *   UART1, TX=CFG_MODEM_TX, RX=CFG_MODEM_RX, 115200 8N1.
 *   RX pin is GPIO_NUM_NC until wired – UART will init but reads return nothing.
 */
#include "modem.h"
#include "../config.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

static const char *TAG = "MODEM";

static bool s_init = false;
static bool s_on   = false;

/* ── PWRKEY helpers ──────────────────────────────────────────────────────── */

static void pwrkey_init(void)
{
    /* NSVDTC143ZET1G NPN digital transistor: GPIO HIGH → collector pulls
     * PWRKEY low (assert), GPIO LOW → transistor off → PWRKEY released.
     * Push-pull output is fine – ESP GPIO never sees the 3.8V PWRKEY rail. */
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CFG_MODEM_PWRKEY,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(CFG_MODEM_PWRKEY, 0);   /* transistor off → PWRKEY released */
}

static void pwrkey_pulse(void)
{
    ESP_LOGI(TAG, "PWRKEY pulse 700 ms");
    gpio_set_level(CFG_MODEM_PWRKEY, 1);   /* transistor on  → PWRKEY low (assert) */
    vTaskDelay(pdMS_TO_TICKS(700));
    gpio_set_level(CFG_MODEM_PWRKEY, 0);   /* transistor off → PWRKEY released     */
}

/* ── UART helpers ────────────────────────────────────────────────────────── */

static esp_err_t uart_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate  = CFG_MODEM_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    esp_err_t ret = uart_param_config(CFG_MODEM_UART, &uart_cfg);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(CFG_MODEM_UART,
                       CFG_MODEM_TX,
                       CFG_MODEM_RX,   /* GPIO_NUM_NC = not wired yet */
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) return ret;

    return uart_driver_install(CFG_MODEM_UART, CFG_MODEM_BUF_SIZE, 0, 0, NULL, 0);
}

/** Read from UART until 'needle' appears in the buffer or timeout expires.
 *  Returns ESP_OK if found, ESP_ERR_TIMEOUT otherwise. */
static esp_err_t uart_wait_for(const char *needle, uint32_t timeout_ms,
                                char *out_buf, size_t out_len)
{
    char   buf[CFG_MODEM_BUF_SIZE];
    size_t filled  = 0;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < end) {
        int n = uart_read_bytes(CFG_MODEM_UART,
                                (uint8_t *)(buf + filled),
                                sizeof(buf) - filled - 1,
                                pdMS_TO_TICKS(50));
        if (n > 0) {
            filled += n;
            buf[filled] = '\0';
            if (strstr(buf, needle)) {
                if (out_buf && out_len > 0) {
                    strncpy(out_buf, buf, out_len - 1);
                    out_buf[out_len - 1] = '\0';
                }
                return ESP_OK;
            }
            /* Prevent overflow: keep only the last half of the buffer */
            if (filled > sizeof(buf) / 2) {
                size_t keep = sizeof(buf) / 4;
                memmove(buf, buf + filled - keep, keep);
                filled = keep;
            }
        }
    }
    return ESP_ERR_TIMEOUT;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t modem_init(void)
{
    if (s_init) return ESP_OK;

    pwrkey_init();

    esp_err_t ret = uart_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_init = true;
    ESP_LOGI(TAG, "init OK  PWRKEY=GPIO%d  TX=GPIO%d  RX=GPIO%d",
             CFG_MODEM_PWRKEY, CFG_MODEM_TX, CFG_MODEM_RX);
    return ESP_OK;
}

esp_err_t modem_deinit(void)
{
    if (!s_init) return ESP_OK;
    uart_driver_delete(CFG_MODEM_UART);
    s_init = false;
    s_on   = false;
    return ESP_OK;
}

bool modem_is_on(void)
{
    return s_on;
}

esp_err_t modem_power_on(uint32_t timeout_ms)
{
    if (!s_init) {
        ESP_LOGE(TAG, "power_on: modem not initialised");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_on) {
        ESP_LOGW(TAG, "already on");
        return ESP_OK;
    }

    /* Flush any stale RX bytes */
    uart_flush(CFG_MODEM_UART);

    pwrkey_pulse();

    ESP_LOGI(TAG, "waiting for RDY URC (timeout %"PRIu32" ms)…", timeout_ms);
    char urc[128] = {0};
    esp_err_t ret = uart_wait_for("RDY", timeout_ms, urc, sizeof(urc));
    if (ret == ESP_OK) {
        s_on = true;
        ESP_LOGI(TAG, "modem ready  URC=[%s]", urc);
    } else {
        ESP_LOGW(TAG, "RDY not received within %"PRIu32" ms – modem may still be booting",
                 timeout_ms);
    }
    return ret;
}

esp_err_t modem_power_off(void)
{
    if (!s_init) return ESP_ERR_INVALID_STATE;
    pwrkey_pulse();
    s_on = false;
    ESP_LOGI(TAG, "power off pulse sent");
    return ESP_OK;
}

esp_err_t modem_at(const char *cmd, char *resp_buf, size_t buf_len,
                   uint32_t timeout_ms)
{
    if (!s_init) return ESP_ERR_INVALID_STATE;
    if (!s_on) {
        ESP_LOGW(TAG, "AT command ignored – modem is off");
        return ESP_ERR_INVALID_STATE;
    }

    /* Send command + CRLF */
    char line[128];
    int  n = snprintf(line, sizeof(line), "%s\r\n", cmd);
    uart_write_bytes(CFG_MODEM_UART, line, n);
    ESP_LOGD(TAG, "TX: %s", cmd);

    /* Wait for OK or ERROR */
    esp_err_t ret = uart_wait_for("OK", timeout_ms, resp_buf, buf_len);
    if (ret != ESP_OK) {
        uart_wait_for("ERROR", 200, resp_buf, buf_len);   /* short retry */
        return ESP_FAIL;
    }
    return ESP_OK;
}
