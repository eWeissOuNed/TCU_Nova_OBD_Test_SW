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
#include "freertos/semphr.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

static const char *TAG = "MODEM";

static bool               s_init        = false;
static bool               s_on          = false;
static SemaphoreHandle_t  s_uart_mtx    = NULL;
static TaskHandle_t       s_stream_task = NULL;
static volatile bool      s_stream_run  = false;
static uint32_t           s_stream_ms   = 5000;

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
    /* Timeout: still copy whatever was received so caller can inspect it */
    if (out_buf && out_len > 0 && filled > 0) {
        strncpy(out_buf, buf, out_len - 1);
        out_buf[out_len - 1] = '\0';
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

    s_uart_mtx = xSemaphoreCreateMutex();
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

    /* Check if modem is already responsive before pulsing PWRKEY.
     * If we pulse when it's already on, we'd turn it off instead. */
    uart_flush(CFG_MODEM_UART);
    uart_write_bytes(CFG_MODEM_UART, "AT\r\n", 4);
    char probe[32];
    if (uart_wait_for("OK", 1000, probe, sizeof(probe)) == ESP_OK) {
        ESP_LOGI(TAG, "modem already on");
        s_on = true;
        return ESP_OK;
    }

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

esp_err_t modem_sms_send(const char *number, const char *text)
{
    if (!s_init) return ESP_ERR_INVALID_STATE;
    if (!s_on)  { ESP_LOGW(TAG, "SMS ignored – modem is off"); return ESP_ERR_INVALID_STATE; }

    char buf[128];
    esp_err_t ret;

    /* 1. Set text mode */
    uart_write_bytes(CFG_MODEM_UART, "AT+CMGF=1\r\n", 11);
    ret = uart_wait_for("OK", 3000, buf, sizeof(buf));
    if (ret != ESP_OK) { ESP_LOGE(TAG, "CMGF failed"); return ESP_FAIL; }

    /* 2. Start SMS – wait for '>' prompt */
    int n = snprintf(buf, sizeof(buf), "AT+CMGS=\"%s\"\r\n", number);
    uart_write_bytes(CFG_MODEM_UART, buf, n);
    ret = uart_wait_for(">", 5000, buf, sizeof(buf));
    if (ret != ESP_OK) { ESP_LOGE(TAG, "no '>' prompt"); return ESP_FAIL; }

    /* 3. Send message text + Ctrl-Z (0x1A) */
    uart_write_bytes(CFG_MODEM_UART, text, strlen(text));
    uart_write_bytes(CFG_MODEM_UART, "\x1A", 1);

    /* 4. Wait for +CMGS confirmation (up to 30 s) */
    ret = uart_wait_for("+CMGS:", 30000, buf, sizeof(buf));
    if (ret != ESP_OK) { ESP_LOGE(TAG, "SMS send timeout"); return ESP_FAIL; }

    ESP_LOGI(TAG, "SMS sent to %s", number);
    return ESP_OK;
}

esp_err_t modem_signal(int *rssi_dbm, char *detail_buf, size_t detail_len)
{
    if (!s_init || !s_on) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_uart_mtx, pdMS_TO_TICKS(3000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    char buf[128];

    /* AT+CSQ → +CSQ: <rssi>,<ber>  rssi 0-31 → -113…-51 dBm, 99=unknown */
    uart_write_bytes(CFG_MODEM_UART, "AT+CSQ\r\n", 8);
    if (uart_wait_for("OK", 3000, buf, sizeof(buf)) == ESP_OK) {
        int rssi_raw = 99, ber = 0;
        char *p = strstr(buf, "+CSQ:");
        if (p) sscanf(p, "+CSQ: %d,%d", &rssi_raw, &ber);
        if (rssi_dbm) {
            *rssi_dbm = (rssi_raw == 99) ? 0 : (-113 + rssi_raw * 2);
        }
    } else {
        xSemaphoreGive(s_uart_mtx);
        return ESP_FAIL;
    }

    /* AT+QCSQ → detailed LTE signal (RSRP, RSRQ, SINR) */
    if (detail_buf && detail_len > 0) {
        uart_write_bytes(CFG_MODEM_UART, "AT+QCSQ\r\n", 9);
        if (uart_wait_for("OK", 3000, buf, sizeof(buf)) == ESP_OK) {
            char *p = strstr(buf, "+QCSQ:");
            if (p) {
                while (*p && (*p == '\r' || *p == '\n')) p++;
                strncpy(detail_buf, p, detail_len - 1);
                detail_buf[detail_len - 1] = '\0';
            }
        }
    }

    xSemaphoreGive(s_uart_mtx);
    return ESP_OK;
}

esp_err_t modem_gnss_enable(bool on)
{
    if (!s_init || !s_on) return ESP_ERR_INVALID_STATE;

    char buf[64];
    if (on) {
        uart_write_bytes(CFG_MODEM_UART, "AT+QGPS=1\r\n", 11);
    } else {
        uart_write_bytes(CFG_MODEM_UART, "AT+QGPSEND\r\n", 12);
    }
    esp_err_t ret = uart_wait_for("OK", 3000, buf, sizeof(buf));
    if (ret != ESP_OK) {
        /* ERROR is normal if GNSS is already in the requested state */
        ESP_LOGW(TAG, "GNSS %s: no OK (may already be in that state)", on ? "ON" : "OFF");
    }
    return ESP_OK;
}

esp_err_t modem_gnss_read(char *out_buf, size_t out_len)
{
    if (!s_init || !s_on) return ESP_ERR_INVALID_STATE;
    if (!out_buf || out_len == 0) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_uart_mtx, pdMS_TO_TICKS(6000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    char buf[256];
    /* Format 2 = decimal degrees: +QGPSLOC: <utc>,<lat>,<lon>,<hdop>,<alt>,... */
    uart_write_bytes(CFG_MODEM_UART, "AT+QGPSLOC=2\r\n", 14);
    /* Response is immediate (OK+data or +CME ERROR). 2 s is generous. */
    esp_err_t ret = uart_wait_for("OK", 2000, buf, sizeof(buf));
    if (ret != ESP_OK) {
        /* buf already contains the received bytes (populated on timeout too) */
        /* +CME ERROR: 516 = not fixed yet, 505 = GNSS not enabled */
        if (strstr(buf, "516")) {
            strncpy(out_buf, "no fix yet – needs sky view", out_len - 1);
        } else if (strstr(buf, "505")) {
            strncpy(out_buf, "GNSS not enabled – run MODEM GNSS ON first", out_len - 1);
        } else {
            strncpy(out_buf, "GNSS read failed", out_len - 1);
        }
        out_buf[out_len - 1] = '\0';
        xSemaphoreGive(s_uart_mtx);
        return ESP_FAIL;
    }

    char *p = strstr(buf, "+QGPSLOC:");
    if (!p) {
        strncpy(out_buf, "parse error", out_len - 1);
        xSemaphoreGive(s_uart_mtx);
        return ESP_FAIL;
    }

    char utc[16], lat[16], lon[16], alt[12], nsat[8] = "0";
    if (sscanf(p, "+QGPSLOC: %15[^,],%15[^,],%15[^,],%*[^,],%11[^,]",
               utc, lat, lon, alt) >= 3) {
        /* nsat is the 11th comma-delimited field after "+QGPSLOC: " */
        char *q = p;
        int   commas = 0;
        while (*q && commas < 10) { if (*q++ == ',') commas++; }
        if (commas == 10) sscanf(q, "%7[^,\r\n ]", nsat);
        snprintf(out_buf, out_len, "lat=%s lon=%s alt=%sm sats=%s utc=%s",
                 lat, lon, alt, nsat, utc);
    } else {
        strncpy(out_buf, p, out_len - 1);
        out_buf[out_len - 1] = '\0';
    }
    xSemaphoreGive(s_uart_mtx);
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
    if (xSemaphoreTake(s_uart_mtx, pdMS_TO_TICKS(timeout_ms + 1000)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    /* Send command + CRLF */
    char line[128];
    int  n = snprintf(line, sizeof(line), "%s\r\n", cmd);
    uart_write_bytes(CFG_MODEM_UART, line, n);
    ESP_LOGD(TAG, "TX: %s", cmd);

    /* Wait for OK or ERROR */
    esp_err_t ret = uart_wait_for("OK", timeout_ms, resp_buf, buf_len);
    if (ret != ESP_OK) {
        uart_wait_for("ERROR", 200, resp_buf, buf_len);
        xSemaphoreGive(s_uart_mtx);
        return ESP_FAIL;
    }
    xSemaphoreGive(s_uart_mtx);
    return ESP_OK;
}

/* ── Periodic stream task ────────────────────────────────────────────────── */

static void modem_stream_task(void *arg)
{
    while (s_stream_run) {
        if (s_on) {
            int  rssi = 0, rsrp = 0, sinr = 0, rsrq = 0;
            char detail[128] = "";
            char gps_str[32] = "no_fix";
            int  sats = 0;

            /* Signal */
            if (modem_signal(&rssi, detail, sizeof(detail)) == ESP_OK) {
                /* +QCSQ: "LTE",<rssi_raw>,<rsrp>,<sinr>,<rsrq> */
                char *p = strstr(detail, "LTE");
                if (p) {
                    int dummy;
                    sscanf(p, "LTE\",%d,%d,%d,%d", &dummy, &rsrp, &sinr, &rsrq);
                }
            }

            /* GPS – skip gracefully if no fix */
            char pos[96];
            if (modem_gnss_read(pos, sizeof(pos)) == ESP_OK) {
                float lat = 0.0f, lon = 0.0f;
                if (sscanf(pos, "lat=%f lon=%f", &lat, &lon) == 2) {
                    char sats_str[8] = "0";
                    sscanf(pos, "lat=%*s lon=%*s alt=%*s sats=%7s", sats_str);
                    sats = atoi(sats_str);
                    snprintf(gps_str, sizeof(gps_str), "%.6f,%.6f", lat, lon);
                }
            }

            printf("MS rssi=%d rsrp=%d sinr=%d rsrq=%d gps=%s sats=%d\n",
                   rssi, rsrp, sinr, rsrq, gps_str, sats);
            fflush(stdout);
        }

        /* Interruptible sleep – checks s_stream_run every 100 ms */
        for (uint32_t i = 0; i < s_stream_ms / 100 && s_stream_run; i++)
            vTaskDelay(pdMS_TO_TICKS(100));
    }

    s_stream_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t modem_stream_set(bool on, uint32_t interval_ms)
{
    if (!s_init) return ESP_ERR_INVALID_STATE;

    if (on) {
        s_stream_ms  = (interval_ms < 1000) ? 1000 : interval_ms;
        s_stream_run = true;
        if (!s_stream_task)
            xTaskCreate(modem_stream_task, "modem_stream", 4096, NULL, 3, &s_stream_task);
        ESP_LOGI(TAG, "stream ON  interval=%"PRIu32" ms", s_stream_ms);
    } else {
        s_stream_run = false;
        ESP_LOGI(TAG, "stream OFF");
    }
    return ESP_OK;
}
