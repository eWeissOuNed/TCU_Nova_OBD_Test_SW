#include "gpio_cmd.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "GPIO_CMD";

/* ── toggle state ────────────────────────────────────────────────────────── */
typedef struct {
    int                pin;
    int                level;
    esp_timer_handle_t timer;
} toggle_entry_t;

static toggle_entry_t s_toggles[GPIO_CMD_MAX_TOGGLE];
static int            s_ntoggle = 0;

/* ── timer callback ──────────────────────────────────────────────────────── */
static void toggle_cb(void *arg)
{
    toggle_entry_t *e = (toggle_entry_t *)arg;
    e->level ^= 1;
    gpio_set_level(e->pin, e->level);
}

/* ── helpers ─────────────────────────────────────────────────────────────── */
static esp_err_t configure_output(int pin)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg);
}

/* ── public API ──────────────────────────────────────────────────────────── */
esp_err_t gpio_cmd_set(int pin, bool high)
{
    esp_err_t ret = configure_output(pin);
    if (ret != ESP_OK) return ret;
    return gpio_set_level(pin, high ? 1 : 0);
}

esp_err_t gpio_cmd_get(int pin, bool *out)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) return ret;
    *out = gpio_get_level(pin) != 0;
    return ESP_OK;
}

esp_err_t gpio_cmd_toggle(int pin, uint32_t freq_hz)
{
    if (freq_hz == 0 || freq_hz > 50000) {
        ESP_LOGE(TAG, "freq_hz must be 1–50000");
        return ESP_ERR_INVALID_ARG;
    }

    /* stop any existing toggle on this pin first */
    gpio_cmd_toggle_stop(pin);

    if (s_ntoggle >= GPIO_CMD_MAX_TOGGLE) {
        ESP_LOGE(TAG, "max %d simultaneous toggles", GPIO_CMD_MAX_TOGGLE);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = configure_output(pin);
    if (ret != ESP_OK) return ret;

    toggle_entry_t *e = &s_toggles[s_ntoggle];
    e->pin   = pin;
    e->level = 0;

    uint64_t half_period_us = 1000000ULL / (2ULL * freq_hz);
    if (half_period_us == 0) half_period_us = 1;

    esp_timer_create_args_t args = {
        .callback = toggle_cb,
        .arg      = e,
        .name     = "gpio_toggle",
    };
    ret = esp_timer_create(&args, &e->timer);
    if (ret != ESP_OK) return ret;

    ret = esp_timer_start_periodic(e->timer, half_period_us);
    if (ret != ESP_OK) {
        esp_timer_delete(e->timer);
        e->timer = NULL;
        return ret;
    }

    s_ntoggle++;
    ESP_LOGI(TAG, "GPIO%d toggling at %"PRIu32" Hz (T/2=%llu µs)",
             pin, freq_hz, (unsigned long long)half_period_us);
    return ESP_OK;
}

esp_err_t gpio_cmd_toggle_stop(int pin)
{
    for (int i = 0; i < s_ntoggle; i++) {
        if (s_toggles[i].pin == pin) {
            esp_timer_stop(s_toggles[i].timer);
            esp_timer_delete(s_toggles[i].timer);
            gpio_set_level(pin, 0);
            /* compact the array */
            memmove(&s_toggles[i], &s_toggles[i + 1],
                    (s_ntoggle - i - 1) * sizeof(toggle_entry_t));
            s_ntoggle--;
            ESP_LOGI(TAG, "GPIO%d toggle stopped", pin);
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;   /* not toggling – not an error for the caller */
}

void gpio_cmd_toggle_stop_all(void)
{
    for (int i = 0; i < s_ntoggle; i++) {
        esp_timer_stop(s_toggles[i].timer);
        esp_timer_delete(s_toggles[i].timer);
        gpio_set_level(s_toggles[i].pin, 0);
    }
    s_ntoggle = 0;
    ESP_LOGI(TAG, "all toggles stopped");
}
