#include "led.h"
#include "../config.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "LED";
static led_strip_handle_t s_strip = NULL;

static uint8_t clamp_u8(float v)
{
    if (v < 0.0f)   return 0;
    if (v > 255.0f) return 255;
    return (uint8_t)v;
}

esp_err_t led_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num         = CFG_LED_GPIO,
        .max_leds               = CFG_LED_COUNT,
        .led_model              = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out       = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src        = RMT_CLK_SRC_DEFAULT,
        .resolution_hz  = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    esp_err_t ret = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip);
    if (ret == ESP_OK) {
        led_strip_clear(s_strip);
        ESP_LOGI(TAG, "init OK – %d LEDs on GPIO%d", CFG_LED_COUNT, CFG_LED_GPIO);
    }
    return ret;
}

esp_err_t led_set(uint8_t idx, uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_strip || idx >= CFG_LED_COUNT) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = led_strip_set_pixel(s_strip, idx, r, g, b);
    if (ret == ESP_OK) ret = led_strip_refresh(s_strip);
    return ret;
}

esp_err_t led_all_off(void)
{
    if (!s_strip) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = led_strip_clear(s_strip);
    if (ret == ESP_OK) ret = led_strip_refresh(s_strip);
    return ret;
}

esp_err_t led_set_axis(uint8_t idx, float value)
{
    if (!s_strip || idx >= CFG_LED_COUNT) return ESP_ERR_INVALID_STATE;
    float mag = fabsf(value);
    if (mag > 1.0f) mag = 1.0f;
    uint8_t r = (value > 0.0f) ? clamp_u8(mag * 255.0f * CFG_LED_BRIGHTNESS) : 0;
    uint8_t b = (value < 0.0f) ? clamp_u8(mag * 255.0f * CFG_LED_BRIGHTNESS) : 0;
    uint8_t g = clamp_u8((1.0f - mag) * 20.0f * CFG_LED_BRIGHTNESS);
    return led_set(idx, r, g, b);
}

void led_shutdown(void)
{
    if (!s_strip) return;
    led_strip_clear(s_strip);
    led_strip_refresh(s_strip);
    vTaskDelay(pdMS_TO_TICKS(50));
    led_strip_del(s_strip);
    s_strip = NULL;
    /* Hold data line low so strip doesn't glitch */
    gpio_set_direction(CFG_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CFG_LED_GPIO, 0);
    ESP_LOGI(TAG, "shutdown");
}
