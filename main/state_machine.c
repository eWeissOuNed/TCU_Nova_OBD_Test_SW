#include "state_machine.h"
#include "config.h"
#include "periph/led.h"
#include "periph/can_bus.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SM";
static app_state_t s_state = STATE_IDLE;

/* ── Transition table ────────────────────────────────────────────────────── *
 * allowed[from][to] = true means the transition is valid.                   */
static const bool allowed[STATE_COUNT][STATE_COUNT] = {
    /*              IDLE   SLEEP  TEST  */
    /* IDLE  */  { false,  true,  true  },
    /* SLEEP */  { false, false, false  },  /* sleep is a one-way door */
    /* TEST  */  {  true,  true, false  },
};

/* ── Enter / exit hooks ──────────────────────────────────────────────────── */
static void on_enter(app_state_t s)
{
    switch (s) {
    case STATE_IDLE:
        ESP_LOGI(TAG, "→ IDLE");
        break;

    case STATE_SLEEP:
        ESP_LOGI(TAG, "→ SLEEP (deep sleep in 500 ms)");
        led_shutdown();
        vTaskDelay(pdMS_TO_TICKS(400));
        can_enter_standby();
        esp_sleep_enable_ext1_wakeup(1ULL << CFG_CAN_RX_GPIO,
                                     ESP_EXT1_WAKEUP_ANY_LOW);
        esp_deep_sleep_start();
        break;  /* unreachable – kept for clarity */

    case STATE_TEST_RUNNING:
        ESP_LOGI(TAG, "→ TEST_RUNNING");
        /* TODO: start test sequence task */
        break;

    default:
        break;
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */
void sm_init(app_state_t initial)
{
    s_state = initial;
    ESP_LOGI(TAG, "init – state=%s", sm_name(initial));
}

app_state_t sm_get(void)
{
    return s_state;
}

const char *sm_name(app_state_t s)
{
    switch (s) {
    case STATE_IDLE:         return "IDLE";
    case STATE_SLEEP:        return "SLEEP";
    case STATE_TEST_RUNNING: return "TEST_RUNNING";
    default:                 return "UNKNOWN";
    }
}

esp_err_t sm_set(app_state_t next)
{
    if (next >= STATE_COUNT) return ESP_ERR_INVALID_ARG;
    if (!allowed[s_state][next]) {
        ESP_LOGW(TAG, "transition %s→%s not allowed",
                 sm_name(s_state), sm_name(next));
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "%s → %s", sm_name(s_state), sm_name(next));
    s_state = next;
    on_enter(next);
    return ESP_OK;
}
