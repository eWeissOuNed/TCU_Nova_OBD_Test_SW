#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* ── States ──────────────────────────────────────────────────────────────── *
 * Add new states here. Give each a name in state_name() below.              */
typedef enum {
    STATE_IDLE = 0,      /* command prompt active, all peripherals accessible */
    STATE_SLEEP,         /* transitioning to deep sleep                       */
    STATE_TEST_RUNNING,  /* running an automated test sequence                */
    /* --- add future states here, e.g.: ---
     * STATE_ZIGBEE_MASTER,
     * STATE_MODEM_DATA,
     */
    STATE_COUNT
} app_state_t;

/** Initialise state machine. Must be called once at boot. */
void         sm_init(app_state_t initial);

/** Return current state. */
app_state_t  sm_get(void);

/** Return human-readable name of a state. */
const char  *sm_name(app_state_t s);

/** Request a state transition.
 *  Returns ESP_OK on success, ESP_ERR_INVALID_STATE if transition not allowed. */
esp_err_t    sm_set(app_state_t next);
