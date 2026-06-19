#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_twai.h"

/** Initialise TWAI peripheral and transceiver STB pin. */
esp_err_t can_init(void);

/** Send a standard CAN frame. id ≤ 0x7FF, data/len up to 8 bytes. */
esp_err_t can_send(uint32_t id, const uint8_t *data, uint8_t len);

/** Enable/disable printing received frames to console. */
void      can_rx_stream_set(bool enable);

/** FreeRTOS task – CAN RX dispatcher. */
void      can_rx_task(void *arg);

/** FreeRTOS task – heartbeat TX (ID=0x001, 1 Hz counter). */
void      can_hb_task(void *arg);

/** Prepare transceiver for deep sleep (release STB → pull-up → standby). */
void      can_enter_standby(void);

/** Returns true if the frame matches the sleep command. */
bool      can_is_sleep_cmd(uint32_t id, uint8_t dlc, const uint8_t *data);
