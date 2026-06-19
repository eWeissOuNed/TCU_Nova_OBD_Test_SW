#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"

/* ── WS2812 LED strip ────────────────────────────────────────────────────── */
#define CFG_LED_GPIO        25
#define CFG_LED_COUNT       3
#define CFG_LED_BRIGHTNESS  0.15f   /* 0.0 – 1.0 */

/* ── CAN / TWAI (TJA1042BT/0Z transceiver) ──────────────────────────────── */
#define CFG_CAN_TX_GPIO      GPIO_NUM_4
#define CFG_CAN_RX_GPIO      GPIO_NUM_5
#define CFG_CAN_STB_GPIO     GPIO_NUM_26   /* pull-up on board: high=standby */
#define CFG_CAN_BITRATE      500000
#define CFG_CAN_HB_ID        0x001         /* heartbeat CAN ID */
#define CFG_CAN_HB_MS        1000
#define CFG_CAN_SLEEP_ID     0x102
#define CFG_CAN_SLEEP_DATA   { 0x00,0x00,0x00,0x00,0x00,0x50,0x50,0x50 }

/* ── Shared SPI bus (SPI2) ───────────────────────────────────────────────── *
 *  Devices on this bus: IMU (LSM6DSOTR), SD card, UWB (DWM3000)            *
 *  Each device has its own CS pin.                                          */
#define CFG_SPI_HOST         SPI2_HOST
#define CFG_SPI_SCK          GPIO_NUM_0
#define CFG_SPI_MOSI         GPIO_NUM_3
#define CFG_SPI_MISO         GPIO_NUM_2

/* ── IMU (LSM6DSOTR) ─────────────────────────────────────────────────────── */
#define CFG_IMU_CS           GPIO_NUM_10
#define CFG_IMU_CLOCK_HZ     1000000

/* ── SD Card (via SPI) ───────────────────────────────────────────────────── *
 *  Enable pin: GPIO7, active-low (pull-up on board → high-Z = disabled)    *
 *  CS:         GPIO24                                                        */
#define CFG_SD_EN_GPIO       GPIO_NUM_7
#define CFG_SD_CS            GPIO_NUM_24
#define CFG_SD_MOUNT         "/sdcard"

/* ── UWB module (DWM3000) ────────────────────────────────────────────────── *
 *  TODO: confirm pins on actual PCB                                         */
#define CFG_UWB_CS           GPIO_NUM_8    /* TBD */
#define CFG_UWB_IRQ          GPIO_NUM_9    /* TBD */
#define CFG_UWB_RST          GPIO_NUM_NC   /* TBD */
#define CFG_UWB_CLOCK_HZ     8000000

/* ── Modem (Quectel EG915, UART) ─────────────────────────────────────────── *
 *  TODO: confirm UART pins on actual PCB                                    *
 *  Note: UART0 is the console (GPIO11/12). Use UART1 for modem.            */
#define CFG_MODEM_UART       UART_NUM_1
#define CFG_MODEM_TX         GPIO_NUM_6    /* TBD */
#define CFG_MODEM_RX         GPIO_NUM_NC   /* TBD */
#define CFG_MODEM_BAUD       115200
#define CFG_MODEM_BUF_SIZE   1024
