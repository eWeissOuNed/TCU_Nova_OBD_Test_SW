/* cmd_handler.c
 *
 * Line-based ASCII protocol over UART0 (console).
 *
 * Protocol
 * ────────
 *   Request :  COMMAND [args…]\n
 *   Response:  OK [payload]\n   or   ERR <reason>\n
 *
 * Commands
 * ────────
 *   PING                      → OK PONG
 *   HELP                      → OK <command list>
 *   STATE?                    → OK <state name>
 *   STATE SET <name>          → OK | ERR
 *   IMU?                      → OK ax=X ay=Y az=Z
 *   IMU STREAM ON|OFF         → OK
 *   LED <idx> <r> <g> <b>     → OK | ERR
 *   LED OFF                   → OK
 *   CAN SEND <id_hex> <hex…>  → OK | ERR
 *   CAN STREAM ON|OFF         → OK
 *   SD INIT                   → OK | ERR
 *   SD LIST                   → OK <files> | ERR
 *   UWB INIT                  → OK | ERR
 *   UWB RANGE                 → OK <mm> | ERR
 *   MODEM AT <command>        → OK <response> | ERR
 *   SLEEP                     → (device sleeps)
 */
#include "cmd_handler.h"
#include "config.h"
#include "state_machine.h"
#include "periph/led.h"
#include "periph/imu.h"
#include "periph/can_bus.h"
#include "periph/sdcard.h"
#include "periph/uwb.h"
#include "periph/modem.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CMD_LINE_MAX 128
#define PROMPT       "\r\n> "

void cmd_prompt(void)
{
    printf(PROMPT);
    fflush(stdout);
}

/* ── helpers ─────────────────────────────────────────────────────────────── */
static void resp_ok(const char *payload)
{
    if (payload && payload[0])
        printf("OK %s\n", payload);
    else
        printf("OK\n");
    fflush(stdout);
}

static void resp_err(const char *reason)
{
    printf("ERR %s\n", reason);
    fflush(stdout);
}

/* ── command dispatch ────────────────────────────────────────────────────── */
static void handle_line(char *line)
{
    /* strip trailing whitespace */
    int len = strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n' ||
                        line[len-1] == ' '))
        line[--len] = '\0';
    if (len == 0) { cmd_prompt(); return; }

    /* tokenise */
    char *tok[8] = {0};
    int   ntok   = 0;
    char *p      = line;
    while (*p && ntok < 8) {
        while (*p == ' ') p++;
        if (!*p) break;
        tok[ntok++] = p;
        while (*p && *p != ' ') p++;
        if (*p) *p++ = '\0';
    }
    if (ntok == 0) { cmd_prompt(); return; }

    /* ── PING ── */
    if (strcmp(tok[0], "PING") == 0) {
        resp_ok("PONG");

    /* ── HELP ── */
    } else if (strcmp(tok[0], "HELP") == 0) {
        printf("OK Commands:\n"
               "  PING\n"
               "  STATE?\n"
               "  STATE SET <IDLE|SLEEP|TEST_RUNNING>\n"
               "  IMU?\n"
               "  IMU STREAM <ON|OFF>\n"
               "  LED <idx 0-%d> <r> <g> <b>\n"
               "  LED OFF\n"
               "  CAN SEND <id_hex> <data_hex_bytes…>\n"
               "  CAN STREAM <ON|OFF>\n"
               "  SD INIT | SD LIST | SD WRITE <msg> | SD READ | SD WIPE | SD EN <ON|OFF>\n"
               "  UWB INIT | UWB RANGE\n"
               "  MODEM AT <command>\n"
               "  SLEEP\n",
               CFG_LED_COUNT - 1);
        fflush(stdout);

    /* ── STATE? ── */
    } else if (strcmp(tok[0], "STATE?") == 0) {
        resp_ok(sm_name(sm_get()));

    /* ── STATE SET ── */
    } else if (strcmp(tok[0], "STATE") == 0 && ntok >= 3 &&
               strcmp(tok[1], "SET") == 0) {
        app_state_t next = STATE_COUNT;
        for (app_state_t s = 0; s < STATE_COUNT; s++) {
            if (strcmp(tok[2], sm_name(s)) == 0) { next = s; break; }
        }
        if (next == STATE_COUNT) { resp_err("unknown state"); }
        else if (sm_set(next) == ESP_OK) { resp_ok(NULL); }
        else { resp_err("transition not allowed"); }

    /* ── IMU? ── */
    } else if (strcmp(tok[0], "IMU?") == 0) {
        imu_data_t d;
        if (imu_read(&d) == ESP_OK) {
            char buf[64];
            snprintf(buf, sizeof(buf), "ax=%.3f ay=%.3f az=%.3f",
                     d.ax_g, d.ay_g, d.az_g);
            resp_ok(buf);
        } else {
            resp_err("IMU read failed");
        }

    /* ── IMU STREAM ── */
    } else if (strcmp(tok[0], "IMU") == 0 && ntok >= 3 &&
               strcmp(tok[1], "STREAM") == 0) {
        bool on = strcmp(tok[2], "ON") == 0;
        imu_stream_set(on);
        resp_ok(on ? "streaming ON" : "streaming OFF");

    /* ── LED <idx> <r> <g> <b> ── */
    } else if (strcmp(tok[0], "LED") == 0 && ntok >= 5 &&
               strcmp(tok[1], "OFF") != 0) {
        uint8_t idx = (uint8_t)atoi(tok[1]);
        uint8_t r   = (uint8_t)atoi(tok[2]);
        uint8_t g   = (uint8_t)atoi(tok[3]);
        uint8_t b   = (uint8_t)atoi(tok[4]);
        if (led_set(idx, r, g, b) == ESP_OK) resp_ok(NULL);
        else resp_err("LED set failed");

    /* ── LED OFF ── */
    } else if (strcmp(tok[0], "LED") == 0 && ntok >= 2 &&
               strcmp(tok[1], "OFF") == 0) {
        led_all_off();
        resp_ok(NULL);

    /* ── CAN SEND <id> <bytes…> ── */
    } else if (strcmp(tok[0], "CAN") == 0 && ntok >= 3 &&
               strcmp(tok[1], "SEND") == 0) {
        uint32_t id   = (uint32_t)strtol(tok[2], NULL, 16);
        uint8_t  data[8] = {0};
        uint8_t  dlen = 0;
        for (int i = 3; i < ntok && dlen < 8; i++, dlen++)
            data[dlen] = (uint8_t)strtol(tok[i], NULL, 16);
        if (can_send(id, data, dlen) == ESP_OK) resp_ok(NULL);
        else resp_err("CAN TX failed");

    /* ── CAN STREAM ── */
    } else if (strcmp(tok[0], "CAN") == 0 && ntok >= 3 &&
               strcmp(tok[1], "STREAM") == 0) {
        bool on = strcmp(tok[2], "ON") == 0;
        can_rx_stream_set(on);
        resp_ok(on ? "streaming ON" : "streaming OFF");

    /* ── SD ── */
    } else if (strcmp(tok[0], "SD") == 0 && ntok >= 2) {
        if (strcmp(tok[1], "INIT") == 0) {
            esp_err_t e = sdcard_init();
            if (e == ESP_OK) resp_ok(NULL);
            else resp_err(esp_err_to_name(e));
        } else if (strcmp(tok[1], "LIST") == 0) {
            char buf[512];
            esp_err_t e = sdcard_list(buf, sizeof(buf));
            if (e == ESP_OK) resp_ok(buf);
            else resp_err(esp_err_to_name(e));
        } else if (strcmp(tok[1], "WRITE") == 0) {
            /* SD WRITE <message words...> */
            char msg[96] = "(no message)";
            if (ntok >= 3) {
                msg[0] = '\0';
                for (int i = 2; i < ntok; i++) {
                    if (i > 2) strncat(msg, " ", sizeof(msg) - strlen(msg) - 1);
                    strncat(msg, tok[i], sizeof(msg) - strlen(msg) - 1);
                }
            }
            esp_err_t e = sdcard_write(msg);
            if (e == ESP_OK) resp_ok("line written");
            else resp_err(esp_err_to_name(e));
        } else if (strcmp(tok[1], "READ") == 0) {
            char buf[1024];
            esp_err_t e = sdcard_read(buf, sizeof(buf));
            /* print line by line so long files don't get cut off */
            if (e == ESP_OK || e == ESP_ERR_NOT_FOUND) {
                printf("OK\n%s\n", buf);
                fflush(stdout);
            } else {
                resp_err(esp_err_to_name(e));
            }
        } else if (strcmp(tok[1], "WIPE") == 0) {
            esp_err_t e = sdcard_wipe();
            if (e == ESP_OK) resp_ok("TEST.TXT deleted");
            else resp_err(esp_err_to_name(e));
        } else if (strcmp(tok[1], "EN") == 0 && ntok >= 3) {
            bool on = strcmp(tok[2], "ON") == 0;
            sdcard_set_enable(on);
            resp_ok(on ? "EN → LOW (card on)" : "EN → HIGH-Z (card off)");
        } else {
            resp_err("SD: INIT | LIST | WRITE <msg> | READ | WIPE | EN <ON|OFF>");
        }

    /* ── UWB ── */
    } else if (strcmp(tok[0], "UWB") == 0 && ntok >= 2) {
        if (strcmp(tok[1], "INIT") == 0) {
            esp_err_t e = uwb_init();
            if (e == ESP_OK) resp_ok(NULL);
            else resp_err(esp_err_to_name(e));
        } else if (strcmp(tok[1], "RANGE") == 0) {
            uint32_t mm;
            esp_err_t e = uwb_range(&mm);
            if (e == ESP_OK) {
                char buf[32];
                snprintf(buf, sizeof(buf), "%"PRIu32" mm", mm);
                resp_ok(buf);
            } else {
                resp_err(esp_err_to_name(e));
            }
        } else {
            resp_err("unknown UWB command");
        }

    /* ── MODEM AT ── */
    } else if (strcmp(tok[0], "MODEM") == 0 && ntok >= 3 &&
               strcmp(tok[1], "AT") == 0) {
        /* rebuild AT command from remaining tokens */
        char at_cmd[64] = "AT";
        for (int i = 2; i < ntok; i++) {
            strncat(at_cmd, tok[i], sizeof(at_cmd) - strlen(at_cmd) - 1);
        }
        char resp[256];
        esp_err_t e = modem_at(at_cmd, resp, sizeof(resp), 5000);
        if (e == ESP_OK) resp_ok(resp);
        else resp_err(esp_err_to_name(e));

    /* ── SLEEP ── */
    } else if (strcmp(tok[0], "SLEEP") == 0) {
        resp_ok("going to sleep");
        vTaskDelay(pdMS_TO_TICKS(50));
        sm_set(STATE_SLEEP);

    } else {
        resp_err("unknown command – type HELP");
    }

    cmd_prompt();
}

/* ── Task ────────────────────────────────────────────────────────────────── *
 * IDF monitor uses raw terminal mode – every keypress arrives immediately.  *
 * We accumulate characters and process the line only on Enter (\r or \n).   */
static void cmd_task(void *arg)
{
    char line[CMD_LINE_MAX];
    int  pos = 0;

    printf("\r\n=== TCU NOVA OBD Test SW ===\r\n");
    printf("Type HELP for available commands.\r\n");
    cmd_prompt();

    while (1) {
        int c = getchar();
        if (c < 0) {                        /* no data yet */
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (c == '\r' || c == '\n') {       /* Enter → process line */
            printf("\r\n");
            fflush(stdout);
            line[pos] = '\0';
            pos = 0;
            handle_line(line);              /* handle_line calls cmd_prompt() */

        } else if (c == 0x7F || c == '\b') { /* Backspace */
            if (pos > 0) {
                pos--;
                printf("\b \b");
                fflush(stdout);
            }

        } else if (c >= 0x20 && pos < CMD_LINE_MAX - 1) { /* printable */
            line[pos++] = (char)c;
            putchar(c);                     /* echo */
            fflush(stdout);
        }
    }
}

void cmd_handler_start(void)
{
    xTaskCreate(cmd_task, "cmd", 4096, NULL, 3, NULL);
}
