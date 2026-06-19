#pragma once

/** Start the command handler task (reads from console UART, line by line).
 *  Call once from app_main after all peripherals are initialised. */
void cmd_handler_start(void);

/** Print the command prompt. Call after any async output to restore it. */
void cmd_prompt(void);
