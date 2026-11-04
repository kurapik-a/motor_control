#include "hpm_soc.h"
#include "hpm_uart_drv.h"
#define BOARD_CONSOLE_UART_BASE ((UART_Type *)1)
#define BOARD_CONSOLE_UART_CLK_NAME 0
#include "motor_config.h"
#define BOARD_CONSOLE_UART_BAUDRATE MOTOR_VOFA_BAUDRATE
void board_init_uart(UART_Type *ptr);
