#include "Debug_UART.h"

void Debug_UART_Init(void) {
    // Custom setup for Debug UART if needed
    // Currently CubeMX initialized `huart2` usually.
    LOG_INFO("System Booted. Debug Interface Ready.");
}

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(DEBUG_UART_HANDLE, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* Also support _write to cover various environments */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(DEBUG_UART_HANDLE, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}
