#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "usart.h"
#include <stdio.h>

/* Note: Configure this to the actual debug UART handle */
#define DEBUG_UART_HANDLE &huart2

/*
 * Thêm dòng này vào syscalls.c hoặc trong _write() để redirect printf:
 * 
 * int _write(int file, char *ptr, int len) {
 *     HAL_UART_Transmit(DEBUG_UART_HANDLE, (uint8_t *)ptr, len, HAL_MAX_DELAY);
 *     return len;
 * }
 */

#ifdef DEBUG
#define LOG_INFO(fmt, ...)   printf("[INFO] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)  printf("[ERROR] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)  printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__)
#else
#define LOG_INFO(fmt, ...)
#define LOG_ERROR(fmt, ...)
#define LOG_DEBUG(fmt, ...)
#endif

void Debug_UART_Init(void);

#endif /* DEBUG_UART_H */
