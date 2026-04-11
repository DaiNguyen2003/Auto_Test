#ifndef COMMON_SYSTEM_ERROR_H_
#define COMMON_SYSTEM_ERROR_H_

#include "stdint.h"

// Define System Error Codes
typedef enum {
    ERR_NONE            = 0x00,
    ERR_HAL_INIT        = 0x01,
    ERR_CAN_TIMEOUT     = 0x10,
    ERR_CAN_BUS_OFF     = 0x11,
    ERR_UART_OVERFLOW   = 0x20,
    ERR_MOTOR_STUCK     = 0x30,
    ERR_I2C_LCD         = 0x40,
    ERR_ASSERT_FAILED   = 0x99,
    ERR_FATAL           = 0xFF
} System_Error_Code_t;

/**
 * @brief  Báo cáo lỗi hệ thống
 * @param  code   Mã lỗi (System_Error_Code_t)
 * @param  msg    Chuỗi giải thích ngắn gọn
 */
void System_ReportError(System_Error_Code_t code, const char* msg);

/**
 * @brief  Hàm Handler thay thế cho while(1) của HAL
 * @param  msg    Tên hệ thống/ngoại vi bị lỗi
 */
void System_ErrorHandler(const char* msg);

#endif /* COMMON_SYSTEM_ERROR_H_ */
