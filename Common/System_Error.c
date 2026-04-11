#include "System_Error.h"
#include "Debug_UART.h"
#include "Protocol.h"
#include "RS485.h"
#include "bsp.h" // For HAL_NVIC_SystemReset or IO_Control
#include "IO_Control.h"
#include "string.h"

void System_ReportError(System_Error_Code_t code, const char* msg) {
    // 1. Log ra Serial màn hình (Console)
    LOG_ERROR("SYS_ERR [0x%02X]: %s", code, msg != NULL ? msg : "Unknown");

    // 2. Gửi RS485 báo cho App PC biết (Frame CMD 0xEE)
    uint8_t tx_buffer[MAX_PAYLOAD_SIZE + 5];
    uint8_t data[64];
    data[0] = (uint8_t)code;
    
    uint8_t msg_len = 0;
    if (msg != NULL) {
        msg_len = strlen(msg);
        if (msg_len > 60) msg_len = 60; // Giới hạn độ dài an toàn
        memcpy(&data[1], msg, msg_len);
    }
    
    uint16_t frame_len = Protocol_PackMessage(CMD_ERROR_REPORT, data, msg_len + 1, tx_buffer);
    
    // Gửi báo cáo lỗi qua cổng RS485 giao tiếp với Master (PC)
    RS485_UART3_Send(tx_buffer, frame_len);
}

void System_ErrorHandler(const char* msg) {
    // Tắt toàn bộ Relay phần cứng để bảo đảm an toàn cháy nổ
    Relay_Off(); 

    // Báo lỗi Fatal
    System_ReportError(ERR_HAL_INIT, msg);

    // Chờ 1 giây để UART gửi xong text
    HAL_Delay(1000); 

    // Reset MCU thay vì treo cứng ở while(1)
    NVIC_SystemReset();
}
