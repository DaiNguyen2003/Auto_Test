#include "Protocol.h"
#include "Test_Manager.h"
#include "Hardware_Control.h"
#include "common.h"
#include "RS485.h"

/**
 * @brief Thực thi các lệnh nhận được từ RS485 (PC Control)
 * Ghi đè hàm weak trong Protocol.c
 */
void Protocol_ProcessMessage(const MessageFrame_t *msg) {
    uint8_t response_data[1] = {0};

    switch (msg->command) {
        case CMD_PING:
            // Phản hồi PING bằng ACK
            RS485_SendResponse(CMD_ACK, NULL, 0);
            break;

        case CMD_START_TEST:
            Test_Manager_Start();
            RS485_SendResponse(CMD_ACK, NULL, 0);
            break;

        case CMD_STOP_TEST:
            Test_Manager_Stop();
            RS485_SendResponse(CMD_ACK, NULL, 0);
            break;

        case CMD_SET_BRAKE:
            if (msg->data_len >= 1) {
                // Chuyển sang chế độ Manual để điều khiển trực tiếp
                Test_Manager_Stop();
                system_debug.debug_mode = 1;
                system_debug.manual_brake_cmd = msg->data[0];
                RS485_SendResponse(CMD_ACK, NULL, 0);
            }
            break;

        case CMD_SET_GEAR:
            if (msg->data_len >= 1) {
                system_debug.debug_mode = 1;
                system_debug.manual_gear_cmd = msg->data[0];
                RS485_SendResponse(CMD_ACK, NULL, 0);
            }
            break;

        case CMD_SET_KEY:
            if (msg->data_len >= 1) {
                system_debug.debug_mode = 1;
                system_debug.manual_key_cmd = msg->data[0];
                RS485_SendResponse(CMD_ACK, NULL, 0);
            }
            break;

        case CMD_READ_CAR_STAT:
            // Gửi lại trạng thái hiện tại (step, status, cycle)
            response_data[0] = system_debug.test_step;
            // (Thêm các thông tin khác nếu cần gói tin dài hơn)
            RS485_SendResponse(msg->command, response_data, 1);
            break;

        default:
            // Lệnh không xác định
            break;
    }
}
