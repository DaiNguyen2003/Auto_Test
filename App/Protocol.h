#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/*
 * FRAME FORMAT CAO CẤP:
 * [HEADER1] [HEADER2] [LENGTH] [COMMAND] [DATA_0] ... [DATA_N] [CRC8]
 * - HEADER1  : 0x55
 * - HEADER2  : 0xAA
 * - LENGTH   : 1 byte (Kích thước của COMMAND + DATA)
 * - COMMAND  : 1 byte (Mã lệnh)
 * - DATA     : N byte (N = LENGTH - 1)
 * - CRC8     : 1 byte (Tính từ [LENGTH] đến hết [DATA_N] sử dụng hàm Checksum đơn giản)
 */

#define PROTOCOL_HEADER_1 0x55
#define PROTOCOL_HEADER_2 0xAA
#define MAX_PAYLOAD_SIZE  128

typedef enum {
    CMD_PING           = 0x00,
    CMD_START_TEST     = 0x01,
    CMD_STOP_TEST      = 0x02,
    CMD_SET_GEAR       = 0x10,
    CMD_SET_BRAKE      = 0x11,
    CMD_SET_KEY        = 0x12,
    CMD_READ_CAR_STAT  = 0x20,
    CMD_ERROR_REPORT   = 0xEE,
    CMD_ACK            = 0xFF
} Protocol_Cmd_t;

typedef enum {
    STATE_WAIT_HEADER_1 = 0,
    STATE_WAIT_HEADER_2,
    STATE_WAIT_LENGTH,
    STATE_WAIT_COMMAND,
    STATE_WAIT_DATA,
    STATE_WAIT_CRC
} Parser_State_t;

typedef struct {
    uint8_t command;
    uint8_t data[MAX_PAYLOAD_SIZE];
    uint8_t data_len;
} MessageFrame_t;

typedef struct {
    Parser_State_t state;
    uint8_t length;
    uint8_t command;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint8_t rx_index;
    uint8_t calc_crc;
} Protocol_Parser_t;

// API
void Protocol_Init(Protocol_Parser_t *parser);
bool Protocol_ParseByte(Protocol_Parser_t *parser, uint8_t byte_in, MessageFrame_t *msg_out);
uint16_t Protocol_PackMessage(uint8_t cmd, const uint8_t* data, uint8_t data_len, uint8_t* tx_buffer);
void Protocol_ProcessMessage(const MessageFrame_t *msg);

#endif /* PROTOCOL_H */
