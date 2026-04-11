#include "Protocol.h"
#include <string.h>

void Protocol_Init(Protocol_Parser_t *parser) {
    parser->state = STATE_WAIT_HEADER_1;
    parser->rx_index = 0;
    parser->length = 0;
    parser->command = 0;
    parser->calc_crc = 0;
}

static uint8_t Calc_CRC8(uint8_t current_crc, uint8_t data) {
    return current_crc ^ data; // XOR simple checksum
}

bool Protocol_ParseByte(Protocol_Parser_t *parser, uint8_t byte_in, MessageFrame_t *msg_out) {
    bool message_ready = false;

    switch (parser->state) {
        case STATE_WAIT_HEADER_1:
            if (byte_in == PROTOCOL_HEADER_1) {
                parser->state = STATE_WAIT_HEADER_2;
            }
            break;

        case STATE_WAIT_HEADER_2:
            if (byte_in == PROTOCOL_HEADER_2) {
                parser->state = STATE_WAIT_LENGTH;
            } else if (byte_in == PROTOCOL_HEADER_1) {
                parser->state = STATE_WAIT_HEADER_2; // Handle 55 55 AA case
            } else {
                parser->state = STATE_WAIT_HEADER_1;
            }
            break;

        case STATE_WAIT_LENGTH:
            parser->length = byte_in;
            parser->calc_crc = Calc_CRC8(0, byte_in); // Start CRC calculation
            if (parser->length == 0 || parser->length > MAX_PAYLOAD_SIZE + 1) {
                // Invalid length
                parser->state = STATE_WAIT_HEADER_1;
            } else {
                parser->state = STATE_WAIT_COMMAND;
            }
            break;

        case STATE_WAIT_COMMAND:
            parser->command = byte_in;
            parser->calc_crc = Calc_CRC8(parser->calc_crc, byte_in);
            if (parser->length > 1) {
                parser->rx_index = 0;
                parser->state = STATE_WAIT_DATA;
            } else {
                parser->state = STATE_WAIT_CRC;
            }
            break;

        case STATE_WAIT_DATA:
            parser->payload[parser->rx_index++] = byte_in;
            parser->calc_crc = Calc_CRC8(parser->calc_crc, byte_in);
            if (parser->rx_index >= parser->length - 1) {
                parser->state = STATE_WAIT_CRC;
            }
            break;

        case STATE_WAIT_CRC:
            if (byte_in == parser->calc_crc) {
                // Frame Valid
                msg_out->command = parser->command;
                msg_out->data_len = parser->length - 1;
                if (msg_out->data_len > 0) {
                    memcpy(msg_out->data, parser->payload, msg_out->data_len);
                }
                message_ready = true;
            }
            // Transition back to Wait Header
            parser->state = STATE_WAIT_HEADER_1;
            break;

        default:
            parser->state = STATE_WAIT_HEADER_1;
            break;
    }

    return message_ready;
}

uint16_t Protocol_PackMessage(uint8_t cmd, const uint8_t* data, uint8_t data_len, uint8_t* tx_buffer) {
    if (data_len > MAX_PAYLOAD_SIZE) {
        data_len = MAX_PAYLOAD_SIZE;
    }
    
    tx_buffer[0] = PROTOCOL_HEADER_1;
    tx_buffer[1] = PROTOCOL_HEADER_2;
    tx_buffer[2] = data_len + 1; // Length of cmd (1) + data_len
    tx_buffer[3] = cmd;
    
    uint8_t crc = Calc_CRC8(0, tx_buffer[2]);
    crc = Calc_CRC8(crc, tx_buffer[3]);
    
    for (uint8_t i = 0; i < data_len; i++) {
        tx_buffer[4 + i] = data[i];
        crc = Calc_CRC8(crc, data[i]);
    }
    
    tx_buffer[4 + data_len] = crc;
    return 5 + data_len; // Total bytes to send
}

/* Prototype for weak function - override in main app */
__attribute__((weak)) void Protocol_ProcessMessage(const MessageFrame_t *msg) {
    // Default empty handler
}
