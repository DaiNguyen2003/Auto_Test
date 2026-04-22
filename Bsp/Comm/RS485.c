#include "RS485.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>

/* --- UART State Variables --- */
RingBuffer_t rx3_ring_buffer;
RingBuffer_t rx4_ring_buffer;

Protocol_Parser_t comm_parser3;
Protocol_Parser_t comm_parser4;

uint8_t uart3_rx_byte;
uint8_t uart4_rx_byte;

/* --- Init: start receiving on UART3 and UART4 --- */
void RS485_Init(void)
{
    RingBuffer_Init(&rx3_ring_buffer);
    RingBuffer_Init(&rx4_ring_buffer);
    Protocol_Init(&comm_parser3);
    Protocol_Init(&comm_parser4);

    // HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1); // Disabled: Modbus uses DMA ReceiveToIdle on UART3
    // HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1); // Disabled: Modbus uses DMA ReceiveToIdle on UART4
}

/* --- Parse buffered data asynchronously in Main Loop/Task --- */
void RS485_Update(void)
{
    uint8_t byte_in;
    MessageFrame_t frame;
    
    // Process UART3
    while(RingBuffer_Pop(&rx3_ring_buffer, &byte_in)) {
        if (Protocol_ParseByte(&comm_parser3, byte_in, &frame)) {
            // LOG_DEBUG("UART3 RX: CMD 0x%02X, Len %d", frame.command, frame.data_len);
            Protocol_ProcessMessage(&frame);
        }
    }
    
    // Process UART4
    while(RingBuffer_Pop(&rx4_ring_buffer, &byte_in)) {
        if (Protocol_ParseByte(&comm_parser4, byte_in, &frame)) {
            // LOG_DEBUG("UART4 RX: CMD 0x%02X, Len %d", frame.command, frame.data_len);
            Protocol_ProcessMessage(&frame);
        }
    }
}

/* --- Send via UART3 (Software DE control on PB14) --- */
void RS485_UART3_Send(uint8_t *data, uint16_t size)
{
    HAL_GPIO_WritePin(DIR3_UART3_GPIO_Port, DIR3_UART3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Led_W_GPIO_Port, Led_W_Pin, 1); // Turn ON LED while sending
    HAL_UART_Transmit(&huart3, data, size, 100);
    HAL_GPIO_WritePin(DIR3_UART3_GPIO_Port, DIR3_UART3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Led_W_GPIO_Port, Led_W_Pin, 0); // Turn OFF LED
}

/* --- Send via UART4 (Hardware DE — auto controlled by STM32) --- */
void RS485_UART4_Send(uint8_t *data, uint16_t size)
{
    HAL_GPIO_WritePin(Led_W_GPIO_Port, Led_W_Pin, 1); // Turn ON LED while sending
    HAL_UART_Transmit(&huart4, data, size, 100);
    HAL_GPIO_WritePin(Led_W_GPIO_Port, Led_W_Pin, 0); // Turn OFF LED
}

/* --- UART RX Callback --- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        RingBuffer_Push(&rx3_ring_buffer, uart3_rx_byte);
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
    else if (huart->Instance == UART4) {
        RingBuffer_Push(&rx4_ring_buffer, uart4_rx_byte);
        HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);
    }
}

void RS485_SendResponse(uint8_t cmd, const uint8_t *data, uint8_t len) {
    uint8_t tx_buf[MAX_PAYLOAD_SIZE + 5];
    uint16_t size = Protocol_PackMessage(cmd, data, len, tx_buf);
    
    // Gửi phản hồi qua cả 2 cổng để đảm bảo PC nhận được
    RS485_UART3_Send(tx_buf, size);
    RS485_UART4_Send(tx_buf, size);
}

void RS485_TestSend(void) {
    char *msg3 = "Test UART3 (RS485-1): OK\r\n";
    char *msg4 = "Test UART4 (RS485-2): OK\r\n";
    RS485_UART3_Send((uint8_t*)msg3, strlen(msg3));
    RS485_UART4_Send((uint8_t*)msg4, strlen(msg4));
    system_debug.rs485_tx_count++; // Increment debug counter
}
