#ifndef RS485_H
#define RS485_H

#include "main.h"
#include "RingBuffer.h"
#include "Protocol.h"

extern RingBuffer_t rx3_ring_buffer;
extern RingBuffer_t rx4_ring_buffer;

/* --- API --- */
void RS485_Init(void);
void RS485_UART3_Send(uint8_t *data, uint16_t size);
void RS485_UART4_Send(uint8_t *data, uint16_t size);
void RS485_Update(void);
void RS485_SendResponse(uint8_t cmd, const uint8_t *data, uint8_t len);
void RS485_TestSend(void);

#endif /* RS485_H */
