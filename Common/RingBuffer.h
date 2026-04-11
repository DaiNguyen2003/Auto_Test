#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdint.h>
#include <stdbool.h>

#define RING_BUFFER_SIZE 256

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer_t;

void RingBuffer_Init(RingBuffer_t *rb);
bool RingBuffer_Push(RingBuffer_t *rb, uint8_t data);
bool RingBuffer_Pop(RingBuffer_t *rb, uint8_t *data);
uint16_t RingBuffer_GetCount(RingBuffer_t *rb);
bool RingBuffer_IsEmpty(RingBuffer_t *rb);
void RingBuffer_Clear(RingBuffer_t *rb);

#endif /* RINGBUFFER_H */
