#include "RingBuffer.h"

void RingBuffer_Init(RingBuffer_t *rb) {
    rb->head = 0;
    rb->tail = 0;
}

bool RingBuffer_Push(RingBuffer_t *rb, uint8_t data) {
    uint16_t next_head = (rb->head + 1) % RING_BUFFER_SIZE;
    if (next_head == rb->tail) {
        return false; // Buffer Full
    }
    rb->buffer[rb->head] = data;
    rb->head = next_head;
    return true;
}

bool RingBuffer_Pop(RingBuffer_t *rb, uint8_t *data) {
    if (rb->head == rb->tail) {
        return false; // Buffer Empty
    }
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    return true;
}

uint16_t RingBuffer_GetCount(RingBuffer_t *rb) {
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    } else {
        return RING_BUFFER_SIZE - rb->tail + rb->head;
    }
}

bool RingBuffer_IsEmpty(RingBuffer_t *rb) {
    return (rb->head == rb->tail);
}

void RingBuffer_Clear(RingBuffer_t *rb) {
    rb->head = 0;
    rb->tail = 0;
}
