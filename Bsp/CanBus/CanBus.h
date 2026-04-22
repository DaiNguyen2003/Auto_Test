#ifndef __CANBUS_H
#define __CANBUS_H

#include "main.h"

#define CANBUS_MAX_DATA_LEN      64U
#define CANBUS_RX_QUEUE_DEPTH    32U
#define CANBUS_MAX_LEGACY_BYTES   8U
#define CANBUS_DLC_INVALID 0xFFFFFFFFUL

typedef enum {
    CAN_PORT_1 = 0,
    CAN_PORT_2 = 1
} CAN_Port_t;

typedef struct {
    CAN_Port_t port;
    uint32_t id;
    uint32_t id_type;
    uint32_t fd_format;
    uint32_t brs;
    uint8_t dlc_code;
    uint8_t len_bytes;
    uint16_t timestamp;
    uint8_t data[CANBUS_MAX_DATA_LEN];
} CAN_RxFrame_t;

typedef struct {
    uint32_t id;
    uint32_t id_type;
    uint32_t fd_format;
    uint32_t brs;
    uint8_t len_bytes;
    uint8_t data[CANBUS_MAX_DATA_LEN];
} CAN_TxFrame_t;

/* Legacy last-frame state kept for UI/virtual compatibility. */
extern volatile uint32_t CAN1_RxID, CAN2_RxID;
extern volatile uint8_t CAN1_RxData[CANBUS_MAX_LEGACY_BYTES], CAN2_RxData[CANBUS_MAX_LEGACY_BYTES];
extern volatile uint8_t CAN1_RxReceived, CAN2_RxReceived;
extern volatile uint8_t CAN1_TxReady;
extern volatile uint8_t CAN2_TxReady;
extern volatile uint32_t CAN_LastRxTick;

void CANBus_Start_Config(void);
uint8_t CANBus_PopRxFrame(CAN_Port_t port, CAN_RxFrame_t *out_frame);
uint8_t CANBus_PopAnyRxFrame(CAN_RxFrame_t *out_frame);
uint8_t CANBus_HasPendingRx(void);
uint8_t CANBus_SendFrame(CAN_Port_t port, const CAN_TxFrame_t *frame);
uint8_t CANBus_DlcCodeToBytes(uint32_t dlc_code);
uint32_t CANBus_BytesToDlcCode(uint8_t len_bytes);

/* Normal send (no TxReady check) */
void FDCAN1_SendMessage_Nomal(uint32_t id, uint8_t *data);
void FDCAN2_SendMessage_Nomal(uint32_t id, uint8_t *data);
uint8_t FDCAN1_SendFrame(const CAN_TxFrame_t *frame);
uint8_t FDCAN2_SendFrame(const CAN_TxFrame_t *frame);

/* Guarded send (checks TxReady flag) */
uint8_t FDCAN1_SendMessage(uint32_t id, uint8_t *data);
uint8_t FDCAN2_SendMessage(uint32_t id, uint8_t *data);

#endif
