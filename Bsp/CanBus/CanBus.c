#include "CanBus.h"
#include "fdcan.h"

#include <string.h>

typedef struct {
    CAN_RxFrame_t frames[CANBUS_RX_QUEUE_DEPTH];
    volatile uint16_t head;
    volatile uint16_t tail;
} CANBus_RxQueue_t;

static CANBus_RxQueue_t s_rx_queues[2];
static uint8_t s_next_pop_port = 0U;

volatile uint32_t CAN1_RxID, CAN2_RxID;
volatile uint8_t CAN1_RxData[CANBUS_MAX_LEGACY_BYTES], CAN2_RxData[CANBUS_MAX_LEGACY_BYTES];
volatile uint8_t CAN1_RxReceived, CAN2_RxReceived;
volatile uint8_t CAN1_TxReady = 1U;
volatile uint8_t CAN2_TxReady = 1U;
volatile uint32_t CAN_LastRxTick = 0U;

static const uint8_t s_dlc_to_bytes[16] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
    8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
};

static uint8_t CANBus_PortToIndex(CAN_Port_t port)
{
    return (port == CAN_PORT_2) ? 1U : 0U;
}

static FDCAN_HandleTypeDef *CANBus_HandleFromPort(CAN_Port_t port)
{
    return (port == CAN_PORT_2) ? &hfdcan2 : &hfdcan1;
}

static CAN_Port_t CANBus_PortFromHandle(FDCAN_HandleTypeDef *hfdcan)
{
    return (hfdcan->Instance == FDCAN2) ? CAN_PORT_2 : CAN_PORT_1;
}

static volatile uint8_t *CANBus_LegacyFlagPtr(CAN_Port_t port)
{
    return (port == CAN_PORT_2) ? &CAN2_RxReceived : &CAN1_RxReceived;
}

static void CANBus_SetLegacyReady(CAN_Port_t port, uint8_t ready)
{
    volatile uint8_t *flag = CANBus_LegacyFlagPtr(port);
    *flag = ready;
}

static void CANBus_CopyLegacyFrame(const CAN_RxFrame_t *frame)
{
    volatile uint32_t *legacy_id = (frame->port == CAN_PORT_2) ? &CAN2_RxID : &CAN1_RxID;
    volatile uint8_t *legacy_data = (frame->port == CAN_PORT_2) ? CAN2_RxData : CAN1_RxData;
    uint8_t copy_len = frame->len_bytes;

    if (copy_len > CANBUS_MAX_LEGACY_BYTES) {
        copy_len = CANBUS_MAX_LEGACY_BYTES;
    }

    *legacy_id = frame->id;
    for (uint8_t i = 0U; i < CANBUS_MAX_LEGACY_BYTES; i++) {
        legacy_data[i] = 0U;
    }
    for (uint8_t i = 0U; i < copy_len; i++) {
        legacy_data[i] = frame->data[i];
    }

    CANBus_SetLegacyReady(frame->port, 1U);
}

static void CANBus_UpdateLegacyFlagFromQueue(CAN_Port_t port)
{
    uint8_t index = CANBus_PortToIndex(port);
    CANBus_SetLegacyReady(port, (s_rx_queues[index].head != s_rx_queues[index].tail) ? 1U : 0U);
}

static void CANBus_UpdateTxReadyFlags(void)
{
    CAN1_TxReady = (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U) ? 1U : 0U;
    CAN2_TxReady = (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0U) ? 1U : 0U;
}

static uint8_t CANBus_RxQueuePush(const CAN_RxFrame_t *frame)
{
    uint8_t index = CANBus_PortToIndex(frame->port);
    CANBus_RxQueue_t *queue = &s_rx_queues[index];
    uint16_t next_head = (uint16_t)((queue->head + 1U) % CANBUS_RX_QUEUE_DEPTH);

    if (next_head == queue->tail) {
        system_debug.can_rx_drop_count++;
        return 0U;
    }

    queue->frames[queue->head] = *frame;
    queue->head = next_head;
    CANBus_SetLegacyReady(frame->port, 1U);
    return 1U;
}

static void CANBus_ConfigPort(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000U;
    filter.FilterID2 = 0x000U;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x00000000U;
    filter.FilterID2 = 0x00000000U;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    HAL_FDCAN_Start(hfdcan);
}

uint8_t CANBus_DlcCodeToBytes(uint32_t dlc_code)
{
    if (dlc_code >= 16U) {
        return 0U;
    }

    return s_dlc_to_bytes[dlc_code];
}

uint32_t CANBus_BytesToDlcCode(uint8_t len_bytes)
{
    for (uint32_t dlc = 0U; dlc < 16U; dlc++) {
        if (s_dlc_to_bytes[dlc] == len_bytes) {
            return dlc;
        }
    }

    return CANBUS_DLC_INVALID;
}

void CANBus_Start_Config(void)
{
    memset((void *)s_rx_queues, 0, sizeof(s_rx_queues));
    CAN1_RxID = 0U;
    CAN2_RxID = 0U;
    memset((void *)CAN1_RxData, 0, sizeof(CAN1_RxData));
    memset((void *)CAN2_RxData, 0, sizeof(CAN2_RxData));
    CAN1_RxReceived = 0U;
    CAN2_RxReceived = 0U;
    CAN_LastRxTick = 0U;
    system_debug.can_rx_seen = 0U;
    system_debug.can_last_rx_port = 0U;
    system_debug.can_last_rx_is_fd = 0U;
    system_debug.can_last_rx_id_type = 0U;
    system_debug.can_last_rx_len = 0U;
    system_debug.can_last_rx_id = 0U;
    system_debug.can_last_rx_tick = 0U;
    system_debug.can1_rx_seen = 0U;
    system_debug.can1_last_rx_is_fd = 0U;
    system_debug.can1_last_rx_id_type = 0U;
    system_debug.can1_last_rx_len = 0U;
    system_debug.can1_last_rx_id = 0U;
    system_debug.can1_last_rx_tick = 0U;
    system_debug.can2_rx_seen = 0U;
    system_debug.can2_last_rx_is_fd = 0U;
    system_debug.can2_last_rx_id_type = 0U;
    system_debug.can2_last_rx_len = 0U;
    system_debug.can2_last_rx_id = 0U;
    system_debug.can2_last_rx_tick = 0U;
    system_debug.can_app_last_loop_frames = 0U;
    system_debug.can_app_budget_hit_count = 0U;
    system_debug.can_app_pending_after_budget = 0U;

    CANBus_ConfigPort(&hfdcan1);
    CANBus_ConfigPort(&hfdcan2);
    CANBus_UpdateTxReadyFlags();
}

uint8_t CANBus_PopRxFrame(CAN_Port_t port, CAN_RxFrame_t *out_frame)
{
    uint8_t index = CANBus_PortToIndex(port);
    CANBus_RxQueue_t *queue = &s_rx_queues[index];

    if (out_frame == NULL) {
        return 0U;
    }

    if (queue->head == queue->tail) {
        CANBus_SetLegacyReady(port, 0U);
        return 0U;
    }

    *out_frame = queue->frames[queue->tail];
    queue->tail = (uint16_t)((queue->tail + 1U) % CANBUS_RX_QUEUE_DEPTH);
    CANBus_UpdateLegacyFlagFromQueue(port);
    return 1U;
}

uint8_t CANBus_PopAnyRxFrame(CAN_RxFrame_t *out_frame)
{
    if (out_frame == NULL) {
        return 0U;
    }

    for (uint8_t attempt = 0U; attempt < 2U; attempt++) {
        CAN_Port_t port = (CAN_Port_t)((s_next_pop_port + attempt) % 2U);

        if (CANBus_PopRxFrame(port, out_frame) != 0U) {
            s_next_pop_port = (uint8_t)((CANBus_PortToIndex(port) + 1U) % 2U);
            return 1U;
        }
    }

    return 0U;
}

uint8_t CANBus_HasPendingRx(void)
{
    if (s_rx_queues[0].head != s_rx_queues[0].tail) {
        return 1U;
    }

    if (s_rx_queues[1].head != s_rx_queues[1].tail) {
        return 1U;
    }

    return 0U;
}

uint8_t CANBus_SendFrame(CAN_Port_t port, const CAN_TxFrame_t *frame)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    FDCAN_HandleTypeDef *hfdcan = CANBus_HandleFromPort(port);
    uint32_t dlc_code;

    if (frame == NULL) {
        return 0U;
    }

    if ((frame->id_type != FDCAN_STANDARD_ID) && (frame->id_type != FDCAN_EXTENDED_ID)) {
        return 0U;
    }

    if ((frame->id_type == FDCAN_STANDARD_ID) && (frame->id > 0x7FFU)) {
        return 0U;
    }

    if ((frame->id_type == FDCAN_EXTENDED_ID) && (frame->id > 0x1FFFFFFFU)) {
        return 0U;
    }

    if (frame->len_bytes > CANBUS_MAX_DATA_LEN) {
        return 0U;
    }

    if ((frame->fd_format == FDCAN_CLASSIC_CAN) && (frame->len_bytes > 8U)) {
        return 0U;
    }

    dlc_code = CANBus_BytesToDlcCode(frame->len_bytes);
    if (dlc_code == CANBUS_DLC_INVALID) {
        return 0U;
    }

    tx_header.Identifier = frame->id;
    tx_header.IdType = frame->id_type;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = dlc_code;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = (frame->fd_format == FDCAN_FD_CAN) ? frame->brs : FDCAN_BRS_OFF;
    tx_header.FDFormat = frame->fd_format;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    CANBus_UpdateTxReadyFlags();
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U) {
        CANBus_UpdateTxReadyFlags();
        return 0U;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, (uint8_t *)frame->data) != HAL_OK) {
        CANBus_UpdateTxReadyFlags();
        return 0U;
    }

    CANBus_UpdateTxReadyFlags();
    return 1U;
}

void FDCAN1_SendMessage_Nomal(uint32_t id, uint8_t *data)
{
    CAN_TxFrame_t frame = {0};

    frame.id = id;
    frame.id_type = FDCAN_STANDARD_ID;
    frame.fd_format = FDCAN_CLASSIC_CAN;
    frame.brs = FDCAN_BRS_OFF;
    frame.len_bytes = 8U;
    if (data != NULL) {
        memcpy(frame.data, data, 8U);
    }

    (void)CANBus_SendFrame(CAN_PORT_1, &frame);
}

void FDCAN2_SendMessage_Nomal(uint32_t id, uint8_t *data)
{
    CAN_TxFrame_t frame = {0};

    frame.id = id;
    frame.id_type = FDCAN_STANDARD_ID;
    frame.fd_format = FDCAN_CLASSIC_CAN;
    frame.brs = FDCAN_BRS_OFF;
    frame.len_bytes = 8U;
    if (data != NULL) {
        memcpy(frame.data, data, 8U);
    }

    (void)CANBus_SendFrame(CAN_PORT_2, &frame);
}

uint8_t FDCAN1_SendFrame(const CAN_TxFrame_t *frame)
{
    return CANBus_SendFrame(CAN_PORT_1, frame);
}

uint8_t FDCAN2_SendFrame(const CAN_TxFrame_t *frame)
{
    return CANBus_SendFrame(CAN_PORT_2, frame);
}

uint8_t FDCAN1_SendMessage(uint32_t id, uint8_t *data)
{
    CAN_TxFrame_t frame = {0};

    CANBus_UpdateTxReadyFlags();
    if (CAN1_TxReady == 0U) {
        return 0U;
    }

    frame.id = id;
    frame.id_type = FDCAN_STANDARD_ID;
    frame.fd_format = FDCAN_CLASSIC_CAN;
    frame.brs = FDCAN_BRS_OFF;
    frame.len_bytes = 8U;
    if (data != NULL) {
        memcpy(frame.data, data, 8U);
    }

    return CANBus_SendFrame(CAN_PORT_1, &frame);
}

uint8_t FDCAN2_SendMessage(uint32_t id, uint8_t *data)
{
    CAN_TxFrame_t frame = {0};

    CANBus_UpdateTxReadyFlags();
    if (CAN2_TxReady == 0U) {
        return 0U;
    }

    frame.id = id;
    frame.id_type = FDCAN_STANDARD_ID;
    frame.fd_format = FDCAN_CLASSIC_CAN;
    frame.brs = FDCAN_BRS_OFF;
    frame.len_bytes = 8U;
    if (data != NULL) {
        memcpy(frame.data, data, 8U);
    }

    return CANBus_SendFrame(CAN_PORT_2, &frame);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef rx_header = {0};
        CAN_RxFrame_t frame = {0};

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, frame.data) != HAL_OK) {
            break;
        }

        frame.port = CANBus_PortFromHandle(hfdcan);
        frame.id = rx_header.Identifier;
        frame.id_type = rx_header.IdType;
        frame.fd_format = rx_header.FDFormat;
        frame.brs = rx_header.BitRateSwitch;
        frame.dlc_code = (uint8_t)rx_header.DataLength;
        frame.len_bytes = CANBus_DlcCodeToBytes(rx_header.DataLength);
        frame.timestamp = rx_header.RxTimestamp;

        system_debug.can_rx_count++;
        system_debug.can_rx_seen = 1U;
        system_debug.can_last_rx_port = (uint8_t)frame.port + 1U;
        system_debug.can_last_rx_is_fd = (frame.fd_format == FDCAN_FD_CAN) ? 1U : 0U;
        system_debug.can_last_rx_id_type = (frame.id_type == FDCAN_EXTENDED_ID) ? 1U : 0U;
        system_debug.can_last_rx_len = frame.len_bytes;
        system_debug.can_last_rx_id = frame.id;
        system_debug.can_last_rx_tick = HAL_GetTick();
        if (frame.port == CAN_PORT_1) {
            system_debug.can1_rx_seen = 1U;
            system_debug.can1_last_rx_is_fd = system_debug.can_last_rx_is_fd;
            system_debug.can1_last_rx_id_type = system_debug.can_last_rx_id_type;
            system_debug.can1_last_rx_len = frame.len_bytes;
            system_debug.can1_last_rx_id = frame.id;
            system_debug.can1_last_rx_tick = system_debug.can_last_rx_tick;
        } else {
            system_debug.can2_rx_seen = 1U;
            system_debug.can2_last_rx_is_fd = system_debug.can_last_rx_is_fd;
            system_debug.can2_last_rx_id_type = system_debug.can_last_rx_id_type;
            system_debug.can2_last_rx_len = frame.len_bytes;
            system_debug.can2_last_rx_id = frame.id;
            system_debug.can2_last_rx_tick = system_debug.can_last_rx_tick;
        }
        if (frame.fd_format == FDCAN_FD_CAN) {
            system_debug.can_rx_fd_count++;
        } else {
            system_debug.can_rx_classic_count++;
        }

        (void)CANBus_RxQueuePush(&frame);
        CANBus_CopyLegacyFrame(&frame);
        CAN_LastRxTick = system_debug.can_last_rx_tick;
    }
}
