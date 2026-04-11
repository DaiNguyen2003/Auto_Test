#include "Car_Signals.h"
#include "Car.h"
#include "common.h"

#include <string.h>

static uint8_t CAN_FrameGetBit(const CAN_RxFrame_t *frame, uint16_t bit_pos, uint8_t *bit_value)
{
    uint32_t payload_bits;

    if ((frame == NULL) || (bit_value == NULL)) {
        return 0U;
    }

    payload_bits = (uint32_t)frame->len_bytes * 8U;
    if (bit_pos >= payload_bits) {
        return 0U;
    }

    *bit_value = (uint8_t)((frame->data[bit_pos / 8U] >> (bit_pos % 8U)) & 0x01U);
    return 1U;
}

static uint8_t CAN_GetBitsIntel(const CAN_RxFrame_t *frame, const CAN_Signal_t *sig, uint64_t *raw_value)
{
    uint32_t payload_bits;
    uint64_t value = 0U;

    if ((frame == NULL) || (sig == NULL) || (raw_value == NULL)) {
        return 0U;
    }

    if ((sig->length == 0U) || (sig->length > 64U)) {
        return 0U;
    }

    payload_bits = (uint32_t)frame->len_bytes * 8U;
    if (((uint32_t)sig->signal_start_bit + (uint32_t)sig->length) > payload_bits) {
        return 0U;
    }

    for (uint8_t i = 0U; i < sig->length; i++) {
        uint16_t bit_pos = (uint16_t)(sig->signal_start_bit + i);
        uint8_t bit_value = 0U;

        if (CAN_FrameGetBit(frame, bit_pos, &bit_value) == 0U) {
            return 0U;
        }

        value |= ((uint64_t)bit_value << i);
    }

    *raw_value = value;
    return 1U;
}

static uint16_t CAN_GetNextMotorolaBit(uint16_t bit_pos)
{
    if ((bit_pos % 8U) == 0U) {
        return (uint16_t)(bit_pos + 15U);
    }

    return (uint16_t)(bit_pos - 1U);
}

static uint8_t CAN_GetBitsMotorola(const CAN_RxFrame_t *frame, const CAN_Signal_t *sig, uint64_t *raw_value)
{
    uint16_t bit_pos;
    uint64_t value = 0U;

    if ((frame == NULL) || (sig == NULL) || (raw_value == NULL)) {
        return 0U;
    }

    if ((sig->length == 0U) || (sig->length > 64U)) {
        return 0U;
    }

    bit_pos = sig->signal_start_bit;

    for (uint8_t i = 0U; i < sig->length; i++) {
        uint8_t bit_value = 0U;

        if (CAN_FrameGetBit(frame, bit_pos, &bit_value) == 0U) {
            return 0U;
        }

        value = (value << 1U) | (uint64_t)bit_value;

        if ((i + 1U) < sig->length) {
            bit_pos = CAN_GetNextMotorolaBit(bit_pos);
        }
    }

    *raw_value = value;
    return 1U;
}

static int64_t CAN_SignExtend(uint64_t raw_value, uint8_t bit_length)
{
    if ((bit_length == 0U) || (bit_length >= 64U)) {
        return (int64_t)raw_value;
    }

    if ((raw_value & (1ULL << (bit_length - 1U))) != 0U) {
        raw_value |= (~0ULL << bit_length);
    }

    return (int64_t)raw_value;
}

static uint8_t CAN_DecodeSignalValue(const CAN_RxFrame_t *frame, const CAN_Signal_t *sig, float *value)
{
    uint64_t raw_value = 0U;
    double physical_value;

    if ((frame == NULL) || (sig == NULL) || (value == NULL)) {
        return 0U;
    }

    if (sig->endian == CAN_INTEL) {
        if (CAN_GetBitsIntel(frame, sig, &raw_value) == 0U) {
            return 0U;
        }
    } else if (CAN_GetBitsMotorola(frame, sig, &raw_value) == 0U) {
        return 0U;
    }

    if (sig->is_signed != 0U) {
        physical_value = (double)CAN_SignExtend(raw_value, sig->length);
    } else {
        physical_value = (double)raw_value;
    }

    physical_value = (physical_value * (double)sig->factor) + (double)sig->offset;
    *value = (float)physical_value;
    return 1U;
}

static uint8_t CAN_MessageMatchesFrame(const CAN_Message_t *msg, const CAN_RxFrame_t *frame)
{
    if ((msg == NULL) || (frame == NULL)) {
        return 0U;
    }

    if ((msg->dlc == 0U) || (msg->id != frame->id)) {
        return 0U;
    }

    if (msg->id_type != frame->id_type) {
        return 0U;
    }

    if ((msg->frame_policy == CAN_FRAME_POLICY_CLASSIC_ONLY) && (frame->fd_format != FDCAN_CLASSIC_CAN)) {
        return 0U;
    }

    if ((msg->frame_policy == CAN_FRAME_POLICY_FD_ONLY) && (frame->fd_format != FDCAN_FD_CAN)) {
        return 0U;
    }

    if (frame->len_bytes < msg->dlc) {
        return 0U;
    }

    return 1U;
}

static void CAN_ApplySignal(Car_Define_Typedef *car, CAN_SignalName_t signal_name, float value)
{
    switch (signal_name) {
        case SIG_GEAR_ACTUAL:
            car->StsCar.Val_Gear_Act = (uint8_t)value;
            break;

        case SIG_BRAKE_PEDAL_VAL:
            car->StsCar.Val_Break_Position = (uint8_t)value;
            break;

        case SIG_BRAKE_SWITCH_STS:
        case SIG_BRAKE_LIGHT:
            car->StsCar.Val_Break_Pedal_Sts = (uint8_t)value;
            break;

        case SIG_KEY_VALID:
            car->StsCar.Val_Key_Act = (uint8_t)value;
            break;

        case SIG_STAT_TERMINAL:
            car->StsCar.Val_STAT_Terminal = (uint8_t)value;
            break;

        case SIG_LV_SOC:
            car->StsCar.Val_LV_soc = (uint8_t)value;
            break;

        case SIG_LV_BATT_CURRENT:
        case SIG_LV_TARGET_CURRENT:
            car->StsCar.Val_LV_current = value;
            break;

        case SIG_LV_TARGET_VOLTAGE:
            car->StsCar.Val_LV_Voltage = value;
            break;

        case SIG_HV_ONOFF_STS:
        case SIG_HV_STS:
            car->StsCar.Val_HV_Sts = (uint8_t)value;
            break;

        case SIG_DCDC_REQUEST:
            car->StsCar.Val_DCDC_Sts = (uint8_t)value;
            break;

        case SIG_DCDC_CURRENT:
            car->StsCar.Val_DCDC_Current = value;
            break;

        case SIG_DCDC_VOLTAGE:
            car->StsCar.Val_DCDC_Voltage = value;
            break;

        default:
            break;
    }
}

uint8_t Car_ParseFrame(const CAN_RxFrame_t *frame)
{
    Car_Define_Typedef *car = Car_GetActiveConfig();
    uint8_t matched_message = 0U;
    uint8_t parsed_signal = 0U;

    if ((car == NULL) || (frame == NULL)) {
        return 0U;
    }

    for (uint8_t i = 0U; i < MAXIMUM_MESS_AT_CAR; i++) {
        CAN_Message_t *msg = &car->Defined_CAN_Msg[i];
        uint8_t signal_count;

        if (CAN_MessageMatchesFrame(msg, frame) == 0U) {
            continue;
        }

        matched_message = 1U;
        signal_count = msg->sig_num;
        if (signal_count > CAN_MAX_SIG_PER_MSG) {
            signal_count = CAN_MAX_SIG_PER_MSG;
        }

        for (uint8_t s = 0U; s < signal_count; s++) {
            float decoded_value = 0.0f;

            if (CAN_DecodeSignalValue(frame, &msg->Signal[s], &decoded_value) == 0U) {
                system_debug.can_rx_parse_error_count++;
                continue;
            }

            CAN_ApplySignal(car, msg->Signal[s].name, decoded_value);
            parsed_signal = 1U;
        }
    }

    if (matched_message == 0U) {
        system_debug.can_rx_unmatched_count++;
    }

    return parsed_signal;
}

void Car_ParseCAN(uint32_t id, const uint8_t *data)
{
    CAN_RxFrame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.port = CAN_PORT_1;
    frame.id = id;
    frame.id_type = FDCAN_STANDARD_ID;
    frame.fd_format = FDCAN_CLASSIC_CAN;
    frame.brs = FDCAN_BRS_OFF;
    frame.dlc_code = (uint8_t)CANBus_BytesToDlcCode(8U);
    frame.len_bytes = 8U;
    if (data != NULL) {
        memcpy(frame.data, data, 8U);
    }

    (void)Car_ParseFrame(&frame);
}
