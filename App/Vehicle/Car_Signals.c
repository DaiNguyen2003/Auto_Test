#include "Car_Signals.h"
#include "Car.h"
#include "common.h"

#include <string.h>

typedef enum {
    CAN_MATCH_NONE = 0,
    CAN_MATCH_OK,
    CAN_MATCH_LENGTH_ERROR
} CAN_FrameMatchResult_t;

static Car_DiagState_t g_car_diag_state;
static Car_SignalMonitorTable_t g_car_signal_monitor;
static uint8_t g_car_signal_monitor_slots[SIG_NUM];

static const char *const kCanSignalNameTokens[SIG_NUM] = {
    [SIG_BRAKE_SWITCH_STS] = "SIG_BRAKE_SWITCH_STS",
    [SIG_BRAKE_REGEN_MODE] = "SIG_BRAKE_REGEN_MODE",
    [SIG_BRAKE_PEDAL_VAL] = "SIG_BRAKE_PEDAL_VAL",
    [SIG_BRAKE_LIGHT] = "SIG_BRAKE_LIGHT",
    [SIG_BRAKE_LIGHT_REQ] = "SIG_BRAKE_LIGHT_REQ",
    [SIG_KEY_VALID] = "SIG_KEY_VALID",
    [SIG_KEY_FOB_LEFT_CAR] = "SIG_KEY_FOB_LEFT_CAR",
    [SIG_GEAR_TARGET] = "SIG_GEAR_TARGET",
    [SIG_GEAR_ACTUAL] = "SIG_GEAR_ACTUAL",
    [SIG_GEAR_VALID] = "SIG_GEAR_VALID",
    [SIG_STAT_TERMINAL] = "SIG_STAT_TERMINAL",
    [SIG_LV_CHARGE_REQ] = "SIG_LV_CHARGE_REQ",
    [SIG_LV_TARGET_CURRENT] = "SIG_LV_TARGET_CURRENT",
    [SIG_LV_TARGET_VOLTAGE] = "SIG_LV_TARGET_VOLTAGE",
    [SIG_LV_MAX_CHARGE_CURRENT] = "SIG_LV_MAX_CHARGE_CURRENT",
    [SIG_LV_MAX_DISCHARGE_CURRENT] = "SIG_LV_MAX_DISCHARGE_CURRENT",
    [SIG_LV_WAKEUP_REASON] = "SIG_LV_WAKEUP_REASON",
    [SIG_LV_BATT_CURRENT] = "SIG_LV_BATT_CURRENT",
    [SIG_LV_BATT_TEMP] = "SIG_LV_BATT_TEMP",
    [SIG_LV_BATT_TYPE] = "SIG_LV_BATT_TYPE",
    [SIG_LV_SOC] = "SIG_LV_SOC",
    [SIG_HV_ONOFF_STS] = "SIG_HV_ONOFF_STS",
    [SIG_HV_ONOFF_REQ] = "SIG_HV_ONOFF_REQ",
    [SIG_HV_STS] = "SIG_HV_STS",
    [SIG_DCDC_REQUEST] = "SIG_DCDC_REQUEST",
    [SIG_DCDC_CURRENT] = "SIG_DCDC_CURRENT",
    [SIG_DCDC_VOLTAGE] = "SIG_DCDC_VOLTAGE",
    [SIG_FMCU_TARGET_TQ_REQ] = "SIG_FMCU_TARGET_TQ_REQ",
    [SIG_FMCU_MOTOR_TQ_VALID] = "SIG_FMCU_MOTOR_TQ_VALID",
    [SIG_FMCU_MAX_SPEED_ALLOW] = "SIG_FMCU_MAX_SPEED_ALLOW",
    [SIG_BMS_REMAIN_CHARGE_TIME] = "SIG_BMS_REMAIN_CHARGE_TIME",
    [SIG_EVCC_AC_CHG_L1_TERMINAL_TEMP] = "SIG_EVCC_AC_CHG_L1_TERMINAL_TEMP",
};

#define CAR_SIGNAL_MONITOR_UNUSED_SLOT 0xFFU

static int32_t CAN_PhysicalValueToMilli(float value)
{
    double scaled = (double)value * (double)CAR_SIGNAL_MONITOR_VALUE_SCALE;

    if (scaled > 2147483647.0) {
        return 2147483647;
    }

    if (scaled < -2147483648.0) {
        return (-2147483647 - 1);
    }

    if (scaled >= 0.0) {
        scaled += 0.5;
    } else {
        scaled -= 0.5;
    }

    return (int32_t)scaled;
}

const char *Car_GetSignalNameToken(CAN_SignalName_t signal_name)
{
    if ((signal_name >= SIG_NUM) || (kCanSignalNameTokens[signal_name] == NULL)) {
        return "";
    }

    return kCanSignalNameTokens[signal_name];
}

const Car_SignalMonitorTable_t *Car_GetSignalMonitor(void)
{
    return &g_car_signal_monitor;
}

void Car_RebuildSignalMonitor(void)
{
    Car_Define_Typedef *car = Car_GetActiveConfig();
    uint16_t next_generation = (uint16_t)(g_car_signal_monitor.generation + 1U);

    if (next_generation == 0U) {
        next_generation = 1U;
    }

    memset(&g_car_signal_monitor, 0, sizeof(g_car_signal_monitor));
    memset(g_car_signal_monitor_slots, CAR_SIGNAL_MONITOR_UNUSED_SLOT, sizeof(g_car_signal_monitor_slots));

    g_car_signal_monitor.generation = next_generation;
    g_car_signal_monitor.active_car_type = (car != NULL) ? car->TypeCar : VF_89;

    if (car == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < MAXIMUM_MESS_AT_CAR; i++) {
        const CAN_Message_t *msg = &car->Defined_CAN_Msg[i];
        uint8_t signal_count = msg->sig_num;

        if ((msg->dlc == 0U) || (signal_count == 0U)) {
            continue;
        }

        if (signal_count > CAN_MAX_SIG_PER_MSG) {
            signal_count = CAN_MAX_SIG_PER_MSG;
        }

        for (uint8_t s = 0U; s < signal_count; s++) {
            CAN_SignalName_t signal_name = msg->Signal[s].name;
            uint16_t slot;

            if (signal_name >= SIG_NUM) {
                continue;
            }

            if (g_car_signal_monitor_slots[signal_name] != CAR_SIGNAL_MONITOR_UNUSED_SLOT) {
                continue;
            }

            if (g_car_signal_monitor.signal_count >= CAR_SIGNAL_MONITOR_MAX_ROWS) {
                return;
            }

            slot = g_car_signal_monitor.signal_count;
            g_car_signal_monitor.rows[slot].signal_name = signal_name;
            g_car_signal_monitor.rows[slot].defined = 1U;
            g_car_signal_monitor.rows[slot].seen = 0U;
            g_car_signal_monitor.rows[slot].value_valid = 0U;
            g_car_signal_monitor.rows[slot].value_milli = 0;
            g_car_signal_monitor.signal_count++;
            g_car_signal_monitor_slots[signal_name] = (uint8_t)slot;
        }
    }
}

static void CAN_UpdateSignalMonitor(CAN_SignalName_t signal_name, float value)
{
    uint8_t slot;
    Car_SignalMonitorRow_t *row;

    if (signal_name >= SIG_NUM) {
        return;
    }

    slot = g_car_signal_monitor_slots[signal_name];
    if ((slot == CAR_SIGNAL_MONITOR_UNUSED_SLOT) || (slot >= g_car_signal_monitor.signal_count)) {
        return;
    }

    row = &g_car_signal_monitor.rows[slot];
    row->seen = 1U;
    row->value_valid = 1U;
    row->value_milli = CAN_PhysicalValueToMilli(value);
}

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

static uint8_t CAN_GetBitsIntel(const CAN_RxFrame_t *frame, uint16_t start_bit, uint8_t length, uint64_t *raw_value)
{
    uint32_t payload_bits;
    uint64_t value = 0U;

    if ((frame == NULL) || (raw_value == NULL)) {
        return 0U;
    }

    if ((length == 0U) || (length > 64U)) {
        return 0U;
    }

    payload_bits = (uint32_t)frame->len_bytes * 8U;
    if (((uint32_t)start_bit + (uint32_t)length) > payload_bits) {
        return 0U;
    }

    for (uint8_t i = 0U; i < length; i++) {
        uint16_t bit_pos = (uint16_t)(start_bit + i);
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

static uint8_t CAN_GetBitsMotorola(const CAN_RxFrame_t *frame, uint16_t start_bit, uint8_t length, uint64_t *raw_value)
{
    uint16_t bit_pos;
    uint64_t value = 0U;

    if ((frame == NULL) || (raw_value == NULL)) {
        return 0U;
    }

    if ((length == 0U) || (length > 64U)) {
        return 0U;
    }

    bit_pos = start_bit;

    for (uint8_t i = 0U; i < length; i++) {
        uint8_t bit_value = 0U;

        if (CAN_FrameGetBit(frame, bit_pos, &bit_value) == 0U) {
            return 0U;
        }

        value = (value << 1U) | (uint64_t)bit_value;

        if ((i + 1U) < length) {
            bit_pos = CAN_GetNextMotorolaBit(bit_pos);
        }
    }

    *raw_value = value;
    return 1U;
}

static uint8_t CAN_DecodeRawValue(const CAN_RxFrame_t *frame,
                                  uint16_t start_bit,
                                  uint8_t length,
                                  CAN_Endian_t endian,
                                  uint64_t *raw_value)
{
    if (endian == CAN_INTEL) {
        return CAN_GetBitsIntel(frame, start_bit, length, raw_value);
    }

    return CAN_GetBitsMotorola(frame, start_bit, length, raw_value);
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

    if (CAN_DecodeRawValue(frame, sig->signal_start_bit, sig->length, sig->endian, &raw_value) == 0U) {
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

static CAN_FrameMatchResult_t CAN_EvaluateFrameMatch(CAN_MessagePort_t port,
                                                     uint32_t id,
                                                     CAN_IdType_t id_type,
                                                     CAN_FramePolicy_t frame_policy,
                                                     uint8_t expected_len,
                                                     const CAN_RxFrame_t *frame)
{
    if (frame == NULL) {
        return CAN_MATCH_NONE;
    }

    if ((port != CAN_MSG_PORT_ANY) && ((CAN_Port_t)port != frame->port)) {
        return CAN_MATCH_NONE;
    }

    if ((expected_len == 0U) || (id != frame->id) || (id_type != frame->id_type)) {
        return CAN_MATCH_NONE;
    }

    if ((frame_policy == CAN_FRAME_POLICY_CLASSIC_ONLY) && (frame->fd_format != FDCAN_CLASSIC_CAN)) {
        return CAN_MATCH_NONE;
    }

    if ((frame_policy == CAN_FRAME_POLICY_FD_ONLY) && (frame->fd_format != FDCAN_FD_CAN)) {
        return CAN_MATCH_NONE;
    }

    if (frame->len_bytes < expected_len) {
        return CAN_MATCH_LENGTH_ERROR;
    }

    return CAN_MATCH_OK;
}

static CAN_FrameMatchResult_t CAN_MessageMatchesFrame(const CAN_Message_t *msg, const CAN_RxFrame_t *frame)
{
    if (msg == NULL) {
        return CAN_MATCH_NONE;
    }

    return CAN_EvaluateFrameMatch(msg->port, msg->id, msg->id_type, msg->frame_policy, msg->dlc, frame);
}

static CAN_FrameMatchResult_t CAN_DiagMessageMatchesFrame(const CAN_DiagMessageDef_t *msg, const CAN_RxFrame_t *frame)
{
    if (msg == NULL) {
        return CAN_MATCH_NONE;
    }

    return CAN_EvaluateFrameMatch(msg->port, msg->id, msg->id_type, msg->frame_policy, msg->expected_len, frame);
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

        case SIG_BRAKE_LIGHT_REQ:
            car->StsCar.Val_Brake_Light_Req = (uint8_t)value;
            break;

        case SIG_KEY_VALID:
            car->StsCar.Val_Key_Act = (uint8_t)value;
            break;

        case SIG_GEAR_VALID:
            car->StsCar.Val_Gear_Valid = (uint8_t)value;
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

        case SIG_LV_BATT_TEMP:
            car->StsCar.Val_LV_Batt_Temp = value;
            break;

        case SIG_LV_BATT_TYPE:
            car->StsCar.Val_LV_Batt_Type = (uint8_t)value;
            break;

        case SIG_LV_TARGET_VOLTAGE:
            car->StsCar.Val_LV_Voltage = value;
            break;

        case SIG_HV_ONOFF_REQ:
            car->StsCar.Val_HV_OnOff_Req = (uint8_t)value;
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

        case SIG_FMCU_TARGET_TQ_REQ:
            car->StsCar.Val_FMCU_TargetTqReq = value;
            break;

        case SIG_FMCU_MOTOR_TQ_VALID:
            car->StsCar.Val_FMCU_MotorTqValid = (uint8_t)value;
            break;

        case SIG_FMCU_MAX_SPEED_ALLOW:
            car->StsCar.Val_FMCU_MaxSpeedAllow = (int16_t)value;
            break;

        case SIG_BMS_REMAIN_CHARGE_TIME:
            car->StsCar.Val_BMS_RemainChargeTime = (uint16_t)value;
            break;

        case SIG_EVCC_AC_CHG_L1_TERMINAL_TEMP:
            car->StsCar.Val_EVCC_ACChgL1Temp = value;
            break;

        default:
            break;
    }
}

static void CAN_DiagResetDebugSummary(void)
{
    system_debug.can2_diag_match_count = 0U;
    system_debug.can2_diag_unmatched_count = 0U;
    system_debug.can2_diag_parse_error_count = 0U;
    system_debug.can2_diag_active_count = 0U;
    system_debug.can2_diag_last_signal = 0U;
    system_debug.can2_diag_last_raw_value = 0U;
    system_debug.can2_diag_last_update_tick = 0U;
}

void Car_ResetDiagState(void)
{
    memset(&g_car_diag_state, 0, sizeof(g_car_diag_state));
    CAN_DiagResetDebugSummary();
}

const Car_DiagState_t *Car_GetDiagState(void)
{
    return &g_car_diag_state;
}

static void CAN_DiagUpdateSignal(CAN_DiagSignalName_t signal_name, uint64_t raw_value)
{
    Car_DiagSignalState_t *state;
    uint32_t now;
    uint8_t was_active;
    uint8_t is_active;

    if ((signal_name <= SIG_DIAG_NONE) || (signal_name >= SIG_DIAG_NUM)) {
        return;
    }

    state = &g_car_diag_state.signal[signal_name];
    now = HAL_GetTick();
    was_active = state->active;
    is_active = (raw_value != 0U) ? 1U : 0U;

    state->seen = 1U;
    state->raw_value = raw_value;
    state->active = is_active;
    state->last_update_tick = now;

    if ((was_active == 0U) && (is_active != 0U)) {
        g_car_diag_state.active_count++;
    } else if ((was_active != 0U) && (is_active == 0U) && (g_car_diag_state.active_count > 0U)) {
        g_car_diag_state.active_count--;
    }

    g_car_diag_state.last_signal = signal_name;
    g_car_diag_state.last_raw_value = raw_value;
    g_car_diag_state.last_update_tick = now;

    system_debug.can2_diag_active_count = g_car_diag_state.active_count;
    system_debug.can2_diag_last_signal = (uint16_t)signal_name;
    system_debug.can2_diag_last_raw_value = raw_value;
    system_debug.can2_diag_last_update_tick = now;
}

static uint8_t CAN_ParseDiagFrame(const CAN_RxFrame_t *frame)
{
    Car_Define_Typedef *car = Car_GetActiveConfig();
    const CAN_DiagProfile_t *profile;
    uint8_t matched_message = 0U;
    uint8_t header_candidate = 0U;
    uint8_t parsed_signal = 0U;
    uint8_t frame_parse_error = 0U;

    if ((car == NULL) || (frame == NULL)) {
        return 0U;
    }

    profile = car->DiagProfile;
    if ((profile == NULL) || (profile->messages == NULL) || (profile->message_count == 0U)) {
        g_car_diag_state.unmatched_frame_count++;
        system_debug.can2_diag_unmatched_count = g_car_diag_state.unmatched_frame_count;
        system_debug.can_rx_unmatched_count++;
        return 0U;
    }

    for (uint16_t i = 0U; i < profile->message_count; i++) {
        const CAN_DiagMessageDef_t *msg = &profile->messages[i];
        CAN_FrameMatchResult_t match_result = CAN_DiagMessageMatchesFrame(msg, frame);

        if (match_result == CAN_MATCH_NONE) {
            continue;
        }

        header_candidate = 1U;

        if (match_result == CAN_MATCH_LENGTH_ERROR) {
            frame_parse_error = 1U;
            continue;
        }

        matched_message = 1U;

        for (uint8_t s = 0U; s < msg->signal_count; s++) {
            uint64_t raw_value = 0U;

            if (CAN_DecodeRawValue(frame,
                                   msg->signals[s].start_bit,
                                   msg->signals[s].length,
                                   msg->signals[s].endian,
                                   &raw_value) == 0U) {
                frame_parse_error = 1U;
                continue;
            }

            CAN_DiagUpdateSignal(msg->signals[s].name, raw_value);
            parsed_signal = 1U;
        }
    }

    if (matched_message != 0U) {
        g_car_diag_state.matched_frame_count++;
        system_debug.can2_diag_match_count = g_car_diag_state.matched_frame_count;
    } else if (header_candidate == 0U) {
        g_car_diag_state.unmatched_frame_count++;
        system_debug.can2_diag_unmatched_count = g_car_diag_state.unmatched_frame_count;
        system_debug.can_rx_unmatched_count++;
    }

    if (frame_parse_error != 0U) {
        g_car_diag_state.parse_error_count++;
        system_debug.can2_diag_parse_error_count = g_car_diag_state.parse_error_count;
        system_debug.can_rx_parse_error_count++;
    }

    return parsed_signal;
}

uint8_t Car_ParseFrame(const CAN_RxFrame_t *frame)
{
    Car_Define_Typedef *car = Car_GetActiveConfig();
    uint8_t matched_message = 0U;
    uint8_t parsed_signal = 0U;
    uint8_t frame_parse_error = 0U;

    if ((car == NULL) || (frame == NULL)) {
        return 0U;
    }

    for (uint8_t i = 0U; i < MAXIMUM_MESS_AT_CAR; i++) {
        CAN_Message_t *msg = &car->Defined_CAN_Msg[i];
        CAN_FrameMatchResult_t match_result = CAN_MessageMatchesFrame(msg, frame);
        uint8_t signal_count;

        if (match_result == CAN_MATCH_NONE) {
            continue;
        }

        matched_message = 1U;

        if (match_result == CAN_MATCH_LENGTH_ERROR) {
            frame_parse_error = 1U;
            continue;
        }

        signal_count = msg->sig_num;
        if (signal_count > CAN_MAX_SIG_PER_MSG) {
            signal_count = CAN_MAX_SIG_PER_MSG;
        }

        for (uint8_t s = 0U; s < signal_count; s++) {
            float decoded_value = 0.0f;

            if (CAN_DecodeSignalValue(frame, &msg->Signal[s], &decoded_value) == 0U) {
                frame_parse_error = 1U;
                continue;
            }

            CAN_ApplySignal(car, msg->Signal[s].name, decoded_value);
            CAN_UpdateSignalMonitor(msg->Signal[s].name, decoded_value);
            parsed_signal = 1U;
        }
    }

    if (matched_message == 0U) {
        system_debug.can_rx_unmatched_count++;
    }

    if (frame_parse_error != 0U) {
        system_debug.can_rx_parse_error_count++;
    }

    return parsed_signal;
}

uint8_t Car_ProcessRxFrame(const CAN_RxFrame_t *frame)
{
    if (frame == NULL) {
        return 0U;
    }

    if (frame->port == CAN_PORT_2) {
        return CAN_ParseDiagFrame(frame);
    }

    return Car_ParseFrame(frame);
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
