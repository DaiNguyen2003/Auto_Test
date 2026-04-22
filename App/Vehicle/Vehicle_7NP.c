#include "Car.h"

#define DIAG_ARRAY_LEN(array) (sizeof(array) / sizeof((array)[0]))
#define DIAG_SIG(_name, _start_bit, _length, _is_signed, _endian) \
    {                                                             \
        .name = (_name),                                          \
        .factor = 1.0f,                                           \
        .offset = 0.0f,                                           \
        .start_bit = (_start_bit),                                \
        .length = (_length),                                      \
        .is_signed = (_is_signed),                                \
        .endian = (_endian)                                       \
    }

/*
 * V1 deferred raw fault/warning rows from Phan_tich_ICAN_Message_Loi:
 * - FCAM_TSR_STATUS.FCAM_TSR_Typ2_Flashing_Warning
 * - FCAM_TSR_STATUS.FCAM_TSR_AudWarning_Feed
 * - FCAM_TSR_STATUS.FCAM_TSR_Typ1_Flashing_Warning
 * - CVC_ACU_Info1.CVC_SeatBelt_2nd_Warning_TIMER
 * - CPD_OUTPUT.CPD_Init_Warning_Delay
 * - CPD_WarnMsg.CPD_SensorFault
 * - BCM_Comfort_Confirm_FL_sts.BCM_FL_STAT_MirrorDefaultSTATtin
 * - BCM_Comfort_Confirm_FR_sts.BCM_FR_STAT_MirrorDefaultSTATtin
 *
 * They are intentionally left out because the current business meaning is not
 * stable enough yet to standardize across vehicle lines without guessing.
 */

static const CAN_DiagSignalDef_t kDiag_FCAM_TSR_STATUS_Signals[] = {
    /* FCAM_TSR_STATUS.FCAM_TSR_Warning_audible */
    DIAG_SIG(SIG_DIAG_TRAFFIC_SIGN_RECOGNITION_AUDIBLE_WARNING, 16U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_FMCU_Status_Signals[] = {
    /* FMCU_Status.FMCU_StsMCUFault */
    DIAG_SIG(SIG_DIAG_MOTOR_CONTROLLER_MCU_FAILURE, 51U, 4U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_CVC_LOGICAL_CLAMP_Signals[] = {
    /* CVC_LOGICAL_CLAMP.Warning_Clamp_Incompatible */
    DIAG_SIG(SIG_DIAG_POWER_CLAMP_INCOMPATIBILITY_WARNING, 37U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_BCM_FL_Measured_Data_Signals[] = {
    /* BCM_FL_Measured_Data.Resp_Error */
    DIAG_SIG(SIG_DIAG_FRONT_LEFT_BODY_MODULE_RESPONSE_FAILURE, 35U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_FL_Measured_Data.EBS_error */
    DIAG_SIG(SIG_DIAG_ELECTRONIC_BRAKE_SYSTEM_FAILURE, 37U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_SS_DriveReq_0x108_Signals[] = {
    /* SS_DriveReq_0x108.SS_Fault */
    DIAG_SIG(SIG_DIAG_SUNSHADE_FAILURE, 35U, 4U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_TRM_LIGHT_Signals[] = {
    /* TRM_LIGHT.TrailerSystemError */
    DIAG_SIG(SIG_DIAG_TRAILER_SYSTEM_FAILURE, 15U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_BCM_LVBAT_Data_Signals[] = {
    /* BCM_LVBAT_Data.LV_ErrorSts */
    DIAG_SIG(SIG_DIAG_LOW_VOLTAGE_SYSTEM_FAILURE, 14U, 3U, 0U, CAN_MOTOROLA),
    /* BCM_LVBAT_Data.LV_CellBalance_Error */
    DIAG_SIG(SIG_DIAG_LOW_VOLTAGE_CELL_BALANCE_FAILURE, 60U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_FCAM_LDW_LKA_ELK_Signals[] = {
    /* FCAM_LDW_LKA_ELK.FCAM_ELK_Warning */
    DIAG_SIG(SIG_DIAG_EMERGENCY_LANE_KEEPING_WARNING, 55U, 4U, 0U, CAN_MOTOROLA),
    /* FCAM_LDW_LKA_ELK.FCAM_LDW_Warning */
    DIAG_SIG(SIG_DIAG_LANE_DEPARTURE_WARNING, 41U, 2U, 0U, CAN_MOTOROLA),
    /* FCAM_LDW_LKA_ELK.FCAM_handsoff_Warning_Mode_2 */
    DIAG_SIG(SIG_DIAG_HANDS_OFF_WARNING, 63U, 3U, 0U, CAN_MOTOROLA),
    /* FCAM_LDW_LKA_ELK.FCAM_LKA_Warning */
    DIAG_SIG(SIG_DIAG_LANE_KEEPING_ASSIST_WARNING, 36U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_WCBS_ESC_Status_Signals[] = {
    /* WCBS_ESC_Status.ESC_HSA_Status_Fault */
    DIAG_SIG(SIG_DIAG_ESC_HSA_FAILURE, 41U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_EPS_SAS_Sensor_Signals[] = {
    /* EPS_SAS_Sensor.SAS_SASFailure */
    DIAG_SIG(SIG_DIAG_STEERING_ANGLE_SENSOR_FAILURE, 33U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_MHU_ADSA02_Signals[] = {
    /* MHU_ADSA02.Nav_errorSts */
    DIAG_SIG(SIG_DIAG_NAVIGATION_FAILURE, 23U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_FCAM_Display_Info_Signals[] = {
    /* FCAM_Display_Info.FCAM_SCAM_Fault */
    DIAG_SIG(SIG_DIAG_SURROUND_CAMERA_SYSTEM_FAILURE, 101U, 2U, 0U, CAN_MOTOROLA),
    /* FCAM_Display_Info.FCAM_MRR_Fault */
    DIAG_SIG(SIG_DIAG_MID_RANGE_RADAR_FAILURE, 99U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_BCM_SunroofSts_Signals[] = {
    /* BCM_SunroofSts.SR_Undervoltage_Fault_Sts */
    DIAG_SIG(SIG_DIAG_SUNROOF_UNDERVOLTAGE_FAULT, 0U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunroofSts.SR_Overvoltage_Fault_Sts */
    DIAG_SIG(SIG_DIAG_SUNROOF_OVERVOLTAGE_FAULT, 1U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunroofSts.SR_MotorReverse_Fault_Sts */
    DIAG_SIG(SIG_DIAG_SUNROOF_MOTOR_REVERSE_FAULT, 8U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunroofSts.SR_Hall_Fault_Sts */
    DIAG_SIG(SIG_DIAG_SUNROOF_HALL_SENSOR_FAULT, 9U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunroofSts.SR_Relay_Fault */
    DIAG_SIG(SIG_DIAG_SUNROOF_RELAY_FAULT, 11U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunroofSts.SR_Commu_Fault_Sts */
    DIAG_SIG(SIG_DIAG_SUNROOF_COMMUNICATION_FAULT, 12U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunroofSts.SR_ThermalProtection_Fault_Sts */
    DIAG_SIG(SIG_DIAG_SUNROOF_THERMAL_PROTECTION_FAULT, 16U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunroofSts.SR_Error_RSP */
    DIAG_SIG(SIG_DIAG_SUNROOF_RESPONSE_FAILURE, 28U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_BCM_SunshadeSts_Signals[] = {
    /* BCM_SunshadeSts.SS_MotorReverse_Fault_Status */
    DIAG_SIG(SIG_DIAG_SUNSHADE_MOTOR_REVERSE_FAULT, 0U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunshadeSts.SS_Undervoltage_Fault_Status */
    DIAG_SIG(SIG_DIAG_SUNSHADE_UNDERVOLTAGE_FAULT, 7U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunshadeSts.SS_Overvoltage_Fault_Status */
    DIAG_SIG(SIG_DIAG_SUNSHADE_OVERVOLTAGE_FAULT, 10U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunshadeSts.SS_Relay_Fault */
    DIAG_SIG(SIG_DIAG_SUNSHADE_RELAY_FAULT, 11U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunshadeSts.SS_Hall_Fault_Status */
    DIAG_SIG(SIG_DIAG_SUNSHADE_HALL_SENSOR_FAULT, 13U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunshadeSts.SS_Commu_Fault_Status */
    DIAG_SIG(SIG_DIAG_SUNSHADE_COMMUNICATION_FAULT, 14U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_SunshadeSts.SS_Error_RSP */
    DIAG_SIG(SIG_DIAG_SUNSHADE_RESPONSE_FAILURE, 63U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_BCM_FKS_STATUS_Signals[] = {
    /* BCM_FKS_STATUS.FKS_RespError */
    DIAG_SIG(SIG_DIAG_KEYLESS_ENTRY_RESPONSE_FAILURE, 0U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_FKS_STATUS.FKS_HWError */
    DIAG_SIG(SIG_DIAG_KEYLESS_ENTRY_HARDWARE_FAILURE, 3U, 1U, 0U, CAN_MOTOROLA),
    /* BCM_FKS_STATUS.FKS_AntennaError */
    DIAG_SIG(SIG_DIAG_KEYLESS_ENTRY_ANTENNA_FAILURE, 5U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_WCBS_STATUS_Signals[] = {
    /* WCBS_STATUS.ABSFailed */
    DIAG_SIG(SIG_DIAG_ABS_FAILURE, 14U, 1U, 0U, CAN_MOTOROLA),
    /* WCBS_STATUS.ABS_EBDFailed */
    DIAG_SIG(SIG_DIAG_EBD_FAILURE, 55U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_FCAM_AEB_FCW_Signals[] = {
    /* FCAM_AEB_FCW.FCAM_AEB_Warning */
    DIAG_SIG(SIG_DIAG_AUTONOMOUS_EMERGENCY_BRAKING_WARNING, 13U, 2U, 0U, CAN_MOTOROLA),
    /* FCAM_AEB_FCW.FCAM_FCW_warning */
    DIAG_SIG(SIG_DIAG_FORWARD_COLLISION_WARNING, 43U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_WCBS_VAFS_Signals[] = {
    /* WCBS_VAFS.WCBS_EPB_Fault */
    DIAG_SIG(SIG_DIAG_ELECTRONIC_PARKING_BRAKE_FAILURE, 26U, 1U, 0U, CAN_MOTOROLA),
    /* WCBS_VAFS.WCBS_ROMFailure */
    DIAG_SIG(SIG_DIAG_BRAKE_CONTROLLER_ROM_FAILURE, 17U, 1U, 0U, CAN_MOTOROLA),
    /* WCBS_VAFS.WCBS_TSMFailure */
    DIAG_SIG(SIG_DIAG_TRACTION_STABILITY_MANAGEMENT_FAILURE, 53U, 1U, 0U, CAN_MOTOROLA),
    /* WCBS_VAFS.WCBS_DTCFailure */
    DIAG_SIG(SIG_DIAG_BRAKE_CONTROLLER_DIAGNOSTIC_STORAGE_FAILURE, 41U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_DoorFLStatus_Signals[] = {
    /* DoorFLStatus.BDLFLResponseError */
    DIAG_SIG(SIG_DIAG_FRONT_LEFT_DOOR_MODULE_RESPONSE_FAILURE, 32U, 1U, 0U, CAN_MOTOROLA),
    /* DoorFLStatus.NDoorFLOutsideHandleFailure */
    DIAG_SIG(SIG_DIAG_FRONT_LEFT_DOOR_OUTER_HANDLE_FAILURE, 26U, 3U, 0U, CAN_MOTOROLA),
    /* DoorFLStatus.NDoorLatchFLFailure */
    DIAG_SIG(SIG_DIAG_FRONT_LEFT_DOOR_LATCH_FAILURE, 55U, 6U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_DoorFRStatus_Signals[] = {
    /* DoorFRStatus.BDLFRResponseError */
    DIAG_SIG(SIG_DIAG_FRONT_RIGHT_DOOR_MODULE_RESPONSE_FAILURE, 32U, 1U, 0U, CAN_MOTOROLA),
    /* DoorFRStatus.NDoorFROutsideHandleFailure */
    DIAG_SIG(SIG_DIAG_FRONT_RIGHT_DOOR_OUTER_HANDLE_FAILURE, 26U, 3U, 0U, CAN_MOTOROLA),
    /* DoorFRStatus.NDoorLatchFRFailure */
    DIAG_SIG(SIG_DIAG_FRONT_RIGHT_DOOR_LATCH_FAILURE, 55U, 6U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_WCBS_Info_Signals[] = {
    /* WCBS_Info.WCBS_Fault_Status */
    DIAG_SIG(SIG_DIAG_BRAKE_CONTROLLER_FAILURE, 14U, 3U, 0U, CAN_MOTOROLA),
    /* WCBS_Info.WCBS_IBS_Sys_Fault_Sts */
    DIAG_SIG(SIG_DIAG_INTEGRATED_BRAKE_SYSTEM_FAILURE, 15U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_APM_WINDOW_POS_FL_Signals[] = {
    /* APM_WINDOW_POS_FL.Window_Warning */
    DIAG_SIG(SIG_DIAG_FRONT_LEFT_WINDOW_WARNING, 17U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_APM_WINDOW_POS_FR_Signals[] = {
    /* APM_WINDOW_POS_FR.Window_Warning */
    DIAG_SIG(SIG_DIAG_FRONT_RIGHT_WINDOW_WARNING, 17U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_APM_WINDOW_POS_RE_Signals[] = {
    /* APM_WINDOW_POS_RE.Window_Warning */
    DIAG_SIG(SIG_DIAG_REAR_WINDOW_WARNING, 17U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_SeatMassage_Output_Signals[] = {
    /* SeatMassage_Output.LSSM_APumpFail */
    DIAG_SIG(SIG_DIAG_SEAT_MASSAGE_PUMP_A_FAILURE, 35U, 2U, 0U, CAN_MOTOROLA),
    /* SeatMassage_Output.LSSM_BPumpFail */
    DIAG_SIG(SIG_DIAG_SEAT_MASSAGE_PUMP_B_FAILURE, 43U, 2U, 0U, CAN_MOTOROLA),
    /* SeatMassage_Output.LSSM_CPumpFail */
    DIAG_SIG(SIG_DIAG_SEAT_MASSAGE_PUMP_C_FAILURE, 51U, 2U, 0U, CAN_MOTOROLA),
    /* SeatMassage_Output.LSSM_DPumpFail */
    DIAG_SIG(SIG_DIAG_SEAT_MASSAGE_PUMP_D_FAILURE, 59U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_MHU_RAEB_Signals[] = {
    /* MHU_RAEB.MHU_RAEB_Warning */
    DIAG_SIG(SIG_DIAG_REAR_AUTONOMOUS_EMERGENCY_BRAKING_WARNING, 29U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_MHU_FCAM_req_Signals[] = {
    /* MHU_FCAM_req.MHU_LA_AudWarning */
    DIAG_SIG(SIG_DIAG_LANE_ASSIST_AUDIBLE_WARNING, 15U, 2U, 0U, CAN_MOTOROLA),
    /* MHU_FCAM_req.MHU_BSD_AudWarning */
    DIAG_SIG(SIG_DIAG_BLIND_SPOT_DETECTION_AUDIBLE_WARNING, 21U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_WCBS_RecuBrkTrqReq_Signals[] = {
    /* WCBS_RecuBrkTrqReq.WCBS_RegenMonFail */
    DIAG_SIG(SIG_DIAG_REGENERATIVE_BRAKE_MONITOR_FAILURE, 13U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_CVC_WMCID_Sts_Signals[] = {
    /* CVC_WMCID_Sts.CVC_WMCID_IVITheftWarning */
    DIAG_SIG(SIG_DIAG_INFOTAINMENT_THEFT_WARNING, 0U, 1U, 0U, CAN_MOTOROLA),
    /* CVC_WMCID_Sts.CVC_WMCID_CAppTheftWarning */
    DIAG_SIG(SIG_DIAG_CONNECTED_APP_THEFT_WARNING, 1U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_EVCC_Charging_Sts_Signals[] = {
    /* EVCC_Charging_Sts.EVCC_VehChgCompError_Sts */
    DIAG_SIG(SIG_DIAG_VEHICLE_CHARGING_COMPATIBILITY_ERROR, 34U, 3U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_MFS_Control_Button_Signals[] = {
    /* MFS_Control_Button.MFS_L_Failure */
    DIAG_SIG(SIG_DIAG_MULTI_FUNCTION_SWITCH_LEFT_FAILURE, 17U, 3U, 0U, CAN_MOTOROLA),
    /* MFS_Control_Button.MFS_LIN_error */
    DIAG_SIG(SIG_DIAG_MULTI_FUNCTION_SWITCH_LIN_COMMUNICATION_FAILURE, 51U, 3U, 0U, CAN_MOTOROLA),
    /* MFS_Control_Button.MFS_R_Failure */
    DIAG_SIG(SIG_DIAG_MULTI_FUNCTION_SWITCH_RIGHT_FAILURE, 48U, 3U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_XGW_FOTA_StsConfirm_Signals[] = {
    /* XGW_FOTA_StsConfirm.XGW_FOTA_Error_Code */
    DIAG_SIG(SIG_DIAG_GATEWAY_FOTA_ERROR, 23U, 8U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_MHU_FOTA_STATUS_Signals[] = {
    /* MHU_FOTA_STATUS.MHU_FOTA_Error_Code */
    DIAG_SIG(SIG_DIAG_HEAD_UNIT_FOTA_ERROR, 23U, 8U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_WLC_Status_Signals[] = {
    /* WLC_Status.WLC_IC_Error */
    DIAG_SIG(SIG_DIAG_WIRELESS_CHARGER_IC_FAILURE, 0U, 1U, 0U, CAN_MOTOROLA),
    /* WLC_Status.WLC_FAN_Error */
    DIAG_SIG(SIG_DIAG_WIRELESS_CHARGER_FAN_FAILURE, 1U, 1U, 0U, CAN_MOTOROLA),
    /* WLC_Status.WLC_TempSNSR_Error */
    DIAG_SIG(SIG_DIAG_WIRELESS_CHARGER_TEMPERATURE_SENSOR_FAILURE, 2U, 1U, 0U, CAN_MOTOROLA),
    /* WLC_Status.WLC_VehSpeed_Error */
    DIAG_SIG(SIG_DIAG_WIRELESS_CHARGER_VEHICLE_SPEED_ERROR, 11U, 1U, 0U, CAN_MOTOROLA),
    /* WLC_Status.WLC_Resp_Error */
    DIAG_SIG(SIG_DIAG_WIRELESS_CHARGER_RESPONSE_FAILURE, 12U, 1U, 0U, CAN_MOTOROLA),
    /* WLC_Status.WLC_FOD_Error */
    DIAG_SIG(SIG_DIAG_WIRELESS_CHARGER_FOREIGN_OBJECT_DETECTION_ERROR, 13U, 1U, 0U, CAN_MOTOROLA),
    /* WLC_Status.WLC_Curr_Error */
    DIAG_SIG(SIG_DIAG_WIRELESS_CHARGER_CURRENT_ERROR, 14U, 1U, 0U, CAN_MOTOROLA),
    /* WLC_Status.WLC_Temp_Error */
    DIAG_SIG(SIG_DIAG_WIRELESS_CHARGER_TEMPERATURE_ERROR, 15U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_MHU_SVM_DMS_STS_Signals[] = {
    /* MHU_SVM_DMS_STS.MHU_SVM_FCamera_Fault */
    DIAG_SIG(SIG_DIAG_SURROUND_VIEW_FRONT_CAMERA_FAILURE, 13U, 2U, 0U, CAN_MOTOROLA),
    /* MHU_SVM_DMS_STS.MHU_SVM_LCamera_Fault */
    DIAG_SIG(SIG_DIAG_SURROUND_VIEW_LEFT_CAMERA_FAILURE, 15U, 2U, 0U, CAN_MOTOROLA),
    /* MHU_SVM_DMS_STS.MHU_SVM_RCamera_Fault */
    DIAG_SIG(SIG_DIAG_SURROUND_VIEW_REAR_CAMERA_FAILURE, 17U, 2U, 0U, CAN_MOTOROLA),
    /* MHU_SVM_DMS_STS.MHU_SVM_RightCamera_Fault */
    DIAG_SIG(SIG_DIAG_SURROUND_VIEW_RIGHT_CAMERA_FAILURE, 19U, 2U, 0U, CAN_MOTOROLA),
    /* MHU_SVM_DMS_STS.MHU_SETTING_DMSWarning */
    DIAG_SIG(SIG_DIAG_DRIVER_MONITORING_WARNING, 25U, 2U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_XGW_WarnMsg_Signals[] = {
    /* XGW_WarnMsg.XGW_AutoHLLevelingFault */
    DIAG_SIG(SIG_DIAG_AUTO_HEADLAMP_LEVELING_FAILURE, 12U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_BMS_WarnMsg_Signals[] = {
    /* BMS_WarnMsg.BMS_Warning_PowerDegrade */
    DIAG_SIG(SIG_DIAG_BMS_POWER_DERATE_WARNING, 16U, 1U, 0U, CAN_MOTOROLA),
    /* BMS_WarnMsg.BMS_Warning_DeltaSoC */
    DIAG_SIG(SIG_DIAG_BMS_CELL_SOC_IMBALANCE_WARNING, 17U, 1U, 0U, CAN_MOTOROLA),
    /* BMS_WarnMsg.BMS_Warning_HVILSts */
    DIAG_SIG(SIG_DIAG_BMS_HVIL_WARNING, 18U, 1U, 0U, CAN_MOTOROLA),
    /* BMS_WarnMsg.BMS_Warning_Isolation */
    DIAG_SIG(SIG_DIAG_BMS_ISOLATION_WARNING, 19U, 1U, 0U, CAN_MOTOROLA),
    /* BMS_WarnMsg.BMS_Warning_CoolantLeakage */
    DIAG_SIG(SIG_DIAG_BMS_COOLANT_LEAKAGE_WARNING, 20U, 1U, 0U, CAN_MOTOROLA),
    /* BMS_WarnMsg.BMS_Warning_DeltaTemperature */
    DIAG_SIG(SIG_DIAG_BMS_CELL_TEMPERATURE_IMBALANCE_WARNING, 31U, 8U, 0U, CAN_MOTOROLA),
    /* BMS_WarnMsg.BMS_Warning_BDUTemperatureSts */
    DIAG_SIG(SIG_DIAG_BMS_BDU_TEMPERATURE_WARNING, 39U, 8U, 0U, CAN_MOTOROLA),
    /* BMS_WarnMsg.BMS_Warning_Contactor_Aging */
    DIAG_SIG(SIG_DIAG_BMS_CONTACTOR_AGING_WARNING, 47U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_RCU_WarnMsg_Signals[] = {
    /* RCU_WarnMsg.RCU_EHandBrk_fault */
    DIAG_SIG(SIG_DIAG_ELECTRONIC_PARKING_BRAKE_FAILURE, 13U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagSignalDef_t kDiag_CVC_WarnMsg_Signals[] = {
    /* CVC_WarnMsg.CVC_12v_Warning */
    DIAG_SIG(SIG_DIAG_TWELVE_VOLT_SYSTEM_WARNING, 29U, 1U, 0U, CAN_MOTOROLA),
};

static const CAN_DiagMessageDef_t kVinfast7NpDiagMessages[] = {
    { .id = 0x080U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_FCAM_TSR_STATUS_Signals), .signals = kDiag_FCAM_TSR_STATUS_Signals },
    { .id = 0x0C9U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_FMCU_Status_Signals), .signals = kDiag_FMCU_Status_Signals },
    { .id = 0x102U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_CVC_LOGICAL_CLAMP_Signals), .signals = kDiag_CVC_LOGICAL_CLAMP_Signals },
    { .id = 0x104U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_FD_ONLY, .port = CAN_MSG_PORT_2, .expected_len = 12U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_BCM_FL_Measured_Data_Signals), .signals = kDiag_BCM_FL_Measured_Data_Signals },
    { .id = 0x108U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_SS_DriveReq_0x108_Signals), .signals = kDiag_SS_DriveReq_0x108_Signals },
    { .id = 0x10BU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_TRM_LIGHT_Signals), .signals = kDiag_TRM_LIGHT_Signals },
    { .id = 0x114U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_BCM_LVBAT_Data_Signals), .signals = kDiag_BCM_LVBAT_Data_Signals },
    { .id = 0x127U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_FCAM_LDW_LKA_ELK_Signals), .signals = kDiag_FCAM_LDW_LKA_ELK_Signals },
    { .id = 0x132U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_WCBS_ESC_Status_Signals), .signals = kDiag_WCBS_ESC_Status_Signals },
    { .id = 0x17EU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_EPS_SAS_Sensor_Signals), .signals = kDiag_EPS_SAS_Sensor_Signals },
    { .id = 0x1F2U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_MHU_ADSA02_Signals), .signals = kDiag_MHU_ADSA02_Signals },
    { .id = 0x203U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_FD_ONLY, .port = CAN_MSG_PORT_2, .expected_len = 16U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_FCAM_Display_Info_Signals), .signals = kDiag_FCAM_Display_Info_Signals },
    { .id = 0x207U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_BCM_SunroofSts_Signals), .signals = kDiag_BCM_SunroofSts_Signals },
    { .id = 0x208U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_BCM_SunshadeSts_Signals), .signals = kDiag_BCM_SunshadeSts_Signals },
    { .id = 0x20BU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 2U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_BCM_FKS_STATUS_Signals), .signals = kDiag_BCM_FKS_STATUS_Signals },
    { .id = 0x20DU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_WCBS_STATUS_Signals), .signals = kDiag_WCBS_STATUS_Signals },
    { .id = 0x21CU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_FD_ONLY, .port = CAN_MSG_PORT_2, .expected_len = 12U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_FCAM_AEB_FCW_Signals), .signals = kDiag_FCAM_AEB_FCW_Signals },
    { .id = 0x230U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_FD_ONLY, .port = CAN_MSG_PORT_2, .expected_len = 12U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_WCBS_VAFS_Signals), .signals = kDiag_WCBS_VAFS_Signals },
    { .id = 0x231U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_DoorFLStatus_Signals), .signals = kDiag_DoorFLStatus_Signals },
    { .id = 0x248U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_DoorFRStatus_Signals), .signals = kDiag_DoorFRStatus_Signals },
    { .id = 0x270U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_FD_ONLY, .port = CAN_MSG_PORT_2, .expected_len = 12U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_WCBS_Info_Signals), .signals = kDiag_WCBS_Info_Signals },
    { .id = 0x28CU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_APM_WINDOW_POS_FL_Signals), .signals = kDiag_APM_WINDOW_POS_FL_Signals },
    { .id = 0x28DU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_APM_WINDOW_POS_FR_Signals), .signals = kDiag_APM_WINDOW_POS_FR_Signals },
    { .id = 0x28EU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_APM_WINDOW_POS_RE_Signals), .signals = kDiag_APM_WINDOW_POS_RE_Signals },
    { .id = 0x2A2U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_SeatMassage_Output_Signals), .signals = kDiag_SeatMassage_Output_Signals },
    { .id = 0x314U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_MHU_RAEB_Signals), .signals = kDiag_MHU_RAEB_Signals },
    { .id = 0x354U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_FD_ONLY, .port = CAN_MSG_PORT_2, .expected_len = 12U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_MHU_FCAM_req_Signals), .signals = kDiag_MHU_FCAM_req_Signals },
    { .id = 0x362U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_FD_ONLY, .port = CAN_MSG_PORT_2, .expected_len = 16U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_WCBS_RecuBrkTrqReq_Signals), .signals = kDiag_WCBS_RecuBrkTrqReq_Signals },
    { .id = 0x373U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_CVC_WMCID_Sts_Signals), .signals = kDiag_CVC_WMCID_Sts_Signals },
    { .id = 0x37CU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_EVCC_Charging_Sts_Signals), .signals = kDiag_EVCC_Charging_Sts_Signals },
    { .id = 0x3F7U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_FD_ONLY, .port = CAN_MSG_PORT_2, .expected_len = 12U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_MFS_Control_Button_Signals), .signals = kDiag_MFS_Control_Button_Signals },
    { .id = 0x3FCU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 4U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_XGW_FOTA_StsConfirm_Signals), .signals = kDiag_XGW_FOTA_StsConfirm_Signals },
    { .id = 0x432U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_MHU_FOTA_STATUS_Signals), .signals = kDiag_MHU_FOTA_STATUS_Signals },
    { .id = 0x442U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 2U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_WLC_Status_Signals), .signals = kDiag_WLC_Status_Signals },
    { .id = 0x45CU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_MHU_SVM_DMS_STS_Signals), .signals = kDiag_MHU_SVM_DMS_STS_Signals },
    { .id = 0x482U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_XGW_WarnMsg_Signals), .signals = kDiag_XGW_WarnMsg_Signals },
    { .id = 0x493U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_BMS_WarnMsg_Signals), .signals = kDiag_BMS_WarnMsg_Signals },
    { .id = 0x4A7U, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_ANY, .port = CAN_MSG_PORT_2, .expected_len = 8U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_RCU_WarnMsg_Signals), .signals = kDiag_RCU_WarnMsg_Signals },
    { .id = 0x4ACU, .id_type = CAN_ID_STANDARD, .frame_policy = CAN_FRAME_POLICY_FD_ONLY, .port = CAN_MSG_PORT_2, .expected_len = 12U, .signal_count = (uint8_t)DIAG_ARRAY_LEN(kDiag_CVC_WarnMsg_Signals), .signals = kDiag_CVC_WarnMsg_Signals },
};

static const CAN_DiagProfile_t kVinfast7NpDiagProfile = {
    .messages = kVinfast7NpDiagMessages,
    .message_count = (uint16_t)DIAG_ARRAY_LEN(kVinfast7NpDiagMessages),
};

/*
 * 7NP currently reuses the standard actuator calibration set while using
 * a dedicated CAN definition parsed from Tim_kiem_Signal_PCAN search data.
 * CAN2 diag profile is normalized from Phan_tich_ICAN_Message_Loi.csv.
 */
Car_Define_Typedef Vinfast_7NP = {
    .TypeCar = VF_7NP,

    .GearCmd = {
        .PossHome12 = 1350,
        .PossHome34 = 1100,
        .Poss = {1990, 680, 2150, 650},
        .Gear_Function_Typedef = 0x01,
        .GearOrder = 0x1B,
        .TimDelay = 180,
        .Tim = &htim3},

    .KeyCmd = {
        .Tim = &htim3,
        .Tim_Press = 120,
        .KeyLock_Poss = 980,
        .KeyUnlock_Poss = 1950,
        .KeyHome_Poss = 1400,
    },

    .BreakCmd = { .BreakPoss = 60.0, .BreakAcc = 300.0, .BreakVel = 20.0, .BreakJerk = 450.0 },
    .AccelCmd = { .AccelPoss = 100.0, .AccelAcc = 500.0, .AccelVel = 20.0, .AccelJerk = 750.0 },

    .Break_Action = Break_Step_Reset,
    .Accel_Action = Accel_Step_Reset,
    .Gear_Action  = Gear_Step_Home,
    .Key_Action   = Key_Home,
    .LV_Action    = LV_Step_Reset,
    .Robot_Action = Robot_Stop,

    .Break_ReqCmd = FC_Break_Home,
    .Accel_ReqCmd = FC_Accel_Home,
    .Gear_ReqCmd  = FC_Gear_Home,
    .LV_ReqCmd    = FC_LV_Reset,
    .Robot_ReqCmd = FC_Robot_Stop,
    .Key_ReqCmd   = FC_Key_Home,

    .Defined_CAN_Msg = {
        /* ================= BRAKE ================= */
        {
            .name = Brake_Msg, .id = 0x0D9, .port = CAN_MSG_PORT_1, .dlc = 8, .sig_num = 2,
            .Signal = {
                { .name = SIG_GEAR_VALID, .factor = 1.0f, .offset = 0.0f, .start_bit = 35, .length = 1, .is_signed = 0, .endian = CAN_MOTOROLA },
                { .name = SIG_BRAKE_LIGHT_REQ, .factor = 1.0f, .offset = 0.0f, .start_bit = 45, .length = 2, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        },
        {
            .name = Brake_Msg, .id = 0x166, .port = CAN_MSG_PORT_1, .dlc = 8, .sig_num = 1,
            .Signal = {
                { .name = SIG_BRAKE_SWITCH_STS, .factor = 1.0f, .offset = 0.0f, .start_bit = 29, .length = 2, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        },

        /* ================= LV ================= */
        {
            .name = LV_Msg, .id = 0x10A, .port = CAN_MSG_PORT_1, .dlc = 8, .sig_num = 4,
            .Signal = {
                { .name = SIG_LV_BATT_TEMP, .factor = 1.0f, .offset = -40.0f, .start_bit = 39, .length = 8, .is_signed = 0, .endian = CAN_MOTOROLA },
                { .name = SIG_LV_BATT_CURRENT, .factor = 0.03125f, .offset = -1536.0f, .start_bit = 55, .length = 16, .is_signed = 0, .endian = CAN_MOTOROLA },
                { .name = SIG_LV_BATT_TYPE, .factor = 1.0f, .offset = 0.0f, .start_bit = 13, .length = 2, .is_signed = 0, .endian = CAN_MOTOROLA },
                { .name = SIG_LV_SOC, .factor = 1.0f, .offset = 0.0f, .start_bit = 47, .length = 8, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        },

        /* ================= HV / CHARGE ================= */
        {
            .name = HV_Msg, .id = 0x0AA, .port = CAN_MSG_PORT_1, .dlc = 8, .sig_num = 1,
            .Signal = {
                { .name = SIG_HV_ONOFF_REQ, .factor = 1.0f, .offset = 0.0f, .start_bit = 19, .length = 2, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        },
        {
            .name = HV_Msg, .id = 0x215, .port = CAN_MSG_PORT_1, .dlc = 8, .sig_num = 1,
            .Signal = {
                { .name = SIG_HV_STS, .factor = 1.0f, .offset = 0.0f, .start_bit = 63, .length = 8, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        },
        {
            .name = Charge_Msg, .id = 0x216, .port = CAN_MSG_PORT_1, .dlc = 8, .sig_num = 1,
            .Signal = {
                { .name = SIG_BMS_REMAIN_CHARGE_TIME, .factor = 1.0f, .offset = 0.0f, .start_bit = 21, .length = 14, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        },
        {
            .name = Charge_Msg, .id = 0x251, .port = CAN_MSG_PORT_1, .frame_policy = CAN_FRAME_POLICY_FD_ONLY, .dlc = 12, .sig_num = 1,
            .Signal = {
                { .name = SIG_EVCC_AC_CHG_L1_TERMINAL_TEMP, .factor = 1.0f, .offset = -40.0f, .start_bit = 55, .length = 8, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        },

        /* ================= MOTOR ================= */
        {
            .name = Motor_Msg, .id = 0x0AB, .port = CAN_MSG_PORT_1, .dlc = 8, .sig_num = 1,
            .Signal = {
                { .name = SIG_FMCU_TARGET_TQ_REQ, .factor = 0.1f, .offset = -400.0f, .start_bit = 23, .length = 16, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        },
        {
            .name = Motor_Msg, .id = 0x0C9, .port = CAN_MSG_PORT_1, .dlc = 8, .sig_num = 1,
            .Signal = {
                { .name = SIG_FMCU_MOTOR_TQ_VALID, .factor = 1.0f, .offset = 0.0f, .start_bit = 55, .length = 1, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        },
        {
            .name = Motor_Msg, .id = 0x170, .port = CAN_MSG_PORT_1, .dlc = 8, .sig_num = 1,
            .Signal = {
                { .name = SIG_FMCU_MAX_SPEED_ALLOW, .factor = 1.0f, .offset = -17000.0f, .start_bit = 23, .length = 16, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        },

        /* ================= DCDC ================= */
        {
            .name = DCDC_Msg, .id = 0x234, .port = CAN_MSG_PORT_1, .dlc = 8, .sig_num = 1,
            .Signal = {
                { .name = SIG_DCDC_REQUEST, .factor = 1.0f, .offset = 0.0f, .start_bit = 14, .length = 2, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        },
        {
            .name = DCDC_Msg, .id = 0x296, .port = CAN_MSG_PORT_1, .dlc = 8, .sig_num = 2,
            .Signal = {
                { .name = SIG_DCDC_CURRENT, .factor = 0.1f, .offset = 0.0f, .start_bit = 43, .length = 12, .is_signed = 0, .endian = CAN_MOTOROLA },
                { .name = SIG_DCDC_VOLTAGE, .factor = 0.1f, .offset = 0.0f, .start_bit = 63, .length = 8, .is_signed = 0, .endian = CAN_MOTOROLA }
            }
        }
    },
    .SequenceTable = std_sequence,
    .SequenceSize = 13,
    .DiagProfile = &kVinfast7NpDiagProfile,
    .GearLayout = GEAR_LAYOUT_PR_ND
};
