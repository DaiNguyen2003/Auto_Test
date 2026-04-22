#ifndef CAR_TYPES_H
#define CAR_TYPES_H

#include "main.h"
#include "CanBus.h"
#include "tim.h"
#include "Motor_Control.h"
#include "Hardware_Control.h"

// Extern các bộ Timer dùng cho PWM/SCurve
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

#define TIM_CH_GEAR12        TIM_CHANNEL_3
#define TIM_CH_GEAR34        TIM_CHANNEL_4

#define TIM_CH_KEY           TIM_CHANNEL_2

#define TIM_CH_POWER_SWITCH  TIM_CHANNEL_3

#define TIM_CH_STEP_BREAK    TIM_CHANNEL_2
// #define TIM_CH_STEP_BREAK    TIM_CHANNEL_2 // trùng lặp cũ

#define CAR_CMD_LENGTH // 24 -> 12 Reg

/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------     Gear Act Define   -----------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

// Define P -> 00 | R -> 01 | N -> 10 | D -> 11
// Encoding: 00=P, 0    1=R, 10=N, 11=D
// Format: [Pos1][Pos2][Pos3][Pos4]
// Example PRND: P(00) R(01) N(10) D(11) -> 00011011 -> 0x1B

#define GEAR_ORDER_PRND 0x1B // 00 01 10 11 (P R N D)
#define GEAR_ORDER_PRDN 0x1E // 00 01 11 10 (P R D N)
#define GEAR_ORDER_PNRD 0x27 // 00 10 01 11 (P N R D)
#define GEAR_ORDER_PNDR 0x2D // 00 10 11 01 (P N D R)
#define GEAR_ORDER_PDRN 0x36 // 00 11 01 10 (P D R N)
#define GEAR_ORDER_PDNR 0x39 // 00 11 10 01 (P D N R)

#define GEAR_ORDER_RPND 0x4B // 01 00 10 11 (R P N D)
#define GEAR_ORDER_RPDN 0x4E // 01 00 11 10 (R P D N)
#define GEAR_ORDER_RNPD 0x63 // 01 10 00 11 (R N P D)
#define GEAR_ORDER_RNDP 0x6C // 01 10 11 00 (R N D P) 
#define GEAR_ORDER_RDPN 0x72 // 01 11 00 10 (R D P N)
#define GEAR_ORDER_RDNP 0x78 // 01 11 10 00 (R D N P)

#define GEAR_ORDER_NPRD 0x87 // 10 00 01 11 (N P R D)
#define GEAR_ORDER_NPDR 0x8D // 10 00 11 01 (N P D R)
#define GEAR_ORDER_NRPD 0x93 // 10 01 00 11 (N R P D)
#define GEAR_ORDER_NRDP 0x9C // 10 01 11 00 (N R D P)
#define GEAR_ORDER_NDPR 0xB1 // 10 11 00 01 (N D P R)
#define GEAR_ORDER_NDRP 0xB4 // 10 11 01 00 (N D R P)

#define GEAR_ORDER_DPRN 0xC6 // 11 00 01 10 (D P R N)
#define GEAR_ORDER_DPNR 0xC9 // 11 00 10 01 (D P N R)
#define GEAR_ORDER_DRPN 0xD2 // 11 01 00 10 (D R P N)
#define GEAR_ORDER_DRNP 0xD8 // 11 01 10 00 (D R N P)
#define GEAR_ORDER_DNPR 0xE1 // 11 10 00 01 (D N P R)
#define GEAR_ORDER_DNRP 0xE4 // 11 10 01 00 (D N R P)

#define P_Order 0x00U
#define R_Order 0x01U
#define N_Order 0x02U
#define D_Order 0x03U

#define P_Act 0x01
#define R_Act 0x02 
#define N_Act 0x03
#define D_Act 0x04

typedef struct{ // -> 7 REG
    uint16_t PossHome12; // REG 05
    uint16_t PossHome34; // REG 06
    
    uint16_t Poss[4];   // REG 07 -> 10 ||  P -> R -> N -> D

    uint8_t Gear_Function_Typedef;  // REG 011_HIGH
    uint8_t GearOrder;  // REG 011_LOW
    
    uint8_t TimDelay; 
    
    TIM_HandleTypeDef* Tim;
    
    uint8_t Servo_Assign[4]; // 0=Servo1(CH3), 1=Servo2(CH4) for [P, R, N, D]
    
}Car_Gear_Control_CMD_Typedef;

/*=========================================================================================================================================================================||
* ||-----------------------------------------------------------------------  KEY Act Define  ------------------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

typedef struct{  // Unlock_Poss -> 12 Bit (Byte1 << 12) | (Byte2 & 0x0F) && Lock_Poss -> 12 Bit
    uint16_t KeyUnlock_Poss;  // REG 0
    uint16_t KeyLock_Poss;    // REG 1
    uint16_t  KeyHome_Poss;    // REG 2
    uint8_t  Tim_Press;       // REG 3
    uint8_t TypeKey;          // REG 02_HIGH
    uint8_t KeyCtrl;           // REG 02_LOW

    TIM_HandleTypeDef* Tim;
}Car_Key_Control_CMD_Typedef;

/*=========================================================================================================================================================================||
* ||----------------------------------------------------------------------   Break Act Define   ---------------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

#define Break_CW     1
#define Break_CCW    0
#define Break_Stop   2
#define Break_Break  3
#define Break_Home   4
#define Break_Rst    5
#define Break_PID_Poss 6

#define MaxRange_Break 120 // mm
#define MinRange_Break 0   // mm
#define MaxPercent_Break 50 // %
#define MinPercent_Break 0   // %
#define DelayTime_Pressed_Break 700 // Thời gian giữ phanh khi nhấn phanh (ms)

typedef struct{
    float BreakPoss;  // 0 -> 120mm / 0 -> 100%
    float BreakAcc;   // mm/s²
    float BreakVel;   // mm/s
    float BreakJerk;  // mm/s³ (S-Curve)
    uint8_t PedalCmd; // REG 03_LOW -> lệnh điều khiển phanh
 
    TIM_HandleTypeDef* Tim;
}Car_Break_Control_CMD_Typedef;


/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------    Accel Pedal Define         ---------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

typedef struct{
    float AccelPoss;  // 0 -> 120mm / 0 -> 100%
    float AccelAcc;   // mm/s²
    float AccelVel;   // mm/s
    float AccelJerk;  // mm/s³ (S-Curve)
    uint8_t PedalCmd; // REG 03_LOW -> lệnh điều khiển chân ga
 
    TIM_HandleTypeDef* Tim;
}Car_Accel_Control_CMD_Typedef;

/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------    Signal CAN Define typedef  ---------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

typedef enum {
    SIG_BRAKE_SWITCH_STS = 0,
    SIG_BRAKE_REGEN_MODE,
    SIG_BRAKE_PEDAL_VAL,
    SIG_BRAKE_LIGHT,
    SIG_BRAKE_LIGHT_REQ,

    SIG_KEY_VALID,
    SIG_KEY_FOB_LEFT_CAR,

    SIG_GEAR_TARGET,
    SIG_GEAR_ACTUAL,
    SIG_GEAR_VALID,

    SIG_STAT_TERMINAL,

    SIG_LV_CHARGE_REQ,
    SIG_LV_TARGET_CURRENT,
    SIG_LV_TARGET_VOLTAGE,
    SIG_LV_MAX_CHARGE_CURRENT,
    SIG_LV_MAX_DISCHARGE_CURRENT,
    SIG_LV_WAKEUP_REASON,
    SIG_LV_BATT_CURRENT,
    SIG_LV_BATT_TEMP,
    SIG_LV_BATT_TYPE,
    SIG_LV_SOC,

    SIG_HV_ONOFF_STS,
    SIG_HV_ONOFF_REQ,
    SIG_HV_STS,

    SIG_DCDC_REQUEST,
    SIG_DCDC_CURRENT,
    SIG_DCDC_VOLTAGE,

    SIG_FMCU_TARGET_TQ_REQ,
    SIG_FMCU_MOTOR_TQ_VALID,
    SIG_FMCU_MAX_SPEED_ALLOW,
    SIG_BMS_REMAIN_CHARGE_TIME,
    SIG_EVCC_AC_CHG_L1_TERMINAL_TEMP,

    SIG_NUM
} CAN_SignalName_t;

typedef enum{
	Brake_Msg = 0,
	Key_Msg,
	Gear_Msg,
	STAT_Terminal_Msg,
    LV_Msg,
	HV_Msg,
	DCDC_Msg,
    Motor_Msg,
    Charge_Msg,
} CAN_MessageName_t;

typedef enum {
    CAN_INTEL = 0,     
    CAN_MOTOROLA       
} CAN_Endian_t;

typedef enum {
    CAN_ID_STANDARD = FDCAN_STANDARD_ID,
    CAN_ID_EXTENDED = FDCAN_EXTENDED_ID
} CAN_IdType_t;

typedef enum {
    CAN_FRAME_POLICY_ANY = 0,
    CAN_FRAME_POLICY_CLASSIC_ONLY,
    CAN_FRAME_POLICY_FD_ONLY
} CAN_FramePolicy_t;

typedef enum {
    CAN_MSG_PORT_1 = CAN_PORT_1,
    CAN_MSG_PORT_2 = CAN_PORT_2,
    CAN_MSG_PORT_ANY = 2
} CAN_MessagePort_t;

/*
* Struct
*/
typedef struct {
    float       factor;      // scale
    float       offset;      // offset
    uint16_t    signal_start_bit; // (0-511), used by mixed CAN/CAN FD parser

    uint8_t     start_bit;   // (0–63)
    uint8_t     length;      // (1–64)
    uint8_t     is_signed;   // 0: unsigned, 1: signed
    CAN_Endian_t endian;     // Intel / Motorola
    CAN_SignalName_t  name;        // Name in stypedef name
} CAN_Signal_t;

/* Keep existing .start_bit initializers/source code working while expanding to 16-bit. */
#define start_bit signal_start_bit

#define CAN_MAX_SIG_PER_MSG  8
typedef struct {
    uint32_t     id;             // CAN ID Msg
    CAN_IdType_t id_type;        // Standard / Extended ID
    CAN_FramePolicy_t frame_policy; // Accept classic only / FD only / any
    CAN_MessagePort_t port;      // Expected receive port

    CAN_Signal_t Signal[CAN_MAX_SIG_PER_MSG];

    uint8_t      dlc;            // 0–16 -> for extern Messenger
    uint8_t      sig_num;        // số signal trong Msg

    CAN_MessageName_t name; 
} CAN_Message_t;

typedef enum {
    SIG_DIAG_NONE = 0,
    SIG_DIAG_TRAFFIC_SIGN_RECOGNITION_AUDIBLE_WARNING,
    SIG_DIAG_MOTOR_CONTROLLER_MCU_FAILURE,
    SIG_DIAG_POWER_CLAMP_INCOMPATIBILITY_WARNING,
    SIG_DIAG_FRONT_LEFT_BODY_MODULE_RESPONSE_FAILURE,
    SIG_DIAG_ELECTRONIC_BRAKE_SYSTEM_FAILURE,
    SIG_DIAG_SUNSHADE_FAILURE,
    SIG_DIAG_TRAILER_SYSTEM_FAILURE,
    SIG_DIAG_LOW_VOLTAGE_SYSTEM_FAILURE,
    SIG_DIAG_LOW_VOLTAGE_CELL_BALANCE_FAILURE,
    SIG_DIAG_EMERGENCY_LANE_KEEPING_WARNING,
    SIG_DIAG_LANE_DEPARTURE_WARNING,
    SIG_DIAG_HANDS_OFF_WARNING,
    SIG_DIAG_LANE_KEEPING_ASSIST_WARNING,
    SIG_DIAG_ESC_HSA_FAILURE,
    SIG_DIAG_STEERING_ANGLE_SENSOR_FAILURE,
    SIG_DIAG_NAVIGATION_FAILURE,
    SIG_DIAG_SURROUND_CAMERA_SYSTEM_FAILURE,
    SIG_DIAG_MID_RANGE_RADAR_FAILURE,
    SIG_DIAG_SUNROOF_UNDERVOLTAGE_FAULT,
    SIG_DIAG_SUNROOF_OVERVOLTAGE_FAULT,
    SIG_DIAG_SUNROOF_MOTOR_REVERSE_FAULT,
    SIG_DIAG_SUNROOF_HALL_SENSOR_FAULT,
    SIG_DIAG_SUNROOF_RELAY_FAULT,
    SIG_DIAG_SUNROOF_COMMUNICATION_FAULT,
    SIG_DIAG_SUNROOF_THERMAL_PROTECTION_FAULT,
    SIG_DIAG_SUNROOF_RESPONSE_FAILURE,
    SIG_DIAG_SUNSHADE_MOTOR_REVERSE_FAULT,
    SIG_DIAG_SUNSHADE_UNDERVOLTAGE_FAULT,
    SIG_DIAG_SUNSHADE_OVERVOLTAGE_FAULT,
    SIG_DIAG_SUNSHADE_RELAY_FAULT,
    SIG_DIAG_SUNSHADE_HALL_SENSOR_FAULT,
    SIG_DIAG_SUNSHADE_COMMUNICATION_FAULT,
    SIG_DIAG_SUNSHADE_RESPONSE_FAILURE,
    SIG_DIAG_KEYLESS_ENTRY_RESPONSE_FAILURE,
    SIG_DIAG_KEYLESS_ENTRY_HARDWARE_FAILURE,
    SIG_DIAG_KEYLESS_ENTRY_ANTENNA_FAILURE,
    SIG_DIAG_ABS_FAILURE,
    SIG_DIAG_EBD_FAILURE,
    SIG_DIAG_AUTONOMOUS_EMERGENCY_BRAKING_WARNING,
    SIG_DIAG_FORWARD_COLLISION_WARNING,
    SIG_DIAG_ELECTRONIC_PARKING_BRAKE_FAILURE,
    SIG_DIAG_BRAKE_CONTROLLER_ROM_FAILURE,
    SIG_DIAG_TRACTION_STABILITY_MANAGEMENT_FAILURE,
    SIG_DIAG_BRAKE_CONTROLLER_DIAGNOSTIC_STORAGE_FAILURE,
    SIG_DIAG_FRONT_LEFT_DOOR_MODULE_RESPONSE_FAILURE,
    SIG_DIAG_FRONT_LEFT_DOOR_OUTER_HANDLE_FAILURE,
    SIG_DIAG_FRONT_LEFT_DOOR_LATCH_FAILURE,
    SIG_DIAG_FRONT_RIGHT_DOOR_MODULE_RESPONSE_FAILURE,
    SIG_DIAG_FRONT_RIGHT_DOOR_OUTER_HANDLE_FAILURE,
    SIG_DIAG_FRONT_RIGHT_DOOR_LATCH_FAILURE,
    SIG_DIAG_BRAKE_CONTROLLER_FAILURE,
    SIG_DIAG_INTEGRATED_BRAKE_SYSTEM_FAILURE,
    SIG_DIAG_FRONT_LEFT_WINDOW_WARNING,
    SIG_DIAG_FRONT_RIGHT_WINDOW_WARNING,
    SIG_DIAG_REAR_WINDOW_WARNING,
    SIG_DIAG_SEAT_MASSAGE_PUMP_A_FAILURE,
    SIG_DIAG_SEAT_MASSAGE_PUMP_B_FAILURE,
    SIG_DIAG_SEAT_MASSAGE_PUMP_C_FAILURE,
    SIG_DIAG_SEAT_MASSAGE_PUMP_D_FAILURE,
    SIG_DIAG_REAR_AUTONOMOUS_EMERGENCY_BRAKING_WARNING,
    SIG_DIAG_LANE_ASSIST_AUDIBLE_WARNING,
    SIG_DIAG_BLIND_SPOT_DETECTION_AUDIBLE_WARNING,
    SIG_DIAG_REGENERATIVE_BRAKE_MONITOR_FAILURE,
    SIG_DIAG_INFOTAINMENT_THEFT_WARNING,
    SIG_DIAG_CONNECTED_APP_THEFT_WARNING,
    SIG_DIAG_VEHICLE_CHARGING_COMPATIBILITY_ERROR,
    SIG_DIAG_MULTI_FUNCTION_SWITCH_LEFT_FAILURE,
    SIG_DIAG_MULTI_FUNCTION_SWITCH_LIN_COMMUNICATION_FAILURE,
    SIG_DIAG_MULTI_FUNCTION_SWITCH_RIGHT_FAILURE,
    SIG_DIAG_GATEWAY_FOTA_ERROR,
    SIG_DIAG_HEAD_UNIT_FOTA_ERROR,
    SIG_DIAG_WIRELESS_CHARGER_IC_FAILURE,
    SIG_DIAG_WIRELESS_CHARGER_FAN_FAILURE,
    SIG_DIAG_WIRELESS_CHARGER_TEMPERATURE_SENSOR_FAILURE,
    SIG_DIAG_WIRELESS_CHARGER_VEHICLE_SPEED_ERROR,
    SIG_DIAG_WIRELESS_CHARGER_RESPONSE_FAILURE,
    SIG_DIAG_WIRELESS_CHARGER_FOREIGN_OBJECT_DETECTION_ERROR,
    SIG_DIAG_WIRELESS_CHARGER_CURRENT_ERROR,
    SIG_DIAG_WIRELESS_CHARGER_TEMPERATURE_ERROR,
    SIG_DIAG_SURROUND_VIEW_FRONT_CAMERA_FAILURE,
    SIG_DIAG_SURROUND_VIEW_LEFT_CAMERA_FAILURE,
    SIG_DIAG_SURROUND_VIEW_REAR_CAMERA_FAILURE,
    SIG_DIAG_SURROUND_VIEW_RIGHT_CAMERA_FAILURE,
    SIG_DIAG_DRIVER_MONITORING_WARNING,
    SIG_DIAG_AUTO_HEADLAMP_LEVELING_FAILURE,
    SIG_DIAG_BMS_POWER_DERATE_WARNING,
    SIG_DIAG_BMS_CELL_SOC_IMBALANCE_WARNING,
    SIG_DIAG_BMS_HVIL_WARNING,
    SIG_DIAG_BMS_ISOLATION_WARNING,
    SIG_DIAG_BMS_COOLANT_LEAKAGE_WARNING,
    SIG_DIAG_BMS_CELL_TEMPERATURE_IMBALANCE_WARNING,
    SIG_DIAG_BMS_BDU_TEMPERATURE_WARNING,
    SIG_DIAG_BMS_CONTACTOR_AGING_WARNING,
    SIG_DIAG_TWELVE_VOLT_SYSTEM_WARNING,
    SIG_DIAG_NUM
} CAN_DiagSignalName_t;

typedef struct {
    CAN_DiagSignalName_t name;
    float factor;
    float offset;
    uint16_t start_bit;
    uint8_t length;
    uint8_t is_signed;
    CAN_Endian_t endian;
} CAN_DiagSignalDef_t;

typedef struct {
    uint32_t id;
    CAN_IdType_t id_type;
    CAN_FramePolicy_t frame_policy;
    CAN_MessagePort_t port;
    uint8_t expected_len;
    uint8_t signal_count;
    const CAN_DiagSignalDef_t *signals;
} CAN_DiagMessageDef_t;

typedef struct {
    const CAN_DiagMessageDef_t *messages;
    uint16_t message_count;
} CAN_DiagProfile_t;

typedef struct {
    uint8_t seen;
    uint8_t active;
    uint64_t raw_value;
    uint32_t last_update_tick;
} Car_DiagSignalState_t;

typedef struct {
    Car_DiagSignalState_t signal[SIG_DIAG_NUM];
    uint32_t matched_frame_count;
    uint32_t unmatched_frame_count;
    uint32_t parse_error_count;
    uint32_t active_count;
    CAN_DiagSignalName_t last_signal;
    uint64_t last_raw_value;
    uint32_t last_update_tick;
} Car_DiagState_t;

#define CAR_SIGNAL_MONITOR_MAX_ROWS    32U
#define CAR_SIGNAL_MONITOR_NAME_CHARS  40U
#define CAR_SIGNAL_MONITOR_VALUE_SCALE 1000L

typedef struct {
    CAN_SignalName_t signal_name;
    uint8_t defined;
    uint8_t seen;
    uint8_t value_valid;
    int32_t value_milli;
} Car_SignalMonitorRow_t;

typedef struct {
    uint16_t active_car_type;
    uint16_t signal_count;
    uint16_t generation;
    Car_SignalMonitorRow_t rows[CAR_SIGNAL_MONITOR_MAX_ROWS];
} Car_SignalMonitorTable_t;

// Signal trạng thái thực tế của xe
typedef struct {
    float Val_LV_current;
    float Val_LV_Voltage;
    float Val_LV_Batt_Temp;
    float Val_FMCU_TargetTqReq;
    float Val_EVCC_ACChgL1Temp;
    float Val_DCDC_Current;
    float Val_DCDC_Voltage;
    int16_t Val_FMCU_MaxSpeedAllow;
    uint16_t Val_BMS_RemainChargeTime;
    uint16_t Val_Car_Speed;
    uint8_t Val_Gear_Act; 
    uint8_t Val_Gear_Valid;
    uint8_t Val_STAT_Terminal;
    uint8_t Val_Break_Position; 
    uint8_t Val_Break_Pedal_Sts;
    uint8_t Val_Brake_Light_Req;
    uint8_t Val_Acc_Pedal_Sts;
    uint8_t Val_Acc_Pedal_Position;
    uint8_t Val_HV_Sts; 
    uint8_t Val_HV_OnOff_Req;
    uint8_t Val_LV_soc; 
    uint8_t Val_LV_Batt_Type;
    uint8_t Val_HV_soc;
    uint8_t Val_Key_Act;
    uint8_t Val_FMCU_MotorTqValid;
    uint8_t Val_CreepMode_Sts; 
    uint8_t Val_DCDC_Sts; 
} Car_Val_Act_t;

/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------    LV control thật VL  ----------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

typedef struct {
    float LV_TargetCurrent;  
    float LV_TargetVoltage;   
    uint8_t LV_SOC_Req;
    uint8_t LV_SOH_Req; 
    uint8_t LV_SOH_Learn;  
} Car_LV_Control_CMD_Typedef;
/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------    Robot control define ---------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

typedef struct {
    float Robot_Speed;  
    float Robot_Acc;   
    float Robot_Distance;   
    uint8_t Robot_Cmd;
} Car_Robot_Control_CMD_Typedef;


/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------    Virtual Car Define  ----------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

typedef struct{  // 8 byte -> ID default : 0x89 
    uint16_t pos_break;    // 2 byte  ( uint16*0.01 = mm) -> Byte 7 + 8
    uint16_t accel_break;  // 2 byte  ( uint16*0.1 = mm/s²) -> Byte 5 + 6
    uint16_t vel_break;    // 2 byte  ( uint16*0.1 = mm/s) -> Byte 3 + 4
    uint16_t ID; // 11 bit ID 
    Break_CMD_Typedef cmd_break; // 2 Byte ( use 1 byte) -> Byte 2
}Pedal_Virtual_Car_Typedef;

typedef struct{ //h23
    uint16_t angle_lock; 
    uint16_t angle_unlock; 
    Key_CMD_Typedef FCCmd;
}Key_Virtual_Car_Typedef; 

/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------    Car Define          ----------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/
typedef enum{
    VF_89 = 0, 
    VF_67, 
    VF_5, 
    VF_e34, 
    VF_Limo, 
    VF_Mgreen, 
    VF_Truck, 
    VF_Van, 
    VF_2, 
    VF_3, 
    VF_67NP, 
    VF_5NP, 
    VF_8NP, 
    VF_LacHong, 
    VF_VanNP, 
    VF_3BCO3, 
    VF_5BCO3, 
    VF_Virtual   // -> là xe ảo, sử dụng cho việc điều khiển qua PM/ Can OE
}Car_Type;

/* Preserve existing Car_Type numeric values used by Modbus/UI while exposing 7NP. */
#define VF_7NP ((Car_Type)(VF_Virtual + 1))


#define MAXIMUM_MESS_AT_CAR 20
// --- Generic Test Engine Types ---
typedef enum {
    ACT_NONE = 0,
    ACT_KEY_LOCK, ACT_KEY_UNLOCK, ACT_KEY_HOME,
    ACT_BREAK_PRESS, ACT_BREAK_RELEASE, ACT_BREAK_HOLD, ACT_BREAK_TRIGGER,
    ACT_GEAR_P, ACT_GEAR_R, ACT_GEAR_N, ACT_GEAR_D, ACT_GEAR_HOME,
    ACT_LV_RESET, ACT_ROBOT_STOP,
    ACT_SEATBELT_BUCKLE, ACT_SEATBELT_RELEASE,
    ACT_WAIT
} TestAction_t;

typedef enum {
    COND_TIME = 0,
    COND_CAN_VAL,
    COND_CAN_STABLE
} TestCondition_t;

typedef struct {
    TestAction_t action;
    uint32_t target_value;
    TestCondition_t condition;
    uint32_t cond_value;
    uint32_t timeout;
} TestStep_t;

typedef struct{
    // Command Structs
    Car_Gear_Control_CMD_Typedef   GearCmd;
    Car_Break_Control_CMD_Typedef  BreakCmd;
    Car_Key_Control_CMD_Typedef    KeyCmd;
    Car_Accel_Control_CMD_Typedef  AccelCmd;
    Car_LV_Control_CMD_Typedef     LVCmd;
    Car_Robot_Control_CMD_Typedef  RobotCmd;

    // Real-time status
    Car_Val_Act_t        StsCar; 

    // Internal actions/steps (from Hardware_Control.h enums)
    uint8_t Break_Action;   // Break_Step_Typedef
    uint8_t Accel_Action;   // Accel_Step_Typedef
    uint8_t Gear_Action;    // Gear_Step_Typedef
    uint8_t Key_Action;     // Key_Step_Typedef
    uint8_t Robot_Action;   // Robot_Mode_Typedef
    uint8_t LV_Action;      // LV_Step_Typedef

    // External Request commands
    uint8_t Break_ReqCmd;   // Break_CMD_Typedef
    uint8_t Accel_ReqCmd;   // Accel_CMD_Typedef
    uint8_t Gear_ReqCmd;    // Gear_CMD_Typedef
    uint8_t LV_ReqCmd;      // LV_CMD_Typedef
    uint8_t Robot_ReqCmd;   // Robot_CMD_Typedef
    uint8_t Key_ReqCmd;     // Key_CMD_Typedef

    // Car Type and CAN Config
    Car_Type           TypeCar;
    CAN_Message_t      Defined_CAN_Msg[MAXIMUM_MESS_AT_CAR];
    
    uint8_t            Camera_Sts;  
    uint8_t            GearLayout;  // Gear_Layout_Typedef

    // --- Dynamic Test Sequence ---
    TestStep_t*        SequenceTable;
    uint8_t            SequenceSize;
    const CAN_DiagProfile_t *DiagProfile;

}Car_Define_Typedef;

extern StepObject Motor1;

#endif // CAR_TYPES_H
