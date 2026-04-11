#ifndef HARDWARE_CONTROL_H
#define HARDWARE_CONTROL_H

#include "main.h"
#include "Motor_Control.h"

#define AccLimit    !HAL_GPIO_ReadPin(CT_ACC_GPIO_Port, CT_ACC_Pin)
#define BreakLimit  !HAL_GPIO_ReadPin(CT_Break_GPIO_Port, CT_Break_Pin)

uint32_t Convert_Poss_to_Step(float poss);
float Convert_Acc_to_Step(float acc);
float Convert_Vel_to_Step(float vel);

void MotorRun(StepObject* motor, uint32_t steps, float acc, float vel, float jerk, MotorDirection_e dir);
void MotorStop(StepObject* motor);

// --- Enums migrated from Car.h for independent use ---

typedef enum {
    Break_Step_Hold = 0, 
    Break_Step_Release, 
    Break_Step_Home, 
    Break_Step_Press_Poss,
    Break_Step_Press_Percent,
    Break_Step_DelayTime,
    Break_Step_IDLE, 
    Break_Step_Reset
} Break_Action_Typedef;

typedef enum {
    FC_Break_Step_IDLE = 0,
    FC_Press_Break_Hold = 1,
    FC_Release_Break,
    FC_Break_Home,
    FC_Trigger_Break,
    FC_Press_Break_Poss,
    FC_Press_Break_Percent,
    FC_Calib_Break_Triger,
    FC_Calib_Break_Percent,
    FC_Break_Reset
} Break_CMD_Typedef;

typedef enum {
    Gear_Step_Reset = 0,
    Gear_Step_Home,
    Gear_Step_Press,
    Gear_Step_IDLE,
    Gear_Calib_Step_IDLE,
    Gear_Calib_Step_Update_Value,
    Gear_Calib_Step_Press_Pcs,
    Gear_Calib_Step_Pass_Poss
} Gear_Action_Typedef;

typedef enum {
    Calib_1 = 1,
    Calib_2,
    Calib_3,
    Calib_4
} Calib_Gear_t;

typedef enum {
    FC_Gear_P = 0,
    FC_Gear_R,
    FC_Gear_N,
    FC_Gear_D,
    FC_Gear_Home
} Gear_CMD_Typedef;

typedef enum {
    GEAR_LAYOUT_PR_ND = 0,
    GEAR_LAYOUT_P_RND,
    GEAR_LAYOUT_RND_P
} Gear_Layout_Typedef;

typedef enum {
    Key_Press_UNLOCK = 0,
    Key_Press_LOCK,
    Key_Home_2,
    Key_ACTIVATE,
    Key_Home
} Key_Action_Typedef;

typedef enum {
    FC_Key_Unlock = 0,
    FC_Key_Lock,
    FC_Key_Home
} Key_CMD_Typedef;

typedef enum {
    FC_Accel_IDLE = 0,
    FC_Accel_Home,
    FC_Accel_Reset,
    FC_Accel_Press_Poss,
    FC_Accel_Press_Percent,
    FC_Accel_Press_Hold,
    FC_Accel_Release
} Accel_CMD_Typedef;

typedef enum {
    Accel_Step_Hold = 0, 
    Accel_Step_Release, 
    Accel_Step_Home, 
    Accel_Step_Press_Poss,
    Accel_Step_Press_Percent,
    Accel_Step_DelayTime,
    Accel_Step_IDLE, 
    Accel_Step_Reset
} Accel_Action_Typedef;

typedef enum {
    FC_LV_Discharge = 0,
    FC_LC_Measure, 
    FC_LV_Charge,
    FC_LV_Reset
} LV_CMD_Typedef;

typedef enum {
    LV_Step_Reset = 0,
    LV_Step_Discharge,
    LV_Step_Charge
} LV_Action_Typedef;

typedef enum {
    Robot_Move_Forward = 0,
    Robot_Move_Backward,
    Robot_Turn_Left,
    Robot_Turn_Right,
    Robot_Stop
} Robot_Action_Typedef;

typedef enum {
    FC_ROBOT_COORD = 0, 
    FC_ROBOT_MOVE,
    FC_Robot_Stop, 
    FC_ROBOT_RESET
} Robot_CMD_Typedef;

// --- Simplified API ---

// Setup functions
void Brake_SetHardware(StepObject* motor, float pos, float acc, float vel, float jerk);
void Brake_UpdateHardwareProfile(float pos, float acc, float vel, float jerk);
void Brake_UpdateHardwareProfileSteps(uint32_t steps, float acc, float vel, float jerk);
void Brake_RearmCommandState(void);
void Accel_SetHardware(StepObject* motor, float pos, float acc, float vel, float jerk);
void Gear_SetHardware(TIM_HandleTypeDef* htim, uint16_t pos_p, uint16_t pos_r, uint16_t pos_n, uint16_t pos_d, uint16_t home12, uint16_t home34, uint8_t s_p, uint8_t s_r, uint8_t s_n, uint8_t s_d);
void Key_SetHardware(TIM_HandleTypeDef* htim, uint16_t unlock, uint16_t lock, uint16_t home, uint8_t time_press);

// Drive functions
void Brake_Drive(Break_CMD_Typedef cmd);
void Accel_Drive(Accel_CMD_Typedef cmd);
void Gear_Drive(Gear_CMD_Typedef cmd);
void Key_Drive(Key_CMD_Typedef cmd);
void LV_Drive(LV_CMD_Typedef cmd);
void Robot_Drive(Robot_CMD_Typedef cmd);
uint8_t Brake_GetStep(void);
uint8_t Brake_IsHomeLimitActive(void);
uint8_t Brake_HasHomeFailure(void);
uint8_t Brake_ServiceHardLimit(void);

#endif // HARDWARE_CONTROL_H
