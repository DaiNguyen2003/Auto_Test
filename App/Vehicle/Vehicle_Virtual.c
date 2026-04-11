#include "Car.h"

Pedal_Virtual_Car_Typedef Break_Virtual_Car = {
    .pos_break = 0,
    .accel_break = 0,
    .vel_break = 0,
    .ID = 0x89,
    .cmd_break = Break_Home
};

Key_Virtual_Car_Typedef Key_Virtual_Car = {
	.angle_lock = 0,
	.angle_unlock = 0,
	.FCCmd = Key_Home
};

Car_Define_Typedef Vinfast_Virtual = {
    .TypeCar = VF_Virtual,

    .GearCmd = {
        .PossHome12 = 1350,             // REG 05
        .PossHome34 = 1100,             // REG 06
        .Poss = {1860, 900, 1750, 800}, // REG 07 -> 10 (P, R, N, D)
        .Gear_Function_Typedef = 0x01,  // REG 11 HIGH
        .GearOrder = 0x1B,
        .TimDelay = 180,
        .Tim = &htim3},

    .KeyCmd = {
        .Tim = &htim3,
        .Tim_Press = 120,
        .KeyLock_Poss = 980,    // 1130,
        .KeyUnlock_Poss = 1950, // 1800,
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
    .LV_ReqCmd      = FC_LV_Reset,
    .Robot_ReqCmd   = FC_Robot_Stop,
    .Key_ReqCmd     = FC_Key_Home,

    .SequenceTable = std_sequence,
    .SequenceSize = 13, // STD_SEQ_SIZE
    .GearLayout = GEAR_LAYOUT_PR_ND
};
