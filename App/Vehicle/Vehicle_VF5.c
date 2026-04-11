#include "Car.h"

Car_Define_Typedef Vinfast_VF5 = {
    .TypeCar = VF_5,

    .GearCmd = {
        .PossHome12 = 1350,             // REG 05
        .PossHome34 = 1100,             // REG 06
        .Poss = {1990, 680, 2150, 650}, // REG 07 -> 10 (P, R, N, D) - Đoạn D đổi từ 800 xuống 700
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

	.Defined_CAN_Msg = {
	    /* ================= BRAKE ================= */
	    {
	        .name = Brake_Msg, .id = 0x1A9, .dlc = 8, .sig_num = 2,
	        .Signal = {
	            { .name = SIG_BRAKE_SWITCH_STS, .factor = 1.0f, .offset = 0.0f, .start_bit = 63, .length = 2, .is_signed = 0, .endian = CAN_MOTOROLA },
                { .name = SIG_BRAKE_REGEN_MODE, .factor = 1.0f, .offset = 0.0f, .start_bit = 29, .length = 2, .is_signed = 0, .endian = CAN_MOTOROLA }
	        }
	    },
	    {
	        .name = Brake_Msg, .id = 0x379, .dlc = 8, .sig_num = 2,
	        .Signal = {
	            { .name = SIG_BRAKE_PEDAL_VAL, .factor = 0.4f, .offset = 0.0f, .start_bit = 47, .length = 8, .is_signed = 0, .endian = CAN_MOTOROLA },
                { .name = SIG_GEAR_ACTUAL, .factor = 1.0f, .offset = 0.0f, .start_bit = 26, .length = 3, .is_signed = 0, .endian = CAN_MOTOROLA }
	        }
	    },

	    /* ================= KEY ================= */
	    {
	        .name = Key_Msg, .id = 0x11B, .dlc = 8, .sig_num = 1,
	        .Signal = {
	            { .name = SIG_KEY_VALID, .factor = 1.0f, .offset = 0.0f, .start_bit = 15, .length = 2, .is_signed = 0, .endian = CAN_MOTOROLA }
	        }
	    },

	    /* ================= TERMINAL ================= */
	    {
	        .name = STAT_Terminal_Msg, .id = 0x112, .dlc = 8, .sig_num = 1,
	        .Signal = {
	            { .name = SIG_STAT_TERMINAL, .factor = 1.0f, .offset = 0.0f, .start_bit = 34, .length = 3, .is_signed = 0, .endian = CAN_MOTOROLA }
	        }
	    },

	    /* ================= LV ================= */
	    {
	        .name = LV_Msg, .id = 0x218, .dlc = 8, .sig_num = 1,
	        .Signal = {
	            { .name = SIG_LV_CHARGE_REQ, .factor = 1.0f, .offset = 0.0f, .start_bit = 20, .length = 1, .is_signed =  0, .endian = CAN_MOTOROLA }
	        }
	    },
        {
	        .name = LV_Msg, .id = 0x216, .dlc = 8, .sig_num = 2,
	        .Signal = {
	            { .name = SIG_LV_TARGET_CURRENT, .factor = 0.1f, .offset = 0.0f, .start_bit = 55, .length = 16, .is_signed = 0, .endian = CAN_MOTOROLA },
	            { .name = SIG_LV_TARGET_VOLTAGE, .factor = 0.1f, .offset = 0.0f, .start_bit = 37, .length = 14, .is_signed = 0, .endian = CAN_MOTOROLA }
	        }
	    },

	    /* ================= DCDC ================= */
	    {
	        .name = DCDC_Msg, .id = 0x2D5, .dlc = 8, .sig_num = 1,
	        .Signal = {
	            { .name = SIG_DCDC_REQUEST, .factor = 1.0f, .offset = 0.0f, .start_bit = 55, .length = 1, .is_signed = 0, .endian = CAN_MOTOROLA }
	        }
	    },
	    {
	        .name = DCDC_Msg, .id = 0x20B, .dlc = 8, .sig_num = 2,
	        .Signal = {
	            { .name = SIG_DCDC_CURRENT, .factor = 1.0f, .offset = 0.0f, .start_bit = 31, .length = 8, .is_signed = 0, .endian = CAN_MOTOROLA },
	            { .name = SIG_DCDC_VOLTAGE, .factor = 1.0f, .offset = 0.0f, .start_bit = 45, .length = 10, .is_signed = 0, .endian = CAN_MOTOROLA }
	        }
	    }
	},
    .SequenceTable = std_sequence,
    .SequenceSize = 13,
    .GearLayout = GEAR_LAYOUT_PR_ND
};
