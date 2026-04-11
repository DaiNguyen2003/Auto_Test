#include "Car.h"

Car_Define_Typedef Vinfast_Limo = {
    .TypeCar = VF_Limo,

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
    
    .BreakCmd = {
        .BreakPoss = 60.0,
        .BreakAcc = 300.0,
        .BreakVel = 20.0,
        .BreakJerk = 450.0,
    },

    .AccelCmd = {
        .AccelPoss = 100.0,
        .AccelAcc = 500.0,
        .AccelVel = 20.0,
        .AccelJerk = 750.0,
    },

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
	        .name = Brake_Msg, .id = 0x109, .dlc = 8, .sig_num = 1,
	        .Signal = {
	            { .name = SIG_BRAKE_SWITCH_STS, .factor = 1.0f, .offset = 0.0f, .start_bit = 12, .length = 2, .is_signed = 0, .endian = CAN_INTEL }
	        }
	    },
	    {
	        .name = Brake_Msg, .id = 0x403, .dlc = 8, .sig_num = 1,
	        .Signal = {
	            { .name = SIG_BRAKE_REGEN_MODE, .factor = 1.0f, .offset = 0.0f, .start_bit = 22, .length = 2, .is_signed = 0, .endian = CAN_INTEL }
	        }
	    },
	    {
	        .name = Brake_Msg, .id = 0x23C, .dlc = 8, .sig_num = 1,
	        .Signal = {
	            { .name = SIG_BRAKE_PEDAL_VAL, .factor = 0.1f, .offset = 0.0f, .start_bit = 33, .length = 15, .is_signed = 0, .endian = CAN_INTEL }
	        }
	    },
	    {
	        .name = Brake_Msg, .id = 0x0D9, .dlc = 8, .sig_num = 2,
	        .Signal = {
	            { .name = SIG_BRAKE_LIGHT, .factor = 1.0f, .offset = 0.0f, .start_bit = 44, .length = 2, .is_signed = 0, .endian = CAN_INTEL },
	            { .name = SIG_GEAR_ACTUAL, .factor = 1.0f, .offset = 0.0f, .start_bit = 32, .length = 3, .is_signed = 0, .endian = CAN_INTEL }
	        }
	    },

	    /* ================= KEY ================= */
	    {
	        .name = Key_Msg, .id = 0x12F, .dlc = 8, .sig_num = 1,
	        .Signal = {
	            { .name = SIG_KEY_VALID, .factor = 1.0f, .offset = 0.0f, .start_bit = 16, .length = 4, .is_signed = 0, .endian = CAN_INTEL }
	        }
	    },
	    {
	        .name = Key_Msg, .id = 0x481, .dlc = 8, .sig_num = 1,
	        .Signal = {
	            { .name = SIG_KEY_FOB_LEFT_CAR, .factor = 1.0f, .offset = 0.0f, .start_bit = 40, .length = 1, .is_signed = 0, .endian = CAN_INTEL }
	        }
	    },

	    /* ================= GEAR ================= */
	    {
	        .name = Gear_Msg, .id = 0x108, .dlc = 8, .sig_num = 1,
	        .Signal = {
	            { .name = SIG_GEAR_TARGET, .factor = 1.0f, .offset = 0.0f, .start_bit = 21, .length = 3, .is_signed = 0, .endian = CAN_INTEL }
	        }
	    },

	    /* ================= LV ================= */
	    {
	        .name = LV_Msg, .id = 0x104, .dlc = 8, .sig_num = 3,
	        .Signal = {
	            { .name = SIG_LV_CHARGE_REQ, .factor = 1.0f, .offset = 0.0f, .start_bit = 39, .length = 1, .is_signed =  0, .endian = CAN_INTEL },
	            { .name = SIG_LV_TARGET_CURRENT, .factor = 0.25f, .offset = -128.0f, .start_bit = 46, .length = 10, .is_signed = 1, .endian = CAN_INTEL },
	            { .name = SIG_LV_TARGET_VOLTAGE, .factor = 0.112903f, .offset = 9.0f, .start_bit = 56, .length = 6, .is_signed = 0, .endian = CAN_INTEL }
	        }
	    },

	    /* ================= DCDC ================= */
	    {
	        .name = DCDC_Msg, .id = 0x213, .dlc = 8, .sig_num = 1,
	        .Signal = {
	            { .name = SIG_DCDC_REQUEST, .factor = 1.0f, .offset = 0.0f, .start_bit = 12, .length = 2, .is_signed = 0, .endian = CAN_MOTOROLA }
	        }
	    },
	    {
	        .name = DCDC_Msg, .id = 0x346, .dlc = 8, .sig_num = 2,
	        .Signal = {
	            { .name = SIG_DCDC_CURRENT, .factor = 0.0625f, .offset = 0.0f, .start_bit = 24, .length = 16, .is_signed = 0, .endian = CAN_INTEL },
	            { .name = SIG_DCDC_VOLTAGE, .factor = 0.0625f, .offset = 0.0f, .start_bit = 40, .length = 16, .is_signed = 0, .endian = CAN_INTEL }
	        }
	    }
	},
    .SequenceTable = std_sequence,
    .SequenceSize = 13, // STD_SEQ_SIZE
    .GearLayout = GEAR_LAYOUT_PR_ND
};
