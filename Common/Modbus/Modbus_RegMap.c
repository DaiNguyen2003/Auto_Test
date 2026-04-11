/**
 * @file    Modbus_RegMap.c
 * @brief   Modbus Register Map Implementation
 * @details Maps Modbus register writes → Hardware_Control commands
 *          Maps Modbus register reads  ← Car status (CAN) + system_debug
 */

#include "Modbus_RegMap.h"
#include "Modbus_Config.h"
#include <string.h>
#include "bsp.h"
#include "Hardware_Control.h"
#include "Car.h"
#include "Test_Manager.h"
#include "common.h"
#include "IO_Control.h"
#include "gpio.h"
#include "CanBus.h"

/* ═══════════════════════════════════════════════════════════════════
 *  GLOBAL: Modbus Register Structure for Debugging
 * ═══════════════════════════════════════════════════════════════════ */
Modbus_Registers_t modbus_regs;
#define cmd_regs modbus_regs.Array
static uint8_t s_brake_cmd_host_latch = FC_Break_Step_IDLE;
static uint8_t s_brake_trigger_rearm_lock = 0U;


/* ═══════════════════════════════════════════════════════════════════
 *  INIT
 * ═══════════════════════════════════════════════════════════════════ */
void ModbusRegMap_Init(void) {
    memset(&modbus_regs, 0, sizeof(modbus_regs));
    memset((void *)&modbus_diag, 0, sizeof(modbus_diag));
    s_brake_cmd_host_latch = FC_Break_Step_IDLE;
    s_brake_trigger_rearm_lock = 0U;
}

/* ═══════════════════════════════════════════════════════════════════
 *  WRITE HANDLER (FC06 / FC10 → Hardware actions)
 * ═══════════════════════════════════════════════════════════════════ */
bool ModbusRegMap_IsWriteAddr(uint16_t addr) {
    return (addr < REG_WRITE_MAX);
}

bool ModbusRegMap_Write(uint16_t addr, uint16_t value) {
    if (!ModbusRegMap_IsWriteAddr(addr)) {
        return false;
    }
    
    /* Store value in local array */
    cmd_regs[addr] = value;
    
    /* Execute action immediately based on register */
    switch (addr) {
        /* --- System Mode --- */
        case REG_CMD_MODE:
            system_debug.debug_mode = (uint8_t)value;
            break;
        
        /* --- Brake --- */
        case REG_CMD_BRAKE: {
            system_debug.debug_mode = 1;  // Auto-switch to manual
            Test_Manager_Stop();
            if ((uint8_t)value == FC_Break_Step_IDLE) {
                s_brake_cmd_host_latch = FC_Break_Step_IDLE;
                s_brake_trigger_rearm_lock = 0U;
                system_debug.manual_brake_cmd = FC_Break_Step_IDLE;
                break;
            }

            if ((uint8_t)value == FC_Trigger_Break) {
                if (s_brake_trigger_rearm_lock != 0U) {
                    system_debug.brake_trigger_ignore_count++;
                    cmd_regs[addr] = FC_Break_Step_IDLE;
                    break;
                }

                s_brake_trigger_rearm_lock = 1U;
                system_debug.brake_trigger_accept_count++;
            } else {
                s_brake_trigger_rearm_lock = 0U;
            }

            if ((uint8_t)value == s_brake_cmd_host_latch) {
                if ((uint8_t)value == FC_Trigger_Break) {
                    system_debug.brake_trigger_ignore_count++;
                }
                cmd_regs[addr] = FC_Break_Step_IDLE;
                break;
            }

            s_brake_cmd_host_latch = (uint8_t)value;
            system_debug.manual_brake_cmd = (uint8_t)value;
            /* Safety: Sync latest calibration to hardware before execution */
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) Brake_UpdateHardwareProfile(car->BreakCmd.BreakPoss, car->BreakCmd.BreakAcc, car->BreakCmd.BreakVel, car->BreakCmd.BreakJerk);
            break;
        }
        
        case REG_CMD_BRAKE_POS: {
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) car->BreakCmd.BreakPoss = (float)value / 100.0f;
            break;
        }
        case REG_CMD_BRAKE_ACC: {
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) car->BreakCmd.BreakAcc = (float)value / 10.0f;
            break;
        }
        case REG_CMD_BRAKE_VEL: {
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) car->BreakCmd.BreakVel = (float)value / 10.0f;
            break;
        }
        case REG_CMD_BRAKE_JERK: {
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) car->BreakCmd.BreakJerk = (float)value / 10.0f;
            break;
        }
        
        /* --- Gear --- */
        case REG_CMD_GEAR: {
            system_debug.debug_mode = 1;
            system_debug.manual_gear_cmd = (uint8_t)value;
            /* Safety: Sync latest calibration to hardware before execution */
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) {
                Gear_SetHardware(car->GearCmd.Tim, car->GearCmd.Poss[FC_Gear_P], car->GearCmd.Poss[FC_Gear_R], 
                                 car->GearCmd.Poss[FC_Gear_N], car->GearCmd.Poss[FC_Gear_D], 
                                 car->GearCmd.PossHome12, car->GearCmd.PossHome34,
                                 car->GearCmd.Servo_Assign[FC_Gear_P], car->GearCmd.Servo_Assign[FC_Gear_R],
                                 car->GearCmd.Servo_Assign[FC_Gear_N], car->GearCmd.Servo_Assign[FC_Gear_D]);
            }
            break;
        }
        
        /* --- Key --- */
        case REG_CMD_KEY: {
            system_debug.debug_mode = 1;
            system_debug.manual_key_cmd = (uint8_t)value;
            /* Safety: Sync latest calibration to hardware before execution */
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) {
                Key_SetHardware(car->KeyCmd.Tim, car->KeyCmd.KeyUnlock_Poss, car->KeyCmd.KeyLock_Poss, car->KeyCmd.KeyHome_Poss, car->KeyCmd.Tim_Press);
            }
            break;
        }
        
        /* --- Accelerator --- */
        case REG_CMD_ACCEL: {
            system_debug.debug_mode = 1;
            system_debug.manual_accel_cmd = (uint8_t)value;
            /* Safety: Sync latest calibration to hardware before execution */
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) Accel_SetHardware(&Motor1, car->AccelCmd.AccelPoss, car->AccelCmd.AccelAcc, car->AccelCmd.AccelVel, car->AccelCmd.AccelJerk);
            break;
        }
        
        case REG_CMD_ACCEL_POS: {
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) car->AccelCmd.AccelPoss = (float)value / 100.0f;
            break;
        }
        case REG_CMD_ACCEL_ACC: {
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) car->AccelCmd.AccelAcc = (float)value / 10.0f;
            break;
        }
        case REG_CMD_ACCEL_VEL: {
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) car->AccelCmd.AccelVel = (float)value / 10.0f;
            break;
        }
        case REG_CMD_ACCEL_JERK: {
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) car->AccelCmd.AccelJerk = (float)value / 10.0f;
            break;
        }

        case REG_CMD_DIAG_LOOPBACK:
            modbus_diag.loopback_value = value;
            break;

        case REG_CMD_DIAG_RESET:
            if (value != 1) {
                cmd_regs[REG_CMD_DIAG_RESET] = 0;
            }
            break;
        
        /* --- Relay Control --- */
        case REG_CMD_RELAY_1:
            K11(value ? 1 : 0);
            system_debug.relay_k11 = value ? 1 : 0;
            break;
        case REG_CMD_RELAY_2:
            K12(value ? 1 : 0);
            system_debug.relay_k12 = value ? 1 : 0;
            break;
        case REG_CMD_RELAY_3:
            K13(value ? 1 : 0);
            system_debug.relay_k13 = value ? 1 : 0;
            break;
        case REG_CMD_RELAY_4:
            K2(value ? 1 : 0);
            break;
        case REG_CMD_RELAY_5:
            K3(value ? 1 : 0);
            break;
        case REG_CMD_RELAY_6:
            K4(value ? 1 : 0);
            break;
        
        /* --- Test Control --- */
        case REG_CMD_TEST_CTRL:
            if (value == 1) {
                Test_Manager_Start();
            } else {
                Test_Manager_Stop();
            }
            break;
        
        case REG_CMD_TEST_INDEX:
            system_debug.test_index = (uint8_t)value;
            break;
        
        case REG_CMD_CAR_TYPE:
            Car_SetActiveConfig((Car_Type)value);
            break;
        
        case REG_CMD_SYSTEM:
            if (value == 1) {
                NVIC_SystemReset();  // Software reset
            }
            break;
        
        /* --- Key Config (Store only, synced on REG_CMD_KEY) --- */
        case REG_WRITE_KEY_UNLOCK_PWM:
        case REG_WRITE_KEY_LOCK_PWM:
        case REG_WRITE_KEY_HOME_PWM: {
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) {
                car->KeyCmd.KeyUnlock_Poss = modbus_regs.Reg.CMD.Key_Unlock_PWM;
                car->KeyCmd.KeyLock_Poss   = modbus_regs.Reg.CMD.Key_Lock_PWM;
                car->KeyCmd.KeyHome_Poss   = modbus_regs.Reg.CMD.Key_Home_PWM;
            }
            break;
        }
        
        /* --- Gear Config (Store only, synced on REG_CMD_GEAR) --- */
        case REG_CMD_GEAR_POS_P:
        case REG_CMD_GEAR_POS_R:
        case REG_CMD_GEAR_POS_N:
        case REG_CMD_GEAR_POS_D:
        case REG_CMD_GEAR_MAP_P:
        case REG_CMD_GEAR_MAP_R:
        case REG_CMD_GEAR_MAP_N:
        case REG_CMD_GEAR_MAP_D: {
            Car_Define_Typedef* car = Car_GetActiveConfig();
            if (car) {
                if (addr <= REG_CMD_GEAR_POS_D) 
                    car->GearCmd.Poss[addr - REG_CMD_GEAR_POS_P] = (uint16_t)value;
                else 
                    car->GearCmd.Servo_Assign[addr - REG_CMD_GEAR_MAP_P] = (uint8_t)value;
            }
            break;
        }
        
        /* --- Seatbelt Control --- */
        case REG_CMD_SEATBELT:
            SeatBelt(value ? 1 : 0);
            break;

        default:
            return false;
    }
    
    return true;
}

/* ═══════════════════════════════════════════════════════════════════
 *  READ HANDLER (FC03 → Status values)
 * ═══════════════════════════════════════════════════════════════════ */
bool ModbusRegMap_IsReadAddr(uint16_t addr) {
    return (addr < REG_WRITE_MAX ||
            (addr >= REG_STS_BASE && addr < (REG_STS_BASE + REG_STS_COUNT)) ||
            (addr == REG_STS_FW_VERSION));
}

uint16_t ModbusRegMap_Read(uint16_t addr) {
    /* If reading from Command range, return value from local array */
    if (addr < REG_WRITE_MAX) {
        return modbus_regs.Array[addr];
    }

    Car_Define_Typedef* car = Car_GetActiveConfig();
    
    switch (addr) {
        case REG_STS_GEAR_ACT:
            return car ? (uint16_t)car->StsCar.Val_Gear_Act : 0;
        
        case REG_STS_BRAKE_STS:
            return car ? (uint16_t)car->StsCar.Val_Break_Pedal_Sts : 0;
        
        case REG_STS_BRAKE_POS:
            return car ? (uint16_t)car->StsCar.Val_Break_Position : 0;
        
        case REG_STS_TERMINAL:
            return car ? (uint16_t)car->StsCar.Val_STAT_Terminal : 0;
        
        case REG_STS_HV:
            return car ? (uint16_t)car->StsCar.Val_HV_Sts : 0;
        
        case REG_STS_KEY_ACT:
            return car ? (uint16_t)car->StsCar.Val_Key_Act : 0;
        
        case REG_STS_SPEED:
            return car ? car->StsCar.Val_Car_Speed : 0;
        
        case REG_STS_LV_SOC:
            return car ? (uint16_t)car->StsCar.Val_LV_soc : 0;
        
        case REG_STS_HV_SOC:
            return car ? (uint16_t)car->StsCar.Val_HV_soc : 0;
        
        case REG_STS_DCDC:
            return car ? (uint16_t)car->StsCar.Val_DCDC_Sts : 0;
        
        case REG_STS_RELAY_1:
            return (uint16_t)system_debug.relay_k11;
        
        case REG_STS_RELAY_2:
            return (uint16_t)system_debug.relay_k12;
        
        case REG_STS_RELAY_3:
            return (uint16_t)system_debug.relay_k13;
        
        case REG_STS_TEST_STATUS:
            return (uint16_t)system_debug.test_status;
        
        case REG_STS_TEST_STEP:
            return (uint16_t)system_debug.test_step;
        
        case REG_STS_TEST_CYCLE:
            return (uint16_t)(system_debug.test_cycle & 0xFFFF);
        
        case REG_STS_BRAKE_STEP:
            return (uint16_t)Brake_GetStep();
        
        case REG_STS_ACCEL_POS:
            return car ? (uint16_t)car->StsCar.Val_Acc_Pedal_Position : 0;
        
        case REG_STS_CREEP:
            return car ? (uint16_t)car->StsCar.Val_CreepMode_Sts : 0;
        
        case REG_STS_CAN_STS:
            return (HAL_GetTick() - CAN_LastRxTick < 2000) ? 1 : 0;

        case REG_STS_DIAG_LOOPBACK:
            return modbus_diag.loopback_value;

        case REG_STS_DIAG_LAST_FC:
            return modbus_diag.last_function;

        case REG_STS_DIAG_LAST_ADDR:
            return modbus_diag.last_start_addr;

        case REG_STS_DIAG_RX_COUNT_L:
            return (uint16_t)(modbus_diag.rx_event_count & 0xFFFF);

        case REG_STS_DIAG_TX_COUNT_L:
            return (uint16_t)(modbus_diag.tx_response_count & 0xFFFF);

        case REG_STS_DIAG_CRC_ERR_L:
            return (uint16_t)(modbus_diag.crc_error_count & 0xFFFF);

        case REG_STS_FW_VERSION:
            return FW_VERSION;
        
        default:
            return 0;
    }
}
void ModbusRegMap_UpdateStatus(void) {
    /* Sync status registers to the naming-friendly struct inside the union */
    modbus_regs.Reg.STS.Gear_Act   = ModbusRegMap_Read(REG_STS_GEAR_ACT);
    modbus_regs.Reg.STS.Brake_Sts  = ModbusRegMap_Read(REG_STS_BRAKE_STS);
    modbus_regs.Reg.STS.Brake_Pos  = ModbusRegMap_Read(REG_STS_BRAKE_POS);
    modbus_regs.Reg.STS.Terminal   = ModbusRegMap_Read(REG_STS_TERMINAL);
    modbus_regs.Reg.STS.HV_Sts     = ModbusRegMap_Read(REG_STS_HV);
    modbus_regs.Reg.STS.Key_Act    = ModbusRegMap_Read(REG_STS_KEY_ACT);
    modbus_regs.Reg.STS.Speed      = ModbusRegMap_Read(REG_STS_SPEED);
    modbus_regs.Reg.STS.LV_Soc     = ModbusRegMap_Read(REG_STS_LV_SOC);
    modbus_regs.Reg.STS.HV_Soc     = ModbusRegMap_Read(REG_STS_HV_SOC);
    modbus_regs.Reg.STS.Dcdc_Sts   = ModbusRegMap_Read(REG_STS_DCDC);
    modbus_regs.Reg.STS.Relay_1    = ModbusRegMap_Read(REG_STS_RELAY_1);
    modbus_regs.Reg.STS.Relay_2    = ModbusRegMap_Read(REG_STS_RELAY_2);
    modbus_regs.Reg.STS.Relay_3    = ModbusRegMap_Read(REG_STS_RELAY_3);
    modbus_regs.Reg.STS.Test_Sts   = ModbusRegMap_Read(REG_STS_TEST_STATUS);
    modbus_regs.Reg.STS.Test_Step  = ModbusRegMap_Read(REG_STS_TEST_STEP);
    modbus_regs.Reg.STS.Test_Cycle = ModbusRegMap_Read(REG_STS_TEST_CYCLE);
    modbus_regs.Reg.STS.Brake_Step = ModbusRegMap_Read(REG_STS_BRAKE_STEP);
    modbus_regs.Reg.STS.Accel_Pos  = ModbusRegMap_Read(REG_STS_ACCEL_POS);
    modbus_regs.Reg.STS.Creep      = ModbusRegMap_Read(REG_STS_CREEP);
    modbus_regs.Reg.STS.CAN_Sts    = ModbusRegMap_Read(REG_STS_CAN_STS);
    modbus_regs.Reg.STS.Diag_Loopback = ModbusRegMap_Read(REG_STS_DIAG_LOOPBACK);
    modbus_regs.Reg.STS.Diag_Last_FC  = ModbusRegMap_Read(REG_STS_DIAG_LAST_FC);
    modbus_regs.Reg.STS.Diag_Last_Addr= ModbusRegMap_Read(REG_STS_DIAG_LAST_ADDR);
    modbus_regs.Reg.STS.Diag_Rx_L     = ModbusRegMap_Read(REG_STS_DIAG_RX_COUNT_L);
    modbus_regs.Reg.STS.Diag_Tx_L     = ModbusRegMap_Read(REG_STS_DIAG_TX_COUNT_L);
    modbus_regs.Reg.STS.Diag_Crc_L    = ModbusRegMap_Read(REG_STS_DIAG_CRC_ERR_L);
    modbus_regs.Reg.FW_Version        = ModbusRegMap_Read(REG_STS_FW_VERSION);
}
#undef cmd_regs
