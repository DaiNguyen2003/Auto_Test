/**
 * @file    Modbus_RegMap.h
 * @brief   Modbus register map accessors
 */
#ifndef MODBUS_REGMAP_H
#define MODBUS_REGMAP_H

#include "Modbus_Config.h"
#include <stdbool.h>

typedef union {
    uint16_t Array[REG_WRITE_MAX + REG_STS_COUNT];
    struct {
        struct {
            uint16_t Mode;
            uint16_t Brake;
            uint16_t Brake_Pos;
            uint16_t Brake_Acc;
            uint16_t Brake_Vel;
            uint16_t Gear;
            uint16_t Key;
            uint16_t Accel;
            uint16_t Accel_Pos;
            uint16_t Accel_Acc;
            uint16_t Accel_Vel;
            uint16_t Relay_1;
            uint16_t Relay_2;
            uint16_t Relay_3;
            uint16_t Relay_4;
            uint16_t Relay_5;
            uint16_t Relay_6;
            uint16_t Test_Ctrl;
            uint16_t Test_Index;
            uint16_t Car_Type;
            uint16_t System;
            uint16_t Key_Unlock_PWM;
            uint16_t Key_Lock_PWM;
            uint16_t Key_Home_PWM;
            uint16_t Gear_Pos_P;
            uint16_t Gear_Pos_R;
            uint16_t Gear_Pos_N;
            uint16_t Gear_Pos_D;
            uint16_t Gear_Map_P;
            uint16_t Gear_Map_R;
            uint16_t Gear_Map_N;
            uint16_t Gear_Map_D;
            uint16_t Brake_Jerk;
            uint16_t Accel_Jerk;
            uint16_t Diag_Loopback;
            uint16_t Diag_Reset;
        } CMD;

        struct {
            uint16_t Gear_Act;
            uint16_t Brake_Sts;
            uint16_t Brake_Pos;
            uint16_t Terminal;
            uint16_t HV_Sts;
            uint16_t Key_Act;
            uint16_t Speed;
            uint16_t LV_Soc;
            uint16_t HV_Soc;
            uint16_t Dcdc_Sts;
            uint16_t Relay_1;
            uint16_t Relay_2;
            uint16_t Relay_3;
            uint16_t Test_Sts;
            uint16_t Test_Step;
            uint16_t Test_Cycle;
            uint16_t Brake_Step;
            uint16_t Accel_Pos;
            uint16_t Creep;
            uint16_t CAN_Sts;
            uint16_t Diag_Loopback;
            uint16_t Diag_Last_FC;
            uint16_t Diag_Last_Addr;
            uint16_t Diag_Rx_L;
            uint16_t Diag_Tx_L;
            uint16_t Diag_Crc_L;
        } STS;

        uint16_t FW_Version;
    } Reg;
} Modbus_Registers_t;

extern Modbus_Registers_t modbus_regs;

void ModbusRegMap_Init(void);
void ModbusRegMap_UpdateStatus(void);
bool ModbusRegMap_Write(uint16_t addr, uint16_t value);
uint16_t ModbusRegMap_Read(uint16_t addr);
bool ModbusRegMap_IsWriteAddr(uint16_t addr);
bool ModbusRegMap_IsReadAddr(uint16_t addr);

#endif /* MODBUS_REGMAP_H */
