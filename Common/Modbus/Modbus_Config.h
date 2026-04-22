/**
 * @file    Modbus_Config.h
 * @brief   Modbus RTU register map definition
 */
#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

#include <stdint.h>

/* Slave configuration */
#define MODBUS_SLAVE_ID             0x01
#define MODBUS_BAUDRATE             115200

/* Write registers: PC -> STM32 */
#define REG_CMD_MODE                0x0000
#define REG_CMD_BRAKE               0x0001
#define REG_CMD_BRAKE_POS           0x0002
#define REG_CMD_BRAKE_ACC           0x0003
#define REG_CMD_BRAKE_VEL           0x0004
#define REG_CMD_GEAR                0x0005
#define REG_CMD_KEY                 0x0006
#define REG_CMD_ACCEL               0x0007
#define REG_CMD_ACCEL_POS           0x0008
#define REG_CMD_ACCEL_ACC           0x0009
#define REG_CMD_ACCEL_VEL           0x000A
#define REG_CMD_RELAY_1             0x000B
#define REG_CMD_RELAY_2             0x000C
#define REG_CMD_RELAY_3             0x000D
#define REG_CMD_RELAY_4             0x000E
#define REG_CMD_RELAY_5             0x000F
#define REG_CMD_RELAY_6             0x0010
#define REG_CMD_TEST_CTRL           0x0011
#define REG_CMD_TEST_INDEX          0x0012
#define REG_CMD_CAR_TYPE            0x0013
#define REG_CMD_SYSTEM              0x0014
#define REG_CMD_KEY_UNLOCK_POS      0x0015
#define REG_WRITE_KEY_UNLOCK_PWM    0x0015
#define REG_WRITE_KEY_LOCK_PWM      0x0016
#define REG_WRITE_KEY_HOME_PWM      0x0017
#define REG_CMD_GEAR_POS_P          0x0018
#define REG_CMD_GEAR_POS_R          0x0019
#define REG_CMD_GEAR_POS_N          0x001A
#define REG_CMD_GEAR_POS_D          0x001B
#define REG_CMD_GEAR_MAP_P          0x001C
#define REG_CMD_GEAR_MAP_R          0x001D
#define REG_CMD_GEAR_MAP_N          0x001E
#define REG_CMD_GEAR_MAP_D          0x001F
#define REG_CMD_BRAKE_JERK          0x0020
#define REG_CMD_ACCEL_JERK          0x0021
#define REG_CMD_DIAG_LOOPBACK       0x0022
#define REG_CMD_DIAG_RESET          0x0023
#define REG_CMD_SEATBELT            0x0024

#define REG_WRITE_MAX               0x0025

/* Read registers: STM32 -> PC */
#define REG_STS_BASE                0x0100

#define REG_STS_GEAR_ACT            0x0100
#define REG_STS_BRAKE_STS           0x0101
#define REG_STS_BRAKE_POS           0x0102
#define REG_STS_TERMINAL            0x0103
#define REG_STS_HV                  0x0104
#define REG_STS_KEY_ACT             0x0105
#define REG_STS_SPEED               0x0106
#define REG_STS_LV_SOC              0x0107
#define REG_STS_HV_SOC              0x0108
#define REG_STS_DCDC                0x0109
#define REG_STS_RELAY_1             0x010A
#define REG_STS_RELAY_2             0x010B
#define REG_STS_RELAY_3             0x010C
#define REG_STS_TEST_STATUS         0x010D
#define REG_STS_TEST_STEP           0x010E
#define REG_STS_TEST_CYCLE          0x010F
#define REG_STS_BRAKE_STEP          0x0110
#define REG_STS_ACCEL_POS           0x0111
#define REG_STS_CREEP               0x0112
#define REG_STS_CAN_STS             0x0113
#define REG_STS_DIAG_LOOPBACK       0x0114
#define REG_STS_DIAG_LAST_FC        0x0115
#define REG_STS_DIAG_LAST_ADDR      0x0116
#define REG_STS_DIAG_RX_COUNT_L     0x0117
#define REG_STS_DIAG_TX_COUNT_L     0x0118
#define REG_STS_DIAG_CRC_ERR_L      0x0119
#define REG_STS_FW_VERSION          0x01FF

#define REG_SIGMON_HEADER_BASE      0x0200
#define REG_SIGMON_SCHEMA_VERSION   0x0200
#define REG_SIGMON_ACTIVE_CAR_TYPE  0x0201
#define REG_SIGMON_SIGNAL_COUNT     0x0202
#define REG_SIGMON_NAME_CHARS       0x0203
#define REG_SIGMON_VALUE_SCALE      0x0204
#define REG_SIGMON_GENERATION       0x0205
#define REG_SIGMON_DATA_ROW_STRIDE  0x0206
#define REG_SIGMON_NAME_ROW_STRIDE  0x0207
#define REG_SIGMON_MAX_ROWS_REG     0x0208

#define REG_SIGMON_DATA_BASE        0x0210
#define REG_SIGMON_NAME_BASE        0x0300

#define SIGMON_SCHEMA_VERSION_VALUE 1U
#define SIGMON_DATA_ROW_STRIDE_REGS 6U
#define SIGMON_NAME_ROW_STRIDE_REGS 20U

#define REG_STS_COUNT               26
#define FW_VERSION                  0x0100

#endif /* MODBUS_CONFIG_H */
