/*
* HungDK_22/01/2026
* Menu.h
* Tổ chức giao diện menu, các mode chạy, mode test, mode config, ...
*/

#ifndef MENU_H
#define MENU_H

#include "Car.h"
#include "LCD_i2C.h"
#include "stdio.h"
#include "string.h"

extern CLCD_I2C_Name LCD;

/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------     Define Gobal Variable -------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

extern volatile uint32_t CAN1_RxID;
extern volatile uint8_t CAN1_RxData[8];
extern volatile uint8_t CAN1_RxReceived;

extern volatile uint32_t CAN2_RxID;
extern volatile uint8_t CAN2_RxData[8];
extern volatile uint8_t CAN2_RxReceived;





void Data_Process(Car_Define_Typedef* car);
void Virtual_Data_Conversion(Car_Define_Typedef* car);
void Mode_Virtual_CAN(Car_Define_Typedef* car);
#endif // Menu_H
