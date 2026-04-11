#ifndef BSP_H
#define BSP_H

#include "main.h"
#include "LCD_i2C.h"

// External instances from main.c or other BSP files
extern CLCD_I2C_Name LCD;

void BSP_Init(void);

// Low-level hardware control wrappers
void SeatBelt(uint8_t stt);

#endif // BSP_H
