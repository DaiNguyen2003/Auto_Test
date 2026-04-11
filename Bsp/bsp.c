#include "bsp.h"

extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim4;

void BSP_Init(void) {
    // 1. LCD Hardware Init
    CLCD_I2C_Init(&LCD, &hi2c2, 0x4e, 20, 4);
    CLCD_I2C_Clear(&LCD);
    
    // 2. Initial Display
    CLCD_I2C_SetCursor(&LCD, 0, 0);
    CLCD_I2C_WriteString(&LCD, "[1] HI Cy:0   S:0   ");
    CLCD_I2C_SetCursor(&LCD, 0, 3);
    CLCD_I2C_WriteString(&LCD, "MINIVAN:             ");
    
    // 3. System States
    SeatBelt(0); // Initial position: nhả
    
    HAL_Delay(500); // Only allowed during boot/init phase
}

void SeatBelt(uint8_t stt) {
    if (stt == 1) { // cài
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2250); // Thắt
    } else {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1250); // nhả
    }
}
