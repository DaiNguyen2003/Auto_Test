#include "common.h"
#include "Motor_Control.h"

System_Debug_Typedef system_debug;

uint32_t millis(void) {
    return HAL_GetTick();
}

