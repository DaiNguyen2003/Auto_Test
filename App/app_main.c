#include "app_main.h"
#include <string.h>
#include "scheduler.h"
#include "UI_Manager.h"
#include "Test_Manager.h"
#include "Car.h"
#include "Hardware_Control.h"
#include "bsp.h"
#include "Modbus_RTU.h"
#include "Modbus_RegMap.h"
#include "Debug_UART.h"
#include "Car_Signals.h"
#include "CanBus.h"

void APP_Init(void) {
    // 1. Initialize BSP / Hardware Control
    Debug_UART_Init();
    
    // Select Car Model Here
    Car_SetActiveConfig(VF_89);
    Car_Define_Typedef* current_car = Car_GetActiveConfig();
    
    Car_Hardware_Init(current_car);
    
    // 2. Initialize Service Layer (UI, Test Manager)
    UI_Manager_Init(&LCD);
    Test_Manager_Init(current_car);
    
    // 3. Initialize Modbus RTU Slave (UART3, DMA)
    ModbusRegMap_Init();
    Modbus_init();
    
    // 4. Initialize Scheduler
    Scheduler_Init();
}

void APP_Run(void) {
    CAN_RxFrame_t frame;

    while (CANBus_PopAnyRxFrame(&frame) != 0U) {
        (void)Car_ParseFrame(&frame);
    }

    Scheduler_Update();
}

// --- Task Implementations ---

void Task_Fast(void) {
    // Modbus RX is handled by DMA + HAL_UARTEx_RxEventCallback
    // No polling needed here
}

void Task_Signal(void) {
    // Raw CAN/FDCAN frames are drained every APP_Run() cycle.
    // Keep this task slot for future signal filtering / debounce logic.
}

void Task_Logic(void) {
    // Primary State Machine - Test Manager Update
    Test_Manager_Update();
}

void Task_UI(void) {
    // UI Refresh / UART Logging
    UI_Manager_Update(); 
    
    // Update Modbus Debug Variable for observation in IDE
    ModbusRegMap_UpdateStatus();
}
