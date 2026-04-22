#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// --- Task Periods (ms) ---
#define TASK_PERIOD_FAST        5   // Fast IRQ / Motor updates
#define TASK_PERIOD_SIGNAL      10  // Signal filtering / Debounce
#define TASK_PERIOD_LOGIC       5   // Main State Machine / brake-limit response
#define TASK_PERIOD_UI          100 // Display updates / Logging

// --- Runtime Mode Config ---
// Default build must boot into the normal app path so LCD / Modbus / APP_Run stay active.
#define MAIN_MOTOR_TEST_MODE          0U
#define MAIN_BRAKE_SEQUENCE_TEST_MODE 0U

// --- CAN Runtime Budget ---
// Drain only a bounded number of CAN frames per APP_Run() so UI / Modbus are not starved.
#define CAN_APP_RX_BUDGET_PER_LOOP    8U

// --- Stepper Driver Config ---
// Set to 1 when the driver is enabled by a HIGH level on EN.
// Set to 0 when the driver is enabled by a LOW level on EN.
#define STEPPER_ENABLE_ACTIVE_HIGH 1


// --- Hardware Constants ---
#define SYSTICK_PERIOD_MS       1

#if ((MAIN_MOTOR_TEST_MODE != 0U) && (MAIN_BRAKE_SEQUENCE_TEST_MODE != 0U))
#error "Only one standalone runtime mode can be enabled at a time."
#endif

#endif // APP_CONFIG_H
