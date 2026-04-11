#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// --- Task Periods (ms) ---
#define TASK_PERIOD_FAST        5   // Fast IRQ / Motor updates
#define TASK_PERIOD_SIGNAL      10  // Signal filtering / Debounce
#define TASK_PERIOD_LOGIC       5   // Main State Machine / brake-limit response
#define TASK_PERIOD_UI          100 // Display updates / Logging

// --- Stepper Driver Config ---
// Set to 1 when the driver is enabled by a HIGH level on EN.
// Set to 0 when the driver is enabled by a LOW level on EN.
#define STEPPER_ENABLE_ACTIVE_HIGH 1


// --- Hardware Constants ---
#define SYSTICK_PERIOD_MS       1

#endif // APP_CONFIG_H
