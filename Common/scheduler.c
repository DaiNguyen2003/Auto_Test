#include "scheduler.h"
#include "app_config.h"

// Task function prototypes (to be implemented in App layer)
extern void Task_Fast(void);
extern void Task_Signal(void);
extern void Task_Logic(void);
extern void Task_UI(void);

static Task_Typedef tasks[] = {
    {Task_Fast,   TASK_PERIOD_FAST,   0},
    {Task_Signal, TASK_PERIOD_SIGNAL, 0},
    {Task_Logic,  TASK_PERIOD_LOGIC,  0},
    {Task_UI,     TASK_PERIOD_UI,     0}
};

#define TASK_COUNT (sizeof(tasks)/sizeof(Task_Typedef))

void Scheduler_Init(void) {
    uint32_t tick = HAL_GetTick();
    for (int i = 0; i < TASK_COUNT; i++) {
        tasks[i].last_tick = tick;
    }
}

void Scheduler_Update(void) {
    uint32_t current_tick = HAL_GetTick();
    for (int i = 0; i < TASK_COUNT; i++) {
        if (current_tick - tasks[i].last_tick >= tasks[i].period) {
            tasks[i].last_tick = current_tick;
            if (tasks[i].func) {
                tasks[i].func();
            }
        }
    }
}
