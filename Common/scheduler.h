#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "main.h"

typedef void (*TaskFunction)(void);

typedef struct {
    TaskFunction func;
    uint32_t period;
    uint32_t last_tick;
} Task_Typedef;

void Scheduler_Init(void);
void Scheduler_Update(void);

#endif // SCHEDULER_H
