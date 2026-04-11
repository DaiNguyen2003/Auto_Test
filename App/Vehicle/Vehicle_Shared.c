#include "Car.h"

TestStep_t std_sequence[] = {
    { ACT_KEY_UNLOCK,     0,   COND_TIME, 10000, 15000 },
    { ACT_WAIT,           0,   COND_TIME, 10000, 15000 },
    { ACT_BREAK_TRIGGER,  0,   COND_TIME, 10000, 15000 },
    { ACT_BREAK_RELEASE,  0,   COND_TIME, 10000, 15000 },
    { ACT_WAIT,           0,   COND_TIME, 10000, 15000 },
    { ACT_BREAK_PRESS,    0,   COND_TIME, 10000, 15000 },
    { ACT_GEAR_D,         0,   COND_TIME, 10000, 15000 },
    { ACT_GEAR_N,         0,   COND_TIME, 10000, 15000 },
    { ACT_GEAR_R,         0,   COND_TIME, 10000, 15000 },
    { ACT_GEAR_P,         0,   COND_TIME, 10000, 15000 },
    { ACT_BREAK_RELEASE,  0,   COND_TIME, 10000, 15000 },
    { ACT_KEY_LOCK,       0,   COND_TIME, 10000, 15000 },
    { ACT_WAIT,           0,   COND_TIME, 300000, 310000 } // Sleep wait 5m
};

TestStep_t e34_sequence[] = {
    { ACT_KEY_UNLOCK,     0,   COND_TIME, 10000, 15000 },
    { ACT_BREAK_PRESS,    0,   COND_TIME, 10000, 15000 },
    { ACT_GEAR_D,         0,   COND_TIME, 10000, 15000 },
    { ACT_GEAR_N,         0,   COND_TIME, 10000, 15000 },
    { ACT_GEAR_R,         0,   COND_TIME, 10000, 15000 },
    { ACT_GEAR_P,         0,   COND_TIME, 10000, 15000 },
    { ACT_BREAK_RELEASE,  0,   COND_TIME, 10000, 15000 },
    { ACT_KEY_LOCK,       0,   COND_TIME, 10000, 15000 }
};

const uint8_t STD_SEQ_SIZE = sizeof(std_sequence)/sizeof(TestStep_t);
const uint8_t E34_SEQ_SIZE = sizeof(e34_sequence)/sizeof(TestStep_t);
