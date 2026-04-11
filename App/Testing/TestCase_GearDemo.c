/*
 * TestCase_GearDemo.c - Demo kịch bản quét số (P->R->N->D)
 */
#include "Test_Manager.h"
#include "common.h"
#include "bsp.h"
#include "string.h"

static Car_Define_Typedef* current_car;
static uint8_t step = 0;
static uint32_t timer = 0;
static uint32_t cycle = 0;

static void Init(Car_Define_Typedef* car) {
    current_car = car;
    step = 0;
    timer = millis();
    cycle = 0;

    Gear_SetHardware(
        current_car->GearCmd.Tim,
        current_car->GearCmd.Poss[FC_Gear_P],
        current_car->GearCmd.Poss[FC_Gear_R],
        current_car->GearCmd.Poss[FC_Gear_N],
        current_car->GearCmd.Poss[FC_Gear_D],
        current_car->GearCmd.PossHome12,
        current_car->GearCmd.PossHome34,
        0, 0, 1, 1 // P,R on S1; N,D on S2
    );
}

static void Update(void) {
    uint32_t now = millis();
    
    // Logic: Mỗi 2 giây đổi một số
    if (now - timer >= 2000) {
        timer = now;
        step = (step + 1) % 4; // 0:P, 1:R, 2:N, 3:D
        if (step == 0) cycle++;
    }

    // Thực thi điều lệnh
    Gear_Drive((Gear_CMD_Typedef)step);

    // Cập nhật debug
    system_debug.test_step = step;
    system_debug.test_cycle = cycle;
    system_debug.rem_seconds = (2000 - (now - timer)) / 1000;
}

static const char* GetStateName(void) {
    switch(step) {
        case 0: return "GEAR P      ";
        case 1: return "GEAR R      ";
        case 2: return "GEAR N      ";
        case 3: return "GEAR D      ";
        default: return "UNKNOWN    ";
    }
}

static uint32_t GetRemainingSeconds(void) {
    return (2000 - (millis() - timer)) / 1000;
}

static uint32_t GetCycleCount(void) {
    return cycle;
}

static uint8_t GetStep(void) {
    return step;
}

TestCase_t TestCase_GearDemo = {
    .name = "Gear Sweep",
    .Init = Init,
    .Update = Update,
    .GetStateName = GetStateName,
    .GetRemainingSeconds = GetRemainingSeconds,
    .GetCycleCount = GetCycleCount,
    .GetStep = GetStep
};
