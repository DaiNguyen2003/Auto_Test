/*
 * TestCase_Creep.c - Kịch bản test Creep Mode
 */
#include "Test_Manager.h"
#include "common.h"
#include "bsp.h"
#include "string.h"
#include "stdio.h"

// Biến trạng thái nội bộ
static uint8_t  step_idx = 0;
static uint32_t step_timer = 0;
static uint32_t cycle_count = 0;
static bool     action_fired = false;

static Break_CMD_Typedef  current_brake_cmd = FC_Break_Step_IDLE;
static Gear_CMD_Typedef   current_gear_cmd  = FC_Gear_Home;
static Key_CMD_Typedef    current_key_cmd   = FC_Key_Home;

static TestStep_t creep_sequence[] = {
    {ACT_KEY_HOME,        0, COND_TIME, 3000, 0},   // 0: Home All
    {ACT_KEY_UNLOCK,      0, COND_TIME, 3000, 0},   // 1: Unlock
    {ACT_BREAK_TRIGGER,   0, COND_TIME, 20000, 0},  // 2: Brake press (Pulse: nhấn xong về home rồi đợi 20s)
    {ACT_SEATBELT_BUCKLE, 0, COND_TIME, 3000, 0},   // 3: Đóng seatbelt
    {ACT_BREAK_HOLD,      0, COND_TIME, 20000, 0},  // 4: Brake hold (nhấn và giữ 20s)
    {ACT_GEAR_D,          0, COND_TIME, 3000, 0},   // 5: Gear D
    {ACT_BREAK_RELEASE,   0, COND_TIME, 20000, 0},  // 6: Brake release (về home 20s)
    {ACT_WAIT,            0, COND_TIME, 60000, 0},  // 7: Wait 60s
    {ACT_BREAK_HOLD,      0, COND_TIME, 20000, 0},  // 8: Brake hold
    {ACT_GEAR_R,          0, COND_TIME, 3000, 0},   // 9: Gear R
    {ACT_BREAK_RELEASE,   0, COND_TIME, 20000, 0},  // 10: Brake release
    {ACT_WAIT,            0, COND_TIME, 60000, 0},  // 11: Wait 60s
    {ACT_BREAK_HOLD,      0, COND_TIME, 20000, 0},  // 12: Brake hold
    {ACT_GEAR_P,          0, COND_TIME, 3000, 0},   // 13: Gear P (lần 1)
    {ACT_GEAR_HOME,       0, COND_TIME, 3000, 0},   // 14: Nhả gear để bấm tiếp
    {ACT_GEAR_P,          0, COND_TIME, 3000, 0},   // 15: Gear P (lần 2)
    {ACT_GEAR_HOME,       0, COND_TIME, 3000, 0},   // 16: Nhả gear
    {ACT_GEAR_P,          0, COND_TIME, 3000, 0},   // 17: Gear P (lần 3)
    {ACT_BREAK_RELEASE,   0, COND_TIME, 20000, 0},  // 18: Brake release
    {ACT_SEATBELT_RELEASE,0, COND_TIME, 3000, 0},   // 19: Nhả seatbelt
    {ACT_KEY_LOCK,        0, COND_TIME, 3000, 0},   // 20: Lock key
    {ACT_WAIT,            0, COND_TIME, 300000, 0}  // 21: Đợi ngủ 5 phút (300,000 ms)
};

#define CREEP_SEQ_SIZE (sizeof(creep_sequence)/sizeof(TestStep_t))

static void Init(Car_Define_Typedef* car) {
    step_idx = 0;
    step_timer = millis();
    cycle_count = 0;
    action_fired = false;
    current_brake_cmd = FC_Break_Step_IDLE;
    current_gear_cmd = FC_Gear_Home;
    current_key_cmd = FC_Key_Home;
    SeatBelt(0);
    
    if (car) {
        // Giảm hành trình phanh xuống 25.0mm (bớt thêm một chút so với trước)
        Brake_UpdateHardwareProfile(25.0f, car->BreakCmd.BreakAcc, car->BreakCmd.BreakVel, car->BreakCmd.BreakJerk);
    }
}

static void Update(void) {
    TestStep_t* step = &creep_sequence[step_idx];
    uint32_t now = millis();

    if (!action_fired) {
        switch (step->action) {
            case ACT_KEY_LOCK:      current_key_cmd = FC_Key_Lock; break;
            case ACT_KEY_UNLOCK:    current_key_cmd = FC_Key_Unlock; break;
            case ACT_KEY_HOME:      current_key_cmd = FC_Key_Home; break;
            
            case ACT_BREAK_PRESS:   current_brake_cmd = FC_Press_Break_Hold; break; // Giữ lại để tương thích
            case ACT_BREAK_HOLD:    current_brake_cmd = FC_Press_Break_Hold; break;
            case ACT_BREAK_TRIGGER: current_brake_cmd = FC_Trigger_Break; break;
            case ACT_BREAK_RELEASE: current_brake_cmd = FC_Break_Home; break;
            
            case ACT_GEAR_P:    current_gear_cmd = FC_Gear_P; break;
            case ACT_GEAR_R:    current_gear_cmd = FC_Gear_R; break;
            case ACT_GEAR_N:    current_gear_cmd = FC_Gear_N; break;
            case ACT_GEAR_D:    current_gear_cmd = FC_Gear_D; break;
            case ACT_GEAR_HOME: current_gear_cmd = FC_Gear_Home; break;

            case ACT_SEATBELT_BUCKLE: SeatBelt(1); break;
            case ACT_SEATBELT_RELEASE: SeatBelt(0); break;
            
            case ACT_WAIT:       break; 
            default: break;
        }
        action_fired = true;
        step_timer = now;
    }

    Brake_Drive(current_brake_cmd);
    Gear_Drive(current_gear_cmd);
    Key_Drive(current_key_cmd);

    bool condition_met = false;
    if (now - step_timer >= step->cond_value) {
        condition_met = true;
    }

    if (condition_met) {
        step_idx++;
        action_fired = false;
        if (step_idx >= CREEP_SEQ_SIZE) {
            step_idx = 0;
            cycle_count++;
        }
    }
}

static const char* GetStateName(void) {
    TestAction_t act = creep_sequence[step_idx].action;
    switch (act) {
        case ACT_KEY_LOCK:    return "KEY LOCK    ";
        case ACT_KEY_UNLOCK:  return "KEY UNLOCK  ";
        case ACT_BREAK_PRESS: return "BRK PRESS   ";
        case ACT_BREAK_HOLD:  return "BRK HOLD    ";
        case ACT_BREAK_TRIGGER: return "BRK PULSE   ";
        case ACT_BREAK_RELEASE: return "BRK RELEASE ";
        case ACT_GEAR_P:      return "GEAR P      ";
        case ACT_GEAR_R:      return "GEAR R      ";
        case ACT_GEAR_D:      return "GEAR D      ";
        case ACT_SEATBELT_BUCKLE: return "BELT BUCKLE ";
        case ACT_SEATBELT_RELEASE: return "BELT RELEASE";
        case ACT_WAIT:        return (step_idx == 21) ? "SLEEP 5M... " : "WAITING...  ";
        default:              return "CREEP RUN   ";
    }
}

static uint32_t GetRemainingSeconds(void) {
    uint32_t now = millis();
    uint32_t elapsed = now - step_timer;
    if (elapsed >= creep_sequence[step_idx].cond_value) return 0;
    return (creep_sequence[step_idx].cond_value - elapsed + 999) / 1000;
}

static uint32_t GetCycleCount(void) { return cycle_count; }
static uint8_t GetStep(void) { return step_idx; }

TestCase_t TestCase_Creep = {
    .name = "Creep Test",
    .Init = Init,
    .Update = Update,
    .GetStateName = GetStateName,
    .GetRemainingSeconds = GetRemainingSeconds,
    .GetCycleCount = GetCycleCount,
    .GetStep = GetStep
};
