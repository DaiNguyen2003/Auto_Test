/*
 * TestCase_Generic.c - Động cơ thực thi kịch bản dựa trên bảng (Table-Driven)
 */
#include "Test_Manager.h"
#include "common.h"
#include "bsp.h"
#include "string.h"
#include "stdio.h"

// Biến trạng thái nội bộ
static Car_Define_Typedef* current_car = NULL;
static uint8_t  step_idx = 0;
static uint32_t step_timer = 0;
static uint32_t cycle_count = 0;
static bool     action_fired = false;
static Break_CMD_Typedef  current_brake_cmd = FC_Break_Step_IDLE;
static Gear_CMD_Typedef   current_gear_cmd  = FC_Gear_Home;
static Key_CMD_Typedef    current_key_cmd   = FC_Key_Home;

static void Init(Car_Define_Typedef* car) {
    current_car = car;
    step_idx = 0;
    step_timer = millis();
    cycle_count = 0;
    action_fired = false;
    current_brake_cmd = FC_Break_Step_IDLE;
    current_gear_cmd = FC_Gear_Home;
    current_key_cmd = FC_Key_Home;

    if (current_car) {
        // Cấu hình phần cứng theo xe hiện tại
        Brake_SetHardware(&Motor1, car->BreakCmd.BreakPoss, car->BreakCmd.BreakAcc, car->BreakCmd.BreakVel, car->BreakCmd.BreakJerk);
        Gear_SetHardware(car->GearCmd.Tim, 
                        car->GearCmd.Poss[0], car->GearCmd.Poss[1], car->GearCmd.Poss[2], car->GearCmd.Poss[3],
                        car->GearCmd.PossHome12, car->GearCmd.PossHome34, 
                        car->GearCmd.Servo_Assign[0], car->GearCmd.Servo_Assign[1], 
                        car->GearCmd.Servo_Assign[2], car->GearCmd.Servo_Assign[3]);
        
        // Keep outputs idle after init so Modbus can come up cleanly on the bench.
        // Home is now serviced only when a test or an explicit command requests it.
    }
}

static void Update(void) {
    if (!current_car || !current_car->SequenceTable || current_car->SequenceSize == 0) {
        return;
    }

    TestStep_t* step = &current_car->SequenceTable[step_idx];
    uint32_t now = millis();

    // 1. Kích hoạt hành động (Chỉ set command 1 lần khi bắt đầu bước)
    if (!action_fired) {
        switch (step->action) {
            case ACT_KEY_LOCK:      current_key_cmd = FC_Key_Lock; break;
            case ACT_KEY_UNLOCK:    current_key_cmd = FC_Key_Unlock; break;
            case ACT_KEY_HOME:      current_key_cmd = FC_Key_Home; break;
            
            case ACT_BREAK_PRESS:   current_brake_cmd = FC_Press_Break_Hold; break;
            case ACT_BREAK_RELEASE: current_brake_cmd = FC_Break_Home; break;
            case ACT_BREAK_TRIGGER: current_brake_cmd = FC_Trigger_Break; break;
            
            case ACT_GEAR_P:    current_gear_cmd = FC_Gear_P; break;
            case ACT_GEAR_R:    current_gear_cmd = FC_Gear_R; break;
            case ACT_GEAR_N:    current_gear_cmd = FC_Gear_N; break;
            case ACT_GEAR_D:    current_gear_cmd = FC_Gear_D; break;
            case ACT_GEAR_HOME: current_gear_cmd = FC_Gear_Home; break;
            
            case ACT_LV_RESET:   LV_Drive(FC_LV_Reset); break;
            case ACT_ROBOT_STOP: Robot_Drive(FC_Robot_Stop); break;
            case ACT_SEATBELT_BUCKLE: SeatBelt(1); break;
            case ACT_SEATBELT_RELEASE: SeatBelt(0); break;
            case ACT_WAIT:       break; // Không làm gì, chỉ chờ
            default: break;
        }
        action_fired = true;
        step_timer = now;
    }

    // 2. Poll các Drive functions liên tục (state machine cần được gọi mỗi vòng)
    Brake_Drive(current_brake_cmd);
    Gear_Drive(current_gear_cmd);
    Key_Drive(current_key_cmd);

    // 3. Kiểm tra điều kiện chuyển bước
    bool condition_met = false;
    switch (step->condition) {
        case COND_TIME:
            if (now - step_timer >= step->cond_value) {
                condition_met = true;
            }
            break;

        case COND_CAN_VAL:
            // Tương lai: Logic check CAN signal
            condition_met = true; 
            break;

        default:
            condition_met = true;
            break;
    }

    // 4. Chuyển bước hoặc kết thúc chu kỳ
    if (condition_met) {
        step_idx++;
        action_fired = false;

        if (step_idx >= current_car->SequenceSize) {
            step_idx = 0;
            cycle_count++;
        }
    }

    // Cập nhật debug
    system_debug.test_step = step_idx;
    system_debug.test_cycle = cycle_count;
}

static const char* GetStateName(void) {
    if (!current_car || !current_car->SequenceTable) return "NO SEQ";
    
    TestAction_t act = current_car->SequenceTable[step_idx].action;
    switch (act) {
        case ACT_KEY_LOCK:    return "KEY LOCK    ";
        case ACT_KEY_UNLOCK:  return "KEY UNLOCK  ";
        case ACT_BREAK_PRESS: return "BRK PRESS   ";
        case ACT_BREAK_RELEASE: return "BRK RELEASE ";
        case ACT_GEAR_P:      return "GEAR P      ";
        case ACT_GEAR_R:      return "GEAR R      ";
        case ACT_GEAR_N:      return "GEAR N      ";
        case ACT_GEAR_D:      return "GEAR D      ";
        case ACT_SEATBELT_BUCKLE: return "BELT BUCKLE ";
        case ACT_SEATBELT_RELEASE: return "BELT RELEASE";
        case ACT_WAIT:        return "WAITING...  ";
        default:              return "STEP RUNNING";
    }
}

static uint32_t GetRemainingSeconds(void) {
    if (!current_car || !current_car->SequenceTable) return 0;
    TestStep_t* step = &current_car->SequenceTable[step_idx];
    if (step->condition != COND_TIME) return 0;

    uint32_t now = millis();
    uint32_t elapsed = now - step_timer;
    if (elapsed >= step->cond_value) return 0;
    return (step->cond_value - elapsed + 999) / 1000;
}

static uint32_t GetCycleCount(void) { return cycle_count; }
static uint8_t GetStep(void) { return step_idx; }

// Object duy nhất được export
TestCase_t TestCase_Generic = {
    .name = "Generic Test",
    .Init = Init,
    .Update = Update,
    .GetStateName = GetStateName,
    .GetRemainingSeconds = GetRemainingSeconds,
    .GetCycleCount = GetCycleCount,
    .GetStep = GetStep
};
