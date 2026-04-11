/*
 * Test_Manager.c - Bộ điều phối các kịch bản test
 */
#include "Test_Manager.h"
#include "stdio.h"
#include "string.h"
#include "bsp.h"
#include "common.h"
#include "CanBus.h"
#include "System_Error.h"
#include "Modbus_RegMap.h"

// Danh sách các kịch bản test có sẵn
static TestCase_t* test_registry[] = {
    &TestCase_Generic, // miniVan
    &TestCase_Generic, // Limo
    &TestCase_Generic, // VF89
    &TestCase_Generic, // VF5
    &TestCase_Generic, // VF67
    &TestCase_Generic, // VF3
    &TestCase_Generic, // VF2
    &TestCase_Generic, // e34
    &TestCase_Generic, // Virtual
    &TestCase_GearDemo
};
#define TEST_COUNT (sizeof(test_registry)/sizeof(TestCase_t*))

static TestCase_t* current_test = &TestCase_Generic;
static Test_Status_Typedef test_status = TEST_IDLE;
static Car_Define_Typedef* current_car_ptr;
static uint8_t safe_home_requested = 0;
static Break_CMD_Typedef manual_brake_active_cmd = FC_Break_Step_IDLE;
static Break_CMD_Typedef manual_brake_last_raw_cmd = FC_Break_Step_IDLE;
static uint8_t manual_brake_trigger_guard = 0U;

static void Test_Manager_ClearManualBrakeRequest(void) {
    system_debug.manual_brake_cmd = FC_Break_Step_IDLE;
    modbus_regs.Reg.CMD.Brake = FC_Break_Step_IDLE;
}

static void Test_Manager_ResetManualBrakeState(void) {
    manual_brake_active_cmd = FC_Break_Step_IDLE;
    manual_brake_trigger_guard = 0U;
    manual_brake_last_raw_cmd = (Break_CMD_Typedef)system_debug.manual_brake_cmd;
    Test_Manager_ClearManualBrakeRequest();
}

static void Test_Manager_LatchManualBrakeRequest(void) {
    Break_CMD_Typedef requested_cmd = (Break_CMD_Typedef)system_debug.manual_brake_cmd;

    if (requested_cmd == FC_Break_Step_IDLE) {
        manual_brake_last_raw_cmd = FC_Break_Step_IDLE;
        return;
    }

    if (requested_cmd == FC_Trigger_Break && manual_brake_last_raw_cmd == FC_Trigger_Break) {
        system_debug.brake_trigger_ignore_count++;
        Test_Manager_ClearManualBrakeRequest();
        return;
    }

    manual_brake_last_raw_cmd = requested_cmd;

    if (requested_cmd == FC_Trigger_Break) {
        if (manual_brake_trigger_guard != 0U) {
            system_debug.brake_trigger_ignore_count++;
            Test_Manager_ClearManualBrakeRequest();
            return;
        }

        Brake_RearmCommandState();
        manual_brake_trigger_guard = 1U;
    } else {
        manual_brake_trigger_guard = 0U;
    }

    manual_brake_active_cmd = requested_cmd;
    Test_Manager_ClearManualBrakeRequest();
}

static void Test_Manager_ServiceManualBrake(void) {
    Test_Manager_LatchManualBrakeRequest();

    if (manual_brake_active_cmd == FC_Break_Step_IDLE) {
        return;
    }

    Brake_Drive(manual_brake_active_cmd);

    switch (manual_brake_active_cmd) {
        case FC_Break_Home:
        case FC_Release_Break:
        case FC_Trigger_Break:
            if ((Brake_HasHomeFailure() != 0U) || (Brake_GetStep() == Break_Step_IDLE)) {
                if (manual_brake_active_cmd == FC_Trigger_Break) {
                    manual_brake_trigger_guard = 0U;
                }
                manual_brake_active_cmd = FC_Break_Step_IDLE;
                Brake_Drive(FC_Break_Step_IDLE);
            }
            break;

        case FC_Press_Break_Hold:
            if (Brake_HasHomeFailure() != 0U) {
                manual_brake_active_cmd = FC_Break_Step_IDLE;
            }
            break;

        case FC_Break_Reset:
        case FC_Break_Step_IDLE:
            manual_brake_active_cmd = FC_Break_Step_IDLE;
            break;

        default:
            if (Brake_GetStep() == Break_Step_IDLE) {
                manual_brake_active_cmd = FC_Break_Step_IDLE;
            }
            break;
    }
}

static uint8_t Test_Manager_HasManualBrakeSession(void) {
    return (uint8_t)((manual_brake_active_cmd != FC_Break_Step_IDLE) ||
                     (system_debug.manual_brake_cmd != FC_Break_Step_IDLE));
}

static void Test_Manager_ApplySafeHome(void) {
    // Brake homing must be serviced continuously so the limit switch can stop the stepper in time.
    Brake_Drive(FC_Break_Home);
    Gear_Drive(FC_Gear_Home);
    Key_Drive(FC_Key_Home);
}

void Test_Manager_Init(Car_Define_Typedef* car) {
    current_car_ptr = car;
    test_status = TEST_IDLE;
    safe_home_requested = 1;
    system_debug.debug_mode = 0;
    Test_Manager_ResetManualBrakeState();
    system_debug.manual_accel_cmd = FC_Accel_IDLE;
    system_debug.manual_gear_cmd = FC_Gear_Home;
    system_debug.manual_key_cmd = FC_Key_Home;
    
    // Khởi tạo test mặc định
    if (current_test && current_test->Init) {
        current_test->Init(car);
    }
    
    // Cập nhật thông tin debug
    system_debug.test_status = 0;
    strcpy(system_debug.current_test_name, current_test->name);
}

void Test_Manager_Select(uint8_t index) {
    if (index >= TEST_COUNT) return;
    
    // Dừng test cũ trước khi chuyển
    Test_Manager_Stop();
    
    current_test = test_registry[index];
    if (current_test->Init) {
        current_test->Init(current_car_ptr);
    }
    safe_home_requested = 1;
    
    strcpy(system_debug.current_test_name, current_test->name);
}

void Test_Manager_Start(void) {
    test_status = TEST_RUNNING;
    safe_home_requested = 0;
    system_debug.test_status = 1;
}

void Test_Manager_Stop(void) {
    test_status = TEST_STOPPED;
    safe_home_requested = 1;
    system_debug.test_status = 2;
}

void Test_Manager_Toggle(void) {
    if (test_status == TEST_RUNNING) {
        Test_Manager_Stop();
    } else {
        Test_Manager_Start();
    }
}

void Test_Manager_Update(void) {
    static uint8_t last_debug_mode = 0;
    uint8_t manual_brake_session_active = Test_Manager_HasManualBrakeSession();

    if (system_debug.debug_mode != last_debug_mode) {
        if (system_debug.debug_mode == 0U && manual_brake_session_active == 0U) {
            Test_Manager_ResetManualBrakeState();
        }
        last_debug_mode = system_debug.debug_mode;
    }

    // 0. Kiểm tra nếu người dùng đổi kịch bản qua IDE (system_debug.test_index)
    static uint8_t last_index = 255;
    if (system_debug.test_index != last_index) {
        if (last_index != 255) {
            Test_Manager_Select(system_debug.test_index);
        }
        last_index = system_debug.test_index;
    }

    // 1. Nếu đang ở chế độ Manual (IDE Watch), ghi đè trực tiếp hardware
    if (system_debug.debug_mode == 1U || manual_brake_session_active != 0U) {
        if (test_status == TEST_RUNNING) {
            Test_Manager_Stop();
        }
        Test_Manager_ServiceManualBrake();
        if (system_debug.debug_mode == 1U) {
            Gear_Drive((Gear_CMD_Typedef)system_debug.manual_gear_cmd);
            Key_Drive((Key_CMD_Typedef)system_debug.manual_key_cmd);
            Accel_Drive((Accel_CMD_Typedef)system_debug.manual_accel_cmd);
        }
        return;
    }

    // 2. Kiểm tra HealthCheck CAN Bus
    // Tạm thời Disable để bạn test RS485 trên bàn không cần cắm CAN
    /*
    if (test_status == TEST_RUNNING) {
        if (HAL_GetTick() - CAN_LastRxTick > 3000) {
            Test_Manager_Stop();
            System_ReportError(ERR_CAN_TIMEOUT, "Mat tin hieu CAN qua 3s");
            return;
        }
    }
    */

    // 3. Service requested home only when an explicit stop requests it.
    if (safe_home_requested && test_status != TEST_RUNNING) {
        Test_Manager_ApplySafeHome();
    }

    // 4. Chế độ Automatic: Chạy logic của kịch bản đang chọn
    if (test_status == TEST_RUNNING) {
        if (current_test && current_test->Update) {
            current_test->Update();
        }
    }
    
    // 3. Cập nhật thông tin chung cho UI/Debug
    system_debug.test_status = (uint8_t)test_status;
    if (current_test) {
        strcpy(system_debug.test_state_name, current_test->GetStateName());
        system_debug.test_cycle = current_test->GetCycleCount();
        if (test_status == TEST_RUNNING) {
            system_debug.rem_seconds = current_test->GetRemainingSeconds();
        }
        system_debug.test_step = current_test->GetStep();
    }
}

Test_Status_Typedef Test_Manager_GetStatus(void) {
    return test_status;
}

uint32_t Test_Manager_GetCycleCount(void) {
    return current_test ? current_test->GetCycleCount() : 0;
}

uint32_t Test_Manager_GetRemainingSeconds(void) {
    return current_test ? current_test->GetRemainingSeconds() : 0;
}

const char* Test_Manager_GetStateName(void) {
    return current_test ? current_test->GetStateName() : "IDLE";
}

uint8_t Test_Manager_GetStep(void) {
    return current_test ? current_test->GetStep() : 255;
}
