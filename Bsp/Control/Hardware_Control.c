#include "Hardware_Control.h"

extern StepObject Motor1;

static struct {
    StepObject* motor;
    float pos;
    float acc;
    float vel;
    float jerk;
    uint32_t press_steps_override;
    Break_Action_Typedef step;
    Break_CMD_Typedef last_cmd;
    Break_CMD_Typedef pending_cmd;
    uint32_t timer;
    uint32_t home_limit_since;
    uint32_t home_retry_tick;
    uint8_t motion_started;
    uint8_t press_steps_override_valid;
    uint8_t pending_cmd_valid;
    uint8_t home_required;
    uint8_t home_failed;
    uint8_t trigger_lockout;
} g_brake = {0};

static struct {
    StepObject* motor;
    float pos;
    float acc;
    float vel;
    float jerk;
    Accel_Action_Typedef step;
    Accel_CMD_Typedef last_cmd;
    uint32_t timer;
} g_accel = {0};

static struct {
    TIM_HandleTypeDef* htim;
    uint16_t pos[5];
    uint16_t home12;
    uint16_t home34;
    uint8_t servo_assign[4]; // 0=Servo1, 1=Servo2 for P,R,N,D
    Gear_Action_Typedef step;
    Gear_CMD_Typedef last_cmd;
    uint32_t timer;
} g_gear = {0};

static struct {
    TIM_HandleTypeDef* htim;
    uint16_t unlock;
    uint16_t lock;
    uint16_t home;
    uint8_t time_press;
    Key_Action_Typedef step;
    Key_CMD_Typedef last_cmd;
    uint32_t timer;
} g_key = {0};

#define BRAKE_HOME_TIMEOUT_MS 15000U
#define BRAKE_HOME_RETRY_MS 500U
#define BRAKE_HOME_LIMIT_DEBOUNCE_MS 30U
#define BRAKE_HOME_MIN_TRAVEL_STEPS 80U
#define BRAKE_HOLD_REFRESH_MS 250U
#define BRAKE_HOLD_REFRESH_STEPS 1U
#define BRAKE_HOME_RUN_STEPS 100000U
#define BRAKE_PRESS_VEL_STEPS 3000.0f
#define BRAKE_PRESS_ACC_STEPS 3500.0f
#define BRAKE_PRESS_JERK_STEPS 6000.0f
#define BRAKE_HOME_VEL_STEPS 4200.0f
#define BRAKE_HOME_ACC_STEPS 5200.0f
#define BRAKE_HOME_JERK_STEPS 8500.0f
#define BRAKE_DIR_PRESS DIR_CW
#define BRAKE_DIR_HOME  DIR_CCW
#define BRAKE_PRESS_TRAVEL_SCALE 1.00f
#define BRAKE_FULL_STROKE_MM 90.0f
#define BRAKE_FULL_STROKE_STEPS 40000.0f
#define BRAKE_STEPS_PER_MM (BRAKE_FULL_STROKE_STEPS / BRAKE_FULL_STROKE_MM)
#define BRAKE_TRIGGER_RELEASE_DELAY_MS 250U
#define ACCEL_HOME_RUN_STEPS 100000U
#define ACCEL_HOME_VEL_STEPS 1800.0f
#define ACCEL_HOME_ACC_STEPS 1800.0f
#define ACCEL_HOME_JERK_STEPS 2700.0f

static void brake_reset_command_state(void);

static float minf_local(float a, float b) {
    return (a < b) ? a : b;
}

static uint32_t brake_convert_mm_to_steps(float pos_mm) {
    if (pos_mm <= 0.0f) {
        return 0U;
    }

    return (uint32_t)((pos_mm * BRAKE_STEPS_PER_MM) + 0.5f);
}

static uint32_t brake_get_press_steps(void) {
    if (g_brake.press_steps_override_valid != 0U) {
        return g_brake.press_steps_override;
    }

    float scaled_pos = g_brake.pos * BRAKE_PRESS_TRAVEL_SCALE;

    if (scaled_pos < 0.0f) {
        scaled_pos = 0.0f;
    }

    return brake_convert_mm_to_steps(scaled_pos);
}

static void brake_run_press_profile(void) {
    float acc_steps = minf_local(Convert_Acc_to_Step(g_brake.acc), BRAKE_PRESS_ACC_STEPS);
    float vel_steps = minf_local(Convert_Vel_to_Step(g_brake.vel), BRAKE_PRESS_VEL_STEPS);
    float jerk_steps = minf_local(Convert_Acc_to_Step(g_brake.jerk), BRAKE_PRESS_JERK_STEPS);

    MotorRun(
        g_brake.motor,
        brake_get_press_steps(),
        acc_steps,
        vel_steps,
        jerk_steps,
        BRAKE_DIR_PRESS
    );
}

static void brake_run_home_profile(void) {
    MotorRun(
        g_brake.motor,
        BRAKE_HOME_RUN_STEPS,
        BRAKE_HOME_ACC_STEPS,
        BRAKE_HOME_VEL_STEPS,
        BRAKE_HOME_JERK_STEPS,
        BRAKE_DIR_HOME
    );
}

static void accel_run_press_profile(void) {
    float acc_steps = minf_local(Convert_Acc_to_Step(g_accel.acc), ACCEL_HOME_ACC_STEPS);
    float vel_steps = minf_local(Convert_Vel_to_Step(g_accel.vel), ACCEL_HOME_VEL_STEPS);
    float jerk_steps = minf_local(Convert_Acc_to_Step(g_accel.jerk), ACCEL_HOME_JERK_STEPS);

    MotorRun(
        g_accel.motor,
        Convert_Poss_to_Step(g_accel.pos),
        acc_steps,
        vel_steps,
        jerk_steps,
        DIR_CCW
    );
}

static void accel_run_home_profile(void) {
    MotorRun(
        g_accel.motor,
        ACCEL_HOME_RUN_STEPS,
        ACCEL_HOME_ACC_STEPS,
        ACCEL_HOME_VEL_STEPS,
        ACCEL_HOME_JERK_STEPS,
        DIR_CW
    );
}

void Brake_SetHardware(StepObject* motor, float pos, float acc, float vel, float jerk) {
    g_brake.motor = motor;
    g_brake.pos = pos;
    g_brake.acc = acc;
    g_brake.vel = vel;
    g_brake.jerk = jerk;
    g_brake.press_steps_override = 0U;
    g_brake.press_steps_override_valid = 0U;
    g_brake.step = Break_Step_Reset;
    g_brake.last_cmd = FC_Break_Reset;
    g_brake.pending_cmd = FC_Break_Step_IDLE;
    g_brake.pending_cmd_valid = 0U;
    g_brake.home_required = 1U;
    g_brake.home_failed = 0U;
    g_brake.trigger_lockout = 0U;
    g_brake.timer = 0U;
    g_brake.home_limit_since = 0U;
    g_brake.home_retry_tick = 0U;
    g_brake.motion_started = 0U;
}

void Brake_UpdateHardwareProfile(float pos, float acc, float vel, float jerk) {
    if (g_brake.motor == NULL) {
        return;
    }

    g_brake.pos = pos;
    g_brake.acc = acc;
    g_brake.vel = vel;
    g_brake.jerk = jerk;
    g_brake.press_steps_override = 0U;
    g_brake.press_steps_override_valid = 0U;
}

void Brake_UpdateHardwareProfileSteps(uint32_t steps, float acc, float vel, float jerk) {
    if (g_brake.motor == NULL) {
        return;
    }

    g_brake.pos = ((float)steps) / 100.0f;
    g_brake.acc = acc;
    g_brake.vel = vel;
    g_brake.jerk = jerk;
    g_brake.press_steps_override = steps;
    g_brake.press_steps_override_valid = 1U;
}

void Brake_RearmCommandState(void) {
    if (g_brake.motor == NULL) {
        return;
    }

    brake_reset_command_state();
    g_brake.trigger_lockout = 0U;
    g_brake.last_cmd = FC_Break_Reset;
}

void Accel_SetHardware(StepObject* motor, float pos, float acc, float vel, float jerk) {
    g_accel.motor = motor;
    g_accel.pos = pos;
    g_accel.acc = acc;
    g_accel.vel = vel;
    g_accel.jerk = jerk;
    g_accel.step = Accel_Step_Reset;
}

static uint8_t brake_is_motion_done(void) {
    return (g_brake.motor->Current_Phase == PHASE_IDLE || g_brake.motor->Current_Phase == PHASE_DONE);
}

static void brake_stop_motion(void) {
    MotorStop(g_brake.motor);
    g_brake.motion_started = 0U;
}

static void brake_reset_command_state(void) {
    brake_stop_motion();
    g_brake.step = Break_Step_Reset;
    g_brake.timer = 0U;
    g_brake.home_limit_since = 0U;
}

static uint8_t brake_is_press_command(Break_CMD_Typedef cmd) {
    return (uint8_t)((cmd == FC_Press_Break_Hold) || (cmd == FC_Trigger_Break));
}

static uint8_t brake_is_trigger_returning_home(Break_CMD_Typedef cmd) {
    return (uint8_t)(
        (cmd == FC_Trigger_Break) &&
        (g_brake.last_cmd == FC_Trigger_Break) &&
        (g_brake.step == Break_Step_Release)
    );
}

static void brake_clear_pending_cmd(void) {
    g_brake.pending_cmd = FC_Break_Step_IDLE;
    g_brake.pending_cmd_valid = 0U;
}

static void brake_queue_pending_cmd(Break_CMD_Typedef cmd) {
    g_brake.pending_cmd = cmd;
    g_brake.pending_cmd_valid = 1U;
}

static void brake_start_press_motion(void) {
    if (brake_is_motion_done()) {
        brake_run_press_profile();
        if (!brake_is_motion_done()) {
            g_brake.motion_started = 1U;
        }
    }
}

uint8_t Brake_IsHomeLimitActive(void) {
    return (uint8_t)(BreakLimit ? 1U : 0U);
}

static void brake_finish_home_sequence(uint8_t failed) {
    brake_stop_motion();
    g_brake.timer = 0U;
    g_brake.step = Break_Step_IDLE;
    g_brake.home_required = failed ? 1U : 0U;
    g_brake.home_failed = failed;
    if (failed != 0U) {
        g_brake.home_retry_tick = HAL_GetTick();
    } else {
        g_brake.home_retry_tick = 0U;
    }

    if (failed != 0U) {
        brake_clear_pending_cmd();
    }
}

static uint8_t brake_service_home_motion(Break_Action_Typedef home_step) {
    uint8_t raw_home_limit = Brake_IsHomeLimitActive();

    if (g_brake.step == Break_Step_Reset || g_brake.step == Break_Step_IDLE || g_brake.step != home_step) {
        g_brake.step = home_step;
        g_brake.timer = 0U;
        g_brake.home_limit_since = 0U;
        g_brake.motion_started = 0U;
    }

    if (g_brake.timer == 0U) {
        g_brake.timer = HAL_GetTick();
    }

    if (raw_home_limit != 0U) {
        if (g_brake.motion_started == 0U) {
            brake_finish_home_sequence(0U);
            return 1;
        }

        if (g_brake.motor->step_counter >= BRAKE_HOME_MIN_TRAVEL_STEPS) {
            if (g_brake.home_limit_since == 0U) {
                g_brake.home_limit_since = HAL_GetTick();
            }

            if ((HAL_GetTick() - g_brake.home_limit_since) >= BRAKE_HOME_LIMIT_DEBOUNCE_MS) {
                brake_finish_home_sequence(0U);
                return 1;
            }
        }
    } else {
        g_brake.home_limit_since = 0U;
    }

    if ((HAL_GetTick() - g_brake.timer) >= BRAKE_HOME_TIMEOUT_MS) {
        brake_finish_home_sequence(1U);
        return 1;
    }

    if (brake_is_motion_done()) {
        brake_run_home_profile();
        if (!brake_is_motion_done()) {
            g_brake.motion_started = 1U;
            g_brake.home_limit_since = 0U;
        }
    }

    return 0;
}

static void brake_keep_hold_torque(void) {
    EN_ENABLE(g_brake.motor);
}

static uint8_t brake_low_level(Break_CMD_Typedef cmd) {
    switch (cmd) {
        case FC_Break_Home:
            g_brake.home_required = 1U;
            return brake_service_home_motion(Break_Step_Home);

        case FC_Release_Break:
            g_brake.home_required = 1U;
            brake_clear_pending_cmd();
            return brake_service_home_motion(Break_Step_Release);

        case FC_Press_Break_Hold:
            if (brake_is_motion_done()) {
                brake_run_press_profile();
            }
            return 0U;

        case FC_Break_Step_IDLE:
        case FC_Break_Reset:
            brake_stop_motion();
            return 1U;

        default:
            return brake_is_motion_done();
    }
}


static uint8_t accel_low_level(Accel_CMD_Typedef cmd) {
    static Accel_CMD_Typedef current_low_cmd = FC_Accel_Reset;
    static uint32_t home_start_tick = 0;

    if (current_low_cmd != cmd) {
        current_low_cmd = cmd;
        if (cmd == FC_Accel_Home || cmd == FC_Accel_Release) {
            home_start_tick = HAL_GetTick();
        }
        MotorStop(g_accel.motor);

        if (cmd == FC_Accel_Press_Hold) {
            accel_run_press_profile();
        }
    }

    if (cmd == FC_Accel_Reset || cmd == FC_Accel_IDLE) {
        // MotorStop already called on transition in current_low_cmd != cmd block
    } else if (cmd == FC_Accel_Home || cmd == FC_Accel_Release) {
        static uint8_t flag_first_home = 1;

        if (AccLimit != 0) {
            flag_first_home = 1;
            MotorStop(g_accel.motor);
            return 1;
        }

        if ((HAL_GetTick() - home_start_tick) >= BRAKE_HOME_TIMEOUT_MS) {
            flag_first_home = 1;
            MotorStop(g_accel.motor);
            return 1;
        }

        if (AccLimit == 0) {
            if (flag_first_home == 1) {
                MotorStop(g_accel.motor);
                flag_first_home = 0;
            }

            if (g_accel.motor->Current_Phase == PHASE_IDLE || g_accel.motor->Current_Phase == PHASE_DONE) {
                accel_run_home_profile();
            }
            return 0;
        }
    } else if (cmd == FC_Accel_Press_Hold) {
        // MotorRun handled in transition block (line 134)
    }

    if (g_accel.motor->Current_Phase == PHASE_IDLE) {
        return 1;
    }

    return 0;
}

void Brake_Drive(Break_CMD_Typedef cmd) {
    Break_CMD_Typedef pending_cmd;

    if (!g_brake.motor) return;

    if (cmd == FC_Break_Home || cmd == FC_Release_Break || cmd == FC_Press_Break_Hold || cmd == FC_Break_Reset) {
        g_brake.trigger_lockout = 0U;
    }

    if (cmd == FC_Trigger_Break && g_brake.trigger_lockout != 0U && g_brake.step == Break_Step_IDLE) {
        brake_low_level(FC_Break_Step_IDLE);
        return;
    }

    if (cmd == FC_Break_Home) {
        brake_clear_pending_cmd();
        if (g_brake.home_failed != 0U) {
            if ((HAL_GetTick() - g_brake.home_retry_tick) < BRAKE_HOME_RETRY_MS) {
                return;
            }
            brake_reset_command_state();
            g_brake.last_cmd = FC_Break_Reset;
        }
        g_brake.home_failed = 0U;
    } else if (g_brake.home_required != 0U) {
        if (g_brake.home_failed != 0U) {
            if (brake_is_press_command(cmd) != 0U) {
                g_brake.home_required = 0U;
            } else if (cmd == FC_Release_Break) {
                cmd = FC_Break_Home;
                g_brake.home_failed = 0U;
            } else {
                if (cmd != FC_Break_Step_IDLE && cmd != FC_Break_Reset) {
                    brake_clear_pending_cmd();
                    g_brake.last_cmd = cmd;
                }
                return;
            }
        }

        if (g_brake.home_required != 0U &&
            brake_is_press_command(cmd) != 0U &&
            brake_is_trigger_returning_home(cmd) == 0U) {
            brake_queue_pending_cmd(cmd);
            cmd = FC_Break_Home;
        }
    }

    if (cmd != g_brake.last_cmd) {
        brake_reset_command_state();
        g_brake.last_cmd = cmd;
    }

    switch (cmd) {
        case FC_Break_Home:
            g_brake.home_required = 1U;
            brake_service_home_motion(Break_Step_Home);
            break;

        case FC_Press_Break_Hold:
            if (g_brake.step == Break_Step_Reset) {
                g_brake.step = Break_Step_Press_Poss;
                g_brake.motion_started = 0U;
                brake_start_press_motion();
            }
            if (0) {
                g_brake.step = Break_Step_Press_Poss;
                // Phát lệnh chạy BẮT ĐẦU từ đây
            }
            if (g_brake.step == Break_Step_Press_Poss) {
                if (g_brake.motion_started == 0U) {
                    brake_start_press_motion();
                    g_brake.timer = 0U;
                } else if (g_brake.motor->Current_Phase == PHASE_IDLE || g_brake.motor->Current_Phase == PHASE_DONE) {
                    if (g_brake.timer == 0) {
                        g_brake.timer = HAL_GetTick(); // Bắt đầu tính thời gian delay sau khi chạy xong
                    } else if (HAL_GetTick() - g_brake.timer > 100) {
                        g_brake.step = Break_Step_Hold;
                        g_brake.timer = 0;
                    }
                } else {
                    g_brake.timer = 0; // Đang chạy thì reset timer
                }
            }
            if (g_brake.step == Break_Step_Hold) {
                brake_keep_hold_torque();
            }
            break;

        case FC_Trigger_Break:
            if (g_brake.step == Break_Step_Reset) {
                g_brake.step = Break_Step_Press_Poss;
                g_brake.timer = 0;
                g_brake.motion_started = 0U;
                brake_start_press_motion();
            }
            if (0 && g_brake.step == Break_Step_Home && brake_low_level(FC_Break_Home)) {
                g_brake.step = Break_Step_Press_Poss;
                brake_low_level(FC_Press_Break_Hold); // ra lệnh đạp
                g_brake.timer = 0;
            }
            if (g_brake.step == Break_Step_Press_Poss) {
                if (g_brake.motion_started == 0U) {
                    brake_start_press_motion();
                    g_brake.timer = 0U;
                } else if (g_brake.motor->Current_Phase == PHASE_IDLE || g_brake.motor->Current_Phase == PHASE_DONE) {
                    if (g_brake.timer == 0) {
                        g_brake.timer = HAL_GetTick();
                    } else if ((HAL_GetTick() - g_brake.timer) >= BRAKE_TRIGGER_RELEASE_DELAY_MS) {
                        g_brake.step = Break_Step_Release;
                        g_brake.timer = 0;
                    }
                } else {
                    g_brake.timer = 0;
                }
            }
            if (g_brake.step == Break_Step_Release) {
                if (brake_service_home_motion(Break_Step_Release)) {
                    g_brake.step = Break_Step_IDLE;
                    g_brake.timer = 0;
                    g_brake.trigger_lockout = 1U;
                }
            }
            if (g_brake.step == Break_Step_IDLE) brake_low_level(FC_Break_Step_IDLE);
            break;

        case FC_Release_Break:
            brake_low_level(FC_Release_Break);
            break;

        case FC_Break_Reset:
            brake_clear_pending_cmd();
            g_brake.home_required = 1U;
            g_brake.home_failed = 0U;
            brake_reset_command_state();
            g_brake.last_cmd = cmd;
            break;

        case FC_Break_Step_IDLE:
            brake_stop_motion();
            break;

        default:
            brake_low_level(cmd);
            break;
    }

    if (g_brake.home_required == 0U && g_brake.home_failed == 0U && g_brake.pending_cmd_valid != 0U) {
        pending_cmd = g_brake.pending_cmd;
        brake_clear_pending_cmd();
        Brake_Drive(pending_cmd);
    }
}

uint8_t Brake_GetStep(void) {
    return (uint8_t)g_brake.step;
}

uint8_t Brake_HasHomeFailure(void) {
    return g_brake.home_failed;
}

uint8_t Brake_ServiceHardLimit(void) {
    if (!g_brake.motor) {
        return 0;
    }

    system_debug.brake_limit_dbg = Brake_IsHomeLimitActive();
    system_debug.brake_phase_dbg = (uint8_t)g_brake.motor->Current_Phase;
    system_debug.brake_step_dbg = (uint8_t)g_brake.step;
    system_debug.brake_cmd_dbg = (uint8_t)g_brake.last_cmd;

    if (Brake_IsHomeLimitActive() == 0U) {
        return 0;
    }

    if (g_brake.motor->Current_Phase == PHASE_IDLE || g_brake.motor->Current_Phase == PHASE_DONE) {
        return 0;
    }

    if (g_brake.motor->profile.direction != BRAKE_DIR_HOME) {
        return 0;
    }

    if (g_brake.step != Break_Step_Home && g_brake.step != Break_Step_Release) {
        return 0;
    }

    MotorStop(g_brake.motor);
    return 1;
}

void Gear_SetHardware(
    TIM_HandleTypeDef* htim,
    uint16_t pos_p,
    uint16_t pos_r,
    uint16_t pos_n,
    uint16_t pos_d,
    uint16_t home12,
    uint16_t home34,
    uint8_t s_p, uint8_t s_r, uint8_t s_n, uint8_t s_d
) {
    g_gear.htim = htim;
    g_gear.pos[FC_Gear_P] = pos_p;
    g_gear.pos[FC_Gear_R] = pos_r;
    g_gear.pos[FC_Gear_N] = pos_n;
    g_gear.pos[FC_Gear_D] = pos_d;
    g_gear.home12 = home12;
    g_gear.home34 = home34;
    g_gear.servo_assign[FC_Gear_P] = s_p;
    g_gear.servo_assign[FC_Gear_R] = s_r;
    g_gear.servo_assign[FC_Gear_N] = s_n;
    g_gear.servo_assign[FC_Gear_D] = s_d;
    g_gear.step = Gear_Step_Home;
}

static uint8_t gear_use_servo12(Gear_CMD_Typedef cmd) {
    if (cmd >= FC_Gear_Home) return 1; // Default to S1 for home if unsure
    return (g_gear.servo_assign[cmd] == 0); // 0 = Servo 1
}

static void gear_action_low(Gear_CMD_Typedef cmd) {
    uint16_t p12 = g_gear.home12;
    uint16_t p34 = g_gear.home34;

    if (cmd != FC_Gear_Home) {
        if (gear_use_servo12(cmd)) {
            p12 = g_gear.pos[cmd];
        } else {
            p34 = g_gear.pos[cmd];
        }
    }

    __HAL_TIM_SET_COMPARE(g_gear.htim, TIM_CHANNEL_3, p12);
    __HAL_TIM_SET_COMPARE(g_gear.htim, TIM_CHANNEL_4, p34);
}

void Gear_Drive(Gear_CMD_Typedef cmd) {
    if (!g_gear.htim) return;

    if (cmd != g_gear.last_cmd) {
        g_gear.last_cmd = cmd;
        g_gear.step = Gear_Step_Reset;
    }

    switch (g_gear.step) {
        case Gear_Step_Reset:
            g_gear.timer = HAL_GetTick();
            g_gear.step = (cmd == FC_Gear_Home) ? Gear_Step_IDLE : Gear_Step_Home;
            break;

        case Gear_Step_Home:
            gear_action_low(FC_Gear_Home);
            if (HAL_GetTick() - g_gear.timer >= 100) {
                g_gear.timer = HAL_GetTick();
                g_gear.step = Gear_Step_Press;
            }
            break;

        case Gear_Step_Press:
            gear_action_low(cmd);
            if (HAL_GetTick() - g_gear.timer >= 1800) g_gear.step = Gear_Step_IDLE;
            break;

        case Gear_Step_IDLE:
            gear_action_low(FC_Gear_Home);
            break;

        default:
            break;
    }
}

void Key_SetHardware(TIM_HandleTypeDef* htim, uint16_t unlock, uint16_t lock, uint16_t home, uint8_t time_press) {
    g_key.htim = htim;
    g_key.unlock = unlock;
    g_key.lock = lock;
    g_key.home = home;
    g_key.time_press = time_press;
    g_key.step = Key_Home;
}

void Key_Drive(Key_CMD_Typedef cmd) {
    if (!g_key.htim) return;

    if (cmd != g_key.last_cmd) {
        g_key.step = Key_Home;
        g_key.last_cmd = cmd;
    }

    switch (g_key.step) {
        case Key_Home:
            if (cmd == FC_Key_Unlock) {
                g_key.step = Key_Press_UNLOCK;
                g_key.timer = HAL_GetTick();
            } else if (cmd == FC_Key_Lock) {
                g_key.step = Key_Press_LOCK;
                g_key.timer = HAL_GetTick();
            } else {
                __HAL_TIM_SET_COMPARE(g_key.htim, TIM_CHANNEL_2, g_key.home);
            }
            break;

        case Key_Press_UNLOCK:
            __HAL_TIM_SET_COMPARE(g_key.htim, TIM_CHANNEL_2, g_key.unlock);
            if (HAL_GetTick() - g_key.timer >= g_key.time_press * 10) g_key.step = Key_Home_2;
            break;

        case Key_Press_LOCK:
            __HAL_TIM_SET_COMPARE(g_key.htim, TIM_CHANNEL_2, g_key.lock);
            if (HAL_GetTick() - g_key.timer >= g_key.time_press * 10) g_key.step = Key_Home_2;
            break;

        case Key_Home_2:
            __HAL_TIM_SET_COMPARE(g_key.htim, TIM_CHANNEL_2, g_key.home);
            break;

        default:
            break;
    }
}

void LV_Drive(LV_CMD_Typedef cmd) {
    // Stub for now to fix build
    (void)cmd;
}

void Robot_Drive(Robot_CMD_Typedef cmd) {
    // Stub for now to fix build
    (void)cmd;
}

void Accel_Drive(Accel_CMD_Typedef cmd) {
    if (!g_accel.motor) return;

    if (cmd != g_accel.last_cmd) {
        g_accel.step = Accel_Step_Reset;
        g_accel.last_cmd = cmd;
    }

    switch (cmd) {
        case FC_Accel_Home:
            if (g_accel.step == Accel_Step_Reset) g_accel.step = Accel_Step_Home;
            if (g_accel.step == Accel_Step_Home && accel_low_level(FC_Accel_Home)) g_accel.step = Accel_Step_IDLE;
            if (g_accel.step == Accel_Step_IDLE) accel_low_level(FC_Accel_IDLE);
            break;

        case FC_Accel_Press_Hold:
            if (g_accel.step == Accel_Step_Reset) {
                g_accel.step = Accel_Step_Press_Poss;
                accel_low_level(FC_Accel_Press_Hold); // Start move
            }
            if (g_accel.step == Accel_Step_Press_Poss) {
                if (g_accel.motor->Current_Phase == PHASE_IDLE || g_accel.motor->Current_Phase == PHASE_DONE) {
                    g_accel.step = Accel_Step_Hold;
                    g_accel.timer = HAL_GetTick();
                }
            }
            if (g_accel.step == Accel_Step_Hold) {
                // Keep the FC_Accel_Press_Hold active at low-level to maintain torque
                EN_ENABLE(g_accel.motor);
            }
            break;

        default:
            accel_low_level(cmd);
            break;
    }
}
