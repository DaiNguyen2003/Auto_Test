#include "Motor_Control.h"
#include <math.h>

static TIM_HandleTypeDef *g_htim_master = NULL;
static volatile uint32_t g_tick_counter = 0;
static bool g_initialized = false;

#define SCURVE_ABS_MIN_VELOCITY_HZ       90.0f
#define SCURVE_MAX_MIN_VELOCITY_HZ       180.0f
#define SCURVE_MIN_VELOCITY_RATIO        0.18f
#define SCURVE_STOP_BLEND_STEPS          80U
#define SCURVE_DIR_EN_SETTLE_MS          3U
#define SCURVE_FORWARD_START_RATIO       0.12f
#define SCURVE_FORWARD_ABS_START_HZ      80.0f
#define SCURVE_FORWARD_MAX_START_HZ      140.0f
#define SCURVE_FORWARD_ACC_SCALE         0.55f
#define SCURVE_FORWARD_JERK_SCALE        0.32f
#define SCURVE_REVERSE_START_RATIO       0.10f
#define SCURVE_REVERSE_ABS_START_HZ      70.0f
#define SCURVE_REVERSE_MAX_START_HZ      120.0f
#define SCURVE_REVERSE_ACC_SCALE         0.35f
#define SCURVE_REVERSE_JERK_SCALE        0.18f
#define SCURVE_REVERSE_SETTLE_MS         8U

static uint32_t get_timer_input_clock_hz(TIM_HandleTypeDef *htim) {
    RCC_ClkInitTypeDef clk_config = {0};
    uint32_t flash_latency = 0;
    uint32_t pclk_hz = HAL_RCC_GetPCLK1Freq();

    HAL_RCC_GetClockConfig(&clk_config, &flash_latency);

    if ((htim->Instance == TIM1) || (htim->Instance == TIM8) ||
        (htim->Instance == TIM16) || (htim->Instance == TIM17)) {
        pclk_hz = HAL_RCC_GetPCLK2Freq();
        if (clk_config.APB2CLKDivider != RCC_APB2_DIV1) {
            pclk_hz *= 2UL;
        }
    } else {
        if (clk_config.APB1CLKDivider != RCC_APB1_DIV1) {
            pclk_hz *= 2UL;
        }
    }

    return pclk_hz / (htim->Instance->PSC + 1UL);
}

// Helper to update Hardware PWM frequency (ARR)
static void update_motor_hardware_frequency(StepObject* motor, float freq_hz) {
    if (freq_hz < 10.0f) {
        // Stop PWM if frequency is too low
        HAL_TIM_PWM_Stop_IT(motor->Tim, motor->Channel);
        return;
    }

    // Timer clock is cached at start to avoid expensive RCC queries in the step ISR.
    uint32_t timer_clk = motor->timer_clk_hz;
    if (timer_clk == 0U) {
        timer_clk = get_timer_input_clock_hz(motor->Tim);
        motor->timer_clk_hz = timer_clk;
    }
    uint32_t arr = (uint32_t)(timer_clk / freq_hz) - 1;
    
    // Safety clamp
    if (arr < 10) arr = 10; 
    
    __HAL_TIM_SET_AUTORELOAD(motor->Tim, arr);
    __HAL_TIM_SET_COMPARE(motor->Tim, motor->Channel, arr / 2); // 50% duty cycle
}

static float scurve_compute_min_velocity(float max_velocity) {
    float min_velocity = max_velocity * SCURVE_MIN_VELOCITY_RATIO;

    if (min_velocity < SCURVE_ABS_MIN_VELOCITY_HZ) {
        min_velocity = SCURVE_ABS_MIN_VELOCITY_HZ;
    }
    if (min_velocity > SCURVE_MAX_MIN_VELOCITY_HZ) {
        min_velocity = SCURVE_MAX_MIN_VELOCITY_HZ;
    }
    if (min_velocity > max_velocity) {
        min_velocity = max_velocity;
    }

    return min_velocity;
}

static float scurve_compute_start_velocity(float max_velocity, bool direction_changed) {
    float start_velocity;

    if (!direction_changed) {
        start_velocity = max_velocity * SCURVE_FORWARD_START_RATIO;
        if (start_velocity < SCURVE_FORWARD_ABS_START_HZ) {
            start_velocity = SCURVE_FORWARD_ABS_START_HZ;
        }
        if (start_velocity > SCURVE_FORWARD_MAX_START_HZ) {
            start_velocity = SCURVE_FORWARD_MAX_START_HZ;
        }
        if (start_velocity > max_velocity) {
            start_velocity = max_velocity;
        }
        return start_velocity;
    }

    start_velocity = max_velocity * SCURVE_REVERSE_START_RATIO;
    if (start_velocity < SCURVE_REVERSE_ABS_START_HZ) {
        start_velocity = SCURVE_REVERSE_ABS_START_HZ;
    }
    if (start_velocity > SCURVE_REVERSE_MAX_START_HZ) {
        start_velocity = SCURVE_REVERSE_MAX_START_HZ;
    }
    if (start_velocity > max_velocity) {
        start_velocity = max_velocity;
    }

    return start_velocity;
}

static float scurve_compute_terminal_floor(const SCurveProfile_t *profile) {
    uint32_t remaining_steps = 0;

    if (profile->target_steps > profile->current_step) {
        remaining_steps = profile->target_steps - profile->current_step;
    }

    if (remaining_steps >= SCURVE_STOP_BLEND_STEPS) {
        return profile->min_velocity;
    }

    if (remaining_steps == 0U) {
        return 0.0f;
    }

    return profile->min_velocity * ((float)remaining_steps / (float)SCURVE_STOP_BLEND_STEPS);
}

static void SCurve_CalculateProfile(SCurveProfile_t *profile) {
    float v_max = profile->max_velocity;
    float a_max = profile->acceleration;
    float j = profile->jerk;
    float min_velocity = profile->min_velocity;

    profile->time_jerk = a_max / j;
    float v_jerk = 0.5f * j * profile->time_jerk * profile->time_jerk;
    profile->steps_jerk_acc = (uint32_t)(j * powf(profile->time_jerk, 3) / 6.0f);
    profile->steps_jerk_dec_acc = profile->steps_jerk_acc;

    float v_after_jerk = 2.0f * v_jerk;

    if (v_after_jerk >= v_max) {
        profile->time_jerk = sqrtf(v_max / j);
        profile->steps_jerk_acc = (uint32_t)(j * powf(profile->time_jerk, 3) / 6.0f);
        profile->steps_jerk_dec_acc = profile->steps_jerk_acc;
        profile->steps_acc_const = 0;
        v_after_jerk = v_max;
    } else {
        float t_acc_const = (v_max - v_after_jerk) / a_max;
        profile->steps_acc_const = (uint32_t)(v_after_jerk * t_acc_const +
            0.5f * a_max * t_acc_const * t_acc_const);
    }

    profile->steps_total_acc = profile->steps_jerk_acc + profile->steps_acc_const +
        profile->steps_jerk_dec_acc;
    profile->steps_total_dec = profile->steps_total_acc;

    if (profile->steps_total_acc + profile->steps_total_dec > profile->target_steps) {
        float new_vmax = profile->max_velocity * 0.8f;
        profile->max_velocity = new_vmax;
        profile->min_velocity = scurve_compute_min_velocity(profile->max_velocity);
        if (profile->start_velocity > profile->max_velocity) {
            profile->start_velocity = profile->max_velocity;
        }

        if (profile->max_velocity < min_velocity) {
            profile->max_velocity = min_velocity;
            profile->steps_total_acc = 0;
            profile->steps_total_dec = 0;
            profile->steps_jerk_acc = 0;
            profile->steps_acc_const = 0;
            profile->steps_jerk_dec_acc = 0;
            profile->steps_cruise = profile->target_steps;
            profile->current_step = 0;
            profile->current_velocity = profile->max_velocity;
            profile->current_acceleration = 0.0f;
            profile->phase = PHASE_CRUISE;
            return;
        }

        SCurve_CalculateProfile(profile);
        return;
    }

    profile->steps_cruise = profile->target_steps - profile->steps_total_acc -
        profile->steps_total_dec;
    profile->current_step = 0;
    profile->current_velocity = profile->start_velocity;
    profile->current_acceleration = 0.0f;
    profile->phase = PHASE_JERK_ACC;
}

static void SCurve_UpdateAfterStep(StepObject* motor, SCurveProfile_t *profile) {
    profile->current_step++;
    motor->step_counter++;

    // Calculate dynamic dT based on current velocity
    float actual_dt = 1.0f / profile->current_velocity;

    switch (profile->phase) {
        case PHASE_JERK_ACC:
            profile->current_acceleration += profile->jerk * actual_dt;
            if (profile->current_acceleration > profile->acceleration) {
                profile->current_acceleration = profile->acceleration;
            }
            profile->current_velocity += profile->current_acceleration * actual_dt;
            if (profile->current_step >= profile->steps_jerk_acc) {
                profile->phase = PHASE_ACC_CONST;
                profile->current_acceleration = profile->acceleration;
            }
            break;

        case PHASE_ACC_CONST:
            profile->current_velocity += profile->current_acceleration * actual_dt;
            if (profile->current_step >= profile->steps_jerk_acc + profile->steps_acc_const) {
                profile->phase = PHASE_JERK_DEC_ACC;
            }
            break;

        case PHASE_JERK_DEC_ACC:
            profile->current_acceleration -= profile->jerk * actual_dt;
            if (profile->current_acceleration < 0.0f) {
                profile->current_acceleration = 0.0f;
            }
            profile->current_velocity += profile->current_acceleration * actual_dt;
            if (profile->current_step >= profile->steps_total_acc) {
                profile->phase = PHASE_CRUISE;
                profile->current_acceleration = 0.0f;
                profile->current_velocity = profile->max_velocity;
            }
            break;

        case PHASE_CRUISE:
            if (profile->current_step >= profile->steps_total_acc + profile->steps_cruise) {
                profile->phase = PHASE_JERK_ACC_DEC;
                profile->current_acceleration = 0.0f;
            }
            break;

        case PHASE_JERK_ACC_DEC:
            profile->current_acceleration -= profile->jerk * actual_dt;
            if (profile->current_acceleration < -profile->acceleration) {
                profile->current_acceleration = -profile->acceleration;
            }
            profile->current_velocity += profile->current_acceleration * actual_dt;
            {
                float terminal_floor = scurve_compute_terminal_floor(profile);
                if (profile->current_velocity < terminal_floor) {
                    profile->current_velocity = terminal_floor;
                }
            }
            if (profile->current_step >= profile->steps_total_acc + profile->steps_cruise +
                profile->steps_jerk_acc) {
                profile->phase = PHASE_DEC_CONST;
                profile->current_acceleration = -profile->acceleration;
            }
            break;

        case PHASE_DEC_CONST:
            profile->current_velocity += profile->current_acceleration * actual_dt;
            {
                float terminal_floor = scurve_compute_terminal_floor(profile);
                if (profile->current_velocity < terminal_floor) {
                    profile->current_velocity = terminal_floor;
                }
            }
            if (profile->current_step >= profile->target_steps - profile->steps_jerk_dec_acc) {
                profile->phase = PHASE_JERK_DEC_DEC;
            }
            break;

        case PHASE_JERK_DEC_DEC:
            profile->current_acceleration += profile->jerk * actual_dt;
            if (profile->current_acceleration > 0.0f) {
                profile->current_acceleration = 0.0f;
            }
            profile->current_velocity += profile->current_acceleration * actual_dt;
            {
                float terminal_floor = scurve_compute_terminal_floor(profile);
                if (profile->current_velocity < terminal_floor) {
                    profile->current_velocity = terminal_floor;
                }
            }
            if (profile->current_step >= profile->target_steps) {
                profile->phase = PHASE_DONE;
                profile->current_velocity = 0.0f;
                profile->current_acceleration = 0.0f;
            }
            break;

        default:
            break;
    }

    motor->Current_Phase = profile->phase;
    if (profile->phase != PHASE_DONE) {
        update_motor_hardware_frequency(motor, profile->current_velocity);
    } else {
        HAL_TIM_PWM_Stop_IT(motor->Tim, motor->Channel);
    }
}

void SCurve_Init(TIM_HandleTypeDef *htim) {
    g_htim_master = htim;
    g_tick_counter = 0;
    g_initialized = true;
    HAL_TIM_Base_Start_IT(htim);
}

bool SCurve_Start(StepObject* Step_Obj, uint32_t steps, float max_vel, float acc, float jerk,
    MotorDirection_e dir) {
    if (!g_initialized) return false;
    bool direction_changed = (Step_Obj->has_direction_history != 0U) &&
        (Step_Obj->last_direction != dir);
    float acc_scale = direction_changed ? SCURVE_REVERSE_ACC_SCALE : SCURVE_FORWARD_ACC_SCALE;
    float jerk_scale = direction_changed ? SCURVE_REVERSE_JERK_SCALE : SCURVE_FORWARD_JERK_SCALE;
    uint32_t settle_delay_ms = direction_changed ? SCURVE_REVERSE_SETTLE_MS : SCURVE_DIR_EN_SETTLE_MS;
    
    SCurveProfile_t *profile = &Step_Obj->profile;
    if (profile->phase != PHASE_IDLE && profile->phase != PHASE_DONE) {
        return false;
    }

    profile->direction = dir;
    if (dir == DIR_CW) {
        DIR_CW_SET(Step_Obj);
    } else {
        DIR_CCW_SET(Step_Obj);
    }

    profile->target_steps = steps;
    profile->max_velocity = max_vel;
    profile->acceleration = acc * acc_scale;
    profile->jerk = jerk * jerk_scale;
    profile->min_velocity = scurve_compute_min_velocity(max_vel);
    profile->start_velocity = scurve_compute_start_velocity(max_vel, direction_changed);

    if (profile->acceleration < 1.0f) {
        profile->acceleration = 1.0f;
    }
    if (profile->jerk < 1.0f) {
        profile->jerk = 1.0f;
    }
    if (profile->start_velocity > profile->max_velocity) {
        profile->start_velocity = profile->max_velocity;
    }

    profile->current_step = 0;
    profile->current_velocity = profile->start_velocity;
    profile->current_acceleration = 0.0f;
    
    Step_Obj->step_counter = 0;
    Step_Obj->target_steps = steps;
    Step_Obj->last_direction = dir;
    Step_Obj->has_direction_history = 1U;
    if (Step_Obj->timer_clk_hz == 0U) {
        Step_Obj->timer_clk_hz = get_timer_input_clock_hz(Step_Obj->Tim);
    }

    SCurve_CalculateProfile(profile);
    EN_ENABLE(Step_Obj);
    HAL_Delay(settle_delay_ms);
    
    // Start Hardware PWM
    update_motor_hardware_frequency(Step_Obj, profile->current_velocity);
    __HAL_TIM_SET_COUNTER(Step_Obj->Tim, 0);
    Step_Obj->Tim->Instance->EGR = TIM_EGR_UG;
    HAL_TIM_PWM_Start_IT(Step_Obj->Tim, Step_Obj->Channel);
    
    Step_Obj->Current_Phase = profile->phase;
    return true;
}

void SCurve_Stop(StepObject* motor) {
    HAL_TIM_PWM_Stop_IT(motor->Tim, motor->Channel);
    SCurveProfile_t *profile = &motor->profile;
    profile->phase = PHASE_DONE;
    profile->current_velocity = 0.0f;
    profile->current_acceleration = 0.0f;
    motor->Current_Phase = PHASE_DONE;
}

bool SCurve_IsRunning(StepObject* motor) {
    return (motor->profile.phase != PHASE_IDLE && motor->profile.phase != PHASE_DONE);
}

void SCurve_GetStatus(StepObject* motor, uint32_t *current_step, float *current_velocity, SCurvePhase_e *phase) {
    SCurveProfile_t *profile = &motor->profile;
    if (current_step) *current_step = profile->current_step;
    if (current_velocity) *current_velocity = profile->current_velocity;
    if (phase) *phase = profile->phase;
}

void SCurve_MasterTickHandler(void) {
    g_tick_counter++;
}

// THIS IS NOW CALLED BY HAL_TIM_PWM_PulseFinishedCallback IN main.c
void SCurve_TimerISR(StepObject* motor) {
    SCurve_UpdateAfterStep(motor, &motor->profile);
}

void SCurve_Process(StepObject* motor) {
    // Hardware PWM handles the process now.
}

void MotorRun(StepObject* motor, uint32_t steps, float acc, float vel, float jerk, MotorDirection_e dir) {
    if (!SCurve_Start(motor, steps, vel, acc, jerk, dir)) {
        return;
    }
}

void MotorStop(StepObject* motor) {
    SCurve_Stop(motor);
}

uint32_t Convert_Poss_to_Step(float poss) { return (uint32_t)(poss * 100.0f); }
float Convert_Acc_to_Step(float acc) { return acc * 100.0f; }
float Convert_Vel_to_Step(float vel) { return vel * 100.0f; }
