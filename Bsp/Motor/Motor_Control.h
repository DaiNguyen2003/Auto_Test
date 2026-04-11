/*
* Motor_Control.h
* Scurve profile Ver 2
* HungDK
*/

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"  
#include "app_config.h"
#include <stdint.h>
#include <stdbool.h>

#define TIMER_FREQ_HZ       1000000UL   // Raised to 1MHz for nanosecond precision in frequency modulation
#define STEP_PULSE_WIDTH_US 5

#define STEP_HIGH(motor)    HAL_GPIO_WritePin((motor)->GPIO_PORT_STEP, (motor)->GPIO_PIN_STEP, GPIO_PIN_SET)
#define STEP_LOW(motor)     HAL_GPIO_WritePin((motor)->GPIO_PORT_STEP, (motor)->GPIO_PIN_STEP, GPIO_PIN_RESET)
#define DIR_CW_SET(motor)   HAL_GPIO_WritePin((motor)->GPIO_PORT_DIR, (motor)->GPIO_PIN_DIR, GPIO_PIN_RESET)
#define DIR_CCW_SET(motor)  HAL_GPIO_WritePin((motor)->GPIO_PORT_DIR, (motor)->GPIO_PIN_DIR, GPIO_PIN_SET)
#define EN_HIGH(motor)      HAL_GPIO_WritePin((motor)->GPIO_PORT_EN, (motor)->GPIO_PIN_EN, GPIO_PIN_SET)
#define EN_LOW(motor)       HAL_GPIO_WritePin((motor)->GPIO_PORT_EN, (motor)->GPIO_PIN_EN, GPIO_PIN_RESET)

#if STEPPER_ENABLE_ACTIVE_HIGH
#define EN_ENABLE(motor)    EN_HIGH(motor)
#define EN_DISABLE(motor)   EN_LOW(motor)
#else
#define EN_ENABLE(motor)    EN_LOW(motor)
#define EN_DISABLE(motor)   EN_HIGH(motor)
#endif


typedef enum {
    PHASE_IDLE = 0,
    PHASE_JERK_ACC,      
    PHASE_ACC_CONST,     
    PHASE_JERK_DEC_ACC,  
    PHASE_CRUISE,        
    PHASE_JERK_ACC_DEC,  
    PHASE_DEC_CONST,     
    PHASE_JERK_DEC_DEC,  
    PHASE_DONE
} SCurvePhase_e;

typedef enum {
    DIR_CW = 0,   
    DIR_CCW = 1   
} MotorDirection_e;

// Define Direction as an alias for legacy code
typedef MotorDirection_e Direction;

typedef struct {
    uint32_t target_steps;      
    float max_velocity;         
    float acceleration;         
    float jerk;                 
    MotorDirection_e direction; 
    
    uint32_t current_step;      
    float current_velocity;     
    float current_acceleration; 
    SCurvePhase_e phase;        
    
    uint32_t steps_jerk_acc;       
    uint32_t steps_acc_const;      
    uint32_t steps_jerk_dec_acc;   
    uint32_t steps_cruise;         
    uint32_t steps_total_acc;      
    uint32_t steps_total_dec;      
    
    float time_jerk;               
    float min_velocity;
    float start_velocity;
    
    uint32_t step_interval_ticks;  
    uint32_t next_step_tick;       
    bool step_ready;               
    
    // Status flags cho ISR xử lý rời rạc
    bool step_due_this_tick;
    bool advanced_this_tick;
} SCurveProfile_t;

typedef struct {
    TIM_HandleTypeDef* Tim;       // Hardware Timer for PWM/Pulses
    uint32_t           Channel;   // Timer Channel (e.g. TIM_CHANNEL_2)
    
    GPIO_TypeDef* GPIO_PORT_STEP; 
    GPIO_TypeDef* GPIO_PORT_DIR; 
    GPIO_TypeDef* GPIO_PORT_EN; 
    uint16_t GPIO_PIN_STEP;       
    uint16_t GPIO_PIN_DIR;       
    uint16_t GPIO_PIN_EN;       

    uint8_t Flag_One_Time;        
    volatile SCurvePhase_e Current_Phase; 
    
    volatile uint32_t step_counter;      // Hardware pulse counter
    volatile uint32_t target_steps;      // End point candidate
    
    uint8_t has_direction_history;
    MotorDirection_e last_direction;
    uint32_t timer_clk_hz;

    SCurveProfile_t profile;     
}StepObject;

void SCurve_Init(TIM_HandleTypeDef *htim);
bool SCurve_Start(StepObject* Step_Obj, uint32_t steps, float max_vel, float acc, float jerk, MotorDirection_e dir);
void SCurve_Stop(StepObject* motor);
bool SCurve_IsRunning(StepObject* motor);
void SCurve_GetStatus(StepObject* motor, uint32_t *current_step, float *current_velocity, SCurvePhase_e *phase);
void SCurve_MasterTickHandler(void);
void SCurve_TimerISR(StepObject* motor);
void SCurve_Process(StepObject* motor);

// Legacy wrappers
void MotorRun(StepObject* motor, uint32_t steps, float acc, float vel, float jerk, MotorDirection_e dir);
void MotorStop(StepObject* motor);

#endif
