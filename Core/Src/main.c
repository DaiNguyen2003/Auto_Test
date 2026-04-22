/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "bsp.h"
#include "app_main.h"
#include "common.h"
#include "IO_Control.h"
#include "Motor_Control.h"
#include "Car.h"
#include "CanBus.h"
#include "RS485.h"
#include "Modbus_RTU.h"
#include "Debug_UART.h"
#include "app_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if MAIN_MOTOR_TEST_MODE
typedef enum {
    MOTOR_TEST_STATE_RUN_FORWARD = 0,
    MOTOR_TEST_STATE_PAUSE_AFTER_FORWARD,
    MOTOR_TEST_STATE_RUN_REVERSE,
    MOTOR_TEST_STATE_PAUSE_AFTER_REVERSE
} MotorTestState_t;
#endif

#if MAIN_BRAKE_SEQUENCE_TEST_MODE
typedef enum {
    BRAKE_CALIB_STATE_IDLE = 0,
    BRAKE_CALIB_STATE_HOME,
    BRAKE_CALIB_STATE_TRIGGER,
    BRAKE_CALIB_STATE_MOVE,
    BRAKE_CALIB_STATE_HOLD,
    BRAKE_CALIB_STATE_FAIL
} BrakeCalibState_t;
#endif

uint8_t Flag_CAN_Tx = 0;
uint32_t cycle_count = 0;
char lcd_buffer[100];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_TEST_MOTOR_OBJ       Motor1
#define MOTOR_TEST_PWM_RUN_HZ      600U
#define MOTOR_TEST_RUN_TIME_MS     3000U
#define MOTOR_TEST_PAUSE_TIME_MS   5000U
#define MOTOR_TEST_DIR_SETTLE_MS   100U
#define BRAKE_CALIB_DEFAULT_TARGET_STEPS 17778U
#define BRAKE_CALIB_MAX_TARGET_STEPS 40000U
#define BRAKE_CALIB_DEFAULT_REF_MM    90.0f
#define BRAKE_CALIB_DEFAULT_CAPTURED_STEPS 40000U
#define BRAKE_CALIB_DEFAULT_STEPS_PER_MM  (40000.0f / 90.0f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t now = 0;
#if MAIN_MOTOR_TEST_MODE
static MotorTestState_t g_motor_test_state = MOTOR_TEST_STATE_RUN_FORWARD;
static uint32_t g_motor_test_tick = 0U;
static uint32_t g_motor_test_last_move_tick = 0U;
static uint32_t g_motor_test_cycle = 0U;
static uint32_t g_motor_test_timer_clk_hz = 0U;
static MotorDirection_e g_motor_test_direction = DIR_CCW;
#endif
#if MAIN_BRAKE_SEQUENCE_TEST_MODE
static BrakeCalibState_t g_brake_calib_state = BRAKE_CALIB_STATE_IDLE;
static uint32_t g_brake_calib_last_target_steps = 0xFFFFFFFFU;
#endif
volatile uint8_t brake_sequence_state_dbg = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if MAIN_MOTOR_TEST_MODE || MAIN_BRAKE_SEQUENCE_TEST_MODE
static void Drain_CanRxQueue(void)
{
  CAN_RxFrame_t frame;

  while (CANBus_PopAnyRxFrame(&frame) != 0U) {
    // In standalone test modes we only need to drain raw frames
    // so the ISR queue does not overflow.
  }
}
#endif

 #if MAIN_BRAKE_SEQUENCE_TEST_MODE
static uint32_t Brake_CalibClampTargetSteps(uint32_t target_steps)
{
  if (target_steps < 1U) {
    target_steps = 1U;
  } else if (target_steps > BRAKE_CALIB_MAX_TARGET_STEPS) {
    target_steps = BRAKE_CALIB_MAX_TARGET_STEPS;
  }

  return target_steps;
}

static float Brake_CalibEstimateMm(uint32_t target_steps)
{
  float steps_per_mm = system_debug.brake_calib_steps_per_mm;

  if (steps_per_mm < 1.0f) {
    steps_per_mm = BRAKE_CALIB_DEFAULT_STEPS_PER_MM;
  }

  return ((float)target_steps) / steps_per_mm;
}

static void Brake_CalibSetStateName(BrakeCalibState_t state)
{
  const char *state_name = "IDLE";

  switch (state) {
    case BRAKE_CALIB_STATE_HOME:
      state_name = "HOME";
      break;

    case BRAKE_CALIB_STATE_TRIGGER:
      state_name = "TRIG";
      break;

    case BRAKE_CALIB_STATE_MOVE:
      state_name = "MOVE";
      break;

    case BRAKE_CALIB_STATE_HOLD:
      state_name = "HOLD";
      break;

    case BRAKE_CALIB_STATE_FAIL:
      state_name = "FAIL";
      break;

    default:
      break;
  }

  snprintf(system_debug.test_state_name, sizeof(system_debug.test_state_name), "%s", state_name);
}

static void Brake_CalibApplyTarget(uint32_t target_steps)
{
  Car_Define_Typedef *car = Car_GetActiveConfig();

  if (car == NULL) {
    return;
  }

  target_steps = Brake_CalibClampTargetSteps(target_steps);
  system_debug.brake_calib_target_steps = target_steps;
  system_debug.manual_brake_pos = Brake_CalibEstimateMm(target_steps);
  system_debug.manual_brake_acc = car->BreakCmd.BreakAcc;
  system_debug.manual_brake_vel = car->BreakCmd.BreakVel;

  Brake_UpdateHardwareProfileSteps(target_steps,
                                   car->BreakCmd.BreakAcc,
                                   car->BreakCmd.BreakVel,
                                   car->BreakCmd.BreakJerk);
}

static void Brake_CalibCaptureCurrentSteps(void)
{
  float ref_mm = system_debug.brake_calib_ref_mm;

  if (ref_mm < 1.0f) {
    ref_mm = BRAKE_CALIB_DEFAULT_REF_MM;
    system_debug.brake_calib_ref_mm = ref_mm;
  }

  system_debug.brake_calib_captured_steps = Motor1.step_counter;
  system_debug.brake_calib_steps_per_mm = ((float)system_debug.brake_calib_captured_steps) / ref_mm;
  system_debug.manual_brake_pos = Brake_CalibEstimateMm(system_debug.brake_calib_target_steps);
}

static void Brake_CalibSyncDebug(void)
{
  uint8_t brake_step = Brake_GetStep();

  system_debug.rem_seconds = 0U;
  brake_sequence_state_dbg = (uint8_t)g_brake_calib_state;
  system_debug.brake_calib_state_dbg = (uint8_t)g_brake_calib_state;
  system_debug.brake_test_stage_dbg = (uint8_t)g_brake_calib_state;
  system_debug.brake_limit_dbg = Brake_IsHomeLimitActive();
  system_debug.brake_cmd_dbg = system_debug.brake_calib_cmd_req;
  system_debug.brake_test_cmd_req = system_debug.brake_calib_cmd_req;
  system_debug.manual_brake_pos = Brake_CalibEstimateMm(system_debug.brake_calib_target_steps);
  system_debug.brake_phase_dbg = (uint8_t)Motor1.Current_Phase;
  system_debug.brake_pos = Motor1.step_counter;
  system_debug.accel_pos = Motor1.target_steps;
  system_debug.brake_step_dbg = brake_step;
  system_debug.test_step = (uint8_t)g_brake_calib_state;
  system_debug.test_status = (Brake_HasHomeFailure() != 0U) ? 2U :
                             ((system_debug.brake_calib_cmd_req != 0U) ? 1U : 0U);
  Brake_CalibSetStateName(g_brake_calib_state);
}
#endif

#if MAIN_MOTOR_TEST_MODE
static uint32_t Motor_TestGetTimerClockHz(TIM_HandleTypeDef *htim)
{
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
  } else if (clk_config.APB1CLKDivider != RCC_APB1_DIV1) {
    pclk_hz *= 2UL;
  }

  return pclk_hz / (htim->Instance->PSC + 1UL);
}

static void Motor_TestStepPinToGpioIdle(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_TIM_PWM_Stop_IT(MOTOR_TEST_MOTOR_OBJ.Tim, MOTOR_TEST_MOTOR_OBJ.Channel);

  GPIO_InitStruct.Pin = MOTOR_TEST_MOTOR_OBJ.GPIO_PIN_STEP;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MOTOR_TEST_MOTOR_OBJ.GPIO_PORT_STEP, &GPIO_InitStruct);
  STEP_LOW(&MOTOR_TEST_MOTOR_OBJ);
}

static void Motor_TestStepPinToTimerAf(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = MOTOR_TEST_MOTOR_OBJ.GPIO_PIN_STEP;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(MOTOR_TEST_MOTOR_OBJ.GPIO_PORT_STEP, &GPIO_InitStruct);
}

static void Motor_TestApplyFrequency(uint32_t freq_hz)
{
  uint32_t arr;

  if (freq_hz < 10U) {
    freq_hz = 10U;
  }

  if (g_motor_test_timer_clk_hz == 0U) {
    g_motor_test_timer_clk_hz = Motor_TestGetTimerClockHz(MOTOR_TEST_MOTOR_OBJ.Tim);
  }

  arr = (g_motor_test_timer_clk_hz / freq_hz) - 1U;
  if (arr < 10U) {
    arr = 10U;
  }

  __HAL_TIM_SET_AUTORELOAD(MOTOR_TEST_MOTOR_OBJ.Tim, arr);
  __HAL_TIM_SET_COMPARE(MOTOR_TEST_MOTOR_OBJ.Tim, MOTOR_TEST_MOTOR_OBJ.Channel, arr / 2U);
  __HAL_TIM_SET_COUNTER(MOTOR_TEST_MOTOR_OBJ.Tim, 0U);
  MOTOR_TEST_MOTOR_OBJ.Tim->Instance->EGR = TIM_EGR_UG;
}

static void Motor_TestSyncDebug(void)
{
  uint32_t tick_now = HAL_GetTick();

  system_debug.motor1_diag_enable = 1U;
  system_debug.motor1_diag_mode = 2U;
  system_debug.motor1_diag_state = (uint8_t)g_motor_test_state;
  system_debug.motor1_diag_dir = (uint8_t)g_motor_test_direction;
  system_debug.motor1_diag_phase = (uint8_t)MOTOR_TEST_MOTOR_OBJ.Current_Phase;
  system_debug.motor1_diag_en_level =
      (uint8_t)HAL_GPIO_ReadPin(MOTOR_TEST_MOTOR_OBJ.GPIO_PORT_EN, MOTOR_TEST_MOTOR_OBJ.GPIO_PIN_EN);
  system_debug.motor1_diag_step_level =
      (uint8_t)HAL_GPIO_ReadPin(MOTOR_TEST_MOTOR_OBJ.GPIO_PORT_STEP, MOTOR_TEST_MOTOR_OBJ.GPIO_PIN_STEP);
  system_debug.motor1_diag_step_active_low = 0U;
  system_debug.motor1_diag_motion_seen = (MOTOR_TEST_MOTOR_OBJ.step_counter > 0U) ? 1U : 0U;
  system_debug.motor1_diag_step = MOTOR_TEST_MOTOR_OBJ.step_counter;
  system_debug.motor1_diag_cycle = g_motor_test_cycle;
  system_debug.motor1_diag_idle_ms = tick_now - g_motor_test_tick;
  system_debug.motor1_diag_last_move_ms = tick_now - g_motor_test_last_move_tick;
  system_debug.accel_pos = MOTOR_TEST_MOTOR_OBJ.step_counter;
}

static void Motor_TestStop(void)
{
  HAL_TIM_PWM_Stop_IT(MOTOR_TEST_MOTOR_OBJ.Tim, MOTOR_TEST_MOTOR_OBJ.Channel);
  Motor_TestStepPinToGpioIdle();
  EN_DISABLE(&MOTOR_TEST_MOTOR_OBJ);
  MOTOR_TEST_MOTOR_OBJ.Current_Phase = PHASE_DONE;
  MOTOR_TEST_MOTOR_OBJ.profile.phase = PHASE_DONE;
  HAL_GPIO_WritePin(Led_W_GPIO_Port, Led_W_Pin, GPIO_PIN_RESET);
  Motor_TestSyncDebug();
}

static void Motor_TestStart(MotorDirection_e dir)
{
  Motor_TestStepPinToGpioIdle();
  if (dir == DIR_CW) {
    DIR_CW_SET(&MOTOR_TEST_MOTOR_OBJ);
  } else {
    DIR_CCW_SET(&MOTOR_TEST_MOTOR_OBJ);
  }
  g_motor_test_direction = dir;
  EN_ENABLE(&MOTOR_TEST_MOTOR_OBJ);
  HAL_GPIO_WritePin(Led_W_GPIO_Port, Led_W_Pin,
                    (dir == DIR_CW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  g_motor_test_tick = HAL_GetTick();
  MOTOR_TEST_MOTOR_OBJ.Current_Phase = PHASE_CRUISE;
  MOTOR_TEST_MOTOR_OBJ.profile.phase = PHASE_CRUISE;
  HAL_Delay(MOTOR_TEST_DIR_SETTLE_MS);
  Motor_TestStepPinToTimerAf();
  Motor_TestApplyFrequency(MOTOR_TEST_PWM_RUN_HZ);
  HAL_TIM_PWM_Start_IT(MOTOR_TEST_MOTOR_OBJ.Tim, MOTOR_TEST_MOTOR_OBJ.Channel);
  Motor_TestSyncDebug();
}

static void Motor_TestInit(void)
{
  Motor_TestStepPinToGpioIdle();
  MOTOR_TEST_MOTOR_OBJ.Current_Phase = PHASE_DONE;
  MOTOR_TEST_MOTOR_OBJ.profile.phase = PHASE_DONE;
  MOTOR_TEST_MOTOR_OBJ.has_direction_history = 0U;
  MOTOR_TEST_MOTOR_OBJ.step_counter = 0U;
  MOTOR_TEST_MOTOR_OBJ.target_steps = 0U;
  g_motor_test_state = MOTOR_TEST_STATE_RUN_FORWARD;
  g_motor_test_timer_clk_hz = Motor_TestGetTimerClockHz(MOTOR_TEST_MOTOR_OBJ.Tim);
  g_motor_test_last_move_tick = HAL_GetTick();
  g_motor_test_cycle = 0U;
  Motor_TestStart(DIR_CCW);
}

static void Motor_TestUpdate(void)
{
  uint32_t tick_now = HAL_GetTick();

  switch (g_motor_test_state) {
    case MOTOR_TEST_STATE_RUN_FORWARD:
      if ((tick_now - g_motor_test_tick) >= MOTOR_TEST_RUN_TIME_MS) {
        Motor_TestStop();
        g_motor_test_tick = tick_now;
        g_motor_test_state = MOTOR_TEST_STATE_PAUSE_AFTER_FORWARD;
      }
      break;

    case MOTOR_TEST_STATE_PAUSE_AFTER_FORWARD:
      if ((tick_now - g_motor_test_tick) >= MOTOR_TEST_PAUSE_TIME_MS) {
        Motor_TestStart(DIR_CW);
        g_motor_test_state = MOTOR_TEST_STATE_RUN_REVERSE;
      }
      break;

    case MOTOR_TEST_STATE_RUN_REVERSE:
      if ((tick_now - g_motor_test_tick) >= MOTOR_TEST_RUN_TIME_MS) {
        Motor_TestStop();
        g_motor_test_tick = tick_now;
        g_motor_test_state = MOTOR_TEST_STATE_PAUSE_AFTER_REVERSE;
      }
      break;

    case MOTOR_TEST_STATE_PAUSE_AFTER_REVERSE:
      if ((tick_now - g_motor_test_tick) >= MOTOR_TEST_PAUSE_TIME_MS) {
        Motor_TestStart(DIR_CCW);
        g_motor_test_state = MOTOR_TEST_STATE_RUN_FORWARD;
        g_motor_test_cycle++;
      }
      break;

    default:
      break;
  }

  Motor_TestSyncDebug();
}
#endif

#if MAIN_BRAKE_SEQUENCE_TEST_MODE
static void Brake_SequenceInit(void)
{
  snprintf(system_debug.current_test_name, sizeof(system_debug.current_test_name), "%s", "BRK_CAL");
  system_debug.brake_calib_cmd_req = 0U;
  system_debug.brake_calib_capture_req = 0U;
  system_debug.brake_calib_state_dbg = (uint8_t)BRAKE_CALIB_STATE_IDLE;
  system_debug.brake_calib_captured_steps = BRAKE_CALIB_DEFAULT_CAPTURED_STEPS;
  system_debug.brake_calib_ref_mm = BRAKE_CALIB_DEFAULT_REF_MM;
  system_debug.brake_calib_steps_per_mm = BRAKE_CALIB_DEFAULT_STEPS_PER_MM;
  system_debug.brake_calib_target_steps = BRAKE_CALIB_DEFAULT_TARGET_STEPS;
  system_debug.manual_brake_pos = Brake_CalibEstimateMm(system_debug.brake_calib_target_steps);
  g_brake_calib_state = BRAKE_CALIB_STATE_IDLE;
  g_brake_calib_last_target_steps = 0xFFFFFFFFU;
  Brake_CalibApplyTarget(system_debug.brake_calib_target_steps);
  g_brake_calib_last_target_steps = system_debug.brake_calib_target_steps;
  Brake_CalibSyncDebug();
}

static void Brake_SequenceUpdate(void)
{
  uint8_t cmd_req = system_debug.brake_calib_cmd_req;
  uint32_t target_steps = Brake_CalibClampTargetSteps(system_debug.brake_calib_target_steps);

  if (cmd_req > 3U) {
    cmd_req = 0U;
    system_debug.brake_calib_cmd_req = 0U;
  }

  if (system_debug.brake_calib_ref_mm < 1.0f) {
    system_debug.brake_calib_ref_mm = BRAKE_CALIB_DEFAULT_REF_MM;
  }

  if (system_debug.brake_calib_steps_per_mm < 1.0f) {
    system_debug.brake_calib_steps_per_mm = BRAKE_CALIB_DEFAULT_STEPS_PER_MM;
  }

  if (target_steps != system_debug.brake_calib_target_steps) {
    system_debug.brake_calib_target_steps = target_steps;
  }

  if (target_steps != g_brake_calib_last_target_steps) {
    Brake_CalibApplyTarget(target_steps);
    g_brake_calib_last_target_steps = target_steps;

    if (cmd_req == 2U || cmd_req == 3U) {
      Brake_RearmCommandState();
    }
  }

  switch (cmd_req) {
    case 1U:
      Brake_Drive(FC_Break_Home);
      g_brake_calib_state = (Brake_HasHomeFailure() != 0U) ? BRAKE_CALIB_STATE_FAIL : BRAKE_CALIB_STATE_HOME;
      break;

    case 2U:
      Brake_Drive(FC_Trigger_Break);
      g_brake_calib_state = (Brake_HasHomeFailure() != 0U) ? BRAKE_CALIB_STATE_FAIL : BRAKE_CALIB_STATE_TRIGGER;
      break;

    case 3U:
      Brake_Drive(FC_Press_Break_Hold);
      if (Brake_HasHomeFailure() != 0U) {
        g_brake_calib_state = BRAKE_CALIB_STATE_FAIL;
      } else if (Brake_GetStep() == (uint8_t)Break_Step_Hold) {
        g_brake_calib_state = BRAKE_CALIB_STATE_HOLD;
      } else {
        g_brake_calib_state = BRAKE_CALIB_STATE_MOVE;
      }
      break;

    default:
      Brake_Drive(FC_Break_Step_IDLE);
      g_brake_calib_state = (Brake_HasHomeFailure() != 0U) ? BRAKE_CALIB_STATE_FAIL : BRAKE_CALIB_STATE_IDLE;
      break;
  }

  system_debug.brake_test_cmd_req = cmd_req;

  if (system_debug.brake_calib_capture_req != 0U) {
    Brake_CalibCaptureCurrentSteps();
    system_debug.brake_calib_capture_req = 0U;
  }

  Brake_CalibSyncDebug();
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART7_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  // Keep CAN/FDCAN sniffing alive in every runtime mode, including motor tests.
  CANBus_Start_Config();

  HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 0);
  HAL_GPIO_WritePin(Relay_1_GPIO_Port,  Relay_1_Pin, 0);
  HAL_GPIO_WritePin(Relay_2_GPIO_Port, Relay_2_Pin, 0);
  HAL_GPIO_WritePin(Relay_3_GPIO_Port, Relay_3_Pin, 0);
  HAL_GPIO_WritePin(Relay_4_GPIO_Port, Relay_4_Pin, 0);
  HAL_GPIO_WritePin(Relay_5_GPIO_Port, Relay_5_Pin, 0);
  HAL_GPIO_WritePin(Relay_6_GPIO_Port, Relay_6_Pin, 0);


  // HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
  SCurve_Init(&htim8);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);



#if !MAIN_MOTOR_TEST_MODE && !MAIN_BRAKE_SEQUENCE_TEST_MODE
  CLCD_I2C_Init(&LCD,&hi2c2,0x4e,20,4);
  CLCD_I2C_Clear(&LCD);

  // Khởi tạo toàn bộ Logic nghiệp vụ App (bao gồm Hardware_Control_Init và Motor Setup)
  APP_Init();
  RS485_Init();
#elif MAIN_BRAKE_SEQUENCE_TEST_MODE
  Debug_UART_Init();
  Car_SetActiveConfig(VF_89);
  Car_Hardware_Init(Car_GetActiveConfig());
  Brake_SequenceInit();
#else
  Motor_TestInit();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* USER CODE BEGIN 3 */
      now = HAL_GetTick();
#if MAIN_MOTOR_TEST_MODE
      Drain_CanRxQueue();
      Motor_TestUpdate();
#elif MAIN_BRAKE_SEQUENCE_TEST_MODE
      Drain_CanRxQueue();
      Brake_SequenceUpdate();
      EN_ENABLE(&Motor1);
      EN_ENABLE(&Motor2);
#else
      Modbus_Service();
      APP_Run();
      Modbus_CheckHealth();
      
      // Keep motors enabled
      EN_ENABLE(&Motor1);
      EN_ENABLE(&Motor2);
#endif

      HAL_Delay(1); 
      /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 20;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVQ);
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLL1QCLK, RCC_MCODIV_1);

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_PIN;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8) // System Master Tick
    {
        SCurve_MasterTickHandler();
#if !MAIN_MOTOR_TEST_MODE
        static uint32_t heartbeat = 0;
        heartbeat++;
        if (heartbeat >= 1000) { // Toggle LED every 1s (at 100kHz tick)
            HAL_GPIO_TogglePin(Led_W_GPIO_Port, Led_W_Pin);
            heartbeat = 0;
        }
#endif
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
#if MAIN_MOTOR_TEST_MODE
    if (htim->Instance == MOTOR_TEST_MOTOR_OBJ.Tim->Instance) {
        MOTOR_TEST_MOTOR_OBJ.step_counter++;
        g_motor_test_last_move_tick = HAL_GetTick();
        return;
    }
#endif
    if (htim->Instance == Motor1.Tim->Instance) {
        SCurve_TimerISR(&Motor1);
    }
    else if (htim->Instance == Motor2.Tim->Instance) {
        SCurve_TimerISR(&Motor2);
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  
  #include "System_Error.h"
  System_ErrorHandler("HAL / HardFault Enountered");
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
