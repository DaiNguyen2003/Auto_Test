/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

extern uint32_t millis();
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    uint8_t relay_k11;   // ACC Relay
    uint8_t relay_k12;   // HV ON Relay
    uint8_t relay_k13;   // eMotor Relay
    uint32_t brake_pos;  // Current step of brake motor
    uint32_t accel_pos;  // Current step of Motor1
    uint32_t brake_debug;
    uint8_t brake_limit_dbg;
    uint8_t brake_phase_dbg;
    uint8_t brake_step_dbg;
    uint8_t brake_test_stage_dbg;
    uint8_t brake_cmd_dbg;
    uint8_t brake_test_cmd_req;
    uint8_t brake_release_req;
    uint8_t brake_calib_cmd_req;      // 0:IDLE, 1:HOME, 2:TRIGGER, 3:HOLD
    uint8_t brake_calib_capture_req;  // Write 1 to latch current brake_pos
    uint8_t brake_calib_state_dbg;
    uint32_t brake_calib_target_steps;
    uint32_t brake_calib_captured_steps;
    float brake_calib_ref_mm;
    float brake_calib_steps_per_mm;
    uint8_t current_page;
    char current_test_name[16];
    uint8_t test_step;
    char test_state_name[16];
    uint32_t test_cycle;
    uint32_t rem_seconds;
    uint8_t test_status; // 0:IDLE, 1:RUNNING, 2:STOPPED
    uint8_t test_index;  // Index of current test case
    
    // Gear IO Feedback
    uint8_t gear_io_p;
    uint8_t gear_io_r;
    uint8_t gear_io_n;
    uint8_t gear_io_d;
    uint16_t gear12_pwm;
    uint16_t gear34_pwm;

    // Manual Control (Write these when debug_mode = 1)
    uint8_t debug_mode;      // 0: Automatic (LCD/Buttons), 1: Manual (IDE Watch)
    uint8_t manual_relay_k11;
    uint8_t manual_relay_k12;
    uint8_t manual_relay_k13;
    uint8_t manual_brake_cmd; // Use FC_Break_... values
    uint8_t manual_accel_cmd; // Use FC_Accel_... values
    float manual_brake_pos;
    float manual_brake_acc;
    float manual_brake_vel;
    uint8_t manual_gear_cmd;  // Use FC_Gear_... values
    uint8_t manual_key_cmd;   // Use FC_Key_... values
    uint8_t limo_gear_test_cmd; // 0:P, 1:R, 2:N, 3:D, 4:HOME
    uint16_t limo_servo1_target_pwm; // TIM3_CH3
    uint16_t limo_servo2_target_pwm; // TIM3_CH4
    uint8_t motor1_diag_enable;
    uint8_t motor1_diag_mode;
    uint8_t motor1_diag_state;
    uint8_t motor1_diag_dir;
    uint8_t motor1_diag_phase;
    uint8_t motor1_diag_en_level;
    uint8_t motor1_diag_step_level;
    uint8_t motor1_diag_step_active_low;
    uint8_t motor1_diag_motion_seen;
    uint32_t motor1_diag_step;
    uint32_t motor1_diag_cycle;
    uint32_t motor1_diag_idle_ms;
    uint32_t motor1_diag_last_move_ms;
    uint32_t can_rx_count;   // Debug counter: total CAN frames received
    uint32_t rs485_tx_count; // Biến debug đếm số lần gửi RS485
    uint32_t can_rx_classic_count;
    uint32_t can_rx_fd_count;
    uint32_t can_rx_drop_count;
    uint32_t can_rx_unmatched_count;
    uint32_t can_rx_parse_error_count;
    uint8_t can_rx_seen;
    uint8_t can_last_rx_port;
    uint8_t can_last_rx_is_fd;
    uint8_t can_last_rx_id_type;
    uint8_t can_last_rx_len;
    uint32_t can_last_rx_id;
    uint32_t can_last_rx_tick;
    uint8_t can1_rx_seen;
    uint8_t can1_last_rx_is_fd;
    uint8_t can1_last_rx_id_type;
    uint8_t can1_last_rx_len;
    uint32_t can1_last_rx_id;
    uint32_t can1_last_rx_tick;
    uint8_t can2_rx_seen;
    uint8_t can2_last_rx_is_fd;
    uint8_t can2_last_rx_id_type;
    uint8_t can2_last_rx_len;
    uint32_t can2_last_rx_id;
    uint32_t can2_last_rx_tick;
    uint32_t brake_trigger_accept_count;
    uint32_t brake_trigger_ignore_count;
} System_Debug_Typedef;

typedef struct {
    uint32_t rx_event_count;
    uint32_t frame_ok_count;
    uint32_t tx_response_count;
    uint32_t crc_error_count;
    uint32_t uart_error_count;
    uint8_t last_port;
    uint8_t last_slave_id;
    uint8_t last_function;
    uint8_t last_status;
    uint16_t last_start_addr;
    uint16_t last_quantity;
    uint16_t last_rx_size;
    uint16_t last_tx_size;
    uint16_t last_crc_calc;
    uint16_t last_crc_recv;
    uint16_t loopback_value;
} Modbus_Diag_Typedef;

extern System_Debug_Typedef system_debug;
extern volatile Modbus_Diag_Typedef modbus_diag;
extern volatile uint8_t brake_sequence_state_dbg;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define INP4_Pin GPIO_PIN_2
#define INP4_GPIO_Port GPIOE
#define INP3_Pin GPIO_PIN_3
#define INP3_GPIO_Port GPIOE
#define INP2_Pin GPIO_PIN_4
#define INP2_GPIO_Port GPIOE
#define INP1_Pin GPIO_PIN_5
#define INP1_GPIO_Port GPIOE
#define STEP1_EN_Pin GPIO_PIN_6
#define STEP1_EN_GPIO_Port GPIOE
#define STEP1_DIR_Pin GPIO_PIN_13
#define STEP1_DIR_GPIO_Port GPIOC
#define Buzz_Pin GPIO_PIN_2
#define Buzz_GPIO_Port GPIOB
#define Led_W_Pin GPIO_PIN_9
#define Led_W_GPIO_Port GPIOE
#define Relay_6_Pin GPIO_PIN_10
#define Relay_6_GPIO_Port GPIOE
#define Relay_5_Pin GPIO_PIN_11
#define Relay_5_GPIO_Port GPIOE
#define Relay_4_Pin GPIO_PIN_12
#define Relay_4_GPIO_Port GPIOE
#define Relay_3_Pin GPIO_PIN_13
#define Relay_3_GPIO_Port GPIOE
#define Relay_2_Pin GPIO_PIN_14
#define Relay_2_GPIO_Port GPIOE
#define Relay_1_Pin GPIO_PIN_15
#define Relay_1_GPIO_Port GPIOE
#define DIR3_UART3_Pin GPIO_PIN_14
#define DIR3_UART3_GPIO_Port GPIOB
#define Button1_Pin GPIO_PIN_10
#define Button1_GPIO_Port GPIOD
#define Button2_Pin GPIO_PIN_11
#define Button2_GPIO_Port GPIOD
#define Button3_Pin GPIO_PIN_8
#define Button3_GPIO_Port GPIOC
#define DIR4_UART4_Pin GPIO_PIN_15
#define DIR4_UART4_GPIO_Port GPIOA
#define CT_ACC_Pin GPIO_PIN_0
#define CT_ACC_GPIO_Port GPIOE
#define CT_Break_Pin GPIO_PIN_1
#define CT_Break_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
