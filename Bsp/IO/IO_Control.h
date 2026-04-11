/*
* HungDk
*/

#ifndef IO_CONTROL_H
#define IO_CONTROL_H

#include "main.h"

/*
** RELAY K1x ( Chung) -> DB_CODE_5
* K11 -> Relay1 -> PE15 -> DB_CODE_8
* K12 -> Relay2 -> PE14 -> DB_CODE_4
* K13 -> Relay3 -> PE13 -> DB_CODE_9
*/

/*
* Relay K2 -> Relay4 -> PE12  ->  DB_3_7 
* Relay K3 -> Relay5 -> PE11  ->  DB_4_8
* Relay K4 -> Relay6 -> PE10  ->  DB_5_9
*/

/*

*/

/*
* Button từ trái qua phải: 1 -> 2 -> 3
*/

#define K11(stt) HAL_GPIO_WritePin(Relay_1_GPIO_Port, Relay_1_Pin, stt)
#define K12(stt) HAL_GPIO_WritePin(Relay_2_GPIO_Port, Relay_2_Pin, stt)
#define K13(stt) HAL_GPIO_WritePin(Relay_3_GPIO_Port, Relay_3_Pin, stt)

#define K2(stt) HAL_GPIO_WritePin(Relay_4_GPIO_Port, Relay_4_Pin, stt)
#define K3(stt) HAL_GPIO_WritePin(Relay_5_GPIO_Port, Relay_5_Pin, stt)
#define K4(stt) HAL_GPIO_WritePin(Relay_6_GPIO_Port, Relay_6_Pin, stt)



#define Button1 !HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin)
#define Button2 !HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin)
#define Button3 !HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin)
    
#define INP_P   !HAL_GPIO_ReadPin(INP1_GPIO_Port, INP1_Pin)
#define INP_R   !HAL_GPIO_ReadPin(INP2_GPIO_Port, INP2_Pin)
#define INP_N   !HAL_GPIO_ReadPin(INP3_GPIO_Port, INP3_Pin)
#define INP_D   !HAL_GPIO_ReadPin(INP4_GPIO_Port, INP4_Pin)

/* --- ByteData union for bit-field IO access --- */
union ByteData {
    uint8_t byte;
    struct {
        uint8_t  stt1 : 1;
        uint8_t  stt2 : 1;
        uint8_t  stt3 : 1;
        uint8_t  stt4 : 1;
        uint8_t  stt5 : 1;
        uint8_t  stt6 : 1;
        uint8_t  stt7 : 1;
        uint8_t  stt8 : 1;
    } val;
};

extern union ByteData SwitchLimit;
extern union ByteData Button;
extern union ByteData StepCtrl;
extern union ByteData InputProtect;
extern union ByteData Relay;

/* --- Relay convenience functions --- */
void Relay_ACC(void);
void Relay_HVon(void);
void Relay_eMotor(void);
void Relay_Off(void);

uint8_t BeepBeep(uint8_t N_Beep, uint16_t Timer_H, uint16_t Timer_L);


#endif
