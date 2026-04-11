/*
* HungDk
*/


#include "IO_Control.h"
#include "gpio.h"
#include "common.h"

/* --- IO State Variables (moved from main.c) --- */
union ByteData SwitchLimit;
union ByteData Button;
union ByteData StepCtrl;
union ByteData InputProtect;
union ByteData Relay;

/* --- Relay convenience functions (moved from main.c: acc/HVon/eMotor/off) --- */
void Relay_ACC(void) {
    K11(1); K12(0); K13(0);
    system_debug.relay_k11 = 1;
    system_debug.relay_k12 = 0;
    system_debug.relay_k13 = 0;
}

void Relay_HVon(void) {
    K11(1); K12(1); K13(0);
    system_debug.relay_k11 = 1;
    system_debug.relay_k12 = 1;
    system_debug.relay_k13 = 0;
}

void Relay_eMotor(void) {
    K11(0); K12(1); K13(1);
    system_debug.relay_k11 = 0;
    system_debug.relay_k12 = 1;
    system_debug.relay_k13 = 1;
}

void Relay_Off(void) {
    K11(0); K12(0); K13(0);
    system_debug.relay_k11 = 0;
    system_debug.relay_k12 = 0;
    system_debug.relay_k13 = 0;
}

// Define các kiểu píp píp  -> 
/*
* Beep 1 phát xong tắt. ó khác nhau, 
* Beep n phát liên tục với cùng tần số
* Beep n phát với tần số khác nhau và duty khác nhau
* => tóm lại tạo một cái hàm beep linh hoạt =)) 
*/
uint8_t BeepBeep(uint8_t N_Beep, uint16_t Timer_H, uint16_t Timer_L){
    static uint8_t State_Beep = 0;
    static uint32_t Timer_Beep = 0;
    static uint8_t  Count_Beep = 0;
    static uint8_t  N_Beep_Last = 0;
    if(N_Beep != N_Beep_Last){
        N_Beep_Last = N_Beep;
        Count_Beep = N_Beep;  // Update một lần duy nhất trong một lần gọi hàm
        State_Beep = 0; 
    }
    if(N_Beep == 0){
        HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 0);  // Tắt còi
        State_Beep = 3;
    }
    
    switch(State_Beep){
        case 0: 
            Timer_Beep = millis(); 
            State_Beep = 1;   // 1 là High, 2 là LoW
            break; 
        case 1: 
            HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 1);  // Bật còi
            if(millis() - Timer_Beep >= Timer_H){
                State_Beep = 2;  //
                Timer_Beep = millis();
                Count_Beep --; 
            }
            break; 
        case 2: 
            HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 0);  // Tắt còi
            if(millis() - Timer_Beep >= Timer_L){
                Timer_Beep = millis();
                if(Count_Beep > 0){
                    State_Beep = 1;  //
                }else{ 
                    State_Beep = 3;  //
                }
            }
            break; 
        case 3: // IDLE
            HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 0);  // CHỜ =))
            return 1;  //-> Kêu xong hehe
            break;                     
    }
    return 0; 
}
