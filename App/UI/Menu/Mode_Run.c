/*
* HungDK -> 22/01/2026
* Mode_Run.c
* Mode chạy. 
*/

#include "Menu.h"

CLCD_I2C_Name LCD;


/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------     Define Gobal Variable -------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/
extern volatile uint32_t CAN1_RxID;
extern volatile uint8_t CAN1_RxData[8];
extern volatile uint8_t CAN1_RxReceived;

extern volatile uint32_t CAN2_RxID;
extern volatile uint8_t CAN2_RxData[8];
extern volatile uint8_t CAN2_RxReceived;

/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------     Data Process          -------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/
void Data_Process(Car_Define_Typedef* car){
    // Xử lý dữ liệu nhận được từ CAN hoặc 485 ở đây. 
    // Chuyển các giá trị nhận được về các giá trị thực tế để điều khiển. 
    // VD: pos_break, vel_break, accel_break
    // Chuyển từ đơn vị mm, mm/s, mm/s^2 sang step, step/s, step/s^2

}

/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------     Mode Virtual  CAN/ 485-------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

void Virtual_Data_Conversion(Car_Define_Typedef* car){
    switch(CAN1_RxID){ // Kiểm tra ID nhận được từ CAN1
        case 0x98 : // Pedal Virtual Car
            Break_Virtual_Car.pos_break   = (uint16_t)(CAN1_RxData[7] << 8 | CAN1_RxData[6]); // Byte 7 + 8 ( đơn vị mm )
            Break_Virtual_Car.accel_break = (uint16_t)(CAN1_RxData[5] << 8 | CAN1_RxData[4]); // Byte 5 + 6
            Break_Virtual_Car.vel_break   = (uint16_t)(CAN1_RxData[3] << 8 | CAN1_RxData[2]); // Byte 3 + 4
            Break_Virtual_Car.cmd_break   = (Break_CMD_Typedef)(CAN1_RxData[0]); // Byte 2 // Byte 0 bỏ vì không dùng
        
            if( Break_Virtual_Car.pos_break != 0 && Break_Virtual_Car.vel_break != 0 && Break_Virtual_Car.accel_break != 0){
                // Chuyển đổi các giá trị từ đơn vị CAN sang đơn vị thực tế
                car -> BreakCmd.BreakPoss = (float)Break_Virtual_Car.pos_break * 0.01f; // mm
                car -> BreakCmd.BreakVel  = (float)Break_Virtual_Car.vel_break * 0.1f;  // mm/s
                car -> BreakCmd.BreakAcc  = (float)Break_Virtual_Car.accel_break * 0.1f; // mm/s²
                car -> Break_ReqCmd       = Break_Virtual_Car.cmd_break;
            }else{
                car -> BreakCmd.BreakPoss   = 60.0f; // mm
                car -> BreakCmd.BreakAcc = 240.0f;  // mm/s²
                car -> BreakCmd.BreakVel   = 120.0f;  // mm/s
                car -> Break_ReqCmd       = Break_Virtual_Car.cmd_break;
            }
            break; 
            
        case 0x8A: // Key Virtual Car
            // Chuyển đổi dữ liệu cho Key
            Key_Virtual_Car.angle_lock   = (uint16_t)(CAN1_RxData[7] << 8 | CAN1_RxData[6]);  // Byte 7  + 8 đơn vị từ 500 -> 2500.
            Key_Virtual_Car.angle_unlock = (uint16_t)(CAN1_RxData[5] << 8 | CAN1_RxData[4]);  // Byte 7  + 8 đơn vị từ 500 -> 2500.
            Key_Virtual_Car.FCCmd        = (Key_CMD_Typedef)(CAN1_RxData[0]);
            if(Key_Virtual_Car.angle_lock == 0 || Key_Virtual_Car.angle_lock == 0 ){
                car -> KeyCmd.KeyLock_Poss = 980;
                car -> KeyCmd.KeyUnlock_Poss = 1950;
                car -> KeyCmd.KeyHome_Poss = 1400;
                car -> Key_ReqCmd = FC_Key_Home; 
            }else{
                car -> KeyCmd.KeyLock_Poss   = Key_Virtual_Car.angle_lock;
                car -> KeyCmd.KeyUnlock_Poss = Key_Virtual_Car.angle_unlock;
                car -> KeyCmd.KeyHome_Poss   = 1400;
                car -> Key_ReqCmd = Key_Virtual_Car.FCCmd ;             
            }
            break;

        default:
            break;
    }
}

void Mode_Virtual_CAN(Car_Define_Typedef* car){
    // Ở đây sẽ là mode chạy thực tế với việc điều khiển qua CAN hoặc 485 điều khiển qua máy tính nhận được cái gì thì điều khiển cái đó thôi.
    static char buffer[10];
    sprintf(buffer, "%4d", (uint16_t)(car -> BreakCmd.BreakAcc));
    CLCD_I2C_SetCursor(&LCD, 4 ,1); 
    CLCD_I2C_WriteString(&LCD,buffer);

    sprintf(buffer, "%4d", (uint16_t)(car -> BreakCmd.BreakVel));
    CLCD_I2C_SetCursor(&LCD, 14 ,1); 
    CLCD_I2C_WriteString(&LCD,buffer);

    sprintf(buffer, "%4d", (uint16_t)(car -> BreakCmd.BreakPoss));
    CLCD_I2C_SetCursor(&LCD, 4 ,2); 
    CLCD_I2C_WriteString(&LCD,buffer);

    switch (car -> Break_ReqCmd)
    {
    case FC_Break_Step_IDLE:
        CLCD_I2C_SetCursor(&LCD, 8 ,3); 
        CLCD_I2C_WriteString(&LCD,"Break_IDLE  ");
        break;
    case FC_Trigger_Break:
        CLCD_I2C_SetCursor(&LCD, 8 ,3); 
        CLCD_I2C_WriteString(&LCD,"Brk_Trigger ");
        break;
    case FC_Press_Break_Hold:
        CLCD_I2C_SetCursor(&LCD, 8 ,3); 
        CLCD_I2C_WriteString(&LCD,"Break_Hold  ");
        break;
    case FC_Release_Break:
        CLCD_I2C_SetCursor(&LCD, 8 ,3); 
        CLCD_I2C_WriteString(&LCD,"Brk_Release ");
        break;
    case FC_Break_Home:
        CLCD_I2C_SetCursor(&LCD, 8 ,3); 
        CLCD_I2C_WriteString(&LCD,"Break_Home  ");
        break;
    default:
        break;
    }

    Car_BrakeControl(car, &Motor1, car -> Break_ReqCmd);
    Car_KeyControl(car, car -> Key_ReqCmd);
    Car_GearControl(car, car -> Gear_ReqCmd);
    
}
