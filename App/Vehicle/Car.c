/*
* Car.c
* HungDK_22/01/2026
*/

#include "Car.h"

StepObject Motor1 = {
    .Tim = &htim2,
    .Channel = TIM_CHANNEL_2,
    .GPIO_PORT_STEP = GPIOA,
    .GPIO_PIN_STEP = GPIO_PIN_1,
    .GPIO_PORT_DIR = STEP1_DIR_GPIO_Port,
    .GPIO_PIN_DIR = STEP1_DIR_Pin,
    .GPIO_PORT_EN = STEP1_EN_GPIO_Port,
    .GPIO_PIN_EN = STEP1_EN_Pin,
    .Flag_One_Time = 1,
};

StepObject Motor2 = {
    .Tim = &htim15,
    .Channel = TIM_CHANNEL_2,
    .GPIO_PORT_STEP = GPIOA,
    .GPIO_PIN_STEP = GPIO_PIN_3,
    .GPIO_PORT_DIR = STEP2_DIR_GPIO_Port,
    .GPIO_PIN_DIR = STEP2_DIR_Pin,
    .GPIO_PORT_EN = STEP2_EN_GPIO_Port,
    .GPIO_PIN_EN = STEP2_EN_Pin,
    .Flag_One_Time = 1,
};

void Car_KeyControl(Car_Define_Typedef* car, Key_CMD_Typedef CmdAction){
    Key_Drive(CmdAction);
}

uint8_t Home_Break(StepObject* Motor){
    static uint8_t Flag1 = 1;
    if(BreakLimit == 0 ){
        if(Flag1 == 1){
            MotorStop(Motor);
            Flag1 = 0;
        }
        MotorRun(Motor, 5500, 2800, 2800, 4200.0f, DIR_CCW);
        return 0;
    }else{
        Flag1 = 1;
        MotorStop(Motor);
        return 1;
    }
}

uint8_t Car_Brake(Car_Define_Typedef* car, StepObject* Motor, Break_CMD_Typedef CmdAction){
    Brake_Drive(CmdAction);
    if (Motor->Current_Phase == PHASE_IDLE) return 1;
    return 0;
}

void Car_BrakeControl(Car_Define_Typedef* car, StepObject* Motor, Break_CMD_Typedef CmdAction){
    Brake_Drive(CmdAction);
}

uint8_t debug_pos = 0;

uint8_t Get_Pos_Gear(uint8_t Gear_Oder_Raw, Gear_CMD_Typedef GearAct){
    for(uint8_t i = 0; i<4; i++){
        if(((Gear_Oder_Raw >> i*2) & 0x03) == GearAct){
            debug_pos = Gear_Oder_Raw;
            return 3 - i;
        }
    }
    return 0;
}

uint8_t Check_Gear_ACT(Car_Define_Typedef* car, uint8_t GearAct){

    return GearAct;
}

void Car_GearControl(Car_Define_Typedef* car, Gear_CMD_Typedef CmdGearAction){
    Gear_Drive(CmdGearAction);
}

void Car_GearCalib(Car_Define_Typedef* car, Gear_CMD_Typedef CmdGearCalib){
    static uint16_t Gear12_Pos;
    static uint16_t Gear34_Pos;
    static uint32_t Timer_Gear = 0;
    static Gear_Action_Typedef  Step_calib;
    static Calib_Gear_t  State_calib;
    static Gear_CMD_Typedef  Last_calib = FC_Gear_Home;

    if(Last_calib != CmdGearCalib ){
        Last_calib = CmdGearCalib;
        Step_calib = Gear_Step_Reset;
        State_calib = Get_Pos_Gear(car -> GearCmd.GearOrder, CmdGearCalib);
    }

    switch(Step_calib){
        case Gear_Step_Reset:
            Timer_Gear = millis();
            Step_calib = Gear_Calib_Step_Press_Pcs;
            Gear12_Pos = car -> GearCmd.PossHome12;
            Gear34_Pos = car -> GearCmd.PossHome34;
            break;

        case Gear_Calib_Step_Press_Pcs:
            if(millis() - Timer_Gear >= 15){
                switch(State_calib){
                    case Calib_1:
                        Gear12_Pos ++;
                        break;
                    case Calib_2:
                        Gear12_Pos --;
                        break;
                    case Calib_3:
                        Gear34_Pos ++;
                        break;
                    case Calib_4:
                        Gear34_Pos --;
                        break;
                    default:
                        break;
                }
                Timer_Gear = millis();
            }

            if(car -> StsCar.Val_Gear_Act == CmdGearCalib){
                if(State_calib == Calib_1 || State_calib == Calib_2){
                    car -> GearCmd.Poss[CmdGearCalib] = Gear12_Pos;
                }else if(State_calib == Calib_3 || State_calib == Calib_4){
                    car -> GearCmd.Poss[CmdGearCalib] = Gear34_Pos;
                }
                Step_calib = Gear_Calib_Step_Pass_Poss;
                Timer_Gear = millis();
            }
            break;

        case Gear_Calib_Step_Pass_Poss:
            Timer_Gear = millis();
            break;
        default:
            break;
    }

    __HAL_TIM_SET_COMPARE(car -> GearCmd.Tim, TIM_CH_GEAR12, Gear12_Pos);
    __HAL_TIM_SET_COMPARE(car -> GearCmd.Tim, TIM_CH_GEAR34, Gear34_Pos);
}

void Car_Hardware_Init(Car_Define_Typedef* car) {
    Gear_Layout_Typedef gear_layout = (car->TypeCar == VF_Limo) ? GEAR_LAYOUT_RND_P : GEAR_LAYOUT_PR_ND;

    EN_ENABLE(&Motor1);
    EN_ENABLE(&Motor2);

    Brake_SetHardware(&Motor1, car->BreakCmd.BreakPoss, car->BreakCmd.BreakAcc, car->BreakCmd.BreakVel, car->BreakCmd.BreakJerk);
    Accel_SetHardware(&Motor2, car->AccelCmd.AccelPoss, car->AccelCmd.AccelAcc, car->AccelCmd.AccelVel, car->AccelCmd.AccelJerk);

    Motor1.Current_Phase = PHASE_DONE;
    Motor2.Current_Phase = PHASE_DONE;

    uint8_t s_p, s_r, s_n, s_d;
    if (gear_layout == GEAR_LAYOUT_RND_P) {
        s_p = 1; s_r = 0; s_n = 0; s_d = 0;
    } else if (gear_layout == GEAR_LAYOUT_P_RND) {
        s_p = 0; s_r = 1; s_n = 1; s_d = 1;
    } else {
        s_p = 0; s_r = 0; s_n = 1; s_d = 1;
    }

    car->GearCmd.Servo_Assign[FC_Gear_P] = s_p;
    car->GearCmd.Servo_Assign[FC_Gear_R] = s_r;
    car->GearCmd.Servo_Assign[FC_Gear_N] = s_n;
    car->GearCmd.Servo_Assign[FC_Gear_D] = s_d;

    Gear_SetHardware(car->GearCmd.Tim, car->GearCmd.Poss[FC_Gear_P], car->GearCmd.Poss[FC_Gear_R],
                     car->GearCmd.Poss[FC_Gear_N], car->GearCmd.Poss[FC_Gear_D],
                     car->GearCmd.PossHome12, car->GearCmd.PossHome34,
                     s_p, s_r, s_n, s_d);
    Key_SetHardware(car->KeyCmd.Tim, car->KeyCmd.KeyUnlock_Poss, car->KeyCmd.KeyLock_Poss, car->KeyCmd.KeyHome_Poss, car->KeyCmd.Tim_Press);
}

static Car_Define_Typedef* active_car = NULL;

void Car_SetActiveConfig(Car_Type type) {
    switch(type) {
        case VF_89:     active_car = &Vinfast_VF89;     break;
        case VF_5:      active_car = &Vinfast_VF5;      break;
        case VF_67:     active_car = &Vinfast_VF67;     break;
        case VF_2:      active_car = &Vinfast_VF2;      break;
        case VF_3:      active_car = &Vinfast_VF3;      break;
        case VF_e34:    active_car = &Vinfast_e34;      break;
        case VF_Van:    active_car = &Vinfast_Van;      break;
        case VF_Limo:   active_car = &Vinfast_Limo;     break;
        case VF_Virtual:active_car = &Vinfast_Virtual;  break;
        default:        active_car = &Vinfast_VF89;     break;
    }
}

Car_Define_Typedef* Car_GetActiveConfig(void) {
    if (active_car == NULL) return &Vinfast_VF89;
    return active_car;
}
