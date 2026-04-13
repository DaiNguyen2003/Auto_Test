#ifndef CAR_API_H
#define CAR_API_H

#include "Car_Types.h"

extern uint8_t debug_pos;

extern StepObject Motor1;
extern StepObject Motor2;

// Shared sequences
extern TestStep_t std_sequence[];
extern TestStep_t e34_sequence[];
extern const uint8_t STD_SEQ_SIZE;
extern const uint8_t E34_SEQ_SIZE;

/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------     Key Act Function  -----------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

void Car_KeyControl(Car_Define_Typedef* Key_Cmd, Key_CMD_Typedef CmdAction);
void Car_KeyCalib(Car_Define_Typedef* Key_Cmd, Key_CMD_Typedef CmdAction); 

/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------     Break Act Function  ---------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

uint8_t Home_Break();

void Car_BrakeControl(Car_Define_Typedef* Break_Cmd, StepObject* Motor, Break_CMD_Typedef CmdAction);
void Car_BrakeCalib(Car_Define_Typedef* Break_Cmd, Break_CMD_Typedef CmdAction); 
uint8_t BreakBreak(StepObject* Motor, Break_CMD_Typedef CmdAction);

/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------     Gear Act Function  ----------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

uint8_t Get_Pos_Gear(uint8_t Gear_Oder_Raw, Gear_CMD_Typedef CmdGearAct); 
void Car_GearAction(Car_Define_Typedef* Gear_Cmd, Gear_CMD_Typedef CmdAction); // Gear_type thứ tự các trường hợp gear.
void Car_GearControl(Car_Define_Typedef* Gear_Cmd,   Gear_CMD_Typedef  CmdGearAction);
void Car_GearCalib(Car_Define_Typedef* Gear_Cmd,   Gear_CMD_Typedef  CmdGearCalib);

/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------     Car Declare Types  ------------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/

extern Car_Define_Typedef Vinfast_Virtual;
extern Key_Virtual_Car_Typedef Key_Virtual_Car;
extern Pedal_Virtual_Car_Typedef Break_Virtual_Car;
extern Car_Define_Typedef Vinfast_VF89;
extern Car_Define_Typedef Vinfast_VF5;
extern Car_Define_Typedef Vinfast_VF67;
extern Car_Define_Typedef Vinfast_VF2;
extern Car_Define_Typedef Vinfast_VF3;
extern Car_Define_Typedef Vinfast_e34;
extern Car_Define_Typedef Vinfast_Van;
extern Car_Define_Typedef Vinfast_Limo;


/*=========================================================================================================================================================================||
* ||-------------------------------------------------------------------------     Init Function  ------------------------------------------------------------------------||
*==========================================================================================================================================================================||
*/
void Car_Hardware_Init(Car_Define_Typedef* car);

void   Car_SetActiveConfig(Car_Type type);
Car_Define_Typedef* Car_GetActiveConfig(void);

#endif // CAR_API_H
