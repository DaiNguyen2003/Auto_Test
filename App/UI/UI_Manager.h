/*
* UI_Manager.h
* Quản lý các trang hiển thị và điều hướng nút bấm.
*/

#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "main.h"
#include "LCD_i2C.h"

typedef enum {
    PAGE_IDLE = 0,
    PAGE_MINIVAN_TEST,
    PAGE_LIMO_TEST,
    PAGE_VF89_TEST,
    PAGE_VF5_TEST,
    PAGE_VF67_TEST,
    PAGE_VF3_TEST,
    PAGE_VF2_TEST,
    PAGE_VFe34_TEST,
    PAGE_VIRTUAL_TEST,
    PAGE_GEARDEMO_TEST,
    PAGE_CREEP_TEST,
    PAGE_MAX
} UI_Page_Typedef;

void UI_Manager_Init(CLCD_I2C_Name* lcd);
void UI_Manager_Update(void);
void UI_Manager_Display(void);
UI_Page_Typedef UI_Manager_GetPage(void);

#endif // UI_MANAGER_H
