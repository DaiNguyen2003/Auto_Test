/*
* Test_Manager.h
* Quản lý các kịch bản test và trạng thái chạy/dừng.
*/

#ifndef TEST_MANAGER_H
#define TEST_MANAGER_H

#include "main.h"
#include "Car.h"

typedef enum {
    TEST_IDLE = 0,
    TEST_RUNNING,
    TEST_STOPPED
} Test_Status_Typedef;

typedef struct {
    const char* name;
    void (*Init)(Car_Define_Typedef* car);
    void (*Update)(void);
    const char* (*GetStateName)(void);
    uint32_t (*GetRemainingSeconds)(void);
    uint32_t (*GetCycleCount)(void);
    uint8_t (*GetStep)(void);
} TestCase_t;

void Test_Manager_Init(Car_Define_Typedef* car);
void Test_Manager_Update(void);
void Test_Manager_Start(void);
void Test_Manager_Stop(void);
void Test_Manager_Toggle(void);
void Test_Manager_Select(uint8_t index);
Test_Status_Typedef Test_Manager_GetStatus(void);
uint32_t Test_Manager_GetCycleCount(void);
uint32_t Test_Manager_GetRemainingSeconds(void);
const char* Test_Manager_GetStateName(void);
uint8_t Test_Manager_GetStep(void);

// Available Test Cases (extern declarations)
// Unified Generic Test Case
extern TestCase_t TestCase_Generic;

// Specialized/Legacy tests (if any)
extern TestCase_t TestCase_GearDemo;

#endif // TEST_MANAGER_H
