/*
* UI_Manager.c
*/

#include "UI_Manager.h"
#include "Test_Manager.h"
#include "stdio.h"
#include "string.h"
#include "bsp.h"
#include "common.h"

static CLCD_I2C_Name* current_lcd;
static UI_Page_Typedef current_page = PAGE_IDLE;
static uint8_t last_btn1 = 0, last_btn2 = 0, last_btn3 = 0;
static uint32_t startup_tick = 0;

static char lcd_shadow[4][20];
static char lcd_frame[4][20];

static const char* UI_GetStatusText(void) {
    switch (Test_Manager_GetStatus()) {
        case TEST_RUNNING: return "RUN";
        case TEST_STOPPED: return "STOP";
        default: return "IDLE";
    }
}

void Frame_Clear(void) {
    for(int r=0; r<4; r++)
        for(int c=0; c<20; c++)
            lcd_frame[r][c] = ' ';
}

void Frame_Print(int col, int row, const char* str) {
    if(row < 0 || row > 3 || col < 0 || col > 19) return;
    int i = 0;
    while(str[i] != '\0' && (col + i) < 20) {
        lcd_frame[row][col+i] = str[i];
        i++;
    }
}

void Frame_Flush(void) {
    for(int r=0; r<4; r++) {
        for(int c=0; c<20; c++) {
            if(lcd_frame[r][c] != lcd_shadow[r][c]) {
                CLCD_I2C_SetCursor(current_lcd, c, r);
                CLCD_I2C_WriteChar(current_lcd, lcd_frame[r][c]);
                lcd_shadow[r][c] = lcd_frame[r][c];
            }
        }
    }
}

void UI_Manager_Init(CLCD_I2C_Name* lcd) {
    current_lcd = lcd;
    current_page = PAGE_IDLE;
    startup_tick = millis();
    CLCD_I2C_Clear(current_lcd);
    // Initialize shadows with spaces to force a full draw on first flush
    for(int r=0; r<4; r++)
        for(int c=0; c<20; c++)
            lcd_shadow[r][c] = '\0'; 
    Frame_Clear();
}

UI_Page_Typedef UI_Manager_GetPage(void) {
    return current_page;
}

void UI_Manager_Update(void) {
    uint32_t now = millis();

    // Auto-transition from IDLE after 15s
    if (current_page == PAGE_IDLE) {
        if (now - startup_tick >= 15000) {
            current_page = PAGE_MINIVAN_TEST;
            CLCD_I2C_Clear(current_lcd);
        }
    }

    // Read Buttons
    uint8_t btn1 = !HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin);
    uint8_t btn2 = !HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin);
    uint8_t btn3 = !HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin);

    // Skip button logic during IDLE
    if (current_page != PAGE_IDLE) {
        // Button 3: Next Page (Right)
        if (btn3 && !last_btn3) {
            current_page++;
            if (current_page >= PAGE_MAX) {
                current_page = PAGE_MINIVAN_TEST; // Wrap around to valid test pages
            }
            Test_Manager_Select((uint8_t)(current_page - 1)); // Map PAGE_MINIVAN=1->0, PAGE_LIMO=2->1
            system_debug.test_index = (uint8_t)(current_page - 1);
        }

        // Button 1: Prev Page (Left)
        if (btn1 && !last_btn1) {
            if (current_page <= PAGE_MINIVAN_TEST) {
                current_page = PAGE_GEARDEMO_TEST; // Wrap backward to latest test page
            } else {
                current_page--;
            }
            Test_Manager_Select((uint8_t)(current_page - 1));
            system_debug.test_index = (uint8_t)(current_page - 1);
        }

        // Button 2: Run/Stop Testcase
        if (btn2 && !last_btn2) {
            Test_Manager_Toggle();
        }
    }

    last_btn1 = btn1;
    last_btn2 = btn2;
    last_btn3 = btn3;

    system_debug.current_page = (uint8_t)current_page;

    UI_Manager_Display();
}

void UI_Manager_Display(void) {
    char buffer[32];
    static uint32_t last_display_tick = 0;
    uint32_t now = millis();
    if (now - last_display_tick < 100) return; // refresh at 10Hz
    last_display_tick = now;

    if (current_page == PAGE_IDLE) {
        // Border Chasing Effect (44 positions)
        static uint8_t anim_pos = 0;
        static const uint8_t border_x[] = {
            0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,
            19,19,19,
            18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,
            0,0
        };
        static const uint8_t border_y[] = {
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            1,2,3,
            3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
            2,1
        };

        // Clear previous position
        uint8_t prev_pos = (anim_pos == 0) ? 43 : anim_pos - 1;
        lcd_frame[border_y[prev_pos]][border_x[prev_pos]] = ' ';
        // Draw new position
        lcd_frame[border_y[anim_pos]][border_x[anim_pos]] = 255;

        anim_pos = (anim_pos + 1) % 44;

        // Intro Text
        Frame_Print(6, 1, "EPT Auto");
        
        // Countdown & Loading Text
        static uint8_t load_dots = 0;
        static uint32_t last_dot_tick = 0;
        if (now - last_dot_tick > 500) {
            load_dots = (load_dots + 1) % 4;
            last_dot_tick = now;
        }
        char dots[4] = "";
        for(uint8_t i=0; i<load_dots; i++) dots[i] = '.';
        sprintf(buffer, "Wait%-3s %2ds  ", dots, 15 - (int)((now - startup_tick)/1000));
        Frame_Print(3, 2, buffer);

        // Progress Bar (Row 3, center)
        uint8_t progress = (now - startup_tick) * 16 / 15000;
        strcpy(buffer, "[                ]");
        for(uint8_t i=0; i<progress && i<16; i++) buffer[i+1] = '=';
        Frame_Print(2, 3, buffer);
        
        Frame_Flush();
        return; 
    }

    // Always clear the virtual frame for test cases so we don't have overlapping characters
    Frame_Clear();

    // Row 0: Header with page and run status
    const char* titles[] = {"", "miniVan", "Limo", "VF89", "VF5", "VF67", "VF3", "VF2", "e34", "Virtual", "GearDemo"};
    if(current_page < PAGE_MAX) {
        snprintf(buffer, sizeof(buffer), "P%d %-8s %4s", current_page, titles[current_page], UI_GetStatusText());
        Frame_Print(0, 0, buffer);
    }
    
    // Rows 1-3 for Test Info
    if (current_page >= PAGE_MINIVAN_TEST && current_page < PAGE_MAX) {
            // Row 1: Step + Cycle
            snprintf(buffer, sizeof(buffer), "Step:%-3u Cy:%-4lu", Test_Manager_GetStep(), Test_Manager_GetCycleCount());
            Frame_Print(0, 1, buffer);
            
            // Row 2: Remaining time
            snprintf(buffer, sizeof(buffer), "Remain:%-4lus", Test_Manager_GetRemainingSeconds());
            Frame_Print(0, 2, buffer);

            // Row 3: Current state
            snprintf(buffer, sizeof(buffer), "State:%-14s", Test_Manager_GetStateName());
            Frame_Print(0, 3, buffer);
    }

    Frame_Flush();
}
