#ifndef TASK_LOOP_H
#define TASK_LOOP_H

#include "main.h"

typedef struct
{
    uint8_t level;
    uint8_t status;
    uint16_t scan_cnt;
} button_t;

void driver_init(void);
void task_loop(void);

void LCD_Clear(void);
void LCD_Display_Logo(void);
void LCD_Display_Page1(void);
void LCD_Display_Page2(void);

#endif  /* TASK_LOOP_H */
