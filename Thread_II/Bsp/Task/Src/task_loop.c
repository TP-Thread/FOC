/**
 * @file task_loop.c
 * @author A-rtos (A-rtos@outlook.com)
 * @brief 循环任务函数
 * @version 0.1
 * @date 2026-01-31
 *
 * @copyright Copyright (c) 2026 A-rtos
 *
 */

#include "task_loop.h"
#include "drv_lcd.h"
#include "drv_motor.h"
#include "stdio.h"

/* 按键 */
uint8_t KeyNum = 0;
button_t button[2] = {0};

volatile uint16_t KeyTaskId = 10;
volatile uint16_t KeyTaskTim = 0;

/* LED 变量 */
volatile uint16_t LedTaskId = 10;
volatile uint16_t LedTaskTim = 0;

/* LCD 变量 */
volatile uint16_t LcdTaskId = 10;
volatile uint16_t LcdTaskTim = 0;

int8_t YCursor = 0;
int8_t YCursorLast = 100;
uint8_t InitPage1 = 0;
uint8_t InitPage2 = 0;
extern const unsigned char gImage_hs_logo[8052];

/**
 * @brief 驱动初始化
 */
void driver_init(void)
{
    printf("Driver Init...\r\n");
    
    HAL_Delay(100);      // 延时等待电源稳定
    
    LCD_Display_Logo();  // LCD显示LOGO

    motor_init(); // 初始化电机控制参数

    /* ADC 设置 */
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); //  ADC校准
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)tMC.Sample.AdcBuff, 3); // 启动常规组ADC转换，并开启DMA搬运

    /* 设置ABZ编码器 */
    // __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_IDX);  // 使能Encoder Index中断
    // HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    
    /* 设置初始占空比 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    /* 开启对应通道PWM输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* 开启定时器中断 */
    HAL_TIM_Base_Start_IT(&htim1); // 用于PWM输出
    HAL_TIM_Base_Start_IT(&htim2); // 用于任务定时

    /* 使能栅极驱动 SD1/2/3/4 */
    HAL_GPIO_WritePin(SD1_GPIO_Port, SD1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD2_GPIO_Port, SD2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD3_GPIO_Port, SD3_Pin, GPIO_PIN_SET);
}

/**
 * @brief 按键扫描函数
 */
void key_scan()
{
    uint8_t index = 0;

    /* 读取按键值 */
    button[0].level = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9); // 按键1
    button[1].level = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12); // 按键2

    /* 按键处理 */
    for (index = 0; index < 2; index++)
    {
        switch (button[index].status)
        {
        case 0:
            if (button[index].level == 0)
            {
                button[index].scan_cnt = 0;
                button[index].status = 1;
            }
            break;
        case 1:
            if (button[index].level == 0)
            {
                button[index].scan_cnt++;
                if (button[index].scan_cnt >= 50) // 长按：按键1→2，按键2→4
                {
                    if (index == 0)
                    {
                        KeyNum = 2;
                        button[index].status = 2;
                    }
                    if (index == 1)
                    {
                        KeyNum = 4;
                        button[index].status = 2;
                    }
                }
            }
            else
            {
                if (button[index].scan_cnt <= 50) // 短按：按键1→1，按键2→3
                {
                    if (index == 0)
                    {
                        KeyNum = 1;
                        button[index].status = 0;
                    }
                    if (index == 1)
                    {
                        KeyNum = 3;
                        button[index].status = 0;
                    }
                }
            }
            break;
        case 2:
            if (button[index].level == 1)
            {
                button[index].status = 0;
            }
            break;
        }
    }
}

/**
 * @brief 循环任务函数
 */
void task_loop(void)
{
    /* 不使用串口VOFA+上位机通信 */
    // task_uart();

    /* 按键扫描 */
    switch (KeyTaskId)
    {
    case 10:
    {
        if (KeyTaskTim >= 100) // 10ms
        {
            KeyTaskTim = 0;
            KeyTaskId = 20;
        }
    }
    break;
    case 20:
    {
        key_scan();
        KeyTaskId = 10;
    }
    break;
    default:
        break;
    }

    /* LED 控制 */
    switch (LedTaskId)
    {
    case 10:
    {
        if (LedTaskTim >= 10000) /* 1s */
        {
            LedTaskTim = 0;
            LedTaskId = 20;

            // printf("tMC.Foc.Ubus = %.2f V\r\n", tMC.Sample.UdcReal);

            printf("KTH78 Angle = %d\r\n", kth78_read_angle());
        }
    }
    break;
    case 20:
    {
        led_b_toggle();
        LedTaskId = 10;
    }
    break;
    default:
        break;
    }

    /* LCD 控制 */
    switch (LcdTaskId)
    {
    case 10:
    {
        if (LcdTaskTim >= 1000) // 200ms
        {
            LcdTaskTim = 0;
            LcdTaskId = 20;
        }
    }
    break;
    case 20:
    {
        LCD_Display_Page1();
        LcdTaskId = 10;

        if (KeyNum == 4)
        {
            KeyNum = 0;
            InitPage2 = 0;
            LCD_Clear();
            LcdTaskId = 30;
        }
    }
    break;
    case 30:
    {
        LCD_Display_Page2();

        if (KeyNum == 1)
        {
            YCursorLast = YCursor;
            KeyNum = 0;
            YCursor += 16;
            if (YCursor > 64)
                YCursor = 0;
        }

        if (KeyNum == 3)
        {
            YCursorLast = YCursor;
            KeyNum = 0;
            YCursor -= 16;
            if (YCursor < 0)
                YCursor = 64;
        }

        if (KeyNum == 2)
        {
            KeyNum = 0;
            InitPage1 = 0;
            LCD_Clear();
            LcdTaskId = 10;
        }

        if (KeyNum == 4)
        {
            KeyNum = 0;
            InitPage1 = 0;
            LCD_Clear();
            LcdTaskId = 10;

            switch (YCursor)
            {
            case 0:
            {
                tMC.Motor.RunState = MOTOR_SENSORUSE;
                tMC.Motor.RunMode = CURRENT_CLOSE_LOOP;
            }
            break;
            case 16:
            {
                tMC.Motor.RunState = MOTOR_SENSORUSE;
                tMC.Motor.RunMode = SPEED_CURRENT_LOOP;
            }
            break;
            case 32:
            {
                tMC.Motor.RunState = MOTOR_SENSORUSE;
                tMC.Motor.RunMode = POS_SPEED_CURRENT_LOOP;

                /* 初始化位置累加器为0 */
                tMC.Position.ElectricalPosSum = 0;
                tMC.Position.ElectricalPosLast = tMC.Encoder.ElectricalVal;
            }
            break;
            case 48:
            {
                tMC.Motor.RunState = MOTOR_SENSORLESS;
                tMC.Motor.RunMode = HFI_SPEED_CURRENT_CLOSE;
            }
            break;
            case 64:
            {
                tMC.Motor.RunState = MOTOR_SENSORLESS;
                tMC.Motor.RunMode = STRONG_DRAG_SMO_SPEED_CURRENT_LOOP;
            }
            break;
            }
        }
    }
    break;
    default:
        break;
    }
}

void LCD_Clear()
{
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK); // 清屏
}

void LCD_Display_Logo() // 显示LOGO 只在运行开始时显示
{
    LCD_Init();                                     // 屏幕初始化
    LCD_ShowPicture(47, 7, 61, 66, gImage_hs_logo); // 显示浩盛LOGO
    HAL_Delay(1500);                                // 延时显示浩盛LOGO
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);            // 清屏
}

void LCD_Display_Page1()
{
    InitPage1++;
    if (InitPage1 == 1) // 静态内容显示
    {
        LCD_ShowChinese(64, 0, "电压", WHITE, RED, 16, 0);
        LCD_ShowString(96, 0, ":", LIGHTGREEN, BLACK, 16, 0);
        LCD_ShowString(152, 0, "V", LIGHTGREEN, BLACK, 16, 0);

        LCD_ShowChinese(64, 16, "电流", BLACK, YELLOW, 16, 0);
        LCD_ShowString(96, 16, ":", LIGHTGREEN, BLACK, 16, 0);
        LCD_ShowString(152, 16, "A", LIGHTGREEN, BLACK, 16, 0);

        LCD_ShowChinese(64, 32, "速度", WHITE, LIGHTBLUE, 16, 0);
        LCD_ShowString(96, 32, ":", LIGHTGREEN, BLACK, 16, 0);
        LCD_ShowString(152, 32, "R", LIGHTGREEN, BLACK, 16, 0);

        LCD_ShowChinese(64, 48, "位置", BLACK, GREEN, 16, 0);
        LCD_ShowString(96, 48, ":", LIGHTGREEN, BLACK, 16, 0);
        LCD_ShowString(152, 48, "R", LIGHTGREEN, BLACK, 16, 0);

        LCD_DrawLine(0, 0, 60, 0, WHITE);   // 画线
        LCD_DrawLine(60, 0, 60, 31, WHITE); // 画线
        LCD_DrawLine(0, 0, 0, 30, WHITE);   // 画线
        LCD_DrawLine(0, 30, 60, 30, WHITE); // 画线
        LCD_DrawLine(1, 1, 59, 1, WHITE);   // 画线
        LCD_DrawLine(59, 1, 59, 30, WHITE); // 画线
        LCD_DrawLine(1, 1, 1, 29, WHITE);   // 画线
        LCD_DrawLine(1, 29, 59, 29, WHITE); // 画线

        LCD_ShowString(7, 7, "TP FOC", WHITE, BLACK, 16, 0);
        LCD_ShowString(4, 32, "RS:", WHITE, BLACK, 16, 0);
        LCD_ShowString(4, 48, "LD:", WHITE, BLACK, 16, 0);
        LCD_ShowString(4, 64, "STATUS:", WHITE, BLACK, 16, 0);
    }

    if (InitPage1 >= 2) // 动态内容显示
    {
        InitPage1 = 2;

        LCD_ShowFloatNum1(112, 0, tMC.Sample.UdcReal, 4, ROSE_PINK, BLACK, 16);

        if (tMC.Foc.Iq < 0)
        {
            LCD_ShowString(104, 16, "-", ROSE_PINK, BLACK, 16, 0);
            LCD_ShowFloatNum1(112, 16, -tMC.Foc.Iq, 4, ROSE_PINK, BLACK, 16);
        }
        else
        {
            LCD_ShowString(104, 16, " ", ROSE_PINK, BLACK, 16, 0);
            LCD_ShowFloatNum1(112, 16, tMC.Foc.Iq, 4, ROSE_PINK, BLACK, 16);
        }

        if (tMC.Speed.MechanicalSpeed < 0)
        {
            LCD_ShowString(104, 32, "-", ROSE_PINK, BLACK, 16, 0);
            LCD_ShowIntNum(112, 32, (uint16_t)(-tMC.Speed.MechanicalSpeed), 5, ROSE_PINK, BLACK, 16);
        }
        else
        {
            LCD_ShowString(104, 32, " ", ROSE_PINK, BLACK, 16, 0);
            LCD_ShowIntNum(112, 32, (uint16_t)tMC.Speed.MechanicalSpeed, 5, ROSE_PINK, BLACK, 16);
        }

        if (tMC.Position.MechanicalPosRaw < 0)
        {
            LCD_ShowString(104, 48, "-", ROSE_PINK, BLACK, 16, 0);
            LCD_ShowFloatNum1(112, 48, -(float)tMC.Position.MechanicalPosRaw / PUL_MAX, 4, ROSE_PINK, BLACK, 16);
        }
        else
        {
            LCD_ShowString(104, 48, " ", ROSE_PINK, BLACK, 16, 0);
            LCD_ShowFloatNum1(112, 48, (float)tMC.Position.MechanicalPosRaw / PUL_MAX, 4, ROSE_PINK, BLACK, 16);
        }

        switch (tMC.Motor.RunState)
        {
        case CURRENT_CALIB: // 相电流校准
        {
            LCD_ShowString(64, 64, "CURRENT_CALIB", BLACK, WHITE, 16, 0);
        }
        break;

        case MOTOR_IDENTIFY: // 参数辨识
        {
            LCD_ShowChinese(64, 64, "电机参数辨识", BLACK, WHITE, 16, 0);
        }
        break;

        case MOTOR_SENSORUSE: // 有感控制
        {
            switch (tMC.Motor.RunMode)
            {
            case ENCODER_CALIB: // 编码器校准
            {
                LCD_ShowChinese(64, 64, "转子零位校准", BLACK, WHITE, 16, 0);
            }
            break;

            case CURRENT_OPEN_LOOP: // 电流开环
            {
                LCD_ShowChinese(64, 64, "有感电流开环", BLACK, WHITE, 16, 0);
            }
            break;

            case CURRENT_CLOSE_LOOP: // 电流闭环
            {
                LCD_ShowChinese(64, 64, "有感电流闭环", BLACK, WHITE, 16, 0);
            }
            break;

            case SPEED_CURRENT_LOOP: // 速度闭环
            {
                LCD_ShowChinese(64, 64, "有感速度闭环", BLACK, WHITE, 16, 0);
            }
            break;

            case POS_SPEED_CURRENT_LOOP: // 位置闭环
            {
                LCD_ShowChinese(64, 64, "有感位置闭环", BLACK, WHITE, 16, 0);
            }
            break;
            }
        }
        break;

        case MOTOR_SENSORLESS: // 无感控制
        {
            switch (tMC.Motor.RunMode)
            {
            case HFI_SPEED_CURRENT_CLOSE:
            {
                LCD_ShowChinese(64, 64, "无感高频注入", BLACK, WHITE, 16, 0);
            }
            break;

            case STRONG_DRAG_SMO_SPEED_CURRENT_LOOP:
            {
                LCD_ShowChinese(64, 64, "无感强拖滑膜", BLACK, WHITE, 16, 0);
            }
            break;
            }
        }
        break;
        }
    }
}

void LCD_Display_Page2()
{
    InitPage2++;
    if (InitPage2 == 1) // 静态内容显示
    {
        LCD_ShowChinese(4, 8, "模", WHITE, BLACK, 16, 0);
        LCD_ShowChinese(4, 24, "式", WHITE, BLACK, 16, 0);
        LCD_ShowChinese(4, 40, "切", WHITE, BLACK, 16, 0);
        LCD_ShowChinese(4, 56, "换", WHITE, BLACK, 16, 0);

        LCD_ShowChinese(64, 0, "有感电流闭环", ROSE_PINK, BLACK, 16, 0);
        LCD_ShowChinese(64, 16, "有感速度闭环", ROSE_PINK, BLACK, 16, 0);
        LCD_ShowChinese(64, 32, "有感位置闭环", ROSE_PINK, BLACK, 16, 0);
        LCD_ShowChinese(64, 48, "无感高频注入", ROSE_PINK, BLACK, 16, 0);
        LCD_ShowChinese(64, 64, "无感强拖滑膜", ROSE_PINK, BLACK, 16, 0);
    }

    if (InitPage1 >= 2) // 动态内容显示
    {
        InitPage1 = 2;
        LCD_ShowString(30, YCursor, "-->", WHITE, BLACK, 16, 0);
        LCD_ShowString(30, YCursorLast, "   ", WHITE, BLACK, 16, 0);
    }
}
