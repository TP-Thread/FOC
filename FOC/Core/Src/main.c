/*!
    \file    main.c
    \brief   initialize the board, motor running state control

    \version 2025-06-30, V1.0.0, GDFOC2.0 for GD32F30x
*/

#include "main.h"
#include "systick.h"
#include <stdio.h>

/* Function prototypes -------------------------------------------------------*/
extern void loop(void);

static void gd_gpio_config(void);
static void gd_uart2_config(void);
static void print_reset_reason(void);

/* Private user code ---------------------------------------------------------*/
/* retarget the C library printf function to the USART, in IAR __VER__ >= 9000000 environment */
size_t __write(int handle, const unsigned char *buffer, size_t size)
{
    size_t nChars = 0;

    for (; size != 0; --size)
    {
        usart_data_transmit(USART2, (uint8_t)*buffer++);
        while (RESET == usart_flag_get(USART2, USART_FLAG_TBE))
            ;
        ++nChars;
    }

    return nChars;
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART2, (uint8_t)ch);
    while (RESET == usart_flag_get(USART2, USART_FLAG_TBE))
        ;
    return ch;
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* enable TIMER behavior when the mcu is in debug mode */
    dbg_periph_enable(DBG_TIMER0_HOLD);
    dbg_periph_enable(DBG_TIMER2_HOLD);

    /* configure systick */
    systick_config();

    /* initialize all configured peripherals */
    gd_gpio_config();
    gd_uart2_config();
    print_reset_reason();

    printf("\r\nGDFOC2.0 for GD32F30x\r\n");

    while (1)
    {
        loop();
    }
}

/**
  * @brief 打印并清除复位来源标志，便于定位“经常复位”的根因
  */
static void print_reset_reason(void)
{
    if (SET == rcu_flag_get(RCU_FLAG_SWRST))
    {
        printf("[RST] software reset\r\n");
    }
    if (SET == rcu_flag_get(RCU_FLAG_WWDGTRST))
    {
        printf("[RST] window watchdog reset\r\n");
    }
    if (SET == rcu_flag_get(RCU_FLAG_FWDGTRST))
    {
        printf("[RST] free watchdog reset\r\n");
    }
    if (SET == rcu_flag_get(RCU_FLAG_LPRST))
    {
        printf("[RST] low-power reset\r\n");
    }
    if (SET == rcu_flag_get(RCU_FLAG_EPRST))
    {
        printf("[RST] external pin reset\r\n");
    }
    if (SET == rcu_flag_get(RCU_FLAG_PORRST))
    {
        printf("[RST] power reset (likely brownout/power dip)\r\n");
    }

    rcu_all_reset_flag_clear();
}

/**
  * @brief 初始化GPIO
  */
static void gd_gpio_config(void)
{
    /* GPIO Ports Clock Enable */
    rcu_periph_clock_enable(RCU_GPIOB);

    /* UART2 */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);       /* UART2_Tx */
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11); /* UART2_Rx */
}

/**
  * @brief UART2 用于调试信息输出
  */
static void gd_uart2_config(void)
{
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART2);

    /* USART configure */
    usart_deinit(USART2);
    usart_baudrate_set(USART2, 115200U);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_enable(USART2);
}
