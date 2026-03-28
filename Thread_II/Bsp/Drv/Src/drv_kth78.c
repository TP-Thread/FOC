/**
 * @file drv_kth78.c
 * @author A-rtos (A-rtos@outlook.com)
 * @brief KTH7823编码器驱动
 * @version 0.1
 * @date 2026-03-28
 *
 * @copyright Copyright (c) 2026 A-rtos
 *
 */

#include "drv_kth78.h"

/**
 * @brief  Transmit and Receive 2 bytes of data.
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
 * @param  pTxData pointer to transmission data buffer.
 * @param  pRxData pointer to reception data buffer.
 * @retval None
 */
void kth78_spi_rw(SPI_HandleTypeDef *hspi, uint16_t *pTxData, uint16_t *pRxData)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, (uint8_t *)pTxData, (uint8_t *)pRxData, 1, 0xFF);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Reads the angle and returns the result.
 * @retval Angle
 */
uint16_t kth78_read_angle(void)
{
    uint16_t u16TxData = 0;
    uint16_t u16RxData;

    kth78_spi_rw(&hspi1, &u16TxData, &u16RxData);
    kth78_spi_rw(&hspi1, &u16TxData, &u16RxData);

    return u16RxData;
}

/**
 * @brief  Reads the register value via the address parameter.
 * @param  Register address.
 * @retval Register value
 */
uint8_t kth78_read_reg(uint8_t addr)
{
    uint8_t u8Data = 0;
    uint16_t u16TxData = (0x40 + addr) << 8;
    uint16_t u16RxData;

    kth78_spi_rw(&hspi1, &u16TxData, &u16RxData);
    kth78_spi_rw(&hspi1, &u16TxData, &u16RxData);

    u8Data = (u16RxData >> 8) & 0xFF;

    return u8Data;
}

/**
 * @brief  Writes the register.
 * @param  Register address.
 * @param  The value you want to write to the register.
 * @retval The value of the newly written register
 */
uint8_t kth78_write_reg(uint8_t addr, uint8_t dataw)
{
    uint8_t u8Data = 0;
    uint16_t u16TxData = ((0x80 + addr) << 8) + dataw;
    uint16_t u16RxData;

    kth78_spi_rw(&hspi1, &u16TxData, &u16RxData);
    HAL_Delay(25);
    kth78_spi_rw(&hspi1, &u16TxData, &u16RxData);

    u8Data = (u16RxData >> 8) & 0xFF;

    return u8Data;
}
