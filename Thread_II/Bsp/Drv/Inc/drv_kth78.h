/**
 * @file drv_kth7823.h
 * @author A-rtos (A-rtos@outlook.com)
 * @brief KTH7823编码器驱动
 * @version 0.1
 * @date 2026-03-28
 * 
 * @copyright Copyright (c) 2026 A-rtos
 * 
 */

#ifndef DRV_KTH78_H
#define DRV_KTH78_H

#include "main.h"

uint16_t kth78_read_angle(void);
uint8_t kth78_read_reg(uint8_t addr);
uint8_t kth78_write_reg(uint8_t addr, uint8_t dataw);

#endif /* DRV_KTH78_H */
