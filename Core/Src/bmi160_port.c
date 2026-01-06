/*
 * bmi160_port.c
 *
 *  Created on: Oct 21, 2025
 *      Author: molik
 */
#include "bmi160_port.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

int8_t bmi160_hal_i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, (dev_addr << 1),
                                            reg, I2C_MEMADD_SIZE_8BIT,
                                            data, len, HAL_MAX_DELAY);
    return (st == HAL_OK) ? 0 : -1;
}

int8_t bmi160_hal_i2c_write(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, (dev_addr << 1),
                                             reg, I2C_MEMADD_SIZE_8BIT,
                                             data, len, HAL_MAX_DELAY);
    return (st == HAL_OK) ? 0 : -1;
}

void bmi160_hal_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}


