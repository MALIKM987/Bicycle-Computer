/*
 * bmi160_port.h
 *
 *  Created on: Oct 21, 2025
 *      Author: molik
 */

#ifndef INC_BMI160_PORT_H_
#define INC_BMI160_PORT_H_
#pragma once
#include "bmi160.h"
#include "i2c.h"   // hi2c1

int8_t bmi160_hal_i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
int8_t bmi160_hal_i2c_write(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
void   bmi160_hal_delay_ms(uint32_t ms);

/* Helper: wypełnia strukturę dev dla I2C */
static inline void bmi160_port_init_i2c(struct bmi160_dev *dev)
{
    dev->id       = 0x68;                 // jeśli SDO=GND; jeśli SDO=3V3 → 0x69
    dev->intf     = BMI160_I2C_INTF;      // interfejs I2C wg definicji Boscha
    dev->read     = bmi160_hal_i2c_read;
    dev->write    = bmi160_hal_i2c_write;
    dev->delay_ms = bmi160_hal_delay_ms;
    dev->read_write_len = 16;             // dowolna sensowna długość bufora
}



#endif /* INC_BMI160_PORT_H_ */
