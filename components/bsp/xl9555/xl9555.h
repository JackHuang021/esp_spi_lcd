/**
 * @file xl9555.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __XL9555_H_
#define __XL9555_H_

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "i2c_new.h"

// xl9555 int gpio define
#define XL9555_INT_MODE			0
#define XL9555_INT_IO			GPIO_NUM_0
#define XL9555_INT				gpio_get_level(XL9555_INT_IO)

// xl9555 registers
#define XL9555_INPUT_PORT0_REG			0x0
#define XL9555_INPUT_PORT1_REG			0x1
#define XL9555_OUTPUT_PORT0_REG			0x2
#define XL9555_OUTPUT_PORT1_REG			0x3
#define XL9555_INVERSION_PORT0_REG		0x4
#define XL9555_INVERSION_PORT1_REG		0x5
#define XL9555_CONFIG_PORT0_REG			0x6
#define XL9555_CONFIG_PORT1_REG			0x7


#define XL9555_ADDR						0x20

void xl9555_init(struct i2c_obj *i2c);
bool xl9555_pin_read(const uint8_t pin);
void xl9555_pin_write(const uint8_t pin, const bool state);




#endif

