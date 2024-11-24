#ifndef __IIC_H_
#define __IIC_H_

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

struct i2c_obj {
	i2c_port_t port;
	gpio_num_t scl;
	gpio_num_t sda;
	bool is_init;
};

struct i2c_buf {
	size_t len;
	uint8_t *buf;
};

#define I2C_FLAG_READ			(0x01)
#define I2C_FLAG_STOP			(0x02)
#define I2C_FLAG_WRITE			(0x04)

// i2c gpio pins define
#define IIC0_SDA_GPIO_PIN		(GPIO_NUM_41)
#define IIC0_SCL_GPIO_PIN		(GPIO_NUM_42)
#define IIC1_SDA_GPIO_PIN		(GPIO_NUM_5)
#define IIC1_SCL_GPIO_PIN		(GPIO_NUM_4)

esp_err_t i2c_init(struct i2c_obj *i2c);
esp_err_t i2c_transfer(i2c_port_t i2c_port, const uint16_t addr, size_t n,
					   struct i2c_buf *bufs, const unsigned int flags);


#endif

