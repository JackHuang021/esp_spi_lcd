#ifndef __IIC_H_
#define __IIC_H_

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

struct i2c_obj {
	i2c_port_t port;
	gpio_num_t scl_io_num;
	gpio_num_t sda_io_num;
	bool is_init;
	i2c_master_bus_handle_t bus_handle;
};

struct i2c_buf {
	size_t len;
	uint8_t *buf;
};

esp_err_t i2c_init(struct i2c_obj *i2c);

#endif

