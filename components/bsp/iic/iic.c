/**
 * @file iic.c
 * @author Jack Huang
 * @brief 
 * @version 0.1
 * @date 2024-11-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "iic.h"
#include "esp_log.h"

static const char *tag = "iic";

/**
 * @brief init i2c bus
 * 
 * @param i2c 
 * @return esp_err_t 
 */
esp_err_t i2c_init(struct i2c_obj *i2c)
{
	esp_err_t ret = ESP_OK;
	i2c_config_t i2c_config = {0};

	i2c_config.mode = I2C_MODE_MASTER;
	i2c_config.sda_io_num = i2c->sda;
	i2c_config.scl_io_num = i2c->scl;
	i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
	i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
	i2c_config.master.clk_speed = 400000;
	ret = i2c_param_config(i2c->port, &i2c_config);
	if (ret != ESP_OK) {
		ESP_LOGE(tag, "i2c param config error %s\n", esp_err_to_name(ret));
		return ret;
	}

	ret = i2c_driver_install(i2c->port, i2c_config.mode, 0, 0, 0);
	if (ret != ESP_OK) {
		ESP_LOGE(tag, "i2c driver install error %s\n", esp_err_to_name(ret));
	}
	i2c->is_init = true;

	return ret;
}

/**
 * @brief transfer data by i2c bus
 * 
 * @param i2c 
 * @param addr 
 * @param n 
 * @param bufs 
 * @param flags 
 * @return esp_err_t 
 */
esp_err_t i2c_transfer(i2c_port_t i2c_port, const uint16_t addr, size_t n,
					   struct i2c_buf *bufs, const unsigned int flags)
{
	esp_err_t ret = ESP_OK;
	int data_len = 0;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	if (flags & I2C_FLAG_WRITE) {
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, addr << 1, true);
		i2c_master_write(cmd, bufs->buf, bufs->len, true);
		data_len += bufs->len;
		n --;
		bufs ++;
	}

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr << 1 | (flags & I2C_FLAG_READ), true);

	while (n --) {
		if (flags & I2C_FLAG_READ)
			i2c_master_read(cmd, bufs->buf, bufs->len,
							n == 0 ? I2C_MASTER_LAST_NACK : I2C_MASTER_ACK);
		else
			i2c_master_write(cmd, bufs->buf, bufs->len, true);
		data_len += bufs->len;
		bufs ++; 
	}

	if (flags & I2C_FLAG_STOP)
		i2c_master_stop(cmd);

	ret = i2c_master_cmd_begin(i2c_port, cmd,
							   100 * (data_len + 1) / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return ret;
}



