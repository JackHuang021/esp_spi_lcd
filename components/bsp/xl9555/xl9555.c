/**
 * @file xl9555.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <string.h>
#include "xl9555.h"
#include "driver/i2c_master.h"

static const char* tag = "xl9555";

static i2c_master_dev_handle_t i2c_dev_handle;

static esp_err_t xl9555_read_data(uint8_t reg,
                                  uint8_t *data, const size_t len)
{
    return i2c_master_transmit_receive(i2c_dev_handle, &reg, 1, data, len, -1);
}

static esp_err_t xl9555_write_data(uint8_t reg, uint8_t *data,
								   const size_t len)
{
    esp_err_t ret = ESP_OK;
    uint8_t *buf = malloc(len + 1);

    buf[0] = reg;
    memcpy(buf + 1, data, len);
    ret = i2c_master_transmit(i2c_dev_handle, buf, len + 1, -1);
    free(buf);
    return ret;
}

void xl9555_pin_write(const uint8_t pin, const bool state)
{
    uint16_t pin_data = 0;

    xl9555_read_data(XL9555_INPUT_PORT0_REG, (uint8_t *)&pin_data, 2);
    if (state)
        pin_data |= (uint16_t)(1 << pin);
    else
        pin_data &= ~(uint16_t)(1 << pin);
    xl9555_write_data(XL9555_OUTPUT_PORT0_REG, (uint8_t *)&pin_data, 2);
}

bool xl9555_pin_read(const uint8_t pin)
{
    uint16_t pin_data = 0;

    xl9555_read_data(XL9555_INPUT_PORT0_REG, (uint8_t *)&pin_data, 2);
    return (pin_data & (1 << pin));
}

static void xl9555_io_config(uint16_t value)
{
    int retry = 3;
    uint16_t temp = 0;
    esp_err_t err = ESP_OK;

    while (retry) {
        err = xl9555_write_data(XL9555_CONFIG_PORT0_REG, (uint8_t *)&value, 2);
        if (err != ESP_OK) {
            ESP_LOGE(tag, "xl9555 io config failed %s", esp_err_to_name(err));
            vTaskDelay(50);
            retry --;
            continue;
}       
            xl9555_read_data(XL9555_CONFIG_PORT0_REG, (uint8_t *)&temp, 2);
            if (temp == value) {
                ESP_LOGI(tag, "xl9555 config done.");
                break;
            }
            ESP_LOGI(tag, "temp: %d", temp);
            retry --;
    }
}

#if XL9555_INT_MODE
static void IRAM_ATTR xl9555_isr_handler(void *arg)
{
    uint16_t temp = 0;

    ESP_LOGI(tag, "xl9555 interrupt");
    xl9555_read_data(XL9555_INPUT_PORT0_REG, (uint8_t *)&temp, 2);
}

static void xl9555_int_gpio_init(void)
{
    gpio_config_t int_gpio = {0};

    int_gpio.mode = GPIO_MODE_INPUT;
    int_gpio.pin_bit_mask = (1ull << XL9555_INT_IO);
    int_gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    int_gpio.pull_up_en = GPIO_PULLUP_ENABLE;
    int_gpio.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&int_gpio);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(XL9555_INT_IO, xl9555_isr_handler, NULL);
}
#endif

void xl9555_init(struct i2c_obj *i2c)
{
    uint16_t temp = 0;

    if (!i2c->is_init)
        i2c_init(i2c);

    i2c_device_config_t i2c_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = XL9555_ADDR,
        .scl_speed_hz = 400000,
    };
    i2c_master_bus_add_device(i2c->bus_handle, &i2c_dev_config, &i2c_dev_handle);


#if XL9555_INT_MODE
    xl9555_int_gpio_init();
#endif

    xl9555_read_data(XL9555_INPUT_PORT0_REG, (uint8_t *)&temp, 2);

    xl9555_io_config(0xf003);
    xl9555_pin_write(3, true);
    xl9555_pin_write(2, true);
    xl9555_pin_write(11, false);

    ESP_LOGI(tag, "xl9555 init done.");
}