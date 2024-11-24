/**
 * @file lcd.c
 * @author Jack
 * @brief 
 * @version 0.1
 * @date 2024-11-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "lcd.h"
#include "lvgl.h"

#define LCD_PIXEL_CLOCK_HZ			(20 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL		1
#define LCD_BK_LIGHT_OFF_LEVEL		!LCD_BK_LIGHT_ON_LEVEL
#define LCD_PIN_NUM_SCLK			GPIO_NUM_12
#define LCD_PIN_NUM_MOSI			GPIO_NUM_11
#define LCD_PIN_NUM_MISO			GPIO_NUM_13
#define LCD_PIN_NUM_DC				GPIO_NUM_40
#define LCD_PIN_NUM_RST				-1
#define LCD_PIN_NUM_CS				GPIO_NUM_21
#define LCD_PIN_NUM_BK_LIGHT		-1
#define LCD_PIN_NUM_TOUCH_CS		-1

#define LCD_H_RES					240
#define LCD_V_RES					320

#define LCD_CMD_BITS				8
#define LCD_PARAM_BITS				8

#define LVGL_TICK_PERIOD_MS			2
#define LVGL_TASK_MAX_DELAY_MS		500
#define LVGL_TASK_MIN_DELAY_MS		1
#define LVGL_TASK_STACK_SIZE		(4 * 1024)
#define LVGL_TASK_PRIORITY			2

static const char *tag = "lcd";

static lv_disp_draw_buf_t disp_buf;
static lv_disp_drv_t disp_drv;
static SemaphoreHandle_t lvgl_mux = NULL;

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
	esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
	int offset_x1 = area->x1;
	int offset_x2 = area->x2;
	int offset_y1 = area->y1;
	int offset_y2 = area->y2;

	esp_lcd_panel_draw_bitmap(panel_handle, offset_x1, offset_y1,
							  offset_x2, offset_y2, color_map);
}

static void lvgl_drv_update_cb(lv_disp_drv_t *drv)
{
	esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

	switch ( drv->rotated)
	{
	case LV_DISP_ROT_NONE:
		esp_lcd_panel_swap_xy(panel_handle, false);
		esp_lcd_panel_mirror(panel_handle, true, false);
		break;

	case LV_DISP_ROT_90:
		esp_lcd_panel_swap_xy(panel_handle, true);
		esp_lcd_panel_mirror(panel_handle, true, true);
		break;

	case LV_DISP_ROT_180:
		esp_lcd_panel_swap_xy(panel_handle, false);
		esp_lcd_panel_mirror(panel_handle, false, true);
		break;

	case LV_DISP_ROT_270:
		esp_lcd_panel_swap_xy(panel_handle, true);
		esp_lcd_panel_mirror(panel_handle, false, false);
		break;

	default:
		break;
	}
}

static void lvgl_tick_increase(void *arg)
{
	lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_port_task(void *args)
{
	ESP_LOGI(tag, "start lvgl task");
	uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;

	while (1) {
		if ()
	}
}

esp_err_t lcd_init(void)
{
	esp_err_t ret = ESP_OK;
	spi_bus_config_t spi_bus_cfg = {0};
	esp_lcd_panel_io_handle_t lcd_io_handle = NULL;
	esp_lcd_panel_io_spi_config_t lcd_io_config;
	esp_lcd_panel_handle_t panel_handle = NULL;
	esp_lcd_panel_dev_config_t panel_config;
	lv_color_t *buf1 = NULL;
	lv_color_t *buf2 = NULL;
	lv_disp_t *disp = NULL;
	const esp_timer_create_args_t lvgl_tick_timer_args = {
		.callback = lvgl_tick_increase,
		.name = "lvgl_tick",
	};
	esp_timer_handle_t lvgl_tick_timer = NULL;

	ESP_LOGI(tag, "init SPI bus");

	spi_bus_cfg.sclk_io_num = LCD_PIN_NUM_SCLK;
	spi_bus_cfg.mosi_io_num = LCD_PIN_NUM_MOSI;
	spi_bus_cfg.miso_io_num = LCD_PIN_NUM_MISO;
	spi_bus_cfg.quadhd_io_num = -1;
	spi_bus_cfg.quadwp_io_num = -1;
	spi_bus_cfg.max_transfer_sz = 0;
	ret = spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);

	ESP_LOGI(tag, "init lcd panel");
	lcd_io_config.dc_gpio_num = LCD_PIN_NUM_DC;
	lcd_io_config.cs_gpio_num = LCD_PIN_NUM_CS;
	lcd_io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ;
	lcd_io_config.lcd_cmd_bits = LCD_CMD_BITS;
	lcd_io_config.lcd_param_bits = LCD_PARAM_BITS;
	lcd_io_config.spi_mode = 0;
	lcd_io_config.trans_queue_depth = 10;
	lcd_io_config.on_color_trans_done = NULL;
	lcd_io_config.user_ctx = NULL;

	// attach lcd to the SPI bus
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &lcd_io_config, &lcd_io_handle));

	panel_config.reset_gpio_num = LCD_PIN_NUM_RST;
	panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
	panel_config.bits_per_pixel = 16;

	ESP_LOGI(tag, "install st7789 panel driver");
	ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io_handle, &panel_config, &panel_handle));

	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
	ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

	ESP_LOGI(tag, "init lvgl library");
	lv_init();
	buf1 = heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
	assert(buf1);
	buf2 = heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
	assert(buf2);

	lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 20);

	ESP_LOGI(tag, "register display driver to lvgl");
	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = LCD_H_RES;
	disp_drv.ver_res = LCD_V_RES;
	disp_drv.flush_cb = lvgl_flush_cb;
	disp_drv.drv_update_cb = lvgl_drv_update_cb;
	disp_drv.draw_buf = &disp_buf;
	disp_drv.user_data = panel_handle;
	disp = lv_disp_drv_register(&disp_drv);

	ESP_LOGI(tag, "install lvgl tick timer");
	ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

	lvgl_mux = xSemaphoreCreateRecursiveMutex();
	assert(lvgl_mux);
	ESP_LOGI(tag, "create lvgl task");
	xTaskCreate()

	


	


	
	return ret;
}



