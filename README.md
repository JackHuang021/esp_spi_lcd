# ESP32S3 SPI LCD & LVGL V9.x demo

## 1. Description

+ Hardware: ESP32S3N16R8 dev board & 2.4 inch ST7789V2 SPI LCD with FT6336 Touch IC
+ Software Stack: esp-idf v5.3.1, LVGL v9.2 & Freetype V2.13.3

![](https://raw.githubusercontent.com/JackHuang021/images/master/lvgl.gif)

## 2. Features

1. Use LCD TE GPIO to avoid tear effect
2. Render ttf font with Freetype libiary
3. Smartconfig to config wifi & save wifi info on nvs
4. Mount Littlefs on flash & FatFs on SD Card

## 3. How to Use

### 3.1 Set target

```bash
idf.py set-target esp32s3
```

### 3.2 Set configs

1. Set Project Config, Touch IIC GPIOs, SPI GPIOs, SD GPIOs, & LCD GPIOs

	![](https://raw.githubusercontent.com/JackHuang021/images/master/20250117163537.png)

2. Set ESP32 System Settings:
	+ CPU Freq 240MHz
	+ Enable PSRAM with Octal Mode & 80MHz clock speed
	+ Flash size set to 16MB with QIO Mode & 80MHz SPI Speed
	+ Set partition table with `partitions-16MB.csv`


3. Set LVGL Config
	+ Color Depth Set to RGB565
	+ Stdio File System
	+ FreeType Support

	![Color Depth](https://raw.githubusercontent.com/JackHuang021/images/master/20250117164610.png)

	![HAL Settings](https://raw.githubusercontent.com/JackHuang021/images/master/20250117164728.png)

	![File System Config](https://raw.githubusercontent.com/JackHuang021/images/master/20250117164822.png)

	![FreeType Config](https://raw.githubusercontent.com/JackHuang021/images/master/20250117164905.png)

## 4. TODO

1. Avoid tear effect when use SPI LCD no TE GPIO, example DaXian 2.0 Inch LCD
2. Support mass storage to update files
