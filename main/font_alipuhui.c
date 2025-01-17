/*
*---------------------------------------------------------------
*                        Lvgl Font Tool                         
*                                                               
* 注:使用unicode编码                                              
* 注:本字体文件由Lvgl Font Tool V0.4 生成                          
* 作者:阿里(qq:617622104)                                         
*---------------------------------------------------------------
*/

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "lvgl.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_heap_caps.h"

/**
 * @brief 决定是否在启动后将字库加载到PSRAM中，可以大幅提高中文显示速度
 * 
 */
#define LOAD_FONT_IN_PSRAM     1

typedef struct{
    uint16_t min;
    uint16_t max;
    uint8_t  bpp;
    uint8_t  reserved[3];
}x_header_t;

typedef struct{
    uint32_t pos;
}x_table_t;

typedef struct{
    uint8_t adv_w;
    uint8_t box_w;
    uint8_t box_h;
    int8_t  ofs_x;
    int8_t  ofs_y;
    uint8_t r;
}glyph_dsc_t;

static x_header_t __g_xbf_hd = {
    .min = 0x0020,
    .max = 0x9fa0,
    .bpp = 4,
};


static const char *tag = "alipuhui_font";

#if LOAD_FONT_IN_PSRAM

static uint8_t *__g_font_buf;
static bool is_font_loaded = false;

esp_err_t load_font_alipuhui(void)
{
    const char *font_file_name = "/littlefs/alipuhui.bin";
    FILE *f = fopen(font_file_name, "r");
    if (f == NULL) {
        ESP_LOGE(tag, "failed to load font file");
        return ESP_FAIL;
    }
    struct stat st;
    int res = stat(font_file_name, &st);
    if (res != 0) {
        ESP_LOGE(tag, "get %s size failed", font_file_name);
        fclose(f);
        return ESP_FAIL;
    }
    ESP_LOGI(tag, "%s file size: %ld bytes", font_file_name, st.st_size);

    __g_font_buf = (uint8_t *)heap_caps_malloc(st.st_size, MALLOC_CAP_SPIRAM);
    size_t len = fread(__g_font_buf, 1, st.st_size, f);
    fclose(f);
    ESP_LOGI(tag, "read %u bytes from %s", len, font_file_name);
    is_font_loaded = true;

    return ESP_OK;
}

#else

static uint8_t __g_font_buf[176];//如bin文件存在SPI FLASH可使用此buff
esp_err_t load_font_alipuhui(void)
{
    return ESP_FAIL;
}

#endif

static uint8_t *__user_font_getdata(int offset, int size){
#if LOAD_FONT_IN_PSRAM
    return __g_font_buf + offset;
#else
    static bool is_font_file_opened = false;
    static FILE *f = NULL;
    if (!is_font_file_opened) {
        f = fopen("/littlefs/alipuhui.bin", "r");
        if (f != NULL) {
            is_font_file_opened = true;
            ESP_LOGI(tag, "font file opened");
        }
        else {
            ESP_LOGE(tag, "font file failed to open");
            return NULL;
        }
    }
    fseek(f, offset, SEEK_SET);
    fread(__g_font_buf, size, 1, f);
    return __g_font_buf;
#endif
}

static const void *  __user_font_get_bitmap(lv_font_glyph_dsc_t *g_desc, lv_draw_buf_t *draw_buf) {
    const lv_font_t *font = g_desc->resolved_font;
    uint32_t unicode_letter = g_desc->gid.index;
    uint8_t bitmap_data = draw_buf->data;

    if( unicode_letter>__g_xbf_hd.max || unicode_letter<__g_xbf_hd.min ) {
        return NULL;
    }
    uint32_t unicode_offset = sizeof(x_header_t)+(unicode_letter-__g_xbf_hd.min)*4;
    uint32_t *p_pos = (uint32_t *)__user_font_getdata(unicode_offset, 4);
    if( p_pos[0] != 0 ) {
        uint32_t pos = p_pos[0];
        glyph_dsc_t * gdsc = (glyph_dsc_t*)__user_font_getdata(pos, sizeof(glyph_dsc_t));
        return __user_font_getdata(pos+sizeof(glyph_dsc_t), gdsc->box_w*gdsc->box_h*__g_xbf_hd.bpp/8);
    }
    return NULL;
}

static bool __user_font_get_glyph_dsc(const lv_font_t * font, lv_font_glyph_dsc_t * dsc_out, uint32_t unicode_letter, uint32_t unicode_letter_next) {
    if( unicode_letter>__g_xbf_hd.max || unicode_letter<__g_xbf_hd.min ) {
        return NULL;
    }
    uint32_t unicode_offset = sizeof(x_header_t)+(unicode_letter-__g_xbf_hd.min)*4;
    uint32_t *p_pos = (uint32_t *)__user_font_getdata(unicode_offset, 4);
    if( p_pos[0] != 0 ) {
        glyph_dsc_t * gdsc = (glyph_dsc_t*)__user_font_getdata(p_pos[0], sizeof(glyph_dsc_t));
        dsc_out->adv_w = gdsc->adv_w;
        dsc_out->box_h = gdsc->box_h;
        dsc_out->box_w = gdsc->box_w;
        dsc_out->ofs_x = gdsc->ofs_x;
        dsc_out->ofs_y = gdsc->ofs_y;
        dsc_out->bpp   = __g_xbf_hd.bpp;
        return true;
    }
    return false;
}


// Alibaba PuHuiTi 3.0 65 Medium,,-1
// 阿里巴巴 普惠字体3.0 Medium
// 字模高度：22
// XBF字体,外部bin文件
lv_font_t alipuhui = {
    .get_glyph_bitmap = __user_font_get_bitmap,
    .get_glyph_dsc = __user_font_get_glyph_dsc,
    .line_height = 22,
    .base_line = 0,
};

