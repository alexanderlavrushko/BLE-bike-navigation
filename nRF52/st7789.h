#ifndef ST7789_H_INCLUDED
#define ST7789_H_INCLUDED

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif


ret_code_t st7789_init(void);
void st7789_uninit(void);

void st7789_powersave_begin();
void st7789_powersave_end();

void st7789_sleep_begin();
void st7789_sleep_end();

uint16_t st7789_get_width();
uint16_t st7789_get_height();
uint8_t* st7789_get_screen_buffer();
void st7789_send_screen();

//void st7789_send_image(const uint16_t* image, int16_t x, int16_t y, int16_t w, int16_t h);
//void st7789_fill_color(uint16_t color);

#ifdef __cplusplus
}
#endif

#endif // ST7789_H_INCLUDED
