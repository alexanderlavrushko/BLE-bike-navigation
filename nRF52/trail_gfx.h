#ifndef TRAIL_GFX_H_INCLUDED
#define TRAIL_GFX_H_INCLUDED

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t* pixel_data;
    uint16_t width;
    uint16_t height;
} trail_gfx_t;

void trail_gfx_fill_rect(trail_gfx_t* p_gfx,
                         int16_t x_start, int16_t y_start,
                         uint16_t width, uint16_t height,
                         uint16_t color);

void trail_gfx_draw_line(trail_gfx_t* p_gfx,
                         int16_t x0, int16_t y0,
                         int16_t x1, int16_t y1,
                         uint16_t color, uint8_t line_width);

void trail_gfx_draw_circle(trail_gfx_t* p_gfx,
                           int16_t x0, int16_t y0,
                           uint16_t r, uint16_t color);

void trail_gfx_fill_circle(trail_gfx_t* p_gfx,
                           int16_t x0, int16_t y0,
                           uint16_t r, uint16_t color);

void trail_gfx_fill_triangle(trail_gfx_t* p_gfx,
                             int16_t x0, int16_t y0,
                             int16_t x1, int16_t y1,
                             int16_t x2, int16_t y2,
                             uint16_t color);

#ifdef __cplusplus
}
#endif

#endif // TRAIL_GFX_H_INCLUDED
