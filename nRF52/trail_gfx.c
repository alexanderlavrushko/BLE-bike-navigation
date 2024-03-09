#include "trail_gfx.h"
#include "nrf_log.h"


static inline void draw_pixel(trail_gfx_t* p_gfx, int16_t x, int16_t y, uint16_t color)
{
    const uint16_t width = p_gfx->width;
    const uint16_t height = p_gfx->height;
    if (x < 0 || y < 0 ||
        x >= width || y >= height)
    {
        return;
    }
    ((uint16_t*)p_gfx->pixel_data)[y * width + x] = (color << 8) | (color >> 8); // most significant byte first
}

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

#ifndef abs
#define abs(a) ((a) > 0 ? (a) : -(a))
#endif



static inline void trail_gfx_draw_v_line(trail_gfx_t* p_gfx, int16_t x, int16_t y, int16_t h, uint16_t color)
{
    trail_gfx_fill_rect(p_gfx, x, y, 1, h, color);
//    trail_gfx_draw_line_1px(p_gfx, x, y, x, y + h - 1, color);
}

static inline void trail_gfx_draw_h_line(trail_gfx_t* p_gfx, int16_t x, int16_t y, int16_t w, uint16_t color)
{
    trail_gfx_fill_rect(p_gfx, x, y, w, 1, color);
//    trail_gfx_draw_line_1px(p_gfx, x, y, x + w - 1, y, color);
}

static inline void trail_gfx_draw_line_1px(trail_gfx_t* p_gfx,
                         int16_t x0, int16_t y0,
                         int16_t x1, int16_t y1,
                         uint16_t color)
{
    if (x0 == x1)
    {
        if (y0 > y1)
            _swap_int16_t(y0, y1);
        trail_gfx_draw_v_line(p_gfx, x0, y0, y1 - y0 + 1, color);
    }
    else if (y0 == y1)
    {
        if (x0 > x1)
            _swap_int16_t(x0, x1);
        trail_gfx_draw_h_line(p_gfx, x0, y0, x1 - x0 + 1, color);
    }
    else
    {
//        startWrite();
        int16_t steep = abs(y1 - y0) > abs(x1 - x0);
        if (steep)
        {
            _swap_int16_t(x0, y0);
            _swap_int16_t(x1, y1);
        }
        
        if (x0 > x1)
        {
            _swap_int16_t(x0, x1);
            _swap_int16_t(y0, y1);
        }
        
        int16_t dx, dy;
        dx = x1 - x0;
        dy = abs(y1 - y0);
        
        int16_t err = dx / 2;
        int16_t ystep;
        
        if (y0 < y1)
        {
            ystep = 1;
        }
        else
        {
            ystep = -1;
        }
        
        for (; x0 <= x1; x0++)
        {
            if (steep)
            {
                draw_pixel(p_gfx, y0, x0, color);
            }
            else
            {
                draw_pixel(p_gfx, x0, y0, color);
            }
            err -= dy;
            if (err < 0)
            {
                y0 += ystep;
                err += dx;
            }
        }
//        endWrite();
    }
}

void trail_gfx_fill_rect(trail_gfx_t* p_gfx,
                         int16_t x_start, int16_t y_start,
                         uint16_t width, uint16_t height,
                         uint16_t color)
{
    for (int16_t y = y_start; y < y_start + height; ++y)
    {
        for (int16_t x = x_start; x < x_start + width; ++x)
        {
            draw_pixel(p_gfx, x, y, color);
        }
    }
}

void trail_gfx_draw_line(trail_gfx_t* p_gfx,
                         int16_t x0, int16_t y0,
                         int16_t x1, int16_t y1,
                         uint16_t color, uint8_t line_width)
{
    if (line_width == 1)
    {
        trail_gfx_draw_line_1px(p_gfx, x0, y0, x1, y1, color);
    }
    else
    {
        int16_t stepX = 1;
        int16_t stepY = 0;
        // if line is more like horizontal than vertical, iterate by vertical axis
        if (abs(x1 - x0) > abs(y1 - y0))
        {
            stepX = 0;
            stepY = 1;
        }
        // width 2:         0, 1
        // width 3:     -1, 0, 1
        // width 4:     -1, 0, 1, 2
        // width 5: -2, -1, 0, 1, 2
        // width 6: -2, -1, 0, 1, 2, 3
        int16_t deltaStart = -line_width / 2 + (1 - line_width % 2);
        int16_t deltaEnd = line_width / 2;
        for (int16_t i = deltaStart; i <= deltaEnd; ++i)
        {
            trail_gfx_draw_line_1px(p_gfx,
                                    x0 + i * stepX,
                                    y0 + i * stepY,
                                    x1 + i * stepX,
                                    y1 + i * stepY, color);
        }
    }
}

void trail_gfx_draw_circle(trail_gfx_t* p_gfx,
                           int16_t x0, int16_t y0,
                           uint16_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

//    startWrite();
    draw_pixel(p_gfx, x0, y0 + r, color);
    draw_pixel(p_gfx, x0, y0 - r, color);
    draw_pixel(p_gfx, x0 + r, y0, color);
    draw_pixel(p_gfx, x0 - r, y0, color);

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        draw_pixel(p_gfx, x0 + x, y0 + y, color);
        draw_pixel(p_gfx, x0 - x, y0 + y, color);
        draw_pixel(p_gfx, x0 + x, y0 - y, color);
        draw_pixel(p_gfx, x0 - x, y0 - y, color);
        draw_pixel(p_gfx, x0 + y, y0 + x, color);
        draw_pixel(p_gfx, x0 - y, y0 + x, color);
        draw_pixel(p_gfx, x0 + y, y0 - x, color);
        draw_pixel(p_gfx, x0 - y, y0 - x, color);
    }
//    endWrite();
}

static inline void trail_gfx_fill_circle_helper(trail_gfx_t* p_gfx,
                                      int16_t x0, int16_t y0, int16_t r,
                                      uint8_t corners, int16_t delta,
                                      uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    int16_t px = x;
    int16_t py = y;
    
    delta++; // Avoid some +1's in the loop
    
    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if (x < (y + 1))
        {
            if (corners & 1)
                trail_gfx_draw_v_line(p_gfx, x0 + x, y0 - y, 2 * y + delta, color);
            if (corners & 2)
                trail_gfx_draw_v_line(p_gfx, x0 - x, y0 - y, 2 * y + delta, color);
        }
        if (y != py)
        {
            if (corners & 1)
                trail_gfx_draw_v_line(p_gfx, x0 + py, y0 - px, 2 * px + delta, color);
            if (corners & 2)
                trail_gfx_draw_v_line(p_gfx, x0 - py, y0 - px, 2 * px + delta, color);
            py = y;
        }
        px = x;
    }
}

void trail_gfx_fill_circle(trail_gfx_t* p_gfx,
                           int16_t x0, int16_t y0,
                           uint16_t r, uint16_t color)
{
//    startWrite();
    trail_gfx_draw_v_line(p_gfx, x0, y0 - r, 2 * r + 1, color);
    trail_gfx_fill_circle_helper(p_gfx, x0, y0, r, 3, 0, color);
//    endWrite();
}

void trail_gfx_fill_triangle(trail_gfx_t* p_gfx,
                             int16_t x0, int16_t y0,
                             int16_t x1, int16_t y1,
                             int16_t x2, int16_t y2,
                             uint16_t color)
{
    int16_t a, b, y, last;
    
    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1)
    {
        _swap_int16_t(y0, y1);
        _swap_int16_t(x0, x1);
    }
    if (y1 > y2)
    {
        _swap_int16_t(y2, y1);
        _swap_int16_t(x2, x1);
    }
    if (y0 > y1)
    {
        _swap_int16_t(y0, y1);
        _swap_int16_t(x0, x1);
    }
    
//    startWrite();
    if (y0 == y2)
    { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if (x1 < a)
            a = x1;
        else if (x1 > b)
            b = x1;
        if (x2 < a)
            a = x2;
        else if (x2 > b)
            b = x2;
        trail_gfx_draw_h_line(p_gfx, a, y0, b - a + 1, color);
//      endWrite();
        return;
    }
    
    int16_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0,
    dx12 = x2 - x1, dy12 = y2 - y1;
    int32_t sa = 0, sb = 0;
    
    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if (y1 == y2)
        last = y1; // Include y1 scanline
    else
        last = y1 - 1; // Skip it
    
    for (y = y0; y <= last; y++)
    {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
         a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
         b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
         */
        if (a > b)
            _swap_int16_t(a, b);
        trail_gfx_draw_h_line(p_gfx, a, y, b - a + 1, color);
    }
    
    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);
    for (; y <= y2; y++)
    {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
         a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
         b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
         */
        if (a > b)
            _swap_int16_t(a, b);
        trail_gfx_draw_h_line(p_gfx, a, y, b - a + 1, color);
    }
//    endWrite();
}
