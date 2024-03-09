#include "st7789.h"
#include "nrf.h"
#include "sdk_common.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"


typedef struct
{
    uint8_t pin_clock;
    uint8_t pin_data;
    uint8_t pin_cs;
    uint8_t pin_dc;
    uint8_t pin_reset;
} pincfg_t;

// UMIDIGI Uwatch 3
const pincfg_t pincfg_umax = {
    .pin_clock = NRF_GPIO_PIN_MAP(0, 2),
    .pin_data = NRF_GPIO_PIN_MAP(0, 29),
    .pin_cs = NRF_GPIO_PIN_MAP(1, 15),
    .pin_dc = NRF_GPIO_PIN_MAP(0, 31),
    .pin_reset = NRF_GPIO_PIN_MAP(1, 14)
};

//#define ST7789_SPI_INSTANCE 0
static nrfx_spim_config_t spi_config;// = NRFX_SPIM_DEFAULT_CONFIG;
static const nrfx_spim_t spi = NRFX_SPIM_INSTANCE(3);
static bool is_spi_powersave = 0;
static bool is_sleep = 0;

#define DISPLAY_WIDTH  240
#define DISPLAY_HEIGHT 240

#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_GAMSET  0x26
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_VSCRDEF 0x33
#define ST7789_TEOFF   0x34
#define ST7789_TEON    0x35
#define ST7789_MADCTL  0x36
#define ST7789_VSCRSADD 0x37
#define ST7789_IGMOFF  0x38
#define ST7789_IDMON   0x39
#define ST7789_COLMOD  0x3A
#define ST7789_RAMWRC  0x3C
#define ST7789_RAMRDC  0x3E
#define ST7789_TESCAN  0x44
#define ST7789_RDTESCAN 0x45

#define ST7789_WRDISBV 0x51
#define ST7789_RDDISBV 0x52
#define ST7789_WRCTRLD 0x53
#define ST7789_RDCTRLD 0x54

// different from ST7735
#define ST7789_RAMCTRL 0xB0
#define ST7789_RGBCTRL 0xB1
#define ST7789_PORCTRL 0xB2 // Porch control
#define ST7789_FRCTRL1 0xB3 // Frame rate control
#define ST7789_PARCTRL 0xB5 // Partial control
#define ST7789_GTADJ   0xB7 // Gate control
#define ST7789_DGMEN   0xBA // Digital gamma enable
#define ST7789_VCOMS   0xBB // VCOM setting (what does it mean?)

// ST7789WV specific
#define ST7789_POWSAVE 0xBC // Power saving mode
#define ST7789_DLPOFFSAVE 0xBD // Display off power save

#define ST7789_PWCTRL1 0xD0

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

#define ST7789_GMCTRP1 0xE0
#define ST7789_GMCTRN1 0xE1
#define ST7789_PWCTRL2 0xE8

#define ST7789_MADCTL_MY  0x80
#define ST7789_MADCTL_MX  0x40
#define ST7789_MADCTL_MV  0x20
#define ST7789_MADCTL_ML  0x10
#define ST7789_MADCTL_RGB 0x00
#define ST7789_MADCTL_BGR 0x08
#define ST7789_MADCTL_MH  0x04


#define PIN_INDEX_CLOCK 0
#define PIN_INDEX_DATA 1
#define PIN_INDEX_CS 2
#define PIN_INDEX_DC 3
#define PIN_INDEX_RESET 4

#define HW_DC 1

static volatile bool spi_xfer_done = true;

static inline uint8_t pin_from_cfg(uint8_t pin_index)
{
    const pincfg_t* cfg = &pincfg_umax;
    switch (pin_index)
    {
        case PIN_INDEX_CLOCK:
            return cfg->pin_clock;
        case PIN_INDEX_DATA:
            return cfg->pin_data;
        case PIN_INDEX_CS:
            return cfg->pin_cs;
        case PIN_INDEX_DC:
            return cfg->pin_dc;
        case PIN_INDEX_RESET:
            return cfg->pin_reset;
    }
    return NRF_GPIO_PIN_MAP(0,30);
}

static inline void pincfg_write(uint8_t pin_index, uint8_t value)
{
    uint8_t pin = pin_from_cfg(pin_index);
    nrf_gpio_pin_write(pin, value);
}

static inline void spi_write(const void* data, size_t size)
{
    while (!spi_xfer_done) {}
    spi_xfer_done = false;
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(data, size);
#if HW_DC
    APP_ERROR_CHECK(nrfx_spim_xfer_dcx(&spi, &xfer_desc, 0, 0));
#else
    APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));
#endif
}

#define SPIM_MAX_LENGTH (SPIM_TXD_MAXCNT_MAXCNT_Msk >> SPIM_TXD_MAXCNT_MAXCNT_Pos)

#if HW_DC
static void spi_write_dcx(const uint8_t* data, size_t size, size_t command_size)
{
    while (!spi_xfer_done) {}
    spi_xfer_done = false;
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(data, size);
    APP_ERROR_CHECK(nrfx_spim_xfer_dcx(&spi, &xfer_desc, 0, command_size));
}

static void spi_write_buffer_dcx(const uint8_t* data, size_t size, size_t command_size)
{
    for (size_t i = 0; i < size; i += SPIM_MAX_LENGTH)
    {
        size_t transfer_size = size - i;
        if (transfer_size > SPIM_MAX_LENGTH)
        {
            transfer_size = SPIM_MAX_LENGTH;
        }
        spi_write_dcx(data + i, transfer_size, command_size);
        command_size = 0; // command appears only in the first portion of the buffer
    }
}
#endif


//static inline void spi_write_buffer(const uint8_t* data, size_t size)
//{
//    for (size_t i = 0; i < size; i += SPIM_MAX_LENGTH)
//    {
//        size_t transfer_size = size - i;
//        if (transfer_size > SPIM_MAX_LENGTH)
//        {
//            transfer_size = SPIM_MAX_LENGTH;
//        }
//        spi_write(data + i, transfer_size);
//    }
//}


static inline void write_command(uint8_t c)
{
#if HW_DC
    spi_write_dcx(&c, sizeof(c), 1);
#else
    pincfg_write(PIN_INDEX_DC, 0);
    spi_write(&c, sizeof(c));
    pincfg_write(PIN_INDEX_DC, 1);
#endif
}

static inline void write_data(uint8_t data)
{
    spi_write(&data, sizeof(data));
}

static inline void write_data16(uint16_t data16)
{
    uint16_t data16e = (data16 << 8) | (data16 >> 8);
    spi_write(&data16e, sizeof(data16e));
}


static void sendCommand(uint8_t cmd)
{
    write_command(cmd);
}

static void sendData(uint8_t data)
{
    write_data(data);
}

static void sendData16(uint16_t data)
{
    write_data16(data);
}

static inline void set_int16_swapped(uint8_t* buffer, uint16_t value)
{
    buffer[0] = (value & 0xFF00) >> 8;
    buffer[1] = (value & 0x00FF);
}

static void set_draw_area(int16_t x, int16_t y, int16_t width, int16_t height)
{
//    y += 80;
#if HW_DC
    uint8_t buf[5] = {};
    buf[0] = 0x2A; // column address set
    set_int16_swapped(buf + 1, x);
    set_int16_swapped(buf + 3, x + width - 1);
    spi_write_dcx(buf, sizeof(buf), 1);

    buf[0] = 0x2A; // row address set
    set_int16_swapped(buf + 1, y);
    set_int16_swapped(buf + 3, y + height - 1);
    spi_write_dcx(buf, sizeof(buf), 1);
#else
    sendCommand(0x2A); // column address set
    sendData16(x);
    sendData16(x + width - 1);
    sendCommand(0x2B); // row address set
    sendData16(y);
    sendData16(y + height - 1);
#endif
}

static void command_list(void)
{
    if(0)
    {
        sendCommand(0x11); // Sleep Out
        nrf_delay_ms(121);

        sendCommand(0x21); // invert colors, no idea why but it works as normal colors

        sendCommand(0x36); // memory data access control, MX, MY, RGB
        sendData(0xC8);

        sendCommand(0x3A); // interface pixel format
        sendData(0x05); // 5-6-5 RGB, 16-bit per pixel

        sendCommand(0x29); // display on
        return;
    }

//    write_command(ST7789_SWRESET);
//    nrf_delay_ms(150);
    write_command(ST7789_SLPOUT);
    nrf_delay_ms(7); // 5 ms to next command, and 120 ms to next SLPIN command

    write_command(ST7789_COLMOD);
    write_data(0x55);

    write_command(ST7789_MADCTL);
    write_data(0x00); // nordic st7735 0xC - draw from cable, adafruit 0x08 - draw from opposite of cable

//    write_command(ST7789_CASET);
//    write_data(0x00);
//    write_data(0x00);
//    write_data(0x00);
//    write_data(239);
//    write_command(ST7789_RASET);
//    write_data(0x00);
//    write_data(0x00);
//    write_data(0x00);
//    write_data(239);

    
//        sendCommand(0x3A); // interface pixel format
//        sendData(0x05); // 5-6-5 RGB, 16-bit per pixel


    write_command(ST7789_INVON);

//    write_command(ST7789_NORON);

    write_command(ST7789_DISPON);
//    nrf_delay_ms(100);
}

static void spim_event_handler(nrfx_spim_evt_t const* p_event, void* p_context)
{
    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.");
//    if (m_rx_buf[0] != 0)
//    {
//        NRF_LOG_INFO(" Received:");
//        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//    }
}

static ret_code_t hardware_init(void)
{
    ret_code_t err_code = NRF_SUCCESS;

    nrfx_spim_config_t default_spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    memcpy(&spi_config, &default_spi_config, sizeof(spi_config));
//    spi_config                = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_32M;
    spi_config.ss_pin         = pin_from_cfg(PIN_INDEX_CS);
    spi_config.mosi_pin       = pin_from_cfg(PIN_INDEX_DATA);
    spi_config.sck_pin        = pin_from_cfg(PIN_INDEX_CLOCK);
#if HW_DC
    spi_config.dcx_pin        = pin_from_cfg(PIN_INDEX_DC);
    spi_config.use_hw_ss      = true;
#else
    nrf_gpio_cfg_output(pin_from_cfg(PIN_INDEX_DC));
    pincfg_write(PIN_INDEX_DC, 0);
#endif
    spi_config.ss_active_high = false;
    APP_ERROR_CHECK(nrfx_spim_init(&spi, &spi_config, spim_event_handler, NULL));

    nrf_gpio_cfg_output(pin_from_cfg(PIN_INDEX_RESET));
    pincfg_write(PIN_INDEX_RESET, 0);
    nrf_delay_us(12); // 10 microseconds
    pincfg_write(PIN_INDEX_RESET, 1);
    nrf_delay_ms(122); // 120 ms before Sleep Out command, 5 ms before other commands

    return err_code;
}

ret_code_t st7789_init(void)
{
    ret_code_t err_code;

    err_code = hardware_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    command_list();

    return err_code;
}

void st7789_uninit(void)
{
//    nrf_drv_spi_uninit(&spi);
}

void st7789_powersave_begin()
{
    if (!is_spi_powersave)
    {
        while (!spi_xfer_done) {}
        nrfx_spim_uninit(&spi);
        is_spi_powersave = true;
    }
}

void st7789_powersave_end()
{
//    is_spi_powersave = is_spi_powersave;
    if (is_spi_powersave)
    {
        APP_ERROR_CHECK(nrfx_spim_init(&spi, &spi_config, spim_event_handler, NULL));
        is_spi_powersave = false;
    }
}

void st7789_sleep_begin()
{
    if (!is_sleep)
    {
        st7789_powersave_end();
        write_command(ST7789_SLPIN);
        st7789_powersave_begin();
        is_sleep = true;
    }
}

void st7789_sleep_end()
{
    if (is_sleep)
    {
        st7789_powersave_end();
        write_command(ST7789_SLPOUT);
        nrf_delay_ms(7); // 5 ms to next command, and 120 ms to next SLPIN command
        st7789_powersave_begin();
        is_sleep = false;
    }
}



void st7789_send_image(const uint16_t* image, int16_t xStart, int16_t yStart, int16_t width, int16_t height)
{
    set_draw_area(xStart, yStart, width, height);
    sendCommand(0x2C); // memory write
    
    const uint32_t pixelCount = width * height;
    for (uint32_t i = 0; i < pixelCount; ++i)
    {
        uint16_t value = image[i];
        sendData16(value);
    }
}

static uint8_t buf[1 + DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t)];

uint16_t st7789_get_width()
{
    return DISPLAY_WIDTH;
}

uint16_t st7789_get_height()
{
    return DISPLAY_HEIGHT;
}

uint8_t* st7789_get_screen_buffer()
{
    return buf + 1;
}

void st7789_send_screen()
{
    set_draw_area(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    buf[0] = 0x2C; // memory write
    const size_t command_size = 1;
    spi_write_buffer_dcx(buf, sizeof(buf), command_size);
}


void st7789_fill_color(uint16_t color)
{
    const int16_t width = 240;
    const int16_t height = 240;

    const uint8_t highByte = color >> 8;
    const uint8_t lowByte = color & 0xFF;
    const size_t pixelCount = width * height;
    if(0)
    {
        set_draw_area((DISPLAY_WIDTH - width) / 2, (DISPLAY_HEIGHT - height) / 2, width, height);
        sendCommand(0x2C); // memory write
        
        for (uint32_t i = 0; i < pixelCount; ++i)
        {
            sendData(highByte);
            sendData(lowByte);
        }
    }
    else if(0)
    {
        set_draw_area((DISPLAY_WIDTH - width) / 2, (DISPLAY_HEIGHT - height) / 2, width, height);
        sendCommand(0x2C); // memory write

        const size_t pixelsInBuf = sizeof(buf) / sizeof(uint16_t);
        for (size_t i = 0; i < pixelsInBuf; ++i)
        {
            buf[i * sizeof(uint16_t)] = highByte;
            buf[i * sizeof(uint16_t) + 1] = lowByte;
        }

        const size_t numberOfBuffers = pixelCount / pixelsInBuf;
        const size_t bytesToSend = pixelsInBuf * sizeof(uint16_t);
        for (size_t i = 0; i < numberOfBuffers; ++i)
        {
            spi_write(buf, bytesToSend);
        }

        const size_t bytesRemaining = (pixelCount % pixelsInBuf) * sizeof(uint16_t);
        if (bytesRemaining)
        {
            spi_write(buf, bytesRemaining);
        }
    }
    else
    {
#if HW_DC
        set_draw_area((DISPLAY_WIDTH - width) / 2, (DISPLAY_HEIGHT - height) / 2, width, height);
        buf[0] = 0x2C; // memory write
        const size_t command_size = 1;

        const size_t pixelsInBuf = sizeof(buf) / sizeof(uint16_t);
        for (size_t i = 0; i < pixelsInBuf; ++i)
        {
            buf[command_size + i * sizeof(uint16_t)] = highByte;
            buf[command_size + i * sizeof(uint16_t) + 1] = lowByte;
        }
        spi_write_buffer_dcx(buf, sizeof(buf), command_size);
#else
        set_draw_area((DISPLAY_WIDTH - width) / 2, (DISPLAY_HEIGHT - height) / 2, width, height);
        sendCommand(0x2C); // memory write

        const size_t pixelsInBuf = sizeof(buf) / sizeof(uint16_t);
        for (size_t i = 0; i < pixelsInBuf; ++i)
        {
            buf[i * sizeof(uint16_t)] = highByte;
            buf[i * sizeof(uint16_t) + 1] = lowByte;
        }
        spi_write_buffer(buf, sizeof(buf));
#endif
    }
}
