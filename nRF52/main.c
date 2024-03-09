/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "app_scheduler.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_service_gfx.h"
#include "st7789.h"
#include "trail_gfx.h"
#include "nrf_delay.h"
#include "nrf_power.h"

//AC21 P0.25 GPIO - unavailable
//AD22 P1.00 GPIO - pin marked as "0"
//Y23  P1.01 GPIO - unavailable
//U24  P1.04 GPIO - pin marked as "T"
//W24  P1.02 GPIO - unavailable

//R24  P1.06 GPIO - unavailable
//T23  P1.05 GPIO - pin marked as "R"
//P23  P1.07 GPIO - unavailable
//N24  DEC5  1.3V flash

//AD14 VDD
//AC13 P0.18 GPIO/nRESET (configurable as system reset) - not in the line to hrs
//AD12 P0.17 GPIO - 8 & 10 HRS line
//AC11 P0.16 GPIO - 6 HRS line
//AD10 P0.15 GPIO - 5 HRS
//AC9  P0.14 GPIO - 1 HRS, LED

//HRS line:
//1 - LED
//2 - VDD
//3 - GND
//4 - ? 4.2V
//5 - AD10
//6 - AC11
//7 - maybe upper line
//8 - AD12
//9 - maybe upper line
//10 - AD12

#define PIN_AC21  NRF_GPIO_PIN_MAP(0,25)
#define PIN_AD22  NRF_GPIO_PIN_MAP(1,0)
#define PIN_Y23   NRF_GPIO_PIN_MAP(1,1)
#define PIN_U24   NRF_GPIO_PIN_MAP(1,4)
#define PIN_W24   NRF_GPIO_PIN_MAP(1,2)

#define PIN_R24   NRF_GPIO_PIN_MAP(1,06)
#define PIN_T23   NRF_GPIO_PIN_MAP(1,05)
#define PIN_P23   NRF_GPIO_PIN_MAP(1,07)
#define PIN_AC13  NRF_GPIO_PIN_MAP(0,18)
#define PIN_AD12  NRF_GPIO_PIN_MAP(0,17)
#define PIN_AC11  NRF_GPIO_PIN_MAP(0,16)
#define PIN_AD10  NRF_GPIO_PIN_MAP(0,15)
#define PIN_AC9   NRF_GPIO_PIN_MAP(0,14)

#define PIN_MOTOR  NRF_GPIO_PIN_MAP(0, 20)
#define PIN_BACKLIGHT0  NRF_GPIO_PIN_MAP(0, 30)
#define PIN_BACKLIGHT1  NRF_GPIO_PIN_MAP(0, 22)
#define PIN_TOUCH_ENABLE  NRF_GPIO_PIN_MAP(1, 13)
#define PIN_TOUCH_RESET  NRF_GPIO_PIN_MAP(0, 24)
#define PIN_TOUCH_SCL  NRF_GPIO_PIN_MAP(1, 10)
#define PIN_TOUCH_SDA  NRF_GPIO_PIN_MAP(1, 11)
#define PIN_TOUCH_INT  NRF_GPIO_PIN_MAP(1, 12)
#define PIN_BUTTON1    NRF_GPIO_PIN_MAP(0, 16) // active low
#define PIN_HEART_SENSOR_ENABLE  NRF_GPIO_PIN_MAP(0, 17)
#define PIN_HEART_SENSOR_LED  NRF_GPIO_PIN_MAP(0, 14)
#define PIN_ACCELEROMETER_ENABLE  NRF_GPIO_PIN_MAP(0, 4)
#define PIN_ACCELEROMETER_STOP  NRF_GPIO_PIN_MAP(0, 6)
#define PIN_BATTERY_CHARGING  NRF_GPIO_PIN_MAP(1, 7) // active low


//#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
//#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
//#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
//#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "nRF5 Trail"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(1000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(3000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(3000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    0                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */


#define SCHED_MAX_EVENT_DATA_SIZE       APP_TIMER_SCHED_EVENT_DATA_SIZE             /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                          /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */
#endif


#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define QUEUED_WRITE_DATA_SIZE   560 // characteristic of 512 bytes needs a bit more space for prepared write


BLE_GFX_DEF(m_ble_gfx);                                                             /**< Graphics Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint8_t m_queued_write_data[QUEUED_WRITE_DATA_SIZE];
static uint8_t m_my_qwr_buff[QUEUED_WRITE_DATA_SIZE];

static trail_gfx_t m_gfx = {};
static uint8_t m_backlight_level = 0;

//static uint8_t m_gfx_data[512];
//static size_t m_gfx_data_size = 0;

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(PIN_MOTOR);
    nrf_gpio_cfg_output(PIN_BACKLIGHT0);
    nrf_gpio_cfg_output(PIN_BACKLIGHT1);
    nrf_gpio_cfg_output(PIN_TOUCH_ENABLE);
    nrf_gpio_cfg_output(PIN_TOUCH_RESET);
    nrf_gpio_cfg_output(PIN_TOUCH_SCL);
    nrf_gpio_cfg_output(PIN_TOUCH_SDA);
    nrf_gpio_cfg_output(PIN_TOUCH_INT);
    nrf_gpio_cfg_output(PIN_HEART_SENSOR_ENABLE);
    nrf_gpio_cfg_output(PIN_HEART_SENSOR_LED);
    nrf_gpio_cfg_output(PIN_ACCELEROMETER_ENABLE);
    nrf_gpio_cfg_output(PIN_ACCELEROMETER_STOP);

    nrf_gpio_pin_write(PIN_MOTOR, 0);
    nrf_gpio_pin_write(PIN_BACKLIGHT0, 0);
    nrf_gpio_pin_write(PIN_BACKLIGHT1, 0);
    nrf_gpio_pin_write(PIN_TOUCH_ENABLE, 0);
    nrf_gpio_pin_write(PIN_TOUCH_RESET, 0);
    nrf_gpio_pin_write(PIN_TOUCH_SCL, 0);
    nrf_gpio_pin_write(PIN_TOUCH_SDA, 0);
    nrf_gpio_pin_write(PIN_TOUCH_INT, 0);
    nrf_gpio_pin_write(PIN_HEART_SENSOR_ENABLE, 0);
    nrf_gpio_pin_write(PIN_HEART_SENSOR_LED, 0);
    nrf_gpio_pin_write(PIN_ACCELEROMETER_ENABLE, 0);
    nrf_gpio_pin_write(PIN_ACCELEROMETER_STOP, 0);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{GFX_UUID_SERVICE, m_ble_gfx.uuid_type}
    };

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;


    memset(&srdata, 0, sizeof(srdata));
//    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
//    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    NRF_LOG_ERROR("ERROR: nrf_qwr_error_handler %i", nrf_error);
//    APP_ERROR_HANDLER(nrf_error);
}

//static uint32_t get_low_bit(uint32_t value)
//{
//    return (value & 1);
//}

//static uint16_t Color8To16bit(uint16_t color8)
//{
//    // |-------------|-------------------|
//    // | color 8 bit |   color 16 bit    |
//    // |-------------|-------------------|
//    // | RGB (3:3:2) | RGB (5:6:5)       |
//    // | rrrgggbb => | rrrrrggg gggbbbbb |
//    // | abcDEFgh => | abcabDEF DEFghghg |
//    // |-------------|-------------------|
//
//    uint16_t color16 = 0;
//    {
//        const uint16_t r8 = color8 & 0xE0; // rrrgggbb => rrr00000
//        color16 |= ((r8 << 8) | (r8 << 5)) & 0xF800; // rxy00000 => rxyrx000 00000000
//    }
//    {
//        const uint16_t g8 = color8 & 0x1C; // rrrgggbb => 000ggg00
//        color16 |= (g8 << 6) | (g8 << 3); // 000gxy00 => 00000gxy gxy00000
//    }
//    {
//        const uint16_t b8 = color8 & 0x03; // rrrgggbb => 000000bb
//        color16 |= (b8 << 3) | (b8 << 1) | (b8 >> 1); // 000000bx => 00000000 000bxbxb
//    }
//    return color16;
//}

#pragma pack(push, 1)
typedef struct
{
    ble_gfx_command_e cmd;
    uint16_t color;
} ble_gfx_command_new_frame_t;

typedef struct
{
    ble_gfx_command_e cmd;
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
    uint16_t color;
    uint8_t width;
} ble_gfx_command_draw_line_t;

typedef struct
{
    ble_gfx_command_e cmd;
    int16_t x0;
    int16_t y0;
    uint8_t r;
    uint16_t color;
} ble_gfx_command_draw_circle_t;

typedef struct
{
    ble_gfx_command_e cmd;
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
    uint16_t color;
} ble_gfx_command_draw_triangle_t;
#pragma pack(pop)


static void gfx_command_execute(const uint8_t* p_data, size_t data_size)
{
    size_t index = 0;
    while (index < data_size)
    {
        const uint8_t* bytes = p_data + index;
        const size_t remaining_size = data_size - index;

        ble_gfx_command_e cmd_type = (ble_gfx_command_e)bytes[0];
        if (cmd_type == ble_gfx_command_new_frame)
        {
            const size_t required_size = sizeof(ble_gfx_command_new_frame_t);
            if (remaining_size >= required_size)
            {
                const ble_gfx_command_new_frame_t* p_cmd = (const ble_gfx_command_new_frame_t*)bytes;
                trail_gfx_fill_rect(&m_gfx, 0, 0, m_gfx.width, m_gfx.height, p_cmd->color);
            }
            else
            {
                NRF_LOG_INFO("ERROR: not enough size cmd=%i, index=%i, size=%i", (int)cmd_type, (int)index, (int)data_size);
            }
            index += required_size;
        }
        else if (cmd_type == ble_gfx_command_show_current_frame)
        {
            index += 1;
            st7789_powersave_end();
            st7789_send_screen();
            st7789_powersave_begin();
        }
        else if (cmd_type == ble_gfx_command_draw_line)
        {
            const size_t required_size = sizeof(ble_gfx_command_draw_line_t);
            if (remaining_size >= required_size)
            {
                const ble_gfx_command_draw_line_t* p_cmd = (const ble_gfx_command_draw_line_t*)bytes;
                trail_gfx_draw_line(&m_gfx, p_cmd->x0, p_cmd->y0, p_cmd->x1, p_cmd->y1, p_cmd->color, p_cmd->width);
            }
            else
            {
                NRF_LOG_INFO("ERROR: not enough size cmd=%i, index=%i, size=%i", (int)cmd_type, (int)index, (int)data_size);
            }
            index += required_size;
        }
        else if (cmd_type == ble_gfx_command_draw_circle)
        {
            const size_t required_size = sizeof(ble_gfx_command_draw_circle_t);
            if (remaining_size >= required_size)
            {
                const ble_gfx_command_draw_circle_t* p_cmd = (const ble_gfx_command_draw_circle_t*)bytes;
                trail_gfx_draw_circle(&m_gfx, p_cmd->x0, p_cmd->y0, p_cmd->r, p_cmd->color);
            }
            else
            {
                NRF_LOG_INFO("ERROR: not enough size cmd=%i, index=%i, size=%i", (int)cmd_type, (int)index, (int)data_size);
            }
            index += required_size;
        }
        else if (cmd_type == ble_gfx_command_fill_circle)
        {
            const size_t required_size = sizeof(ble_gfx_command_draw_circle_t);
            if (remaining_size >= required_size)
            {
                const ble_gfx_command_draw_circle_t* p_cmd = (const ble_gfx_command_draw_circle_t*)bytes;
                trail_gfx_fill_circle(&m_gfx, p_cmd->x0, p_cmd->y0, p_cmd->r, p_cmd->color);
            }
            else
            {
                NRF_LOG_INFO("ERROR: not enough size cmd=%i, index=%i, size=%i", (int)cmd_type, (int)index, (int)data_size);
            }
            index += required_size;
        }
        else if (cmd_type == ble_gfx_command_fill_triangle)
        {
            const size_t required_size = sizeof(ble_gfx_command_draw_triangle_t);
            if (remaining_size >= required_size)
            {
                const ble_gfx_command_draw_triangle_t* p_cmd = (const ble_gfx_command_draw_triangle_t*)bytes;
                trail_gfx_fill_triangle(&m_gfx, p_cmd->x0, p_cmd->y0, p_cmd->x1, p_cmd->y1, p_cmd->x2, p_cmd->y2, p_cmd->color);
            }
            else
            {
                NRF_LOG_INFO("ERROR: not enough size cmd=%i, index=%i, size=%i", (int)cmd_type, (int)index, (int)data_size);
            }
            index += required_size;
        }
        else
        {
            NRF_LOG_INFO("ERROR: unknown cmd=%i, index=%i, size=%i,", (int)cmd_type, (int)index, (int)data_size);
            break;
        }
    }
}

/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_gfx     Instance of Graphics Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void data_write_handler(uint16_t conn_handle, const ble_gfx_t* p_gfx, const uint8_t* p_data, size_t data_size)
{
    NRF_LOG_INFO("data_write_handler: size=%i", data_size);
//    m_gfx_data_size = data_size;
//    memcpy(m_gfx_data, p_data, data_size);
    gfx_command_execute(p_data, data_size);
    
//    if (data_size == 3)
//    {
//        st7789_fill_color(*((uint16_t*)p_data));
//        NRF_LOG_INFO("Received BLE value: 0x%02x 0x%02x 0x%02x", p_data[0], p_data[1], p_data[2]);
//    }
//    else
//    {
//        NRF_LOG_INFO("Received BLE value, size=%i", data_size);
//    }
    
    
//    st7789_fill_color(Color8To16bit(led_state));
//    nrf_gpio_pin_write(PIN_MOTOR, led_state & 0xF0);
//    nrf_gpio_pin_write(PIN_BACKLIGHT0, led_state & 0x01);
//    nrf_gpio_pin_write(PIN_BACKLIGHT1, led_state & 0x02);
//    NRF_LOG_INFO("Received BLE value: 0x%02x", led_state);

//    nrf_gpio_pin_write(PIN_R24,  get_low_bit(led_state >> 0));
//    nrf_gpio_pin_write(PIN_T23,  get_low_bit(led_state >> 1));
//    nrf_gpio_pin_write(PIN_P23,  get_low_bit(led_state >> 2));
//    nrf_gpio_pin_write(PIN_AC13, get_low_bit(led_state >> 3));
//    nrf_gpio_pin_write(PIN_AD12, get_low_bit(led_state >> 4));
//    nrf_gpio_pin_write(PIN_AC11, get_low_bit(led_state >> 5));
//    nrf_gpio_pin_write(PIN_AD10, get_low_bit(led_state >> 6));
//    nrf_gpio_pin_write(PIN_AC9,  get_low_bit(led_state >> 7));
//
//    if (led_state)
//    {
//        bsp_board_led_on(LEDBUTTON_LED);
//        NRF_LOG_INFO("Received LED ON!");
//    }
//    else
//    {
//        bsp_board_led_off(LEDBUTTON_LED);
//        NRF_LOG_INFO("Received LED OFF!");
//    }
}


/**@brief Function for handling events from the Queued Write module.
 */
uint16_t queued_write_handler(nrf_ble_qwr_t * p_qwr, nrf_ble_qwr_evt_t * p_evt)
{
    if(p_evt->evt_type == NRF_BLE_QWR_EVT_AUTH_REQUEST)
    {
//        NRF_LOG_INFO("queued_write_handler AUTH");
    }
    else if(p_evt->evt_type == NRF_BLE_QWR_EVT_EXECUTE_WRITE)
    {
        uint16_t data_size = QUEUED_WRITE_DATA_SIZE;
        ret_code_t err_code = nrf_ble_qwr_value_get(p_qwr, p_evt->attr_handle, m_my_qwr_buff, &data_size);
        NRF_LOG_INFO("queued_write_handler WRITE qwr_value_get returned %i, data_size=%i", err_code, data_size);
        if (p_evt->attr_handle == m_ble_gfx.data_char_handles.value_handle)
        {
            gfx_command_execute(m_my_qwr_buff, data_size);
        }
    }
    return BLE_GATT_STATUS_SUCCESS;//nrf_ble_qwrs_on_qwr_evt(&m_qwrs, p_qwr, p_evt);
}



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_gfx_init_t     gfx_init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.mem_buffer.len   = QUEUED_WRITE_DATA_SIZE;
    qwr_init.mem_buffer.p_mem = m_queued_write_data;
    qwr_init.error_handler    = nrf_qwr_error_handler;
    qwr_init.callback         = queued_write_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize GFX.
    gfx_init.width = 240;
    gfx_init.height = 240;
    gfx_init.data_write_handler = data_write_handler;
    gfx_init.p_qwr_ctx = &m_qwr;

    err_code = ble_gfx_init(&m_ble_gfx, &gfx_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
//    ret_code_t err_code;
//
//    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
//    {
//        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
//        APP_ERROR_CHECK(err_code);
//    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    NRF_LOG_ERROR("conn_params_error_handler %i", nrf_error);
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

//    bsp_board_led_on(ADVERTISING_LED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
//            bsp_board_led_on(CONNECTED_LED);
//            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
//            err_code = app_button_enable();
//            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
//            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
//            err_code = app_button_disable();
//            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
    
//        // my customized events instead of QWR service
//        case BLE_GATTS_EVT_WRITE:
//            NRF_LOG_INFO("BLE_GATTS_EVT_WRITE");
//            break;
//        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
//        {
//            ble_gatts_rw_authorize_reply_params_t auth_reply;
//            memset(&auth_reply, 0, sizeof(auth_reply));
//
////            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
//            auth_reply.params.write.gatt_status = NRF_BLE_QWR_REJ_REQUEST_ERR_CODE;
////            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_REQUEST_NOT_SUPPORTED;
//            auth_reply.type                     = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
//
//            err_code = sd_ble_gatts_rw_authorize_reply(m_conn_handle, &auth_reply);
//            NRF_LOG_INFO("sd_ble_gatts_rw_authorize_reply %i", err_code);
//            break;
//        }

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


static void set_backlight_level(uint8_t level)
{
    // level 1 - 4-5 mA
    // level 2 - 21 mA
    // level 3 - 23-24 mA
    m_backlight_level = level;
    nrf_gpio_pin_write(PIN_BACKLIGHT0, level & 0x01);
    nrf_gpio_pin_write(PIN_BACKLIGHT1, level & 0x02);
}



/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
//    ret_code_t err_code;

    switch (pin_no)
    {
        case PIN_BUTTON1:
            if (button_action == 0)
            {
                NRF_LOG_INFO("button press");
                ++m_backlight_level;
                if (m_backlight_level > 3)
                {
                    m_backlight_level = 0;
                }
                else if (m_backlight_level > 0)
                {
                    m_backlight_level = 3;
                }
                set_backlight_level(m_backlight_level);

                if (m_backlight_level == 0)
                {
                    st7789_sleep_begin();
//                    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
//                    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_STAY_IN_SYSOFF);
//                    ret_code_t err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
//                    if (err_code != NRF_SUCCESS)
//                    {
//                        NRF_LOG_INFO("sd_power_mode_set(NRF_POWER_MODE_LOWPWR) %i");
//                    }
                }
                else
                {
                    st7789_sleep_end();
//                    ret_code_t err_code = sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);
//                    if (err_code != NRF_SUCCESS)
//                    {
//                        NRF_LOG_INFO("sd_power_mode_set(NRF_POWER_MODE_CONSTLAT) %i");
//                    }
                }
            }
//            err_code = ble_lbs_on_button_change(m_conn_handle, &m_ble_gfx, button_action);
//            if (err_code != NRF_SUCCESS &&
//                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//                err_code != NRF_ERROR_INVALID_STATE &&
//                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
            break;
            
        case PIN_BATTERY_CHARGING:
            if (button_action == APP_BUTTON_PUSH)
            {
                NRF_LOG_INFO("charging");
            }
            else
            {
                NRF_LOG_INFO("not charging");
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {PIN_BUTTON1, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_event_handler},
        {PIN_BATTERY_CHARGING, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void motor_beep_ms(uint16_t duration_ms)
{
    nrf_gpio_pin_write(PIN_MOTOR, 1);
    nrf_delay_ms(duration_ms);
    nrf_gpio_pin_write(PIN_MOTOR, 0);
}

static void gfx_init(void)
{
    ret_code_t result = st7789_init();
    if (result == NRF_SUCCESS)
    {
//        for (uint8_t i = 0; i < 3; ++i)
//        {
//            nrf_gpio_pin_write(PIN_BACKLIGHT, 0);
//            nrf_delay_ms(200);
//            nrf_gpio_pin_write(PIN_BACKLIGHT, 1);
//            nrf_delay_ms(200);
//        }
//
        m_gfx.pixel_data = st7789_get_screen_buffer();
        m_gfx.width = st7789_get_width();
        m_gfx.height = st7789_get_height();
        trail_gfx_fill_rect(&m_gfx, 0, 0, m_gfx.width, m_gfx.height, 0x1111);
        NRF_LOG_INFO("before st7789_send_screen");
        st7789_send_screen();
        st7789_powersave_begin();
        NRF_LOG_INFO("after st7789_send_screen");
        set_backlight_level(3);
//        st7789_fill_color(0x6108);
        motor_beep_ms(50);

//        for (uint8_t i = 0; i < 3; ++i)
//        {
//            nrf_gpio_pin_write(PIN_BACKLIGHT, 0);
//            nrf_delay_ms(200);
//            nrf_gpio_pin_write(PIN_BACKLIGHT, 1);
//            nrf_delay_ms(200);
//        }
    }
    else
    {
        for (uint8_t i = 0; i < 10; ++i)
        {
            set_backlight_level(3);
            nrf_delay_ms(200);
            set_backlight_level(0);
            nrf_delay_ms(200);
        }
    }
}

static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


static void log_resetreason(void)
{
    /* Reset reason */
    uint32_t rr = nrf_power_resetreas_get();
    NRF_LOG_INFO("Reset reasons:");
    if (0 == rr)
    {
        NRF_LOG_INFO("- NONE");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_RESETPIN_MASK))
    {
        NRF_LOG_INFO("- RESETPIN");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_DOG_MASK     ))
    {
        NRF_LOG_INFO("- DOG");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_SREQ_MASK    ))
    {
        NRF_LOG_INFO("- SREQ");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_LOCKUP_MASK  ))
    {
        NRF_LOG_INFO("- LOCKUP");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_OFF_MASK     ))
    {
        NRF_LOG_INFO("- OFF");
    }
#if defined(NRF_POWER_RESETREAS_LPCOMP_MASK)
    if (0 != (rr & NRF_POWER_RESETREAS_LPCOMP_MASK  ))
    {
        NRF_LOG_INFO("- LPCOMP");
    }
#endif
    if (0 != (rr & NRF_POWER_RESETREAS_DIF_MASK     ))
    {
        NRF_LOG_INFO("- DIF");
    }
#if defined(NRF_POWER_RESETREAS_NFC_MASK)
    if (0 != (rr & NRF_POWER_RESETREAS_NFC_MASK     ))
    {
        NRF_LOG_INFO("- NFC");
    }
#endif
    if (0 != (rr & NRF_POWER_RESETREAS_VBUS_MASK    ))
    {
        NRF_LOG_INFO("- VBUS");
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    
    log_resetreason();
    nrf_power_resetreas_clear(nrf_power_resetreas_get());

    leds_init();
    timers_init();
    buttons_init();
    scheduler_init();
    power_management_init();
    // my
    {
        gfx_init();
        set_backlight_level(3);
        ret_code_t err_code = app_button_enable();
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_INFO("app_button_enable %i", err_code);
        }
    }
    ble_stack_init();
    NRF_LOG_INFO("after ble_stack_init");
    gap_params_init();
    NRF_LOG_INFO("after gap_params_init");
    gatt_init();
    NRF_LOG_INFO("after gatt_init");
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    NRF_LOG_INFO("TrailNavi example started.");
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
