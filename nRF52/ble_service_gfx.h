/** @file
 *
 * @defgroup ble_service_gfx Graphics Service
 * @{
 * @brief BLE Graphics Service module.
 *
 * @details This module implements the Graphics Service over BLE - drawing 2D primitives using commands.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_gfx_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_GFX_BLE_OBSERVER_PRIO,
 *                                   ble_gls_on_ble_evt, &instance);
 *          @endcode
 */


#ifndef BLE_SERVICE_GFX_H_INCLUDED
#define BLE_SERVICE_GFX_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_qwr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_GFX_BLE_OBSERVER_PRIO 2
/**@brief   Macro for defining a ble_gls instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_GFX_DEF(_name)                          \
    static ble_gfx_t _name;                         \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,             \
                         BLE_GFX_BLE_OBSERVER_PRIO, \
                         ble_gfx_on_ble_evt, &_name)


//000098E2-1C69-4041-A14A-C573893C4B1F
//1E6387F0-BE8C-40DA-8F76-8ED84C42065D

#define GFX_UUID_BASE          {0x5D, 0x06, 0x42, 0x4C, 0xD8, 0x8E, 0x76, 0x8F, \
                                0xDA, 0x40, 0x8C, 0xBE, 0x00, 0x00, 0x63, 0x1E}
#define GFX_UUID_SERVICE          0x87F0
#define GFX_UUID_CHAR_READ_STATE  0x87F1
#define GFX_UUID_CHAR_WRITE_DATA  0x87F2
#define GFX_UUID_CHAR_INDICATE    0x87F3

typedef enum
{
    ble_gfx_command_new_frame = 1,
    ble_gfx_command_show_current_frame = 2,
    ble_gfx_command_draw_line = 3,
    ble_gfx_command_draw_circle = 4,
    ble_gfx_command_fill_circle = 5,
    ble_gfx_command_fill_triangle = 6
} ble_gfx_command_e;

typedef struct ble_gfx_s ble_gfx_t;

typedef void (*ble_gfx_data_write_handler_t) (uint16_t conn_handle, const ble_gfx_t* p_gfx, const uint8_t* p_data, size_t data_size);

typedef struct
{
    ble_gfx_data_write_handler_t data_write_handler;
    uint8_t width;
    uint8_t height;
    nrf_ble_qwr_t*              p_qwr_ctx;         //!< pointer to the initialized queued write context
} ble_gfx_init_t;

/**@brief Graphics Service structure. This structure contains various status information for the service. */
struct ble_gfx_s
{
    uint16_t                    service_handle;     /**< Handle of Graphics Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    state_char_handles; /**< Handles related to the State Characteristic. */
    ble_gatts_char_handles_t    data_char_handles;  /**< Handles related to the Data Characteristic. */
    ble_gatts_char_handles_t    indicate_char_handles; /**< Handles related to the Indicate Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the Graphics Service. */
    ble_gfx_data_write_handler_t data_write_handler;   /**< Event handler to be called when the Data Characteristic is written. */
};

/**@brief Function for initializing the Graphics Service.
 *
 * @param[out] p_gfx      Graphics Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_gfx_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_gfx_init(ble_gfx_t* p_gfx, const ble_gfx_init_t* p_gfx_init);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the Graphics Service.
 *
 * @param[in] p_gfx      Graphics Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
void ble_gfx_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for sending an indication to request data update.
 *
 * @param[in] p_gfx      Gfx Service structure.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_gfx_request_data_update(uint16_t conn_handle, ble_gfx_t* p_gfx);


#ifdef __cplusplus
}
#endif

#endif // BLE_SERVICE_GFX_H_INCLUDED

/** @} */
