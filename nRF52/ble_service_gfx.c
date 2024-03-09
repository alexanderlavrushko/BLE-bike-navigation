#include "sdk_common.h"
//#if NRF_MODULE_ENABLED(BLE_SERVO)
#include "ble_service_gfx.h"
#include "ble_srv_common.h"
#include "nrf_log.h"

/**@brief Function for handling the Write event.
 *
 * @param[in] p_gfx      Graphics Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(const ble_gfx_t* p_gfx, const ble_evt_t* p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (   (p_evt_write->handle == p_gfx->data_char_handles.value_handle)
        && (p_gfx->data_write_handler != NULL))
    {
        p_gfx->data_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_gfx, p_evt_write->data, p_evt_write->len);
    }
}

static void on_rw_authorize_request(const ble_gfx_t* p_gfx, const ble_evt_t* p_ble_evt)
{
    const ble_gatts_evt_rw_authorize_request_t * p_auth_req = &p_ble_evt->evt.gatts_evt.params.authorize_request;

    if ((p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) &&
        (p_auth_req->request.write.handle == p_gfx->data_char_handles.value_handle) &&
        (p_auth_req->request.write.op != BLE_GATTS_OP_PREP_WRITE_REQ) &&
        (p_auth_req->request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) &&
        (p_auth_req->request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
    {
        ble_gatts_rw_authorize_reply_params_t auth_reply;
        memset(&auth_reply, 0, sizeof(auth_reply));

        auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
        auth_reply.type                     = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
        auth_reply.params.write.update      = 1;

        ret_code_t err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("sd_ble_gatts_rw_authorize_reply=%i", err_code);
        }

        if (p_gfx->data_write_handler != NULL)
        {
            size_t len = p_auth_req->request.write.len;
            const uint8_t* data = p_auth_req->request.write.data;
            p_gfx->data_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_gfx, data, len);
        }
    }
}

void ble_gfx_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
//    NRF_LOG_DEBUG("ble_gfx_on_ble_evt %i", p_ble_evt->header.evt_id);

    ble_gfx_t* p_gfx = (ble_gfx_t*)p_context;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_gfx, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_rw_authorize_request(p_gfx, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding the State Characteristic.
 *
 * @param[in] p_gfx      Graphics Service structure.
 * @param[in] p_gfx_init Graphics Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t state_char_add(ble_gfx_t* p_gfx, const ble_gfx_init_t* p_gfx_init)
{
    uint8_t value[2] = { p_gfx_init->width, p_gfx_init->height };

    ble_add_char_params_t add_char_params;
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = GFX_UUID_CHAR_READ_STATE;
    add_char_params.uuid_type        = p_gfx->uuid_type;
    add_char_params.init_len         = 2 * sizeof(uint8_t);
    add_char_params.p_init_value     = value;
    add_char_params.max_len          = 2 * sizeof(uint8_t);
    add_char_params.char_props.read  = 1;
    add_char_params.char_props.write = 0;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_NO_ACCESS;

    return characteristic_add(p_gfx->service_handle, &add_char_params, &p_gfx->state_char_handles);
}


/**@brief Function for adding the Data Characteristic.
 *
 * @param[in] p_gfx      Graphics Service structure.
 * @param[in] p_gfx_init Graphics Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t write_char_add(ble_gfx_t* p_gfx, const ble_gfx_init_t* p_gfx_init)
{
//    ble_add_char_params_t add_char_params;
//    memset(&add_char_params, 0, sizeof(add_char_params));
//    add_char_params.uuid             = GFX_UUID_CHAR_WRITE_DATA;
//    add_char_params.uuid_type        = p_gfx->uuid_type;
//    add_char_params.init_len         = 0;
//    add_char_params.max_len          = 510;
////    add_char_params.char_props.write_wo_resp = 1;
//    add_char_params.char_props.write = 1;
//
//    add_char_params.write_access = SEC_OPEN;
//
//    ret_code_t err_code = characteristic_add(p_gfx->service_handle, &add_char_params, &p_gfx->data_char_handles);
//    VERIFY_SUCCESS(err_code);
//    return err_code;

//    if(qwr)
    {
        ble_add_char_params_t add_char_params;
        memset(&add_char_params, 0, sizeof(add_char_params));
        add_char_params.uuid             = GFX_UUID_CHAR_WRITE_DATA;
        add_char_params.uuid_type        = p_gfx->uuid_type;
        add_char_params.init_len         = 0;
        add_char_params.max_len          = 510;
        add_char_params.char_props.read  = 0;
        add_char_params.char_props.write = 1;

        add_char_params.read_access  = SEC_NO_ACCESS;
        add_char_params.write_access = SEC_OPEN;
        add_char_params.is_defered_write = 1;

        ret_code_t err_code = characteristic_add(p_gfx->service_handle, &add_char_params, &p_gfx->data_char_handles);
        VERIFY_SUCCESS(err_code);

        if (p_gfx_init->p_qwr_ctx != NULL)
        {
            err_code = nrf_ble_qwr_attr_register(p_gfx_init->p_qwr_ctx,
                                                 p_gfx->data_char_handles.value_handle);
            NRF_LOG_INFO("nrf_ble_qwr_attr_register=%i", err_code);
            VERIFY_SUCCESS(err_code);
        }
        return err_code;
    }
}

/**@brief Function for adding the Indicate Characteristic.
 *
 * @param[in] p_gfx      Graphics Service structure.
 * @param[in] p_gfx_init Graphics Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t indicate_char_add(ble_gfx_t* p_gfx, const ble_gfx_init_t* p_gfx_init)
{
    ble_add_char_params_t add_char_params;
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = GFX_UUID_CHAR_INDICATE;
    add_char_params.uuid_type         = p_gfx->uuid_type;
    add_char_params.init_len          = 0;
    add_char_params.max_len           = 0;
    add_char_params.char_props.read   = 0;
    add_char_params.char_props.indicate = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_gfx->service_handle,
                                  &add_char_params,
                                  &p_gfx->indicate_char_handles);
}


uint32_t ble_gfx_init(ble_gfx_t* p_gfx, const ble_gfx_init_t* p_gfx_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_gfx->data_write_handler = p_gfx_init->data_write_handler;

    // Add service.
    ble_uuid128_t base_uuid = {GFX_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_gfx->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_gfx->uuid_type;
    ble_uuid.uuid = GFX_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_gfx->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = state_char_add(p_gfx, p_gfx_init);
    VERIFY_SUCCESS(err_code);

    err_code = write_char_add(p_gfx, p_gfx_init);
    VERIFY_SUCCESS(err_code);

    err_code = indicate_char_add(p_gfx, p_gfx_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t ble_gfx_request_data_update(uint16_t conn_handle, ble_gfx_t* p_gfx)
{
    ble_gatts_hvx_params_t params;
    memset(&params, 0, sizeof(params));
    uint16_t len = 0;

    params.type = BLE_GATT_HVX_INDICATION;
    params.handle = p_gfx->indicate_char_handles.value_handle;
    params.p_data = NULL;
    params.p_len = &len;

    return sd_ble_gatts_hvx(conn_handle, &params);
}
//#endif // NRF_MODULE_ENABLED(ble_servo)
