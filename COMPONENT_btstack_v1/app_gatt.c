/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#ifdef WICED_APP_LE_INCLUDED

/** @file
 *
 * This file implement BTLE application controlled over UART.
 * The GATT database is defined in this file and is not changed by the MCU.
 *
 */

#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "app.h"
#include "hci_control_le.h"
#include "cycfg_gatt_db.h"
#include "le_peripheral.h"

/******************************************************
 *                     Constants
 ******************************************************/

/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/
extern hci_control_le_cb_t le_control_cb;

/******************************************************
 *                   static Functions
 ******************************************************/
/* Get a Value */
wiced_bt_gatt_status_t app_gatt_get_value( uint16_t attr_handle, uint16_t conn_idx, uint8_t *p_val, uint16_t len_requested, uint16_t *p_len )
{
    int                    i;
    wiced_bool_t           is_handle_in_table = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            is_handle_in_table = WICED_TRUE;
            // Detected a matching handle in the external lookup table
            if (app_gatt_db_ext_attr_tbl[i].cur_len <= len_requested)
            {
                // Value fits within the supplied buffer; copy over the value
                *p_len = app_gatt_db_ext_attr_tbl[i].cur_len;
                memcpy(p_val, app_gatt_db_ext_attr_tbl[i].p_data, app_gatt_db_ext_attr_tbl[i].cur_len);
                res = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                // Value to read will not fit within the buffer
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!is_handle_in_table)
    {
        // TBD. If handle is not contained within external lookup table pass the read req to the host
        // res = WICED_BT_GATT_PENDING;
    }

    return res;
}

static wiced_bt_gatt_status_t app_gatt_read_handler( uint16_t conn_idx, wiced_bt_gatt_read_t *p_req )
{
    return app_gatt_get_value(p_req->handle, conn_idx, p_req->p_val, *p_req->p_val_len, p_req->p_val_len);
}

static wiced_result_t app_gatt_mtu_handler( uint16_t conn_idx, uint16_t mtu )
{
    le_control_cb.conn[conn_idx].peer_mtu   = mtu;
    return ( WICED_SUCCESS );
}

/******************************************************
 *                     Functions
 ******************************************************/
/*
 * Operation complete received from the GATT server
 */
wiced_result_t app_gatt_operation_comp_cb( wiced_bt_gatt_operation_complete_t *p_complete )
{
    uint16_t conn_idx = app_gatt_get_conn_idx(p_complete->conn_id);

    switch ( p_complete->op )
    {
    case GATTC_OPTYPE_DISCOVERY:
        WICED_BT_TRACE( "!!! Disc compl conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );
        break;

    case GATTC_OPTYPE_READ:
        hci_control_le_gatt_op_comp_read_handle(conn_idx, p_complete);
        break;

    case GATTC_OPTYPE_WRITE:
    case GATTC_OPTYPE_EXE_WRITE:
        hci_control_le_gatt_op_comp_write_handle(conn_idx, p_complete->status);
        break;

    case GATTC_OPTYPE_CONFIG:
        WICED_BT_TRACE( "Config conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        WICED_BT_TRACE( "Notification conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );
        hci_control_le_notification_handler( conn_idx,
                p_complete->response_data.att_value.handle,
                p_complete->response_data.att_value.p_data,
                p_complete->response_data.att_value.len );
        break;

    case GATTC_OPTYPE_INDICATION:
        WICED_BT_TRACE( "Indication conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );
        hci_control_le_indication_handler( conn_idx,
                p_complete->response_data.att_value.handle,
                p_complete->response_data.att_value.p_data,
                p_complete->response_data.att_value.len );
        break;
    }
    return ( WICED_SUCCESS );
}

/*
 * This is a GATT request callback
 */
wiced_bt_gatt_status_t app_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_req )
{
    wiced_bt_gatt_status_t result  = WICED_BT_GATT_SUCCESS;

    uint16_t conn_idx = p_req->conn_id;

    WICED_BT_TRACE( "GATT request conn_idx:%d type:%d\n", conn_idx, p_req->request_type );

    switch ( p_req->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = app_gatt_read_handler( conn_idx, &p_req->data.read_req );
            break;

        case GATTS_REQ_TYPE_WRITE:
            result = hci_control_le_write_handler( conn_idx, &p_req->data.write_req );
             break;

        case GATTS_REQ_TYPE_WRITE_EXEC:
            result = hci_control_le_write_exec_handler( conn_idx, p_req->data.exec_write );
            break;

        case GATTS_REQ_TYPE_MTU:
            result = app_gatt_mtu_handler( conn_idx, p_req->data.mtu );
            break;

        case GATTS_REQ_TYPE_CONF:
            result = hci_control_le_conf_handler( conn_idx, p_req->data.handle );
            break;

       default:
            break;
    }

    return result;
}

/*
 * This function sends write to the peer GATT server
 */
wiced_bt_gatt_status_t app_gatt_send_write( uint8_t conn_idx, uint16_t attr_handle, uint8_t *p_data, uint16_t len, wiced_bt_gatt_write_type_t type )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INSUF_RESOURCE;
    uint16_t conn_id = conn_idx;

    // Allocating a buffer to send the write request
    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( GATT_RESPONSE_SIZE( len ) );

    if ( p_write )
    {
        p_write->handle   = attr_handle;
        p_write->offset   = 0;
        p_write->len      = len;
        p_write->auth_req = GATT_AUTH_REQ_NONE;
        memcpy( p_write->value, p_data, len );

        // Register with the server to receive notification
        status = wiced_bt_gatt_send_write ( conn_id, type, p_write );

        WICED_BT_TRACE( "wiced_bt_gatt_send_write status:%d\n", status );

        wiced_bt_free_buffer( p_write );
    }
    return ( status );
}

wiced_bt_gatt_status_t app_gatt_client_send_read_handle( uint16_t conn_idx, uint16_t handle )
{
    wiced_bt_gatt_read_param_t read_req;

    // execute read procedure
    memset( &read_req, 0, sizeof( wiced_bt_gatt_read_param_t ) );

    read_req.by_handle.auth_req = GATT_AUTH_REQ_NONE;
    read_req.by_handle.handle = handle;

    return wiced_bt_gatt_send_read( conn_idx, GATT_READ_BY_HANDLE, &read_req );
}

wiced_bt_gatt_status_t app_gatt_send_response(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    return wiced_bt_gatt_send_response( WICED_BT_GATT_SUCCESS, conn_id, handle, len, 0, p_data );
}

/*
 * GATT operation started by the client has been completed
 */
wiced_bt_gatt_status_t app_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch (p_data->op)
    {
    case GATTC_OPTYPE_READ:
        watch_process_read_rsp(p_data);
        break;

    case GATTC_OPTYPE_WRITE:
        watch_process_write_rsp(p_data);
        break;

    case GATTC_OPTYPE_CONFIG:
        WICED_BT_TRACE("peer mtu:%d\n", p_data->response_data.mtu);
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        watch_notification_handler(p_data);
        break;

    case GATTC_OPTYPE_INDICATION:
        watch_indication_handler(p_data);
        break;
    }
    return WICED_BT_GATT_SUCCESS;
}

void app_gatt_send_read_by_handle(uint16_t conn_id, uint16_t handle)
{
    wiced_bt_gatt_status_t     status;
    wiced_bt_gatt_read_param_t param;

    memset(&param, 0, sizeof(param));
    param.by_handle.handle = handle;

    status = wiced_bt_gatt_send_read(conn_id, GATT_READ_BY_HANDLE, &param);

    WICED_BT_TRACE("wiced_bt_gatt_send_read %d\n", status);
}

wiced_bool_t app_gatt_send_read_by_type(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle, uint16_t uuid)
{
    wiced_bt_gatt_status_t     status;
    wiced_bt_gatt_read_param_t param;

    memset(&param, 0, sizeof(param));
    param.char_type.s_handle        = s_handle;
    param.char_type.e_handle        = e_handle;
    param.char_type.uuid.len        = 2;
    param.char_type.uuid.uu.uuid16  = uuid;

    status = wiced_bt_gatt_send_read(conn_id, GATT_READ_BY_TYPE, &param);

    WICED_BT_TRACE("wiced_bt_gatt_send_read %d\n", status);
    return (status == WICED_BT_SUCCESS);
}

#endif // WICED_APP_LE_INCLUDED
