/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "app.h"
#include "hci_control_le.h"
#include "wiced_app_cfg.h"
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
static wiced_bt_gatt_status_t app_gatt_get_value( uint16_t attr_handle, uint16_t conn_idx,
        wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_t *p_read_data, uint16_t len_requested)
{
    int                    i;
    wiced_bool_t           is_handle_in_table = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;
    uint16_t conn_id = app_gatt_get_conn_id(conn_idx);

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
                int attr_len_to_copy = app_gatt_db_ext_attr_tbl[i].cur_len;
                if (attr_len_to_copy != 0)
                {
                    uint8_t *from;
                    int     to_copy = MIN(len_requested, attr_len_to_copy - p_read_data->offset);
                    from = ((uint8_t *)app_gatt_db_ext_attr_tbl[i].p_data) + p_read_data->offset;
                    if (to_copy < 0)
                    {
                        res = WICED_BT_GATT_INVALID_OFFSET;
                    }
                    else
                    {
                        wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_copy, from, NULL);
                        res = WICED_BT_GATT_SUCCESS;
                    }
                }
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

    if (res != WICED_BT_GATT_SUCCESS)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_data->handle,
                res);
    }
    return res;
}

static wiced_bt_gatt_status_t app_gatt_read_handler( uint16_t conn_idx, wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_t *p_req, uint16_t len_requested)
{
    return app_gatt_get_value(p_req->handle, conn_idx, opcode, p_req, len_requested);
}

static wiced_result_t app_gatt_mtu_handler( uint16_t conn_idx, uint16_t mtu )
{
    le_control_cb.conn[conn_idx].peer_mtu   = mtu;

    wiced_bt_gatt_server_send_mtu_rsp(app_gatt_get_conn_id(conn_idx),
            mtu, wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);

    return ( WICED_SUCCESS );
}

/******************************************************
 *                     Functions
 ******************************************************/
/*
 * Helper function for mapping between conn_id and conn_idx
 */
uint16_t app_gatt_allocate_conn_cb (void)
{
    uint8_t i;
    for (i = 0; i < (sizeof(le_control_cb.conn)/sizeof(hci_control_le_conn_state_t)); i++)
    {
        if (le_control_cb.conn[i].conn_id == 0)
        {
            return i;
        }
    }

    WICED_BT_TRACE("app_gatt_allocate_conn_cb fail!!! \n");
    return 0xffff;
}

uint16_t app_gatt_get_conn_idx (uint16_t conn_id)
{
    uint8_t i;
    for (i = 0; i < (sizeof(le_control_cb.conn)/sizeof(hci_control_le_conn_state_t)); i++)
    {
        if (le_control_cb.conn[i].conn_id == conn_id)
        {
            return i;
        }
    }
    WICED_BT_TRACE("hci_control_le_get_conn_idx fail %d !!! \n", conn_id);
    return 0xffff;
}

uint16_t app_gatt_get_conn_id (uint16_t conn_idx)
{
    if (conn_idx < (sizeof(le_control_cb.conn)/sizeof(hci_control_le_conn_state_t)))
        return le_control_cb.conn[conn_idx].conn_id;

    WICED_BT_TRACE("hci_control_le_get_conn_id fail %d !!! \n", conn_idx);
    return 0xffff;
}

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

    case GATTC_OPTYPE_READ_HANDLE:
        hci_control_le_gatt_op_comp_read_handle(conn_idx, p_complete);
        break;

    case GATTC_OPTYPE_WRITE_WITH_RSP:
    case GATTC_OPTYPE_WRITE_NO_RSP:
    case GATTC_OPTYPE_EXECUTE_WRITE:
        hci_control_le_gatt_op_comp_write_handle(conn_idx, p_complete->status);
        break;

    case GATTC_OPTYPE_CONFIG_MTU:
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

    uint16_t conn_idx = app_gatt_get_conn_idx(p_req->conn_id);

    WICED_BT_TRACE( "GATT request conn_idx:%d type:%d\n", conn_idx, p_req->opcode );

    switch ( p_req->opcode )
    {
        case GATT_REQ_READ:
            result = app_gatt_read_handler( conn_idx, p_req->opcode, &p_req->data.read_req, p_req->len_requested);
            break;

        case GATT_REQ_READ_BY_TYPE:
            /* TODO: support for read_by_type handler */
            WICED_BT_TRACE("READ_BY_TYPE not support\n");
            wiced_bt_gatt_server_send_error_rsp(p_req->conn_id, p_req->opcode,
                    p_req->data.read_by_type.s_handle, WICED_BT_GATT_REQ_NOT_SUPPORTED);
            break;

        case GATT_REQ_WRITE:
            result = hci_control_le_write_handler( conn_idx, &p_req->data.write_req );
            if (result == WICED_BT_GATT_SUCCESS)
            {
                wiced_bt_gatt_server_send_write_rsp(p_req->conn_id, p_req->opcode, p_req->data.write_req.handle);
            }
            else
            {
                wiced_bt_gatt_server_send_error_rsp(p_req->conn_id, p_req->opcode, p_req->data.write_req.handle,
                        result);
            }
             break;

        case GATT_REQ_EXECUTE_WRITE:
            result = hci_control_le_write_exec_handler( conn_idx, p_req->data.exec_write_req.exec_write );
            wiced_bt_gatt_server_send_execute_write_rsp(p_req->conn_id, p_req->opcode);
            break;

        case GATT_REQ_MTU:
            result = app_gatt_mtu_handler( conn_idx, p_req->data.remote_mtu );
            break;

        case GATT_HANDLE_VALUE_CONF:
            result = hci_control_le_conf_handler( conn_idx, p_req->data.confirm.handle );
            break;

       default:
            WICED_BT_TRACE("Unhandled GATT request\n");
            break;
    }

    return result;
}

/*
 * This function sends write to the peer GATT server
 */
wiced_bt_gatt_status_t app_gatt_send_write( uint8_t conn_idx, uint16_t attr_handle, uint8_t *p_data, uint16_t len, wiced_bt_gatt_opcode_t type )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INSUF_RESOURCE;
    wiced_bt_gatt_write_hdr_t hdr;
    uint16_t conn_id = app_gatt_get_conn_id(conn_idx);

    // Allocating a buffer to send the write request
    uint8_t *p_write = (uint8_t *)wiced_bt_get_buffer(len);

    if ( p_write )
    {
        hdr.handle   = attr_handle;
        hdr.offset   = 0;
        hdr.len      = len;
        hdr.auth_req = GATT_AUTH_REQ_NONE;
        memcpy(p_write, p_data, len );

        // Register with the server to receive notification
        status = wiced_bt_gatt_client_send_write ( conn_id, type, &hdr, p_write, NULL);

        WICED_BT_TRACE( "wiced_bt_gatt_client_send_write status:%d", status );

        wiced_bt_free_buffer( p_write );
    }

    return ( status );
}

wiced_bt_gatt_status_t app_gatt_send_response(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len)
{
    return wiced_bt_gatt_server_send_read_handle_rsp( conn_id, GATT_RSP_READ, len, p_data, NULL );
}

/*
 * GATT operation started by the client has been completed
 */
wiced_bt_gatt_status_t app_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch (p_data->op)
    {
    case GATTC_OPTYPE_READ_HANDLE:
        watch_process_read_rsp(p_data);
        break;

    case GATTC_OPTYPE_WRITE_WITH_RSP:
    case GATTC_OPTYPE_WRITE_NO_RSP:
        watch_process_write_rsp(p_data);
        break;

    case GATTC_OPTYPE_CONFIG_MTU:
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

void watch_util_send_read_by_handle(uint16_t conn_id, uint16_t handle)
{
    wiced_bt_gatt_status_t     status;
    status = wiced_bt_gatt_client_send_read_handle(conn_id, handle, 0,
            NULL, 0, GATT_AUTH_REQ_NONE);

    WICED_BT_TRACE("wiced_bt_gatt_client_send_read_handle %d\n", status);
}

wiced_bool_t watch_util_send_read_by_type(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle, uint16_t uuid)
{
    wiced_bt_gatt_status_t     status;
    wiced_bt_uuid_t uuid_buf;

    uuid_buf.len = 2;
    uuid_buf.uu.uuid16 = uuid;

    status = wiced_bt_gatt_client_send_read_by_type(conn_id, s_handle, e_handle,
            &uuid_buf, NULL, 0, GATT_AUTH_REQ_NONE);

    WICED_BT_TRACE("wiced_bt_gatt_client_send_read_by_type %d\n", status);
    return (status != WICED_BT_GATT_SUCCESS);
}

#endif // WICED_APP_LE_INCLUDED
