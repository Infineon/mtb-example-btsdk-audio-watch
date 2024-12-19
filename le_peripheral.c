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

/** @file
*
* Watch Reference application
*
* The watch reference application combines multiple services and clients
* commonly used in Bluetooth LE watches including Apple's vendor specific ANCS,
* AMS, and Time.  Device works in the
* peripheral mode accepting connection from central typically a phone.
* The GATT database for the device includes definitions for all services.
*
* After pairing application performs GATT discovery of the connected
* device.  This module figures out what services are available and then
* each particular modules performs discovery of the characteristics of
* each particular service.  Similarly for each notification/indication
* message is passed to each particular module.
*
* Features demonstrated
*  - Executing multiple clients/services in the same application
*  - GATT database and Device configuration initialization
*  - Registration with LE stack for various events
*  - NVRAM read/write operation
*  - Processing control and data from the client
*  - Processing indications and notifications from the servers
*  - Sending data to the client
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Start tracing (see Kit Guide)
* 3. Pair with a client (iOS device)
* 4. Send SMS/incoming call to the phone and verify traces
* 5. Change Time on the phone and see notification in traces
* 6. Play music on the phone.  Push button on the tag to toggle play/stop
*
*/

#include "le_peripheral.h"

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_uuid.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_result.h"

#include "hci_control_api.h"
#ifdef WICED_APP_ANCS_INCLUDED
#include "wiced_bt_ancs.h"
#endif
#ifdef WICED_APP_AMS_INCLUDED
#include "wiced_bt_ams.h"
#endif
#include "hci_control.h"
#include "string.h"
#include "wiced_memory.h"
#include "wiced_transport.h"
#include "app.h"
#include "wiced_timer.h"

/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *               Function Prototypes
 ******************************************************/

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static void                   peripheral_timeout           (TIMER_PARAM_TYPE count);

static void                   watch_init_next_client       (uint8_t index);

/******************************************************
 *               Variables Definitions
 ******************************************************/
#ifdef CYW55572
#ifndef PACKED
#define PACKED
#endif
#endif

#pragma pack(1)
// host information for NVRAM
typedef PACKED struct
{
    // BD address of the bonded host
    BD_ADDR  bdaddr;

    // Current value of the client configuration descriptor for characteristic 'Report'
    uint16_t ancs_s_handle;
    uint16_t ancs_e_handle;
    uint16_t ams_s_handle;
    uint16_t ams_e_handle;
}  HOSTINFO;

#pragma pack()

#ifndef MAX_PHONE_CONNECTIONS
#define MAX_PHONE_CONNECTIONS 2
#endif
// NVRAM save area
HOSTINFO watch_hostinfo[MAX_PHONE_CONNECTIONS];

watch_app_state_t watch_app_state[MAX_PHONE_CONNECTIONS];
#define        ANCS_EMPTY_CONN_INDEX    0x00
#define        ANCS_INVALID_CONN_INDEX  0xff
/******************************************************
 *               Function Definitions
 ******************************************************/
#ifdef WICED_APP_ANCS_INCLUDED
static void le_peripheral_ancs_client_event_handler_notification(uint8_t index, wiced_bt_ancs_client_notification_data_t *p_data)
{
    // Allocating a buffer to send the trace
    uint8_t *p_tx_buf = (uint8_t *) wiced_bt_get_buffer(sizeof(wiced_bt_ancs_client_notification_data_t)+2);

    WICED_BT_TRACE("ANCS notification index:%d, UID:%d command:%d category:%d flags:%04x\n", index, p_data->basic.notification_uid, p_data->basic.command, p_data->basic.category, p_data->basic.flags);
    WICED_BT_TRACE("*******:%s:%s:%s:%s:\n", p_data->info.title, p_data->info.message, p_data->info.positive_action_label, p_data->info.negative_action_label);

    if (p_tx_buf)
    {
        int len;
        p_tx_buf[0] = watch_app_state[index].conn_id;
        p_tx_buf[1] = watch_app_state[index].conn_id>>8;

        p_tx_buf[2] = p_data->basic.notification_uid & 0xff;
        p_tx_buf[3] = (p_data->basic.notification_uid >> 8) & 0xff;
        p_tx_buf[4] = (p_data->basic.notification_uid >> 16) & 0xff;
        p_tx_buf[5] = (p_data->basic.notification_uid >> 24) & 0xff;

        p_tx_buf[6] = p_data->basic.command;

        p_tx_buf[7] = p_data->basic.category;

        p_tx_buf[8] = p_data->basic.flags;

        len = 9;

        utl_strcpy((char *) &p_tx_buf[len], (char *) p_data->info.title);
        len += (strlen((const char *) p_data->info.title) + 1);

        utl_strcpy((char *) &p_tx_buf[len],(char *) p_data->info.message);
        len += (strlen((const char *) p_data->info.message) + 1);

        utl_strcpy((char *) &p_tx_buf[len], (char *) p_data->info.positive_action_label);
        len += (strlen((const char *) p_data->info.positive_action_label) + 1);

        utl_strcpy((char *) &p_tx_buf[len], (char *) p_data->info.negative_action_label);
        len += (strlen((const char *) p_data->info.negative_action_label) + 1);

        wiced_transport_send_data(HCI_CONTROL_ANCS_EVENT_NOTIFICATION, p_tx_buf, len);
        wiced_bt_free_buffer( p_tx_buf );
    }
    else
    {
        WICED_BT_TRACE("le_peripheral_ancs_client_event_handler_notification FAILED!!\n");
    }
}

static void le_peripheral_ancs_client_event_handler(uint8_t index, wiced_bt_ancs_client_event_t event, wiced_bt_ancs_client_event_data_t *p_data)
{
    int i;
    uint8_t tx_buf[3];
    uint8_t *p = tx_buf;

    switch (event)
    {
    case WICED_BT_ANCS_CLIENT_EVENT_INITIALIZED:
        *p++ = (watch_app_state[index].conn_id) & 0xff;
        *p++ = (watch_app_state[index].conn_id >> 8) & 0xff;
        *p++ = 0;
        WICED_BT_TRACE("send HCI_CONTROL_ANCS_EVENT_CONNECTED\n");
        wiced_transport_send_data(HCI_CONTROL_ANCS_EVENT_CONNECTED, tx_buf, sizeof(tx_buf));

        watch_init_next_client(index);
        break;

    case WICED_BT_ANCS_CLIENT_EVENT_NOTIFICATION:
        le_peripheral_ancs_client_event_handler_notification(index, p_data->notification.p_data);
        break;

    default:
        break;
    }
}

#endif

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
void le_peripheral_app_init(void)
{
    uint8_t i = 0;
#ifdef WICED_APP_ANCS_INCLUDED
    wiced_bt_ancs_client_config_t ancs_client_config = {0};
#endif
    for (i = 0; i < MAX_PHONE_CONNECTIONS; i++)
    {
        memset(&watch_hostinfo[i], 0, sizeof(HOSTINFO));
        memset(&watch_app_state[i], 0, sizeof(watch_app_state_t));
        watch_app_state[i].timer_param = i;
#if BTSTACK_VER >= 0x03000001
        wiced_init_timer(&watch_app_state[i].timer, peripheral_timeout,
                         (TIMER_PARAM_TYPE)&watch_app_state[i].timer_param,
                         WICED_SECONDS_TIMER);
#else
        wiced_init_timer(&watch_app_state[i].timer, peripheral_timeout,
                         (TIMER_PARAM_TYPE)watch_app_state[i].timer_param,
                         WICED_SECONDS_TIMER);
#endif
    }
#ifdef WICED_APP_ANCS_INCLUDED
    /* Initialize the ANCS client. */
    ancs_client_config.p_event_handler = &le_peripheral_ancs_client_event_handler;

    if (wiced_bt_ancs_client_initialize(MAX_PHONE_CONNECTIONS, &ancs_client_config) == WICED_FALSE)
    {
        WICED_BT_TRACE("Err: wiced_bt_ancs_client_initialize failed\n");
    }
#endif
}

uint8_t find_index_by_conn_id(uint16_t conn_id)
{
    uint8_t index = 0;
    for (index = 0; index < MAX_PHONE_CONNECTIONS; index++)
    {
        if (watch_app_state[index].conn_id == conn_id)
        {
            return index;
        }
    }
    return ANCS_INVALID_CONN_INDEX;
}

uint8_t find_index_by_address(BD_ADDR remote_addr)
{
    uint8_t index = 0;
    for (index = 0; index < MAX_PHONE_CONNECTIONS; index++)
    {
        if (memcmp((void *) watch_app_state[index].remote_addr,
                   (void *) remote_addr,
                   sizeof(BD_ADDR)) == 0)
        {
            return index;
        }
    }
    return ANCS_INVALID_CONN_INDEX;
}

wiced_bool_t is_le_peripheral_new_connection_avalialbe(void)
{
    if (ANCS_INVALID_CONN_INDEX == find_index_by_conn_id(ANCS_EMPTY_CONN_INDEX))
    {
        return WICED_FALSE;
    }
    else
    {
        return WICED_TRUE;
    }
}
/*
 * This function will be called when a connection is established in LE Peripheral Role
 */
void le_peripheral_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    // find empty slot for save new connection.
    uint8_t index = find_index_by_conn_id(ANCS_EMPTY_CONN_INDEX);
    if (index == ANCS_INVALID_CONN_INDEX)
    {
        WICED_BT_TRACE("exceed max connection number:%d\n", MAX_PHONE_CONNECTIONS);
        return;
    }
    watch_app_state[index].conn_id = p_conn_status->conn_id;

    // save address of the connected device and print it out.
    memcpy(watch_app_state[index].remote_addr, p_conn_status->bd_addr, sizeof(watch_app_state[index].remote_addr));
    watch_app_state[index].addr_type = p_conn_status->addr_type;
    watch_app_state[index].transport = p_conn_status->transport;

#ifdef WICED_APP_ANCS_INCLUDED
    wiced_bt_ancs_client_connection_up(p_conn_status);
#endif
#ifdef WICED_APP_AMS_INCLUDED
    wiced_bt_ams_client_connection_up(p_conn_status);
#endif

    /* Connected as Peripheral. Start discovery in couple of seconds to give time to the peer device
     * to find/configure our services */
    wiced_start_timer(&watch_app_state[index].timer, 2);
}

// This function will be called when connection goes down
void le_peripheral_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    uint8_t index = find_index_by_conn_id(p_conn_status->conn_id);
    if (index ==  ANCS_INVALID_CONN_INDEX)
    {
        WICED_BT_TRACE("disconnect conn_id:[%d] error\n", p_conn_status->conn_id);
        return;
    }
    watch_app_state[index].conn_id = 0;
    watch_app_state[index].encrypted = WICED_FALSE;
    memset(&watch_hostinfo[index], 0, sizeof(HOSTINFO));

    wiced_stop_timer(&watch_app_state[index].timer);
#ifdef WICED_APP_ANCS_INCLUDED
    wiced_bt_ancs_client_connection_down(index, p_conn_status);
#endif
#ifdef WICED_APP_AMS_INCLUDED
    wiced_bt_ams_client_connection_down(index, p_conn_status);
#endif
}

// Process encryption status changed notification from the stack
void le_peripheral_encryption_status_changed(wiced_bt_dev_encryption_status_t *p_status)
{
    wiced_result_t result;
    uint8_t role;
    uint8_t index = find_index_by_address(p_status->bd_addr);
    if (index == ANCS_INVALID_CONN_INDEX)
    {
        WICED_BT_TRACE("[%s] find conn_id error\n", __FUNCTION__);
        return;
    }
    /* Ignore event if Encryption failed */
    if (p_status->result != WICED_BT_SUCCESS)
        return;

    /* Check if it's a Peripheral/Client device */
    if (memcmp(watch_app_state[index].remote_addr, p_status->bd_addr, sizeof(watch_app_state[index].remote_addr)))
    {
        /* Handle Race condition with already paired iPhone. In this case,
         * BTM_ENCRYPTION_STATUS_EVT is received before GATT_CONNECTION_STATUS_EVT
         */
        result = wiced_bt_dev_get_role(p_status->bd_addr, &role, BT_TRANSPORT_LE);
        if ((result != WICED_BT_SUCCESS) || (role != HCI_ROLE_PERIPHERAL))
        {
            /* This is, definitely, not a Peripheral LE connection. Ignore it. */
            return;
        }
    }

    watch_app_state[index].encrypted = WICED_TRUE;
    WICED_BT_TRACE("LE Peripheral Link is Encrypted\n");

    /* Handle race connection again. If GATT_CONNECTION_STATUS_EVT not yet received, we don't
     * know the Connection Id. We need to wait for the GATT_CONNECTION_STATUS_EVT event. */
    if (watch_app_state[index].conn_id == 0)
    {
        WICED_BT_TRACE("ConnId not yet known. Wait.\n");
        return;
    }


#if (defined (WICED_APP_ANCS_INCLUDED) || defined (WICED_APP_AMS_INCLUDED))
    /* If at ANCS or AMS Service already found */
    if ((watch_hostinfo[index].ancs_s_handle && watch_hostinfo[index].ancs_e_handle) ||
        (watch_hostinfo[index].ams_s_handle && watch_hostinfo[index].ams_e_handle))
    {
        /* Link is encrypted => Start Service configuration */
        watch_app_state[index].init_state = WATCH_INIT_STATE_NONE;
        watch_init_next_client(index);
    }
#endif
}


/*
 * Process discovery results from the stack
 */
wiced_bt_gatt_status_t le_peripheral_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    uint8_t index = find_index_by_conn_id(p_data->conn_id);
    if (index == ANCS_INVALID_CONN_INDEX)
    {
        WICED_BT_TRACE("[%s]find conn_id error\n", __FUNCTION__);
        return WICED_BT_GATT_ERROR;
    }
    WICED_BT_TRACE("[%s] conn %d type %d state 0x%02x\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type, watch_app_state[index].init_state);

    switch (watch_app_state[index].init_state)
    {
#ifdef WICED_APP_ANCS_INCLUDED
    case WATCH_INIT_STATE_ANCS:
        wiced_bt_ancs_client_discovery_result(index, p_data);
        break;
#endif
#ifdef WICED_APP_AMS_INCLUDED
    case WATCH_INIT_STATE_AMS:
        wiced_bt_ams_client_discovery_result(index, p_data);
        break;
#endif
    default:
        if (p_data->discovery_type  == GATT_DISCOVER_SERVICES_ALL)
        {
            if (p_data->discovery_data.group_value.service_type.len == 16)
            {
                WICED_BT_TRACE("%04x e:%04x uuid\n", p_data->discovery_data.group_value.s_handle, p_data->discovery_data.group_value.e_handle);
#ifdef WICED_APP_ANCS_INCLUDED
                if (memcmp(p_data->discovery_data.group_value.service_type.uu.uuid128, ANCS_SERVICE, 16) == 0)
                {
                    WICED_BT_TRACE("ANCS Service found s:%04x e:%04x\n",
                            p_data->discovery_data.group_value.s_handle,
                            p_data->discovery_data.group_value.e_handle);
                    watch_hostinfo[index].ancs_s_handle = p_data->discovery_data.group_value.s_handle;
                    watch_hostinfo[index].ancs_e_handle = p_data->discovery_data.group_value.e_handle;
                }
#endif
#ifdef WICED_APP_AMS_INCLUDED
                if (memcmp(p_data->discovery_data.group_value.service_type.uu.uuid128, AMS_SERVICE, 16) == 0)
                {
                    WICED_BT_TRACE("AMS Service found s:%04x e:%04x\n",
                            p_data->discovery_data.group_value.s_handle,
                            p_data->discovery_data.group_value.e_handle);
                    watch_hostinfo[index].ams_s_handle = p_data->discovery_data.group_value.s_handle;
                    watch_hostinfo[index].ams_e_handle = p_data->discovery_data.group_value.e_handle;
                }
#endif
            }
        }
        else
        {
            WICED_BT_TRACE("!!!! invalid op:%d\n", p_data->discovery_type);
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process discovery complete from the stack
 */
wiced_bt_gatt_status_t le_peripheral_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    wiced_result_t result;
    uint8_t index = find_index_by_conn_id(p_data->conn_id);
    if (index == ANCS_INVALID_CONN_INDEX)
    {
        WICED_BT_TRACE("[%s]find conn_id error\n", __FUNCTION__);
        return WICED_BT_GATT_ERROR;
    }

    WICED_BT_TRACE("[%s] conn %d type %d state %d\n", __FUNCTION__, p_data->conn_id, app_gatt_discovery_type(p_data), watch_app_state[index].init_state);

    switch (watch_app_state[index].init_state)
    {
#ifdef WICED_APP_ANCS_INCLUDED
    case WATCH_INIT_STATE_ANCS:
        wiced_bt_ancs_client_discovery_complete(index, p_data);
        break;
#endif
#ifdef WICED_APP_AMS_INCLUDED
    case WATCH_INIT_STATE_AMS:
        wiced_bt_ams_client_discovery_complete(index, p_data);
        break;
#endif
    default:
        if (app_gatt_discovery_type(p_data) == GATT_DISCOVER_SERVICES_ALL)
        {
#if (defined (WICED_APP_ANCS_INCLUDED) || defined (WICED_APP_AMS_INCLUDED))
            WICED_BT_TRACE("ANCS:%04x-%04x AMS:%04x-%04x\n",
                            watch_hostinfo[index].ancs_s_handle, watch_hostinfo[index].ancs_e_handle,
                            watch_hostinfo[index].ams_s_handle, watch_hostinfo[index].ams_e_handle);

            /* If at ANCS or AMS Service found */
            if ((watch_hostinfo[index].ancs_s_handle && watch_hostinfo[index].ancs_e_handle) ||
                (watch_hostinfo[index].ams_s_handle && watch_hostinfo[index].ams_e_handle))
            {
                /* These Services require Authentication/Encryption */
                if (!watch_app_state[index].encrypted)
                {
                    WICED_BT_TRACE( "Start Authentication\n");
                    /* Link is Not encrypted => Initiate Authorization */
                    result = wiced_bt_dev_sec_bond(watch_app_state[index].remote_addr,
                            watch_app_state[index].addr_type, watch_app_state[index].transport, 0, NULL);
                    WICED_BT_TRACE( "wiced_bt_dev_sec_bond returns:%d\n", result);
                    // If call to the Bond returns success, device is bonded, and we just need
                    // to setup encryption
                    if( result == WICED_BT_SUCCESS )
                    {
                        WICED_BT_TRACE( "starting encryption\n" );
                        wiced_bt_dev_set_encryption(watch_app_state[index].remote_addr,
                                BT_TRANSPORT_LE, NULL );
                    }
                }
                else
                {
                    WICED_BT_TRACE( "LE Peripheral Link encrypted. Let's start LE Services config\n");
                    /* Link is encrypted => Start Service configuration */
                    watch_app_state[index].init_state = WATCH_INIT_STATE_NONE;
                    watch_init_next_client(index);
                }
            }
#endif
        }
        else
        {
            WICED_BT_TRACE("!!!! invalid op:%d\n", app_gatt_discovery_type(p_data));
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Pass read response to appropriate client based on the attribute handle
 */
void watch_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    uint8_t index = find_index_by_conn_id(p_data->conn_id);
    if (index == ANCS_INVALID_CONN_INDEX)
    {
        WICED_BT_TRACE("[%s]find conn_id error\n", __FUNCTION__);
        return;
    }
    WICED_BT_TRACE("read response handle:%04x\n", p_data->response_data.att_value.handle);

    // Check the handle to figure out which client this answer belongs to
#ifdef WICED_APP_ANCS_INCLUDED
    if ((p_data->response_data.att_value.handle >= watch_hostinfo[index].ancs_s_handle) &&
             (p_data->response_data.att_value.handle <= watch_hostinfo[index].ancs_e_handle))
    {
        wiced_bt_ancs_client_read_rsp(index, p_data);
    }
#endif
#ifdef WICED_APP_AMS_INCLUDED
    if ((p_data->response_data.att_value.handle >= watch_hostinfo[index].ams_s_handle) &&
             (p_data->response_data.att_value.handle <= watch_hostinfo[index].ams_e_handle))
    {
        wiced_bt_ams_client_read_rsp(index, p_data);
    }
#endif
}

/*
 * Pass write response to appropriate client based on the attribute handle
 */
void watch_process_write_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    uint8_t index = find_index_by_conn_id(p_data->conn_id);
    if (index == ANCS_INVALID_CONN_INDEX)
    {
        WICED_BT_TRACE("[%s]find conn_id error\n", __FUNCTION__);
        return;
    }
    WICED_BT_TRACE("write response handle:%04x\n", p_data->response_data.handle);

    // Check the handle to figure out which client this answer belongs to
#ifdef WICED_APP_ANCS_INCLUDED
    if ((p_data->response_data.handle >= watch_hostinfo[index].ancs_s_handle) &&
        (p_data->response_data.handle <= watch_hostinfo[index].ancs_e_handle))
    {
        wiced_bt_ancs_client_write_rsp(index, p_data);
    }
#endif
#ifdef WICED_APP_AMS_INCLUDED
    if ((p_data->response_data.handle >= watch_hostinfo[index].ams_s_handle) &&
             (p_data->response_data.handle <= watch_hostinfo[index].ams_e_handle))
    {
        wiced_bt_ams_client_write_rsp(index, p_data);
    }
#endif
}

/*
* Pass notification to appropriate client based on the attribute handle
*/
void watch_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    uint8_t index = find_index_by_conn_id(p_data->conn_id);
    if (index == ANCS_INVALID_CONN_INDEX)
    {
        WICED_BT_TRACE("[%s]find conn_id error\n", __FUNCTION__);
        return;
    }
    WICED_BT_TRACE("notification handle:%04x\n", p_data->response_data.att_value.handle);

    // Check the handle to figure out which client this answer belongs to
#ifdef WICED_APP_ANCS_INCLUDED
    if ((p_data->response_data.att_value.handle >= watch_hostinfo[index].ancs_s_handle) &&
        (p_data->response_data.att_value.handle < watch_hostinfo[index].ancs_e_handle))
    {
        wiced_bt_ancs_client_notification_handler(index, p_data);
    }
#endif
#ifdef WICED_APP_AMS_INCLUDED
    if ((p_data->response_data.att_value.handle >= watch_hostinfo[index].ams_s_handle) &&
        (p_data->response_data.att_value.handle < watch_hostinfo[index].ams_e_handle))
    {
        wiced_bt_ams_client_notification_handler(index, p_data);
    }
#endif
}

/*
* Pass read response to appropriate client based on the attribute handle
*/
void watch_indication_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    uint8_t index = find_index_by_conn_id(p_data->conn_id);
    if (index == ANCS_INVALID_CONN_INDEX)
    {
        WICED_BT_TRACE("[%s] find conn_id error\n", __FUNCTION__);
        return;
    }
    // remember GATT service start and end handles
#ifdef WICED_APP_ANCS_INCLUDED
    if ((p_data->response_data.att_value.handle >= watch_hostinfo[index].ancs_s_handle) &&
        (p_data->response_data.att_value.handle < watch_hostinfo[index].ancs_e_handle))
    {
        wiced_bt_ancs_client_indication_handler(index, p_data);
    }
#endif
#ifdef WICED_APP_AMS_INCLUDED
    if ((p_data->response_data.att_value.handle >= watch_hostinfo[index].ams_s_handle) &&
        (p_data->response_data.att_value.handle < watch_hostinfo[index].ams_e_handle))
    {
        wiced_bt_ams_client_indication_handler(index, p_data);
    }
#endif
}

#ifdef WICED_APP_AMS_INCLUDED
static void le_peripheral_ams_client_event_handler_notification(uint8_t index, wiced_bt_ams_client_notification_id_t opcode, uint16_t data_len, uint8_t *p_data)
{
    uint8_t event_data[62];
    uint16_t event_code;
    uint8_t i = 0;

    event_data[i++] = watch_app_state[index].conn_id;
    event_data[i++] = watch_app_state[index].conn_id>>8;

    switch (opcode)
    {
    case WICED_BT_AMS_CLIENT_NOTIFICATION_PLAYER_NAME:
        event_code = HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAYER_CHANGE;
        break;

    case WICED_BT_AMS_CLIENT_NOTIFICATION_PLAY_STATUS:
        event_code = HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS;
        break;

    case WICED_BT_AMS_CLIENT_NOTIFICATION_PALY_POSITION:
        event_code = HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_POSITION;
        break;

    case WICED_BT_AMS_CLIENT_NOTIFICATION_SETTING_CHANGE:
        event_data[i++] = 1;    // number of settings

        event_code = HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_CHANGE;
        break;

    case WICED_BT_AMS_CLIENT_NOTIFICATION_TRACK_INFO:
        event_data[i++] = 0;    // status

        event_code = HCI_CONTROL_AVRC_CONTROLLER_EVENT_CURRENT_TRACK_INFO;
        break;

    case WICED_BT_AMS_CLIENT_NOTIFICATION_VOLUME_LEVEL:
        event_code = HCI_CONTROL_AVRC_TARGET_EVENT_VOLUME_LEVEL;
        break;

    default:
        return;
    }

    if (data_len > (sizeof(event_data) - i))
        data_len = (sizeof(event_data) - i);

    memcpy((void *) &event_data[i], (void *) p_data, data_len);

    wiced_transport_send_data(event_code, event_data, data_len + i);
}

static void le_peripheral_ams_client_event_handler(uint8_t index, wiced_bt_ams_client_event_t event, wiced_bt_ams_client_event_data_t *p_event_data)
{
    int i;
    uint8_t tx_buf[3];
    uint8_t *p = tx_buf;

    switch (event)
    {
    case WICED_BT_AMS_CLIENT_EVENT_INITIALIZED:
        *p++ = (watch_app_state[index].conn_id) & 0xff;
        *p++ = (watch_app_state[index].conn_id >> 8) & 0xff;
        *p++ = 0;
        WICED_BT_TRACE("send HCI_CONTROL_AMS_EVENT_CONNECTED\n");
        wiced_transport_send_data(HCI_CONTROL_AMS_EVENT_CONNECTED, tx_buf, sizeof(tx_buf));

        watch_init_next_client(index);
        break;

    case WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION:
        le_peripheral_ams_client_event_handler_notification(index, p_event_data->notification.opcode, p_event_data->notification.data_len, p_event_data->notification.p_data);
        break;

    default:
        break;
    }
}
#endif

/*
 * This function is called during startup operation to start initialization of the next client
 */
void watch_init_next_client(uint8_t index)
{
#ifdef WICED_APP_AMS_INCLUDED
    wiced_bt_ams_client_config_t ams_client_config;
#endif

    WICED_BT_TRACE("%s state:%d\n", __FUNCTION__, watch_app_state[index].init_state);

    switch (watch_app_state[index].init_state)
    {
    case WATCH_INIT_STATE_NONE:
#ifdef WICED_APP_ANCS_INCLUDED
        watch_app_state[index].init_state = WATCH_INIT_STATE_ANCS;

        if (wiced_bt_ancs_client_start(index, watch_app_state[index].conn_id, watch_hostinfo[index].ancs_s_handle, watch_hostinfo[index].ancs_e_handle))
            break;
        /* No break on purpose (if not ANCS Service found) */

    case WATCH_INIT_STATE_ANCS:
#endif
#ifdef WICED_APP_AMS_INCLUDED
        watch_app_state[index].init_state = WATCH_INIT_STATE_AMS;

        ams_client_config.conn_id           = watch_app_state[index].conn_id;
        ams_client_config.s_handle          = watch_hostinfo[index].ams_s_handle;
        ams_client_config.e_handle          = watch_hostinfo[index].ams_e_handle;
        ams_client_config.p_event_handler   = &le_peripheral_ams_client_event_handler;

        if (wiced_bt_ams_client_initialize(MAX_PHONE_CONNECTIONS, index, &ams_client_config))
            break;

        /* No break on purpose  (if not AMS Service found) */

    case WATCH_INIT_STATE_AMS:
#endif
        // We are done with initial settings, and need to stay connected.
        watch_app_state[index].init_state = WATCH_INIT_STATE_NONE;
    }
}

void watch_util_send_discover(uint16_t conn_id, wiced_bt_gatt_discovery_type_t type, uint16_t uuid,
                              uint16_t s_handle, uint16_t e_handle)
{
    wiced_bt_gatt_discovery_param_t param;
    wiced_bt_gatt_status_t          status;

    memset(&param, 0, sizeof(param));
    if (uuid != 0)
    {
        param.uuid.len = LEN_UUID_16;
        param.uuid.uu.uuid16 = uuid;
    }
    param.s_handle = s_handle;
    param.e_handle = e_handle;

    status = wiced_bt_gatt_client_send_discover(conn_id, type, &param);

    WICED_BT_TRACE("app_gatt_client_send_discover %d\n", status);
}

/*
 * peripheral_timeout
 */
void peripheral_timeout(TIMER_PARAM_TYPE arg)
{
#if BTSTACK_VER >= 0x03000001
    uint8_t index = *(uint8_t *)arg;
#else
    uint8_t index = (uint8_t)arg;
#endif

    WICED_BT_TRACE("index:%d, timer expited\n",index);
    /* If Pairing is not allowed AND peer device not yet Paired */
    if ((hci_control_cb.pairing_allowed == WICED_FALSE) &&
        (hci_control_find_nvram_id(watch_app_state[index].remote_addr, BD_ADDR_LEN) == HCI_CONTROL_INVALID_NVRAM_ID))
    {
        WICED_BT_TRACE("Peripheral timeout. Pairing not allowed and Device not Paired. Do nothing\n");
        return;
    }

    WICED_BT_TRACE("Peripheral timeout. Starting Service Search\n");


    // perform primary service search
    watch_app_state[index].init_state = WATCH_INIT_STATE_NONE;

    watch_hostinfo[index].ams_s_handle = 0;
    watch_hostinfo[index].ams_e_handle = 0;
    watch_hostinfo[index].ancs_s_handle = 0;
    watch_hostinfo[index].ancs_e_handle = 0;

    watch_util_send_discover(watch_app_state[index].conn_id, GATT_DISCOVER_SERVICES_ALL,
            UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
}
