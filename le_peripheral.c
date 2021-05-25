/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
* commonly used in the BLE watches including Apple's vendor specific ANCS,
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
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
#include "wiced_bt_ams.h"
#endif
#include "hci_control.h"
#include "string.h"
#include "wiced_memory.h"
#include "wiced_transport.h"

#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED


/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *               Function Prototypes
 ******************************************************/

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static void                   watch_process_read_rsp       (wiced_bt_gatt_operation_complete_t *p_data);
static void                   watch_process_write_rsp      (wiced_bt_gatt_operation_complete_t *p_data);
static void                   watch_notification_handler   (wiced_bt_gatt_operation_complete_t *p_data);
static void                   watch_indication_handler     (wiced_bt_gatt_operation_complete_t *p_data);

static void                   peripheral_timeout           (uint32_t count);

static void                   watch_init_next_client       (void);

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

// NVRAM save area
HOSTINFO watch_hostinfo;

watch_app_state_t watch_app_state;

/******************************************************
 *               Function Definitions
 ******************************************************/

static void le_peripheral_ancs_client_event_handler_notification(wiced_bt_ancs_client_notification_data_t *p_data)
{
    // Allocating a buffer to send the trace
    uint8_t *p_tx_buf = (uint8_t *) wiced_bt_get_buffer(sizeof(wiced_bt_ancs_client_notification_data_t));

    WICED_BT_TRACE("ANCS notification UID:%d command:%d category:%d flags:%04x\n", p_data->basic.notification_uid, p_data->basic.command, p_data->basic.category, p_data->basic.flags);
    WICED_BT_TRACE("T%s %s %s\n", p_data->info.title, p_data->info.message, p_data->info.positive_action_label, p_data->info.negative_action_label);

    if (p_tx_buf)
    {
        int len;
        p_tx_buf[0] = p_data->basic.notification_uid & 0xff;
        p_tx_buf[1] = (p_data->basic.notification_uid >> 8) & 0xff;
        p_tx_buf[2] = (p_data->basic.notification_uid >> 16) & 0xff;
        p_tx_buf[3] = (p_data->basic.notification_uid >> 24) & 0xff;

        p_tx_buf[4] = p_data->basic.command;

        p_tx_buf[5] = p_data->basic.category;

        p_tx_buf[6] = p_data->basic.flags;

        len = 7;

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
}

static void le_peripheral_ancs_client_event_handler(wiced_bt_ancs_client_event_t event, wiced_bt_ancs_client_event_data_t *p_data)
{
    switch (event)
    {
    case WICED_BT_ANCS_CLIENT_EVENT_INITIALIZED:
        watch_init_next_client();
        break;

    case WICED_BT_ANCS_CLIENT_EVENT_NOTIFICATION:
        le_peripheral_ancs_client_event_handler_notification(p_data->notification.p_data);
        break;

    default:
        break;
    }
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
void le_peripheral_app_init(void)
{
    wiced_bt_ancs_client_config_t ancs_client_config = {0};

    memset(&watch_hostinfo, 0, sizeof(watch_hostinfo));
    memset(&watch_app_state, 0, sizeof(watch_app_state));

    if (wiced_init_timer(&watch_app_state.timer, peripheral_timeout, 0,
            WICED_SECONDS_TIMER ) != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Err: wiced_init_timer le_peripheral failed\n");
    }

    /* Initialize the ANCS client. */
    ancs_client_config.p_event_handler = &le_peripheral_ancs_client_event_handler;

    if (wiced_bt_ancs_client_initialize(&ancs_client_config) == WICED_FALSE)
    {
        WICED_BT_TRACE("Err: wiced_bt_ancs_client_initialize failed\n");
    }
}

/*
 * This function will be called when a connection is established in LE Peripheral Role
 */
void le_peripheral_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    watch_app_state.conn_id = p_conn_status->conn_id;

    // save address of the connected device and print it out.
    memcpy(watch_app_state.remote_addr, p_conn_status->bd_addr, sizeof(watch_app_state.remote_addr));
    watch_app_state.addr_type = p_conn_status->addr_type;
    watch_app_state.transport = p_conn_status->transport;

    wiced_bt_ancs_client_connection_up(p_conn_status);

    wiced_bt_ams_client_connection_up(p_conn_status);

    /* Connected as Peripheral. Start discovery in couple of seconds to give time to the peer device
     * to find/configure our services */
    wiced_start_timer(&watch_app_state.timer, 2);
}

// This function will be called when connection goes down
void le_peripheral_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    watch_app_state.conn_id = 0;
    watch_app_state.encrypted = WICED_FALSE;
    memset(&watch_hostinfo, 0, sizeof(watch_hostinfo));

    wiced_stop_timer(&watch_app_state.timer);

    wiced_bt_ancs_client_connection_down(p_conn_status);

    wiced_bt_ams_client_connection_down(p_conn_status);
}


// Process encryption status changed notification from the stack
void le_peripheral_encryption_status_changed(wiced_bt_dev_encryption_status_t *p_status)
{
    wiced_result_t result;
    uint8_t role;

    /* Ignore event if Encryption failed */
    if (p_status->result != WICED_BT_SUCCESS)
        return;

    /* Check if it's a Peripheral/Client device */
    if (memcmp(watch_app_state.remote_addr, p_status->bd_addr, sizeof(watch_app_state.remote_addr)))
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

    watch_app_state.encrypted = WICED_TRUE;
    WICED_BT_TRACE("LE Peripheral Link is Encrypted\n");

    /* Handle race connection again. If GATT_CONNECTION_STATUS_EVT not yet received, we don't
     * know the Connection Id. We need to wait for the GATT_CONNECTION_STATUS_EVT event. */
    if (watch_app_state.conn_id == 0)
    {
        WICED_BT_TRACE("ConnId not yet known. Wait.\n");
        return;
    }

    /* If at ANCS or AMS Service already found */
    if ((watch_hostinfo.ancs_s_handle && watch_hostinfo.ancs_e_handle) ||
        (watch_hostinfo.ams_s_handle && watch_hostinfo.ams_e_handle))
    {
        /* Link is encrypted => Start Service configuration */
        watch_app_state.init_state = WATCH_INIT_STATE_NONE;
        watch_init_next_client();
    }
}


/*
 * GATT operation started by the client has been completed
 */
wiced_bt_gatt_status_t le_peripheral_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch (p_data->op)
    {
#if BTSTACK_VER > 0x01020000
    case GATTC_OPTYPE_READ_HANDLE:
#else
    case GATTC_OPTYPE_READ:
#endif
        watch_process_read_rsp(p_data);
        break;

#if BTSTACK_VER > 0x01020000
    case GATTC_OPTYPE_WRITE_WITH_RSP:
    case GATTC_OPTYPE_WRITE_NO_RSP:
#else
    case GATTC_OPTYPE_WRITE:
#endif
        watch_process_write_rsp(p_data);
        break;

#if BTSTACK_VER > 0x01020000
    case GATTC_OPTYPE_CONFIG_MTU:
#else
    case GATTC_OPTYPE_CONFIG:
#endif
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

/*
 * Process discovery results from the stack
 */
wiced_bt_gatt_status_t le_peripheral_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    WICED_BT_TRACE("[%s] conn %d type %d state 0x%02x\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type, watch_app_state.init_state);

    switch (watch_app_state.init_state)
    {
    case WATCH_INIT_STATE_ANCS:
        wiced_bt_ancs_client_discovery_result(p_data);
        break;
    case WATCH_INIT_STATE_AMS:
        wiced_bt_ams_client_discovery_result(p_data);
        break;
    default:
        if (p_data->discovery_type  == GATT_DISCOVER_SERVICES_ALL)
        {
            if (p_data->discovery_data.group_value.service_type.len == 16)
            {
                WICED_BT_TRACE("%04x e:%04x uuid\n", p_data->discovery_data.group_value.s_handle, p_data->discovery_data.group_value.e_handle);
                if (memcmp(p_data->discovery_data.group_value.service_type.uu.uuid128, ANCS_SERVICE, 16) == 0)
                {
                    WICED_BT_TRACE("ANCS Service found s:%04x e:%04x\n",
                            p_data->discovery_data.group_value.s_handle,
                            p_data->discovery_data.group_value.e_handle);
                    watch_hostinfo.ancs_s_handle = p_data->discovery_data.group_value.s_handle;
                    watch_hostinfo.ancs_e_handle = p_data->discovery_data.group_value.e_handle;
                }
                else if (memcmp(p_data->discovery_data.group_value.service_type.uu.uuid128, AMS_SERVICE, 16) == 0)
                {
                    WICED_BT_TRACE("AMS Service found s:%04x e:%04x\n",
                            p_data->discovery_data.group_value.s_handle,
                            p_data->discovery_data.group_value.e_handle);
                    watch_hostinfo.ams_s_handle = p_data->discovery_data.group_value.s_handle;
                    watch_hostinfo.ams_e_handle = p_data->discovery_data.group_value.e_handle;
                }
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
#if BTSTACK_VER > 0x01020000
    wiced_bt_gatt_discovery_type_t discovery_type = p_data->discovery_type;
#else
    wiced_bt_gatt_discovery_type_t discovery_type = p_data->disc_type;
#endif

    WICED_BT_TRACE("[%s] conn %d type %d state %d\n", __FUNCTION__, p_data->conn_id, discovery_type, watch_app_state.init_state);

    switch (watch_app_state.init_state)
    {
    case WATCH_INIT_STATE_ANCS:
        wiced_bt_ancs_client_discovery_complete(p_data);
        break;
    case WATCH_INIT_STATE_AMS:
        wiced_bt_ams_client_discovery_complete(p_data);
        break;
    default:
        if (discovery_type == GATT_DISCOVER_SERVICES_ALL)
        {
            WICED_BT_TRACE("ANCS:%04x-%04x AMS:%04x-%04x\n",
                            watch_hostinfo.ancs_s_handle, watch_hostinfo.ancs_e_handle,
                            watch_hostinfo.ams_s_handle, watch_hostinfo.ams_e_handle);

            /* If at ANCS or AMS Service found */
            if ((watch_hostinfo.ancs_s_handle && watch_hostinfo.ancs_e_handle) ||
                (watch_hostinfo.ams_s_handle && watch_hostinfo.ams_e_handle))
            {
                /* These Services require Authentication/Encryption */
                if (watch_app_state.encrypted == WICED_FALSE)
                {
                    WICED_BT_TRACE( "Start Authentication\n");
                    /* Link is Not encrypted => Initiate Authorization */
                    result = wiced_bt_dev_sec_bond(watch_app_state.remote_addr,
                            watch_app_state.addr_type, watch_app_state.transport, 0, NULL);
                    WICED_BT_TRACE( "wiced_bt_dev_sec_bond returns:%d\n", result);
                    // If call to the Bond returns success, device is bonded, and we just need
                    // to setup encryption
                    if( result == WICED_BT_SUCCESS )
                    {
                        WICED_BT_TRACE( "starting encryption\n" );
                        wiced_bt_dev_set_encryption(watch_app_state.remote_addr,
                                BT_TRANSPORT_LE, NULL );
                    }
                }
                else
                {
                    WICED_BT_TRACE( "LE Peripheral Link encrypted. Let's start LE Services config\n");
                    /* Link is encrypted => Start Service configuration */
                    watch_app_state.init_state = WATCH_INIT_STATE_NONE;
                    watch_init_next_client();
                }
            }
        }
        else
        {
            WICED_BT_TRACE("!!!! invalid op:%d\n", discovery_type);
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Pass read response to appropriate client based on the attribute handle
 */
void watch_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("read response handle:%04x\n", p_data->response_data.att_value.handle);

    // Check the handle to figure out which client this answer belongs to
    if ((p_data->response_data.att_value.handle >= watch_hostinfo.ancs_s_handle) &&
             (p_data->response_data.att_value.handle <= watch_hostinfo.ancs_e_handle))
    {
        wiced_bt_ancs_client_read_rsp(p_data);
    }
    else if ((p_data->response_data.att_value.handle >= watch_hostinfo.ams_s_handle) &&
             (p_data->response_data.att_value.handle <= watch_hostinfo.ams_e_handle))
    {
        wiced_bt_ams_client_read_rsp(p_data);
    }
}

/*
 * Pass write response to appropriate client based on the attribute handle
 */
void watch_process_write_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("write response handle:%04x\n", p_data->response_data.handle);

    // Check the handle to figure out which client this answer belongs to
    if ((p_data->response_data.handle >= watch_hostinfo.ancs_s_handle) &&
        (p_data->response_data.handle <= watch_hostinfo.ancs_e_handle))
    {
        wiced_bt_ancs_client_write_rsp(p_data);
    }
    else if ((p_data->response_data.handle >= watch_hostinfo.ams_s_handle) &&
             (p_data->response_data.handle <= watch_hostinfo.ams_e_handle))
    {
        wiced_bt_ams_client_write_rsp(p_data);
    }
}

/*
* Pass notification to appropriate client based on the attribute handle
*/
void watch_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("notification handle:%04x\n", p_data->response_data.att_value.handle);

    // Check the handle to figure out which client this answer belongs to
    if ((p_data->response_data.att_value.handle >= watch_hostinfo.ancs_s_handle) &&
        (p_data->response_data.att_value.handle < watch_hostinfo.ancs_e_handle))
    {
        wiced_bt_ancs_client_notification_handler(p_data);
    }
    else if ((p_data->response_data.att_value.handle >= watch_hostinfo.ams_s_handle) &&
        (p_data->response_data.att_value.handle < watch_hostinfo.ams_e_handle))
    {
        wiced_bt_ams_client_notification_handler(p_data);
    }
}

/*
* Pass read response to appropriate client based on the attribute handle
*/
void watch_indication_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    // remember GATT service start and end handles
    if ((p_data->response_data.att_value.handle >= watch_hostinfo.ancs_s_handle) &&
        (p_data->response_data.att_value.handle < watch_hostinfo.ancs_e_handle))
    {
        wiced_bt_ancs_client_indication_handler(p_data);
    }
    else if ((p_data->response_data.att_value.handle >= watch_hostinfo.ams_s_handle) &&
        (p_data->response_data.att_value.handle < watch_hostinfo.ams_e_handle))
    {
        wiced_bt_ams_client_indication_handler(p_data);
    }
}

static void le_peripheral_ams_client_event_handler_notification(wiced_bt_ams_client_notification_id_t opcode, uint16_t data_len, uint8_t *p_data)
{
    uint8_t event_data[60];
    uint16_t event_code;
    uint8_t i = 0;

    event_data[i++] = 0;    // handle
    event_data[i++] = 0;

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

static void le_peripheral_ams_client_event_handler(wiced_bt_ams_client_event_t event, wiced_bt_ams_client_event_data_t *p_event_data)
{
    switch (event)
    {
    case WICED_BT_AMS_CLIENT_EVENT_INITIALIZED:
        watch_init_next_client();
        break;

    case WICED_BT_AMS_CLIENT_EVENT_NOTIFICATION:
        le_peripheral_ams_client_event_handler_notification(p_event_data->notification.opcode, p_event_data->notification.data_len, p_event_data->notification.p_data);
        break;

    default:
        break;
    }
}

/*
 * This function is called during startup operation to start initialization of the next client
 */
void watch_init_next_client(void)
{
    wiced_bt_ams_client_config_t ams_client_config;

    WICED_BT_TRACE("%s state:%d\n", __FUNCTION__, watch_app_state.init_state);

    switch (watch_app_state.init_state)
    {
    case WATCH_INIT_STATE_NONE:
        watch_app_state.init_state = WATCH_INIT_STATE_ANCS;

        if (wiced_bt_ancs_client_start(watch_app_state.conn_id, watch_hostinfo.ancs_s_handle, watch_hostinfo.ancs_e_handle))
            break;
        /* No break on purpose (if not ANCS Service found) */

    case WATCH_INIT_STATE_ANCS:
        watch_app_state.init_state = WATCH_INIT_STATE_AMS;

        ams_client_config.conn_id           = watch_app_state.conn_id;
        ams_client_config.s_handle          = watch_hostinfo.ams_s_handle;
        ams_client_config.e_handle          = watch_hostinfo.ams_e_handle;
        ams_client_config.p_event_handler   = &le_peripheral_ams_client_event_handler;

        if (wiced_bt_ams_client_initialize(&ams_client_config))
            break;

        /* No break on purpose  (if not AMS Service found) */

    case WATCH_INIT_STATE_AMS:

        // We are done with initial settings, and need to stay connected.
        watch_app_state.init_state = WATCH_INIT_STATE_NONE;
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

#if BTSTACK_VER > 0x01020000
    status = wiced_bt_gatt_client_send_discover(conn_id, type, &param);

    WICED_BT_TRACE("wiced_bt_gatt_client_send_discover %d\n", status);
#else
    status = wiced_bt_gatt_send_discover(conn_id, type, &param);

    WICED_BT_TRACE("wiced_bt_gatt_send_discover %d\n", status);
#endif
}

void watch_util_send_read_by_handle(uint16_t conn_id, uint16_t handle)
{
    wiced_bt_gatt_status_t     status;
#if BTSTACK_VER > 0x01020000
    status = wiced_bt_gatt_client_send_read_handle(conn_id, handle, 0,
            NULL, 0, GATT_AUTH_REQ_NONE);

    WICED_BT_TRACE("wiced_bt_gatt_client_send_read_handle %d\n", status);
#else /* !BTSTACK_VER */
    wiced_bt_gatt_read_param_t param;

    memset(&param, 0, sizeof(param));
    param.by_handle.handle = handle;

    status = wiced_bt_gatt_send_read(conn_id, GATT_READ_BY_HANDLE, &param);

    WICED_BT_TRACE("wiced_bt_gatt_send_read %d\n", status);
#endif /* BTSTACK_VER */
}

wiced_bool_t watch_util_send_read_by_type(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle, uint16_t uuid)
{
    wiced_bt_gatt_status_t     status;
#if BTSTACK_VER > 0x01020000
    wiced_bt_uuid_t uuid_buf;

    uuid_buf.len = 2;
    uuid_buf.uu.uuid16 = uuid;

    status = wiced_bt_gatt_client_send_read_by_type(conn_id, s_handle, e_handle,
            &uuid_buf, NULL, 0, GATT_AUTH_REQ_NONE);

    WICED_BT_TRACE("wiced_bt_gatt_client_send_read_by_type %d\n", status);
    return (status != WICED_BT_GATT_SUCCESS);
#else /* !BTSTACK_VER */
    wiced_bt_gatt_read_param_t param;

    memset(&param, 0, sizeof(param));
    param.char_type.s_handle        = s_handle;
    param.char_type.e_handle        = e_handle;
    param.char_type.uuid.len        = 2;
    param.char_type.uuid.uu.uuid16  = uuid;

    status = wiced_bt_gatt_send_read(conn_id, GATT_READ_BY_TYPE, &param);

    WICED_BT_TRACE("wiced_bt_gatt_send_read %d\n", status);
    return (status == WICED_BT_SUCCESS);
#endif /* BTSTACK_VER */
}

/*
 * peripheral_timeout
 */
void peripheral_timeout(uint32_t arg)
{
    /* If Pairing is not allowed AND peer device not yet Paired */
    if ((hci_control_cb.pairing_allowed == WICED_FALSE) &&
        (hci_control_find_nvram_id(watch_app_state.remote_addr, BD_ADDR_LEN) == HCI_CONTROL_INVALID_NVRAM_ID))
    {
        WICED_BT_TRACE("Peripheral timeout. Pairing not allowed and Device not Paired. Do nothing\n");
        return;
    }

    WICED_BT_TRACE("Peripheral timeout. Starting Service Search\n");


    // perform primary service search
    watch_app_state.init_state = WATCH_INIT_STATE_NONE;

    watch_hostinfo.ams_s_handle = 0;
    watch_hostinfo.ams_e_handle = 0;
    watch_hostinfo.ancs_s_handle = 0;
    watch_hostinfo.ancs_e_handle = 0;

    watch_util_send_discover(watch_app_state.conn_id, GATT_DISCOVER_SERVICES_ALL,
            UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
}

#endif /* WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED */
