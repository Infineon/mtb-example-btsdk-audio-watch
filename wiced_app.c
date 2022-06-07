/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
 * This file implements the entry point of the Wiced Application
 */
#include <sparcommon.h>
#ifdef WICED_APP_AMS_INCLUDED
#include <wiced_bt_ams.h>
#endif
#ifdef WICED_APP_HFP_AG_INCLUDED
#include "hci_control_hfp_ag.h"
#include <wiced_bt_hfp_ag.h>
#endif
#ifdef WICED_APP_HFP_HF_INCLUDED
#include "hci_control_hfp_hf.h"
#include "wiced_bt_hfp_hf_int.h"
#endif
#include <wiced_bt_stack.h>
#include <wiced_bt_trace.h>
#include <wiced_memory.h>
#if (defined(CYW20721B2) || defined(CYW43012C0))
#include <wiced_utilities.h>
#endif
#include <wiced_platform.h>
#include <wiced_hal_gpio.h>
#include <wiced_hal_puart.h>
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW43012C0) )
#include <wiced_bt_app_common.h>
#endif
#ifdef WICED_APP_AUDIO_SNK_INCLUDED
#include "a2dp_sink.h"
#endif
#include "hci_control.h"
#include "hci_control_le.h"
#include "hci_control_rc_controller.h"
#include "le_peripheral.h"
#include "wiced_app_cfg.h"
#include "wiced_app.h"
#ifdef CYW9BT_AUDIO
#include <wiced_audio_manager.h>
#endif

#ifdef CYW43012C0
#include "wiced_memory.h"

static wiced_bt_buffer_pool_t* watch_app_pool_big = NULL;
static wiced_bt_buffer_pool_t* watch_app_pool_small = NULL;
#endif

#define WICED_HS_EIR_BUF_MAX_SIZE 264


static void write_eir(void);
static wiced_result_t btm_enabled_event_handler(wiced_bt_dev_enabled_t *event_data);

#if defined(CYW20819A1)
#include "wiced_memory_pre_init.h"
/* override weak version of this struct to adjust memory for audio application */
WICED_MEM_PRE_INIT_CONTROL g_mem_pre_init =
{
    .max_ble_connections = WICED_MEM_PRE_INIT_IGNORE,
    .max_peripheral_piconet = WICED_MEM_PRE_INIT_IGNORE, /* use to reduce bt connections */
    .max_resolving_list = WICED_MEM_PRE_INIT_IGNORE,
    .onfound_list_len = 0,
    .max_multi_adv_instances = WICED_MEM_PRE_INIT_IGNORE,
};
#endif

static app_identity_random_mapping_t addr_mapping[ADDR_MAPPING_MAX_COUNT] = {0};

/*
 * Application Start, ie, entry point to the application.
 */
APPLICATION_START()
{
    wiced_result_t result;

    hci_control_init();

#ifdef WICED_BT_TRACE_ENABLE
#ifdef NO_PUART_SUPPORT
#if defined(CYW43012C0)
    wiced_debug_uart = WICED_ROUTE_DEBUG_TO_DBG_UART;
    debug_uart_enable(3000000);
#else // CYW43012C0
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif // CYW43012C0
#else
#ifdef CYW55572
    // Default PUART baudrate is 115200, update it to 3M before calling wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    wiced_set_debug_uart_baudrate(3000000);
#else /* !CYW55572 */
    wiced_hal_puart_init();
#if ( defined(CYW20706A2) )
    wiced_hal_puart_configuration( 3000000, PARITY_NONE, STOP_BIT_2 );
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#else
    wiced_hal_puart_configuration( 3000000, PARITY_NONE, STOP_BIT_2 );
#endif // CYW20706A2
#endif /* CYW55572 */
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#endif // NO_PUART_SUPPORT
#endif // WICED_BT_TRACE_ENABLE

    result = app_stack_init();

    if (WICED_SUCCESS != result)
    {
        WICED_BT_TRACE("ERROR bt_stack_init %u\n", result);
        return;
    }

    result = wiced_audio_buffer_initialize(wiced_bt_audio_buf_config);
    if (WICED_SUCCESS != result)
    {
        WICED_BT_TRACE("ERROR audio_buffer_init %u\n", result);
        return;
    }

#ifdef WICED_APP_LE_INCLUDED
    hci_control_le_init();
#endif

#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
    le_peripheral_app_init();
#endif
#ifndef DEBUG
    //Enable the bt coex functionality
//    wiced_bt_coex_enable();
#endif

    WICED_BT_TRACE("Watch App Start\n");
}

app_identity_random_mapping_t * get_addr_mapping_by_random_addr(wiced_bt_device_address_t random_addr)
{
    for (int i=0; i<ADDR_MAPPING_MAX_COUNT;i++)
    {
        if (memcmp(random_addr, addr_mapping[i].random_addr, sizeof(wiced_bt_device_address_t))==0)
            return &addr_mapping[i];
    }
    return NULL;
}

app_identity_random_mapping_t * get_addr_mapping_by_identity_addr(wiced_bt_device_address_t identity_addr)
{
    for (int i=0; i<ADDR_MAPPING_MAX_COUNT;i++)
    {
        if (memcmp(identity_addr, addr_mapping[i].identity_addr, sizeof(wiced_bt_device_address_t))==0)
            return &addr_mapping[i];
    }
    return NULL;
}

app_identity_random_mapping_t * get_empty_addr_mapping()
{
    wiced_bt_device_address_t empty_addr={0};

    return get_addr_mapping_by_identity_addr(empty_addr);
}

void write_eir(void)
{
    uint8_t *pBuf;
    uint8_t *p, *p_tmp;
    uint8_t nb_uuid = 0;
    uint8_t length;

    //Allocating a buffer from the public pool
    pBuf = (uint8_t *)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );

    WICED_BT_TRACE("%s %x\n", __func__, pBuf);

    if ( !pBuf )
    {
        return;
    }

    p = pBuf;

    length = strlen( (char *)wiced_bt_cfg_settings.device_name );
    UINT8_TO_STREAM(p, length + 1);
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_LOCAL_NAME_TYPE);
    memcpy( p, wiced_bt_cfg_settings.device_name, length );
    p += length;

    // Add other BR/EDR UUIDs
    p_tmp = p;      // We don't now the number of UUIDs for the moment
    p++;
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_16BITS_UUID_TYPE);
#ifdef WICED_APP_AUDIO_SRC_INCLUDED
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AUDIO_SOURCE);       nb_uuid++;
#endif
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AV_REM_CTRL_TARGET); nb_uuid++;
#endif
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AV_REMOTE_CONTROL);  nb_uuid++;
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AUDIO_SINK);         nb_uuid++;
#endif
#if (defined(WICED_APP_HFP_AG_INCLUDED) || defined(WICED_APP_HFP_HF_INCLUDED))
    UINT16_TO_STREAM(p, UUID_SERVCLASS_HEADSET);            nb_uuid++;
    UINT16_TO_STREAM(p, UUID_SERVCLASS_HF_HANDSFREE);       nb_uuid++;
    UINT16_TO_STREAM(p, UUID_SERVCLASS_GENERIC_AUDIO);      nb_uuid++;
#endif

#ifdef WICED_APP_PANU_INCLUDED
    UINT16_TO_STREAM(p, UUID_SERVCLASS_PANU);               nb_uuid++;
#endif
#ifdef WICED_APP_PANNAP_INCLUDED
    UINT16_TO_STREAM(p, UUID_SERVCLASS_NAP);                nb_uuid++;
#endif
    /* Now, we can update the UUID Tag's length */
    UINT8_TO_STREAM(p_tmp, (nb_uuid * LEN_UUID_16) + 1);

    // Last Tag
    UINT8_TO_STREAM(p, 0x00);

    // print EIR data
    WICED_BT_TRACE_ARRAY( ( uint8_t* )( pBuf ), MIN( p - ( uint8_t* )pBuf, 100 ), "EIR :" );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    /* Allocated buffer not anymore needed. Free it */
    wiced_bt_free_buffer( pBuf );

    return;
}

wiced_result_t btm_event_handler(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    int                                bytes_written, bytes_read;
    int                                nvram_id;
    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    uint8_t                             pairing_result;

    WICED_BT_TRACE( "btm_event_handler 0x%02x\n", event );

    switch( event )
    {
        /* Bluetooth stack enabled */
        case BTM_ENABLED_EVT:
            result = btm_enabled_event_handler(&p_event_data->enabled);
            break;

        case BTM_DISABLED_EVT:
            hci_control_send_device_error_evt(p_event_data->disabled.reason, 0);
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS, WICED_PIN_CODE_LEN, (uint8_t *)&pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            // If this is just works pairing, accept. Otherwise send event to the MCU to confirm the same value.
            if (p_event_data->user_confirmation_request.just_works)
            {
                wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
            }
            else
            {
                hci_control_send_user_confirmation_request_evt(p_event_data->user_confirmation_request.bd_addr, p_event_data->user_confirmation_request.numeric_value );
            }
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            hci_control_send_user_confirmation_request_evt(p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_ALL_PROFILES_NO;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* Use the default security for BLE */
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %B\n",
                    p_event_data->pairing_io_capabilities_ble_request.bd_addr);
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_MITM_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;
            if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
                hci_control_send_pairing_completed_evt( pairing_result, p_event_data->pairing_complete.bd_addr );
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
                app_identity_random_mapping_t * addr_map = get_addr_mapping_by_random_addr(p_event_data->pairing_complete.bd_addr);
                if ( addr_map != NULL )
                {
                    hci_control_send_pairing_completed_evt( pairing_result, addr_map->identity_addr );
                }
            }
            WICED_BT_TRACE( "Pairing Result: %d\n", pairing_result );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;

            WICED_BT_TRACE( "Encryption Status:(%B) res:%d\n", p_encryption_status->bd_addr, p_encryption_status->result );

#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
            if (p_encryption_status->transport == BT_TRANSPORT_LE)
                le_peripheral_encryption_status_changed(p_encryption_status);
#endif
            hci_control_send_encryption_changed_evt( p_encryption_status->result, p_encryption_status->bd_addr );
            break;

        case BTM_SECURITY_REQUEST_EVT:
            WICED_BT_TRACE( "Security Request Event, Pairing allowed %d\n", hci_control_cb.pairing_allowed );
            if ( hci_control_cb.pairing_allowed )
            {
                wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            }
            else
            {
                // Pairing not allowed, return error
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            app_paired_device_link_keys_update( p_event_data );
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */
            WICED_BT_TRACE("\t\tfind device %B\n", p_event_data->paired_device_link_keys_request.bd_addr);

            if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN ) ) != 0)
            {
                 bytes_read = hci_control_read_nvram( nvram_id, &p_event_data->paired_device_link_keys_request, sizeof( wiced_bt_device_link_keys_t ) );

                 result = WICED_BT_SUCCESS;
                 WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, bytes_read);
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n");
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Request to store newly generated local identity keys to NVRAM */
            /* (sample app does not store keys to NVRAM) */

            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /*
             * Request to restore local identity keys from NVRAM
             * (requested during Bluetooth start up)
             * */
            /* (sample app does not store keys to NVRAM)
             * New local identity keys will be generated
             * */
            result = WICED_BT_NO_RESOURCES;
            break;
#ifdef WICED_APP_LE_INCLUDED

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            hci_control_le_scan_state_changed( p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            WICED_BT_TRACE("BLE_ADVERT_STATE_CHANGED_EVT:%d\n", p_event_data->ble_advert_state_changed);
            hci_control_le_advert_state_changed( p_event_data->ble_advert_state_changed );
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            WICED_BT_TRACE ("BTM BLE Connection Update event status %d BDA [%B] interval %d latency %d supervision timeout %d \n",
                                p_event_data->ble_connection_param_update.status, p_event_data->ble_connection_param_update.bd_addr,
                                p_event_data->ble_connection_param_update.conn_interval, p_event_data->ble_connection_param_update.conn_latency,
                                p_event_data->ble_connection_param_update.supervision_timeout);
            break;
#endif

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
            WICED_BT_TRACE("Power mgmt status event: bd ( %B ) status:%d hci_status:%d\n", p_power_mgmt_notification->bd_addr,
                    p_power_mgmt_notification->status, p_power_mgmt_notification->hci_status);
            break;

        case BTM_SCO_CONNECTED_EVT:
        case BTM_SCO_DISCONNECTED_EVT:
        case BTM_SCO_CONNECTION_REQUEST_EVT:
        case BTM_SCO_CONNECTION_CHANGE_EVT:
#ifdef WICED_APP_HFP_AG_INCLUDED
            if (hfp_profile_role == HFP_AUDIO_GATEWAY_ROLE)
            {
                hfp_ag_sco_management_callback( event, p_event_data );
            }
#endif
#ifdef WICED_APP_HFP_HF_INCLUDED
            if (hfp_profile_role == HFP_HANDSFREE_UNIT_ROLE)
            {
                hci_control_hf_sco_management_callback( event, p_event_data );
            }
#endif
            break;

        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
    }
    return result;
}

wiced_result_t btm_enabled_event_handler(wiced_bt_dev_enabled_t *event_data)
{
    wiced_result_t result;

#ifdef CYW20706A2
    wiced_bt_app_init();
#endif

    write_eir( );

    /* create SDP records */
    if (!wiced_bt_sdp_db_init((uint8_t *)wiced_app_cfg_sdp_record, wiced_app_cfg_sdp_record_get_size()))
    {
        WICED_BT_TRACE("ERROR sdp_db_init\n");
        return WICED_BT_ERROR;
    }

    hci_control_post_init();

#ifdef CYW9BT_AUDIO
    /* Initialize AudioManager */
    wiced_am_init();
#endif

    WICED_BT_TRACE("Free Bytes After Init:%d\n", wiced_memory_get_free_bytes());

    return WICED_BT_SUCCESS;
}
