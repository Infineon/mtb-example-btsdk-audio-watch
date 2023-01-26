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

/** @file
 *
 * app.c
 * This file is applicable for all devices with BTSTACK version 3.0 and greater, for example 55572
 *
 */
#include "wiced_bt_dev.h"
#include "wiced_bt_avrc_ct.h"
#include "wiced_bt_stack.h"
#include "wiced_transport.h"
#include "wiced_app.h"
#include "wiced_app_cfg.h"
#include "app.h"
#include "hci_control_api.h"
#include "hci_control_le.h"
#include "hci_control_rc_controller.h"

/******************************************************
 *               defines
 ******************************************************/

/******************************************************
 *               extern variables
 ******************************************************/

/******************************************************
 *               static function
 ******************************************************/
/*
 * app_transport_tx_cplt_cback.
 * This function is called when a Transport Buffer has been sent to the MCU
 */
static void app_transport_tx_cplt_cback(void)
{
    return;
}

/******************************************************
 *               data structure
 ******************************************************/
const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
    .heap_config =
    {
        .data_heap_size = 1024 * 4 + 1500 * 2,
        .hci_trace_heap_size = 1024 * 2,
        .debug_trace_heap_size = 1024,
    },
    .p_status_handler    = hci_control_transport_status,
    .p_data_handler      = hci_control_proc_rx_cmd,
    .p_tx_complete_cback = app_transport_tx_cplt_cback
};

/******************************************************
 *               variables
 ******************************************************/
static wiced_bt_heap_t * p_default_heap = NULL;
wiced_bt_db_hash_t       watch_db_hash;
wiced_bt_pool_t *        p_key_info_pool;  //Pool for storing the  key info

/******************************************************
 *               Functions
 ******************************************************/
wiced_result_t app_read_buffer_stats()
{
    /*
     * Get statistics of default heap.
     * TODO: get statistics of stack heap (btu_cb.p_heap)
     */
    wiced_bt_heap_statistics_t heap_stat;

    if (wiced_bt_get_heap_statistics(p_default_heap, &heap_stat))
    {
        WICED_BT_TRACE("--- heap_size:%d ---\n", heap_stat.heap_size);
        WICED_BT_TRACE("max_single_allocation:%d max_heap_size_used:%d\n",
                heap_stat.max_single_allocation,
                heap_stat.max_heap_size_used);
        WICED_BT_TRACE("allocation_failure_count:%d current_largest_free_size:%d\n",
                heap_stat.allocation_failure_count,
                heap_stat.current_largest_free_size);
        WICED_BT_TRACE("current_num_allocations:%d current_size_allocated:%d\n",
                heap_stat.current_num_allocations,
                heap_stat.current_size_allocated);
        WICED_BT_TRACE("current_num_free_fragments:%d current_free_size\n",
                heap_stat.current_num_free_fragments,
                heap_stat.current_free_size);

        wiced_transport_send_data( HCI_CONTROL_EVENT_READ_BUFFER_STATS,
                (uint8_t *)&heap_stat, sizeof(heap_stat) );
        return WICED_BT_SUCCESS;
    }
    return WICED_BT_ERROR;
}

void app_pr_dev_started_evt()
{
    WICED_BT_TRACE( "maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n",
            wiced_bt_cfg_settings.p_l2cap_app_cfg->max_app_l2cap_channels,
            wiced_bt_cfg_settings.p_l2cap_app_cfg->max_app_l2cap_psms,
            wiced_bt_cfg_settings.p_br_cfg->rfcomm_cfg.max_links,
            wiced_bt_cfg_settings.p_br_cfg->rfcomm_cfg.max_ports );
}

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to WICED_FALSE indicating that data does not need to be forwarded.
 */
int app_write_nvram( int nvram_id, int data_len, void *p_data, wiced_bool_t from_host )
{
    uint8_t                    tx_buf[257];
    uint8_t                   *p = tx_buf;
    hci_control_nvram_chunk_t *p1;
    wiced_result_t            result;

    wiced_bt_device_link_keys_t data;
    wiced_bt_device_link_keys_t_20721 * p_data_from_host;
    wiced_bt_device_link_keys_t_20721 device_link_key_data;

    if (from_host)
    {
        p_data_from_host = (wiced_bt_device_link_keys_t_20721 *)p_data;
        memset(&data, 0, sizeof(wiced_bt_device_link_keys_t));
        memcpy(&data.bd_addr, &p_data_from_host->bd_addr, sizeof(wiced_bt_device_address_t));
        data.key_data.br_edr_key_type = p_data_from_host->key_data.br_edr_key_type;
        memcpy(&data.key_data.br_edr_key, &p_data_from_host->key_data.br_edr_key, sizeof(wiced_bt_link_key_t));
        data.key_data.le_keys_available_mask = p_data_from_host->key_data.le_keys_available_mask;
        data.key_data.ble_addr_type = p_data_from_host->key_data.ble_addr_type;
        memcpy(&data.key_data.le_keys, &p_data_from_host->key_data.le_keys, sizeof(wiced_bt_ble_keys_t));
        data_len = sizeof(wiced_bt_device_link_keys_t);
        p_data = &data;
    }

    /* first check if this ID is being reused and release the memory chunk */
    hci_control_delete_nvram( nvram_id, WICED_FALSE );

    /* Allocating a buffer from the pool created for storing the peer info */
    if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
    {
        WICED_BT_TRACE( "Failed to alloc:%d\n", data_len );
        return ( 0 );
    }

    if ( wiced_bt_get_buffer_size( p1 ) < ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) )
    {
        WICED_BT_TRACE( "Insufficient buffer size, Buff Size %d, Len %d  \n",
                        wiced_bt_get_buffer_size( p1 ),
                        ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) );
        wiced_bt_free_buffer( p1 );
        return ( 0 );
    }

    p1->p_next    = p_nvram_first;
    p1->nvram_id  = nvram_id;
    p1->chunk_len = data_len;
    memcpy( p1->data, p_data, data_len );

    p_nvram_first = p1;

    wiced_bt_device_link_keys_t * p_keys = ( wiced_bt_device_link_keys_t *) p_data;

    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys );

    WICED_BT_TRACE("Updated Addr Resolution DB:%d\n", result );

    // If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport
    if (!from_host)
    {
        *p++ = nvram_id & 0xff;
        *p++ = (nvram_id >> 8) & 0xff;

        memset(&device_link_key_data, 0, sizeof(wiced_bt_device_link_keys_t_20721));
        memcpy(&device_link_key_data.bd_addr, &p_keys->bd_addr, sizeof(wiced_bt_device_address_t));

        device_link_key_data.key_data.br_edr_key_type = p_keys->key_data.br_edr_key_type;
        memcpy(&device_link_key_data.key_data.br_edr_key, &p_keys->key_data.br_edr_key, sizeof(wiced_bt_link_key_t));
        device_link_key_data.key_data.le_keys_available_mask = p_keys->key_data.le_keys_available_mask;
        device_link_key_data.key_data.ble_addr_type = p_keys->key_data.ble_addr_type;
        memcpy(&device_link_key_data.key_data.le_keys, &p_keys->key_data.le_keys, sizeof(wiced_bt_ble_keys_t));
        memcpy(p, &device_link_key_data, sizeof(wiced_bt_device_link_keys_t_20721));
        data_len = sizeof(wiced_bt_device_link_keys_t_20721);

        wiced_transport_send_data( HCI_CONTROL_EVENT_NVRAM_DATA, tx_buf, ( int )( data_len + 2 ) );
    }
    else
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
    return (data_len);
}

wiced_result_t app_stack_init( void )
{
    /* Create default heap */
    p_default_heap = wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL,
            WICED_TRUE);
    if (p_default_heap == NULL)
    {
        WICED_BT_TRACE("create default heap error: size %d\n", BT_STACK_HEAP_SIZE);
        return WICED_BT_NO_RESOURCES;
    }
    return wiced_bt_stack_init(btm_event_handler, &wiced_bt_cfg_settings);
}

wiced_bt_gatt_status_t app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    switch( event )
    {
    case GATT_GET_RESPONSE_BUFFER_EVT:
        p_data->buffer_request.buffer.p_app_rsp_buffer = wiced_bt_get_buffer(p_data->buffer_request.len_requested);
        p_data->buffer_request.buffer.p_app_ctxt = wiced_bt_free_buffer;
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            void (*pfn_free)(uint8_t *) =
                (void (*)(uint8_t *))p_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
                pfn_free(p_data->buffer_xmitted.p_app_data);
        }
        break;

    default:
        break;
    }

    return WICED_BT_GATT_SUCCESS;
}

void app_btu_hcif_send_cmd( uint16_t opcode, uint8_t* params, uint8_t params_length )
{
    /* TODO: allocate buffer for sending HCI command */
#if 0
        BT_HDR  *p_command;
        uint8_t *p;

        if ( ( p_command = HCI_GET_CMD_BUF( params_length ) ) == NULL )
        {
            hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED );
            return;
        }

        p = ( uint8_t * )( p_command + 1 );

        p_command->event  = BT_EVT_TO_LM_HCI_CMD;
        p_command->len    = HCIC_PREAMBLE_SIZE + params_length;
        p_command->offset = 0;

        UINT16_TO_STREAM( p, opcode );
        UINT8_TO_STREAM( p, params_length );

        if ( params_length )
        {
            ARRAY_TO_STREAM( p, params, params_length );
        }

        btu_hcif_send_cmd (LOCAL_BR_EDR_CONTROLLER_ID,  p_command);
#endif
}

void app_paired_device_link_keys_update( wiced_bt_management_evt_data_t *p_event_data )
{
    int nvram_id, bytes_written;
    wiced_bt_device_address_t zero_bda = {0};
    wiced_bt_management_evt_data_t modified_event_data;

    /* Check if we already have information saved for this bd_addr */
    if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN ) ) == 0)
    {
        // This is the first time, allocate id for the new memory chunk
        nvram_id = hci_control_alloc_nvram_id( );
        WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
    }
    bytes_written = hci_control_write_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), &p_event_data->paired_device_link_keys_update, WICED_FALSE );

    WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, p_event_data->paired_device_link_keys_update.bd_addr);

    // addr_mapping table is used to send identity address when pairing complete
    if ( memcmp(p_event_data->paired_device_link_keys_update.conn_addr, zero_bda, sizeof(wiced_bt_device_address_t)) != 0 )
    {
        app_identity_random_mapping_t * addr_map = get_empty_addr_mapping();
        if (addr_map != NULL)
        {
            memcpy(addr_map->identity_addr, p_event_data->paired_device_link_keys_update.bd_addr, sizeof(wiced_bt_device_address_t));
            memcpy(addr_map->random_addr, p_event_data->paired_device_link_keys_update.conn_addr, sizeof(wiced_bt_device_address_t));
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT identity addr [%B] random addr [%B]\n", addr_map->identity_addr, addr_map->random_addr);
        }
    }
}

/*
 *  copy advertisement data to report buffer
 *
 *  parameter p_adv_data -- pointer to advertisement data
 *            buff       -- pointer to buffer
 *            buff_len   -- buffer length
 *
 *  returns   number of bytes copied
 */
uint32_t app_copy_advertisement_data( uint8_t *p_adv_data, uint8_t * buff, uint32_t buff_len )
{
    uint8_t   len = 0;

    if ( p_adv_data != NULL )
    {
        // new BTSTACK will send pointer to start of ADV data, and length is the first byte.
        len = *p_adv_data++;  // number of bytes to copy
        if (len > buff_len)
        {
            WICED_BT_TRACE("Bad data\n");  // buffer is not enough to copy data
        }
        else
        {
            memcpy(buff, p_adv_data, len);
        }
    }

    return (uint32_t) len;
}

