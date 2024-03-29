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
 * app.c
 * This file is applicable for all devices with BTSTACK version lower than 3.0, i.e. 20xxx and 43012C0
 *
 */
#include "wiced_bt_dev.h"
#include "wiced_bt_stack.h"
#include "hci_control.h"
#include "wiced_bt_dev.h"
#include "wiced_app_cfg.h"
#include "wiced_app.h"
#include "app.h"
#include "cycfg_gatt_db.h"
#include "hci_control_rc_controller.h"

/******************************************************
 *               defines
 ******************************************************/
#ifdef CYW20819A1
#define WICED_TRANSPORT_BUFFER_COUNT    1
#else
#define WICED_TRANSPORT_BUFFER_COUNT    2
#endif

/******************************************************
 *               extern variables
 ******************************************************/
extern void btu_hcif_send_cmd (uint8_t controller_id, BT_HDR *p_buf);

/******************************************************
 *               static function
 ******************************************************/
/*
 * app_transport_tx_cplt_cback.
 * This function is called when a Transport Buffer has been sent to the MCU
 */
static void app_transport_tx_cplt_cback(wiced_transport_buffer_pool_t* p_pool)
{
    WICED_BT_TRACE( " hci_control_transport_tx_cplt_cback %x \n", p_pool );
}

/******************************************************
 *               data structure
 ******************************************************/
#if ( !defined(CYW43012C0) && (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI))

 #ifndef CYW20706A2
  #ifdef CYW20819A1
   #define SPI_GPIO_CFG    SPI_PIN_CONFIG(WICED_P09, WICED_P15, WICED_P06, WICED_P17)
  #else
   #define SLAVE1_P01_CS_P10_CLK_P28_MOSI_P29_MISO     0x010A1C1D
   #define SPI_GPIO_CFG    SLAVE1_P01_CS_P10_CLK_P28_MOSI_P29_MISO
  #endif
 #endif
const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_SPI,
    .cfg =
    {
        .spi_cfg =
        {
 #ifdef CYW20706A2
            .dev_role            = SPI_SLAVE_ROLE,
            .spi_gpio_cfg        = SLAVE1_P02_CS_P03_CLK_P00_MOSI_P25_MISO, /**< Pins to use for the data and clk lines. Refer  spiffdriver.h for details */
            .spi_pin_pull_config = INPUT_PIN_FLOATING,
 #elif (CYW20719B2 || CYW20721B2)
 #else
            .dev_role            = SPI_SLAVE,
            .spi_gpio_cfg        = SPI_GPIO_CFG, /**< Pins to use for the data and clk lines. Refer  spiffdriver.h for details */
            .spi_pin_pull_config = INPUT_PIN_PULL_DOWN,
 #endif
            .clock_speed         = 0,
            .endian              = SPI_MSB_FIRST,
            .polarity            = SPI_SS_ACTIVE_LOW,
            .mode                = SPI_MODE_0,
 #ifdef CYW20706A2
            .cs_pin              =  0,
            .slave_ready_pin     =  WICED_P15
 #elif (CYW20719B2 || CYW20721B2)
 #else
            .cs_pin              =  0,
            .slave_ready_pin     =  WICED_P06
 #endif
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_SPI_BUFFER_SIZE,
        .buffer_count = WICED_TRANSPORT_BUFFER_COUNT
    },
    .p_status_handler    = hci_control_transport_status,
    .p_data_handler      = hci_control_proc_rx_cmd,
    .p_tx_complete_cback = app_transport_tx_cplt_cback
};

#else // !( !defined(CYW43012C0) && (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI))

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
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = WICED_TRANSPORT_BUFFER_COUNT
    },
    .p_status_handler    = hci_control_transport_status,
    .p_data_handler      = hci_control_proc_rx_cmd,
    .p_tx_complete_cback = app_transport_tx_cplt_cback
};

#endif

const uint8_t cy_bt_adv_packet_elem_0[1] = { BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED };

wiced_bt_ble_advert_elem_t cy_bt_adv_packet_data[CY_BT_ADV_PACKET_DATA_SIZE] =
{
    /* Flags */
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_FLAG,
        .len = 1,
        .p_data = (uint8_t*)cy_bt_adv_packet_elem_0,
    },
    /* Complete local name */
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,
        .len = 5,           // app_gap_device_name_len,
        .p_data = ( uint8_t* )app_gap_device_name,
    },
};

/******************************************************
 *               variables
 ******************************************************/
wiced_bt_buffer_pool_t*     p_key_info_pool;  //Pool for storing the  key info

/******************************************************
 *               Functions
 ******************************************************/
wiced_result_t app_read_buffer_stats()
{
    uint8_t buff_pools = 0;
#ifdef WICEDX
#define BUFF_POOLS 5
    wiced_bt_buffer_statistics_t buff_stats[BUFF_POOLS];
    buff_pools = BUFF_POOLS;
#else
    wiced_bt_buffer_statistics_t buff_stats[wiced_bt_get_number_of_buffer_pools()];
    buff_pools = wiced_bt_get_number_of_buffer_pools();
#endif
    wiced_result_t result;
    uint8_t i;

    result = wiced_bt_get_buffer_usage( buff_stats, sizeof( buff_stats ) );

    if( result == WICED_BT_SUCCESS )
    {
        // Print out the stats to trace
        WICED_BT_TRACE( "Buffer usage statistics:\n");

        for( i=0; i < buff_pools; i++) {
            WICED_BT_TRACE("pool_id:%d size:%d curr_cnt:%d max_cnt:%d total:%d\n",
                           buff_stats[i].pool_id, buff_stats[i].pool_size,
                           buff_stats[i].current_allocated_count, buff_stats[i].max_allocated_count,
                           buff_stats[i].total_count);
        }

        // Return the stats via WICED-HCI
        wiced_transport_send_data( HCI_CONTROL_EVENT_READ_BUFFER_STATS, (uint8_t*)&buff_stats, sizeof( buff_stats ) );
    }
    return result;
}

void app_pr_dev_started_evt()
{
    WICED_BT_TRACE( "maxLinks:%d maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n",
            wiced_bt_cfg_settings.l2cap_application.max_links,
            wiced_bt_cfg_settings.l2cap_application.max_channels,
            wiced_bt_cfg_settings.l2cap_application.max_psm,
            wiced_bt_cfg_settings.rfcomm_cfg.max_links,
            wiced_bt_cfg_settings.rfcomm_cfg.max_ports );
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
#ifdef CYW20706A2
    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys, p_keys->key_data.ble_addr_type );
#else
    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys );
#endif
    WICED_BT_TRACE("Updated Addr Resolution DB:%d\n", result );

    // If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport
    if (!from_host)
    {
        *p++ = nvram_id & 0xff;
        *p++ = (nvram_id >> 8) & 0xff;

        memcpy(p, p_data, data_len);

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
    return wiced_bt_stack_init(btm_event_handler, &wiced_bt_cfg_settings, wiced_app_cfg_buf_pools);
}

void app_btu_hcif_send_cmd( uint16_t opcode, uint8_t* params, uint8_t params_length )
{
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
}

void app_paired_device_link_keys_update( wiced_bt_management_evt_data_t *p_event_data )
{
    int nvram_id, bytes_written;
    wiced_bt_device_address_t zero_bda = {0};
    wiced_bt_management_evt_data_t modified_event_data;

    /* Check if we already have information saved for this bd_addr */
    if ( memcmp(p_event_data->paired_device_link_keys_update.key_data.static_addr, zero_bda, sizeof(wiced_bt_device_address_t)) != 0 )
    {
        // Save link key infomation for static_addr
        memcpy(&modified_event_data, p_event_data, sizeof(wiced_bt_management_evt_data_t));
        memcpy(modified_event_data.paired_device_link_keys_update.bd_addr, modified_event_data.paired_device_link_keys_update.key_data.static_addr, sizeof(wiced_bt_device_address_t));

        if ( ( nvram_id = hci_control_find_nvram_id( modified_event_data.paired_device_link_keys_update.bd_addr, BD_ADDR_LEN ) ) == 0)
        {
            // This is the first time, allocate id for the new memory chunk
            nvram_id = hci_control_alloc_nvram_id( );
            WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
        }
        bytes_written = hci_control_write_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), &modified_event_data.paired_device_link_keys_update, WICED_FALSE );

        WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, modified_event_data.paired_device_link_keys_update.bd_addr);
    }
    else
    {
        if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN ) ) == 0)
        {
            // This is the first time, allocate id for the new memory chunk
            nvram_id = hci_control_alloc_nvram_id( );
            WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
        }
        bytes_written = hci_control_write_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), &p_event_data->paired_device_link_keys_update, WICED_FALSE );

        WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, p_event_data->paired_device_link_keys_update.bd_addr);
    }

    if ( memcmp(p_event_data->paired_device_link_keys_update.key_data.static_addr, zero_bda, sizeof(wiced_bt_device_address_t)) != 0 )
    {
        app_identity_random_mapping_t * addr_map = get_empty_addr_mapping();
        if (addr_map != NULL)
        {
            memcpy(addr_map->random_addr, p_event_data->paired_device_link_keys_update.bd_addr, sizeof(wiced_bt_device_address_t));
            memcpy(addr_map->identity_addr, p_event_data->paired_device_link_keys_update.key_data.static_addr, sizeof(wiced_bt_device_address_t));
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT identity addr [%B] random addr [%B]\n", addr_map->identity_addr, addr_map->random_addr);
        }
    }
}

/*
 *  copy advertisement data to report buffer
 *
 *  parameter p_adv_data -- pointer to advertisement data
 *                          new BTSTACK will send pointer to start of ADV data, and length is the first byte.
 *            buff       -- pointer to buffer
 *            buff_len   -- buffer length
 *
 *  returns   number of bytes copied
 */
uint32_t app_copy_advertisement_data( uint8_t *p_adv_data, uint8_t * buff, uint32_t buff_len )
{
    uint8_t   len;
    uint32_t  total_len = 0;

    // currently callback does not pass the data of the adv data, need to go through the data
    // zero len in the LTV means that there is no more data
    while ( ( p_adv_data != NULL ) && ( len = *p_adv_data ) != 0 )
    {
        // In the HCI event all parameters should fit into buffer limit (255 bytes)
        if ( len + total_len > buff_len )
        {
            WICED_BT_TRACE("Bad data\n");
            break;
        }

        len++; // include length byte itself
        total_len += len;

        while (len--)
        {
            *buff++ = *p_adv_data++;
        }
    }
    return total_len;
}
