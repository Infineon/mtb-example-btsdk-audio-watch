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
 * app.h
 * This file is applicable for all devices with BTSTACK version lower than 3.0, i.e. 20xxx and 43012C0
 *
 */
#ifndef __APP_H_
#define __APP_H_

#include "wiced_bt_gatt.h"
#include "wiced_transport.h"
#include "wiced_memory.h"
#include "hci_control.h"
#include "app_avrc.h"
#include "app_gatt.h"

/******************************************************
 *               defines
 ******************************************************/
#define CY_BT_ADV_PACKET_DATA_SIZE  3

/******************************************************
 *               typedef
 ******************************************************/
typedef wiced_bt_gatt_write_t app_gatt_write_req_t;

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t  app_stack_init( void );
wiced_result_t  app_read_buffer_stats( void );
void            app_pr_dev_started_evt();
int             app_write_nvram( int nvram_id, int data_len, void *p_data, wiced_bool_t from_host );
void            app_btu_hcif_send_cmd( uint16_t opcode, uint8_t* params, uint8_t params_length );
uint32_t        app_copy_advertisement_data( uint8_t *p_adv_data, uint8_t * buff, uint32_t buff_len );
void            app_paired_device_link_keys_update( wiced_bt_management_evt_data_t *p_event_data );

/******************************************************
 *               Macro Function Definitions
 ******************************************************/
#define app_create_pool( size, count ) wiced_bt_create_pool( size, count )
#define app_transport_send_hci_trace( type, data, len ) wiced_transport_send_hci_trace( NULL, type, len, data )
#define app_free_rx_cmd_buffer( ptr ) wiced_transport_free_buffer( ptr )
#define app_get_pool_free_count( pool ) wiced_bt_get_buffer_count( pool )
#define app_cfg_sec_mask() ( wiced_bt_cfg_settings.security_requirement_mask )
#define APP_AVDT_CB p_ctrl_cback
#define app_gatt_db_init( d, l ) wiced_bt_gatt_db_init( d, l )
#define app_allocate_conn_id( id ) ( id )
#define GATT_BLE_DEFAULT_MTU_SIZE GATT_DEF_BLE_MTU_SIZE
#define app_gatt_callback( e, d ) ( WICED_SUCCESS )


/******************************************************
 *               extern variables
 ******************************************************/
extern wiced_bt_buffer_pool_t*          p_key_info_pool;  //Pool for storing the  key info
extern const wiced_transport_cfg_t      transport_cfg;
extern const wiced_bt_cfg_buf_pool_t    wiced_app_cfg_buf_pools[];
extern wiced_bt_ble_advert_elem_t       cy_bt_adv_packet_data[];

#endif // __APP_H_
