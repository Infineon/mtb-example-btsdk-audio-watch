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

#ifndef _APP_HCI_H_
#define _APP_HCI_H_

/******************************************************
 *               Macro Function Definitions
 ******************************************************/
#define app_gatt_valid_conn_id( idx ) ( le_control_cb.conn[idx].conn_id != 0 )
#define app_gatt_discovery_type( p ) ( p->discovery_type )

#define APP_GATT_WRITE_NRSP GATT_CMD_WRITE   // should change to GATT_REQ_WRITE?
#define APP_GATT_WRITE GATT_REQ_WRITE

/******************************************************
 *               Function Definitions
 ******************************************************/
uint16_t               app_gatt_get_conn_idx(uint16_t conn_id);
uint16_t               app_gatt_allocate_conn_cb(void);
uint16_t               app_gatt_get_conn_id(uint16_t conn_idx);
wiced_result_t         app_gatt_operation_comp_cb( wiced_bt_gatt_operation_complete_t *p_complete);
wiced_bt_gatt_status_t app_gatt_req_cb(wiced_bt_gatt_attribute_request_t *p_req);
wiced_bt_gatt_status_t app_gatt_send_write(uint8_t conn_idx, uint16_t attr_handle, uint8_t *p_data, uint16_t len, wiced_bt_gatt_opcode_t type);
wiced_bt_gatt_status_t app_gatt_client_send_read_handle(uint16_t conn_idx, uint16_t handle);
wiced_bt_gatt_status_t app_gatt_send_response(uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len);
wiced_bt_gatt_status_t app_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data);
void                   app_gatt_send_read_by_handle(uint16_t conn_id, uint16_t handle);
wiced_bool_t           app_gatt_send_read_by_type(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle, uint16_t uuid);

#endif // _APP_HCI_H_
