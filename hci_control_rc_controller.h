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

#pragma once

#include <stdint.h>
#include <wiced_result.h>
#include "wiced_bt_avrc_ct.h"
#include "app.h"

#define MAX_POSSIBLE_APP_ATTR_VALUES    4
#define MAX_POSSIBLE_APP_ATTR_SETTINGS  4

typedef struct
{
    wiced_bool_t available;
    uint8_t      current_index;
    uint8_t      num_possible_values;
    uint8_t      possible_values[MAX_POSSIBLE_APP_ATTR_VALUES]; /* Values are all 1 based */
} tAVRC_APP_SETTING_ATTR;

typedef struct
{
    wiced_bt_device_address_t remote_addr;
    wiced_bt_avrc_ct_connection_state_t connection_state;

    uint8_t num_app_settings;
    uint8_t num_app_settings_init;
    tAVRC_APP_SETTING_ATTR app_setting[MAX_POSSIBLE_APP_ATTR_SETTINGS + 1];
    uint8_t handle;
} tRC_APP_CB;

void hci_control_rc_controller_init(void);
wiced_result_t hci_control_rc_controller_send_pass_through_cmd(uint8_t op_id, uint8_t state);
void avrc_handle_registered_notification_rsp(uint8_t handle, app_avrc_reg_notif_rsp_t *reg_notif);
void avrc_passthrough_cback( uint8_t handle, uint8_t operation_id, wiced_bt_avrc_ctype_t ctype, app_avrc_pass_thru_hdr_t *avrc_pass_rsp);
