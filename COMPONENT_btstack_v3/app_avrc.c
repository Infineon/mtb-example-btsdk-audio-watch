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
#include "hci_control_rc_controller.h"

/******************************************************
 *               defines
 ******************************************************/
#define GET_ELEMENT_ATTR_RSP_LEN(str_len)   (sizeof(uint16_t) + sizeof(uint8_t)  + sizeof(uint8_t)  + sizeof(uint16_t) + str_len) /* handle(2) + status(1) + type_id(1) + len(2)) */

/******************************************************
 *               extern variables
 ******************************************************/
extern tRC_APP_CB rc_app_cb;

wiced_result_t app_avrc_set_player_settings (uint16_t handle, uint8_t attr_id )
{
    wiced_bt_avrc_metadata_set_app_value_cmd_t app_param;
    wiced_bt_avrc_app_setting_t app_setting;

    app_setting.attr_id = attr_id;
    app_param.num_val      = 1;
    app_param.p_vals   = &app_setting;

    rc_app_cb.app_setting[attr_id].current_index =
                    (rc_app_cb.app_setting[attr_id].current_index + 1) %
                     rc_app_cb.app_setting[attr_id].num_possible_values;

    app_setting.attr_val =
              rc_app_cb.app_setting[attr_id].possible_values[rc_app_cb.app_setting[attr_id].current_index];

    return wiced_bt_avrc_ct_set_player_value_cmd((uint8_t) handle, &app_param );
}

void app_avrc_handle_registered_notification_rsp(uint8_t handle, app_avrc_response_t *avrc_rsp)
{
    avrc_handle_registered_notification_rsp(handle, &avrc_rsp->type.metadata.u.reg_notif);
}

void app_avrc_setting_change(uint8_t event_data[], uint16_t * idx, wiced_bt_avrc_player_app_param_t * setting)
{
    int i;

    for (i = 0; i < setting->num_attr && setting->p_attrs; i = i+2)
    {
        // Update shuffle status in app_setting
        if ( setting->p_attrs[i] == AVRC_PLAYER_SETTING_SHUFFLE )
        {
            if ( setting->p_attrs[i+1] == AVRC_PLAYER_VAL_OFF )
            {
                //Shuffle Off
                rc_app_cb.app_setting[AVRC_PLAYER_SETTING_SHUFFLE].current_index = 0;
            }
            else if (  setting->p_attrs[i+1] == AVRC_PLAYER_VAL_ALL_SHUFFLE )
            {
                //Shuffle All
                rc_app_cb.app_setting[AVRC_PLAYER_SETTING_SHUFFLE].current_index = 1;
            }
            else
            {
                //Unexpected attr_value
            }
        }

        event_data[(*idx)++] = setting->p_attrs[i];
        event_data[(*idx)++] = setting->p_attrs[i+1];
    }
}

/**
 *
 * Function         avrc_handle_element_attribute_rsp
 *
 *                  Callback invoked by avrc_response_cback to inform of a response to the
 *                  get element attributes request.
 *
 * @param[in]       remote_addr : Address of the peer device
 * @param[in]       avrc_rsp    : AVRC response messages
 *
 * @return          Nothing
 */
void app_avrc_handle_element_attribute_rsp(uint8_t handle, app_avrc_response_t *avrc_rsp)
{
    int i;
    int rsp_size;
    uint8_t *rsp;

    wiced_bt_avrc_metadata_get_element_attrs_rsp_t *elem_attrs_rsp = &avrc_rsp->type.metadata.u.get_elem_attrs;
    wiced_bt_avrc_attr_entry_t attr_entry;
    /* If successful, make room for the response. */
    /* Determine the number of bytes necessary to transport each element separately to MCU */
    for ( i = 0; i < elem_attrs_rsp->num_attr; i++ )
    {
        wiced_bt_avrc_parse_get_element_attr_rsp_from_stream(elem_attrs_rsp->p_attr_stream, elem_attrs_rsp->length, &attr_entry);

        rsp_size = GET_ELEMENT_ATTR_RSP_LEN( attr_entry.name.name.str_len);

        /* Make sure that there is enough room in the allocated buffer for the result */
        if ( rsp_size <= WICED_BUFF_MAX_SIZE )
        {
            WICED_BT_TRACE( "[%s]: rsp_size: %d attr: %d, strlen: %d\n", __FUNCTION__,
                            rsp_size,
                            attr_entry.attr_id,
                            attr_entry.name.name.str_len);
            rsp = (uint8_t *)wiced_bt_get_buffer( rsp_size );
            if (rsp != NULL)
            {
                /* Playing Time attribute is an ASCII string containing milli-sec */
                /* We need to check case where a duration of 0 is received */
                if ((attr_entry.attr_id == AVRC_MEDIA_ATTR_ID_PLAYING_TIME) &&
                    (attr_entry.name.name.str_len >= 3))
                {
                    /* Convert from milli-sec to sec (by ignoring the last 3 digits) */
                    attr_entry.name.name.str_len -= 3;
                    rsp_size -= 3;
                }
                rsp[0] = handle;
                rsp[1] = 0;
                rsp[2] = WICED_SUCCESS; //avrc_status_to_wiced_result(avrc_rsp->rsp.status);
                rsp[3] = ( uint8_t ) attr_entry.attr_id;
                rsp[4] = attr_entry.name.name.str_len & 0xff;
                rsp[5] = ( attr_entry.name.name.str_len >> 8) & 0xff;
                memcpy( &rsp[6], attr_entry.name.name.p_str, attr_entry.name.name.str_len );

                hci_control_send_avrc_event( HCI_CONTROL_AVRC_CONTROLLER_EVENT_CURRENT_TRACK_INFO, rsp, (uint16_t)rsp_size );
                wiced_bt_free_buffer(rsp);
            }
        }
    }
}

/**
 *
 * Function         avrc_passthrough_cback
 *
 *                  Callback invoked on completion of a passthrough command. If the
 *                  command had been a "PRESS" state command then the "RELEASE" is automatically
 *                  sent by this callback except in the case of Fast Forward and Rewind which
 *                  require MCU intervention.
 *
 * @param[in]       remote_addr   : Address of the peer device
 * @param[in]       avrc_pass_rsp : AVRC passthrough command response
 *
 * @return          Nothing
 */
void app_avrc_passthrough_cback(uint8_t handle, wiced_bt_avrc_ctype_t ctype, wiced_bt_avrc_pass_thru_hdr_t *avrc_pass_rsp)
{
    avrc_passthrough_cback( handle, avrc_pass_rsp->operation_id, ctype, avrc_pass_rsp);
}
