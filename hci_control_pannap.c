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


#ifdef WICED_APP_PANNAP_INCLUDED

#include <wiced_bt_cfg.h>
#include "hci_control_api.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_pan.h"
#include "wiced_memory.h"

static void pannap_conn_state_cback(uint16_t handle, BD_ADDR bd_addr, tPAN_RESULT state,
                                     BOOLEAN is_role_change, uint8_t src_role, uint8_t dst_role)
{
    WICED_BT_TRACE("pannap_conn_state_cback handle = %d, state = %d, is_role_change = %d, src_role = %d, dst_role = %d\n",
                   handle, state, is_role_change, src_role, dst_role);
}

static void pannap_data_flow_cb(uint16_t handle, tPAN_RESULT result)
{
    WICED_BT_TRACE("pannap_data_flow_cb handle:0x%x, result:0x%x \n", handle, result);
}

static void pannap_data_buf_ind_cback(uint16_t handle, BD_ADDR src, BD_ADDR dst, uint16_t protocol,
                                                    uint8_t *data_buf, uint16_t data_len, BOOLEAN ext, BOOLEAN forward)
{
    WICED_BT_TRACE("pannap_data_buf_ind_cback handle = %d, BD_ADDR src = %B, BD_ADDR dst = %B \n", handle, src, dst);
    WICED_BT_TRACE("pannap_data_buf_ind_cback protocol = %d, ext = %d, forward = %d, data_len = %d \n",
                   protocol, ext, forward, data_len);
}

static void pannap_pfilt_ind_cback(uint16_t handle, BOOLEAN indication,tBNEP_RESULT result,
                                    uint16_t num_filters, uint8_t *p_filters)
{
    WICED_BT_TRACE("pannap_pfilt_ind_cback handle:0x%x, indication:0x%x \n", handle, indication);
    WICED_BT_TRACE("pannap_pfilt_ind_cback result:0x%x, num_filters:0x%x \n", result, num_filters);
}

static void pannap_mfilt_ind_cback(uint16_t handle, BOOLEAN indication,tBNEP_RESULT result,
                                    uint16_t num_mfilters, uint8_t *p_mfilters)
{
    WICED_BT_TRACE( "pannap_mfilt_ind_cback handle:0x%x, indication:0x%x \n", handle, indication);
    WICED_BT_TRACE( "pannap_mfilt_ind_cback result:0x%x, num_mfilters:0x%x \n", result, num_mfilters);
}

void hci_control_pannap_init( void )
{
    tPAN_REGISTER reg_data;

    WICED_BT_TRACE( "hci_control_pannap_init\n");

    wiced_bt_bnep_init();
    wiced_bt_pan_init();

    WICED_BT_TRACE( "bta_pan_enable\n");

    reg_data.pan_conn_state_cb  = pannap_conn_state_cback;
    reg_data.pan_bridge_req_cb  = NULL;
    reg_data.pan_data_buf_ind_cb = pannap_data_buf_ind_cback;
    reg_data.pan_data_ind_cb = NULL;
    reg_data.pan_pfilt_ind_cb = pannap_pfilt_ind_cback;
    reg_data.pan_mfilt_ind_cb = pannap_mfilt_ind_cback;
    reg_data.pan_tx_data_flow_cb = pannap_data_flow_cb;

    wiced_bt_pan_register(&reg_data);
    wiced_bt_pan_setrole(PAN_ROLE_NAP_SERVER);
}

#ifdef WICED_APP_PAN_PTS_INCLUDED
void testcase_pan_nap_bridge_rx_bv_02_i(uint16_t handle)
{
    BD_ADDR remote_addr;
    remote_addr[0] = 0x00;
    remote_addr[1] = 0x1b;
    remote_addr[2] = 0xdc;
    remote_addr[3] = 0x08;
    remote_addr[4] = 0xe3;
    remote_addr[5] = 0xde;

    BD_ADDR local_addr;
    wiced_bt_dev_read_local_addr(local_addr);

    {
        uint8_t ping_data[84] = {0x45, 0x00, 0x00, 0x54, 0x15, 0xeb, 0x40, 0x00, 0x40, 0x01,

                                             /* 192.168.167.152  */  /* 192.168.168.100  */
                                 0x4b, 0x69, 0xc0, 0xa8, 0xa7, 0x98, 0xc0, 0xa8, 0xa8, 0x64,

                                 0x08, 0x00, 0xc3, 0x84, 0x32, 0xd2, 0x00, 0x01, 0x09, 0x78,
                                 0x02, 0x61, 0x00, 0x00, 0x00, 0x00, 0x30, 0xfc, 0x06, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
                                 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
                                 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
                                 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
                                 0x34, 0x35, 0x36, 0x37};
        uint16_t  protocol = 2048;
        BOOLEAN ext = 0;
        BT_HDR  *ping_buf;
        uint8_t *p;

        if ((ping_buf = (BT_HDR  *)wiced_bt_get_buffer(1024)) == NULL)
        {
            WICED_BT_TRACE( "PAN_NAP_BRIDGE_RX_BV_02_I wiced_bt_get_buffer fail\n" );
            return;
        }
        ping_buf->event  = 3333;
        ping_buf->len    = 84;
        ping_buf->offset = 22;
        ping_buf->layer_specific = 1;

        p = (uint8_t *)(ping_buf + 1) + ping_buf->offset;
        ARRAY_TO_STREAM(p, ping_data, 84);

        wiced_bt_pan_writebuf(handle, remote_addr, local_addr, protocol, ping_buf, ext);

        wiced_bt_free_buffer(ping_buf);
    }
}
#endif //WICED_APP_PAN_PTS_INCLUDED

#endif //WICED_APP_PANNAP_INCLUDED
