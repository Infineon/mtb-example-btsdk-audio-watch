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


#ifdef WICED_APP_PANU_INCLUDED

#include <wiced_bt_cfg.h>
#include "hci_control_api.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_pan.h"
#include "pan_api.h"
#include "panu_sdp.h"
#include "hci_control_panu.h"
#include "wiced_transport.h"
#include "wiced_memory.h"

extern pan_session_cb_t sdp_panu_scb;

void panu_hci_send_panu_event(uint16_t evt, uint16_t handle, panu_event_t *p_data)
{
    uint8_t   tx_buf[300];
    uint8_t  *p = tx_buf;
    int       i;

    WICED_BT_TRACE("panu_hci_send_panu_event: Sending Event: 0x%04x  to UART\n", evt);

    switch ( evt )
    {
    case HCI_CONTROL_PANU_EVENT_OPEN:
        for ( i = 0; i < BD_ADDR_LEN; i++ )
            *p++ = p_data->open.bd_addr[BD_ADDR_LEN - 1 - i];
        break;

    case HCI_CONTROL_PANU_EVENT_CONNECTED:
        for ( i = 0; i < BD_ADDR_LEN; i++ )
            *p++ = p_data->conn.bd_addr[BD_ADDR_LEN - 1 - i];
        break;

    case HCI_CONTROL_PANU_EVENT_SERVICE_NOT_FOUND:
    case HCI_CONTROL_PANU_EVENT_CONNECTION_FAILED:
    case HCI_CONTROL_PANU_EVENT_DISCONNECTED:
    default:
        break;
    }

    *p++ = ( uint8_t ) ( handle );
    *p++ = ( uint8_t ) ( handle >> 8 );

    wiced_transport_send_data(evt, tx_buf, ( int ) ( p - tx_buf ));
}

void panu_connected(BD_ADDR bd_addr)
{
    pan_session_cb_t *p_scb = &sdp_panu_scb;
    p_scb->state = PANU_STATE_CONNECT;

    panu_connect_t    connect;
    utl_bdcpy( connect.bd_addr, bd_addr );

    panu_hci_send_panu_event( HCI_CONTROL_PANU_EVENT_CONNECTED, p_scb->app_handle, (panu_event_t *)&connect );
}

void panu_disconnected(uint16_t handle)
{
    pan_session_cb_t *p_scb = &sdp_panu_scb;
    p_scb->state = PANU_STATE_IDLE;

    if (p_scb->app_handle != handle)
    {
        WICED_BT_TRACE( "panu_disconnected, p_scb->app_handle = %d, handle = %d\n", p_scb->app_handle, handle );
        return;
    }

    panu_hci_send_panu_event( HCI_CONTROL_PANU_EVENT_DISCONNECTED, p_scb->app_handle, NULL );
}

void panu_connect_failed(uint16_t handle)
{
    pan_session_cb_t *p_scb = &sdp_panu_scb;
    p_scb->state = PANU_STATE_IDLE;

    if (p_scb->app_handle != handle)
    {
        WICED_BT_TRACE( "panu_connect_failed, p_scb->app_handle = %d, handle = %d\n", p_scb->app_handle, handle );
        return;
    }

    panu_hci_send_panu_event(HCI_CONTROL_PANU_EVENT_CONNECTION_FAILED, p_scb->app_handle, NULL);
}

static void panu_conn_state_cback(uint16_t handle, BD_ADDR bd_addr, tPAN_RESULT state,
                                     BOOLEAN is_role_change, uint8_t src_role, uint8_t dst_role)
{
    WICED_BT_TRACE("panu_conn_state_cback handle = %d, state = %d, is_role_change = %d, src_role = %d, dst_role = %d\n",
                    handle, state, is_role_change, src_role, dst_role);

    if (state == PAN_SUCCESS)
    {
        panu_connected(bd_addr);
    }
    else if (state == PAN_DISCONNECTED)
    {
        panu_disconnected(handle);
    }
    else
    {
        panu_connect_failed(handle);
    }
}

static void panu_data_buf_ind_cback(uint16_t handle, BD_ADDR src, BD_ADDR dst, uint16_t protocol,
                                                    uint8_t *data_buf, uint16_t data_len, BOOLEAN ext, BOOLEAN forward)
{
    WICED_BT_TRACE("panu_data_buf_ind_cback handle = %d, BD_ADDR src = %B, BD_ADDR dst = %B \n", handle, src, dst);
    WICED_BT_TRACE("panu_data_buf_ind_cback protocol = %d, ext = %d, forward = %d, data_len = %d \n", protocol, ext, forward, data_len);
}

static void panu_data_flow_cb(uint16_t handle, tPAN_RESULT result)
{
    WICED_BT_TRACE("panu_data_flow_cb handle:0x%x, result:0x%x \n", handle, result);
}

static void panu_pfilt_ind_cback(uint16_t handle, BOOLEAN indication,tBNEP_RESULT result,
                                    uint16_t num_filters, uint8_t *p_filters)
{
    WICED_BT_TRACE("panu_pfilt_ind_cback handle:0x%x, indication:0x%x \n", handle, indication);
    WICED_BT_TRACE("panu_pfilt_ind_cback result:0x%x, num_filters:0x%x \n", result, num_filters);
}

static void panu_mfilt_ind_cback(uint16_t handle, BOOLEAN indication,tBNEP_RESULT result,
                                    uint16_t num_mfilters, uint8_t *p_mfilters)
{
    WICED_BT_TRACE("panu_mfilt_ind_cback handle:0x%x, indication:0x%x \n", handle, indication);
    WICED_BT_TRACE("panu_mfilt_ind_cback result:0x%x, num_mfilters:0x%x \n", result, num_mfilters);
}

void hci_control_panu_init(void)
{
    tPAN_REGISTER reg_data;

    wiced_bt_panu_sdp_init();
    wiced_bt_bnep_init();
    wiced_bt_pan_init();

    WICED_BT_TRACE( "bta_pan_enable\n");

    reg_data.pan_conn_state_cb  = panu_conn_state_cback;
    reg_data.pan_bridge_req_cb  = NULL;
    reg_data.pan_data_buf_ind_cb = panu_data_buf_ind_cback;
    reg_data.pan_data_ind_cb = NULL;
    reg_data.pan_pfilt_ind_cb = panu_pfilt_ind_cback;
    reg_data.pan_mfilt_ind_cb = panu_mfilt_ind_cback;
    reg_data.pan_tx_data_flow_cb = panu_data_flow_cb;
    wiced_bt_pan_register(&reg_data);

    wiced_bt_pan_setrole(PAN_ROLE_CLIENT);
}

void hci_control_panu_handle_command(uint16_t opcode, uint8_t* p_data, uint32_t length)
{
    uint16_t handle;
    uint8_t  hs_cmd;
    uint8_t  *p = ( uint8_t * ) p_data;

    switch (opcode)
    {
    case HCI_CONTROL_PANU_COMMAND_CONNECT:
        WICED_BT_TRACE("HCI_CONTROL_PANU_COMMAND_CONNECT\n");
        wiced_bt_panu_connect(p);
        break;

    case HCI_CONTROL_PANU_COMMAND_DISCONNECT:
        WICED_BT_TRACE("HCI_CONTROL_PANU_COMMAND_DISCONNECT\n");
        handle = p[0] | ( p[1] << 8 );
        wiced_bt_panu_disconnect(handle);
        break;
    }
}


typedef struct {
    uint16_t handle;
    uint16_t num_filter;
    uint16_t data[BNEP_MAX_PROT_FILTERS * 2];
} tAPP_PAN_PFILT;

typedef struct
{
    uint16_t start;
    uint16_t end;
} tAPP_PAN_PFILTER;

tAPP_PAN_PFILTER app_pan_pfilter1[] = {
    { 0x0800, 0x0800 },
    { 0x86DD, 0x86DD },
    { 0x0806, 0x0806 },
};

void hci_control_panu_set_pfilter(uint16_t handle)
{
    uint8_t i;
    uint8_t j;
    pan_session_cb_t *p_scb = &sdp_panu_scb;
    tAPP_PAN_PFILT pfilter;

    WICED_BT_TRACE("panu_set_pfilter State: %d\n", p_scb->state);

    if (handle != p_scb->app_handle)
    {
        WICED_BT_TRACE("handle error \n");
        return;
    }

    if (p_scb->state != PANU_STATE_CONNECT)
    {
        WICED_BT_TRACE( "p_scb->state error \n" );
        return;
    }

    memset(&pfilter, 0, sizeof(tAPP_PAN_PFILT));
    pfilter.handle = p_scb->app_handle;

    WICED_BT_TRACE("pfilter list:");
    for (i = 0; i < (sizeof(app_pan_pfilter1) / sizeof(app_pan_pfilter1[0])); i++)
    {
        WICED_BT_TRACE("    Start:0x%04X    End:%04X", app_pan_pfilter1[i].start, app_pan_pfilter1[i].end);
    }

    pfilter.num_filter = sizeof(app_pan_pfilter1) / sizeof(app_pan_pfilter1[0]);
    if (pfilter.num_filter > BNEP_MAX_PROT_FILTERS)
    {
        WICED_BT_TRACE("Too big array");
        return;
    }

    for (j = 0; j < pfilter.num_filter; j++)
    {
        pfilter.data[j] = app_pan_pfilter1[j].start;
        pfilter.data[j + pfilter.num_filter] = app_pan_pfilter1[j].end;
    }
    wiced_bt_pan_set_protocol_filters(pfilter.handle, pfilter.num_filter,
                                   pfilter.data, pfilter.data + pfilter.num_filter);
}

typedef struct {
    uint16_t handle;
    uint16_t num_filter;
    BD_ADDR data[BNEP_MAX_MULTI_FILTERS * 2];
} tAPP_PAN_MFILT;

typedef struct
{
    BD_ADDR start;
    BD_ADDR end;
} tAPP_PAN_MFILTER;

tAPP_PAN_MFILTER app_pan_mfilter3[] = {
    {{ 0x03, 0x00, 0x02, 0x30, 0x00, 0x02},
            { 0x03, 0x00, 0x02, 0x30, 0x00, 0x04}},
    {{ 0x03, 0x00, 0x02, 0x30, 0x00, 0x06},
            { 0x03, 0x00, 0x02, 0x30, 0x00, 0x08}},
};

void bdcpy(BD_ADDR a, const BD_ADDR b)
{
    int i;

    for (i = BD_ADDR_LEN; i != 0; i--)
    {
        *a++ = *b++;
    }
}
void hci_control_panu_set_mfilter(uint16_t handle)
{
    uint8_t i;
    uint8_t j;
    BD_ADDR *p;
    BD_ADDR *q;
    pan_session_cb_t *p_scb = &sdp_panu_scb;
    tAPP_PAN_MFILT mfilter;

    WICED_BT_TRACE("panu_set_mfilter State: %d\n", p_scb->state);

    if (handle != p_scb->app_handle)
    {
        WICED_BT_TRACE("handle error \n");
        return;
    }

    if (p_scb->state != PANU_STATE_CONNECT)
    {
        WICED_BT_TRACE( "p_scb->state error \n" );
        return;
    }

    memset(&mfilter, 0, sizeof(tAPP_PAN_MFILT));
    mfilter.handle = p_scb->app_handle;

    WICED_BT_TRACE("mfilter list:");
    for (i = 0; i < (sizeof(app_pan_mfilter3) / sizeof(app_pan_mfilter3[0])); i++)
    {
        WICED_BT_TRACE("    Start  %02X:%02X:%02X:%02X:%02X:%02X     "
                       "End  %02X:%02X:%02X:%02X:%02X:%02X",
                            app_pan_mfilter3[i].start[0],
                            app_pan_mfilter3[i].start[1],
                            app_pan_mfilter3[i].start[2],
                            app_pan_mfilter3[i].start[3],
                            app_pan_mfilter3[i].start[4],
                            app_pan_mfilter3[i].start[5],
                            app_pan_mfilter3[i].end[0],
                            app_pan_mfilter3[i].end[1],
                            app_pan_mfilter3[i].end[2],
                            app_pan_mfilter3[i].end[3],
                            app_pan_mfilter3[i].end[4],
                            app_pan_mfilter3[i].end[5]);
    }

    mfilter.num_filter = sizeof(app_pan_mfilter3) / sizeof(app_pan_mfilter3[0]);
    if (mfilter.num_filter > BNEP_MAX_MULTI_FILTERS)
    {
        WICED_BT_TRACE("Too big array");
        return;
    }

    p = (BD_ADDR*)&mfilter.data[0];
    q = (BD_ADDR*)&mfilter.data[mfilter.num_filter];
    for (j = 0; j < mfilter.num_filter; j++)
    {
        bdcpy((uint8_t *)(p++), app_pan_mfilter3[j].start);
        bdcpy((uint8_t *)(q++), app_pan_mfilter3[j].end);
    }
    wiced_bt_pan_set_multicast_filters(mfilter.handle, mfilter.num_filter,
        (uint8_t *)mfilter.data, (uint8_t *)(mfilter.data + mfilter.num_filter));
}

#ifdef WICED_APP_PAN_PTS_INCLUDED
void testcase_pan_panu_ip_app_bv_05_i(uint16_t handle)
{
    BD_ADDR local_addr;
    pan_session_cb_t *p_scb = &sdp_panu_scb;

    WICED_BT_TRACE("[%u]PAN_PANU_IP_APP_BV_05_I State: %u\n", p_scb->app_handle, p_scb->state);

    if (handle != p_scb->app_handle)
    {
        WICED_BT_TRACE("handle error \n");
        return;
    }

    if (p_scb->state != PANU_STATE_CONNECT)
    {
        WICED_BT_TRACE("p_scb->state error \n");
        return;
    }
    wiced_bt_dev_read_local_addr(local_addr);

    {
        uint8_t ping_data[84] = {0x45, 0x00, 0x00, 0x54, 0xe9, 0x7b, 0x00, 0x00, 0x40, 0x01,

                                             /* 192.168.168.100  */  /* 192.168.167.152  */
                                 0xb7, 0xd9, 0xc0, 0xa8, 0xa8, 0x64, 0xc0, 0xa8, 0xa7, 0x98,

                                 0x00, 0x00, 0x4f, 0xad, 0x44, 0x30, 0x00, 0x01, 0x87, 0x49,
                                 0x44, 0x61, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xa3, 0x01, 0x00,
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
            WICED_BT_TRACE( "PAN_PANU_IP_APP_BV_05_I wiced_bt_get_buffer fail\n" );
            return;
        }
        ping_buf->event  = 3333;
        ping_buf->len    = 84;
        ping_buf->offset = 22;
        ping_buf->layer_specific = 1;

        p = (uint8_t *)(ping_buf + 1) + ping_buf->offset;
        ARRAY_TO_STREAM(p, ping_data, 84);
        wiced_bt_pan_writebuf(p_scb->app_handle, p_scb->remote_addr, local_addr, protocol, ping_buf, ext);
        wiced_bt_free_buffer(ping_buf);
    }
}

void testcase_pan_panu_ipv4_autonet_bv_01_I(uint16_t handle)
{
    BD_ADDR local_addr;
    pan_session_cb_t *p_scb = &sdp_panu_scb;

    WICED_BT_TRACE("[%u]PAN_PANU_IPv4_AUTONET_BV_01_I State: %u\n", p_scb->app_handle, p_scb->state);

    if (handle != p_scb->app_handle)
    {
        WICED_BT_TRACE("handle error \n");
        return;
    }

    if (p_scb->state != PANU_STATE_CONNECT)
    {
        WICED_BT_TRACE("p_scb->state error \n");
        return;
    }
    wiced_bt_dev_read_local_addr(local_addr);

    {
        uint8_t arp_data[28] =  {0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xbe, 0xef,
                                 0xbe, 0xef, 0x07, 0x6e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

                                                         /* 192.168.167.152  */
                                 0x00, 0x00, 0x00, 0x00, 0xc0, 0xa8, 0xa7, 0x98};
        uint16_t  protocol = 2054;
        BOOLEAN ext = 0;
        BT_HDR  *arp_buf;
        uint8_t *p;

        if ((arp_buf = (BT_HDR  *)wiced_bt_get_buffer(1024)) == NULL)
        {
            WICED_BT_TRACE( "panu_pingpacket wiced_bt_get_buffer fail\n" );
            return;
        }
        arp_buf->event  = 3333;
        arp_buf->len    = 28;
        arp_buf->offset = 22;
        arp_buf->layer_specific = 1;

        p = (uint8_t *)(arp_buf + 1) + arp_buf->offset;
        ARRAY_TO_STREAM(p, arp_data, 28);

        wiced_bt_pan_writebuf(p_scb->app_handle, p_scb->remote_addr, local_addr, protocol, arp_buf, ext);
        wiced_bt_free_buffer(arp_buf);
    }
}
#endif //WICED_APP_PAN_PTS_INCLUDED

#endif //WICED_APP_PANU_INCLUDED
