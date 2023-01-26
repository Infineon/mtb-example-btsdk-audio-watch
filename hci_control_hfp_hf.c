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
 * This file implements Hands-free profile Handsfree Unit
 *
 */

#ifdef WICED_APP_HFP_HF_INCLUDED

#include "hci_control_api.h"
#include "wiced_bt_trace.h"
#include "hci_control_hfp_hf.h"
#include "wiced_bt_sco.h"
#include "wiced_transport.h"
#include "wiced_memory.h"
#include "hci_control.h"
#if defined(CYW20721B2) || defined(CYW43012C0)
#define HFP_VOLUME_HIGH 15
#include "wiced_audio_manager.h"
#endif

#include "hci_control_audio.h"

/******************************************************
 *               Variables Definitions
 ******************************************************/
bluetooth_hfp_context_t handsfree_ctxt_data;
hci_control_hfp_hf_app_cb handsfree_app_states;

wiced_bt_sco_params_t handsfree_esco_params =
{
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        0x000D,             /* Latency: 13 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( T2 ) */
#else
        0x000C,             /* Latency: 12 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( S4 ) */
#endif
        HANDS_FREE_SCO_PKT_TYPES,
        BTM_ESCO_RETRANS_POWER, /* Retrans Effort ( At least one retrans, opt for power ) ( S4 ) */
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        WICED_TRUE
#else
        WICED_FALSE
#endif
};

#if defined(CYW20721B2) || defined(CYW43012C0)
static int32_t stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
static audio_config_t audio_config =
    {
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        .sr = AM_PLAYBACK_SR_16K,
#else
        .sr = AM_PLAYBACK_SR_8K,
#endif
       .channels = 1,
       .bits_per_sample = DEFAULT_BITSPSAM,
       .volume = AM_VOL_LEVEL_HIGH-2,
       .mic_gain = AM_VOL_LEVEL_HIGH-2,
       .sink = AM_HEADPHONES,
    };
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/
static void hfp_timer_expiry_handler( uint32_t param )
{
    /* if sco is not created as an acceptor then remove the sco and create it as initiator. */
    if( handsfree_ctxt_data.call_active && !handsfree_ctxt_data.is_sco_connected )
    {
        wiced_bt_sco_remove( handsfree_ctxt_data.sco_index );
        wiced_bt_sco_create_as_initiator( handsfree_ctxt_data.peer_bd_addr, &handsfree_ctxt_data.sco_index, (wiced_bt_sco_params_t *) &handsfree_esco_params );
    }
}

static void handsfree_init_context_data(void)
{
    handsfree_ctxt_data.call_active         = 0;
    handsfree_ctxt_data.call_held           = 0;
    handsfree_ctxt_data.call_setup          = WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE;
    handsfree_ctxt_data.connection_status   = WICED_BT_HFP_HF_STATE_DISCONNECTED;
    handsfree_ctxt_data.spkr_volume         = 8;
    handsfree_ctxt_data.mic_volume          = 8;
    handsfree_ctxt_data.sco_index           = BT_AUDIO_INVALID_SCO_INDEX;
    handsfree_ctxt_data.init_sco_conn       = WICED_FALSE;
}

static void hci_control_send_hf_event(uint16_t evt, uint16_t handle, hci_control_hfp_hf_event_t *p_data)
{
    uint8_t   tx_buf[300];
    uint8_t  *p = tx_buf;
    int       i;

    WICED_BT_TRACE("[%u]hci_control_send_hf_event: Sending Event: %u  to UART\n", handle, evt);

    *p++ = (uint8_t)(handle);
    *p++ = (uint8_t)(handle >> 8);

    switch (evt)
    {
        case HCI_CONTROL_HF_EVENT_OPEN:                 /* HS connection opened or connection attempt failed  */
            for (i = 0; i < BD_ADDR_LEN; i++)
                *p++ = p_data->open.bd_addr[BD_ADDR_LEN - 1 - i];
            *p++ = p_data->open.status;
            break;

        case HCI_CONTROL_HF_EVENT_CLOSE:                /* HS connection closed */
            break;

        case HCI_CONTROL_HF_EVENT_AUDIO_OPEN:           /* Audio connection open */
            break;

        case HCI_CONTROL_HF_EVENT_AUDIO_CLOSE:          /* Audio connection closed */
            break;

        case HCI_CONTROL_HF_EVENT_CONNECTED:            /* HS Service Level Connection is UP */
            UINT32_TO_STREAM(p,p_data->conn.peer_features);
            break;

        case HCI_CONTROL_HF_EVENT_PROFILE_TYPE:
            UINT8_TO_STREAM(p,p_data->conn.profile_selected);
            break;
        default:                                        /* AT response */
            if (p_data)
            {
                *p++ = (uint8_t)(p_data->val.num);
                *p++ = (uint8_t)(p_data->val.num >> 8);
                utl_strcpy((char *)p, p_data->val.str);
                p += strlen(p_data->val.str) + 1;
            }
            else
            {
                *p++ = 0;               // val.num
                *p++ = 0;
                *p++ = 0;               // empty val.str
            }
            break;
    }
    wiced_transport_send_data(evt, tx_buf, (int)(p - tx_buf));
}

static void handsfree_connection_event_handler(wiced_bt_hfp_hf_event_data_t* p_data)
{
    wiced_bt_dev_status_t status;

    if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_CONNECTED)
    {
        hci_control_hfp_hf_open_t    open;
        wiced_bt_hfp_hf_scb_t *p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr (p_data->conn_data.remote_address);
        memcpy(open.bd_addr,p_data->conn_data.remote_address,BD_ADDR_LEN);
        open.status = WICED_BT_SUCCESS;
        handsfree_ctxt_data.rfcomm_handle = p_scb->rfcomm_handle;
        hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_OPEN, p_scb->rfcomm_handle, (hci_control_hfp_hf_event_t *) &open);

        if( p_data->conn_data.connected_profile == WICED_BT_HFP_PROFILE )
        {
            handsfree_app_states.connect.profile_selected = WICED_BT_HFP_PROFILE;
            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_PROFILE_TYPE, p_scb->rfcomm_handle, (hci_control_hfp_hf_event_t *) &handsfree_app_states.connect);
        }
        else
        {
            handsfree_app_states.connect.profile_selected = WICED_BT_HSP_PROFILE;
            memcpy( handsfree_ctxt_data.peer_bd_addr, p_data->conn_data.remote_address, sizeof(wiced_bt_device_address_t));
            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_PROFILE_TYPE, p_scb->rfcomm_handle, (hci_control_hfp_hf_event_t *) &handsfree_app_states.connect);
        }

        status = wiced_bt_sco_create_as_acceptor(&handsfree_ctxt_data.sco_index);
        WICED_BT_TRACE("%s: status [%d] SCO INDEX [%d] \n", __func__, status, handsfree_ctxt_data.sco_index);
    }
    else if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_SLC_CONNECTED)
    {
        WICED_BT_TRACE("%s: Peer BD Addr [%B]\n", __func__,p_data->conn_data.remote_address);

        memcpy( handsfree_ctxt_data.peer_bd_addr, p_data->conn_data.remote_address, sizeof(wiced_bt_device_address_t));
    }
    else if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_DISCONNECTED)
    {
        memset(handsfree_ctxt_data.peer_bd_addr, 0, sizeof(wiced_bt_device_address_t));
        if(handsfree_ctxt_data.sco_index != BT_AUDIO_INVALID_SCO_INDEX)
        {
            status = wiced_bt_sco_remove(handsfree_ctxt_data.sco_index);
            handsfree_ctxt_data.sco_index = BT_AUDIO_INVALID_SCO_INDEX;
            WICED_BT_TRACE("%s: remove sco status [%d] \n", __func__, status);
        }
        hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_CLOSE, handsfree_ctxt_data.rfcomm_handle, NULL);
    }
    UNUSED_VARIABLE(status);
}

static void handsfree_send_ciev_cmd (uint16_t handle, uint8_t ind_id,uint8_t ind_val, hci_control_hfp_hf_value_t *p_val)
{
    wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(handle);
    p_val->str[0] = '0'+ind_id;
    p_val->str[1] = ',';
    p_val->str[2] = '0'+ind_val;
    p_val->str[3] = '\0';
    hci_control_send_hf_event( HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CIEV, p_scb->rfcomm_handle, (hci_control_hfp_hf_event_t *)p_val );
}

static void handsfree_send_clcc_evt (uint16_t handle, wiced_bt_hfp_hf_active_call_t *active_call, hci_control_hfp_hf_value_t *p_val)
{
    wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(handle);
    int i = 0;

    p_val->str[i++] = '0'+active_call->idx;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->dir;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->status;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->mode;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->is_conference;

    if(active_call->type)
    {
        p_val->str[i++] = ',';
        memcpy(&p_val->str[i],active_call->num,strlen(active_call->num));
        i +=  strlen(active_call->num);
        p_val->str[i++] = ',';
        i += utl_itoa (active_call->type,&p_val->str[i]);
    }
    p_val->str[i++] = '\0';
    hci_control_send_hf_event( HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CLCC, p_scb->rfcomm_handle, (hci_control_hfp_hf_event_t *)p_val );
}

static void handsfree_call_setup_event_handler(wiced_bt_hfp_hf_call_data_t* call_data)
{
    switch (call_data->setup_state)
    {
        case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING:
            WICED_BT_TRACE("%s: Call(incoming) setting-up\n", __func__);
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:
            if(call_data->active_call_present == 0)
            {
                if(handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING ||
                        handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING ||
                        handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING )
                {
                    WICED_BT_TRACE("Call: Inactive; Call Set-up: IDLE\n");
                    break;
                }
                /* If previous context has an active-call and active_call_present is 0 */
                if(handsfree_ctxt_data.call_active == 1)
                {
                    WICED_BT_TRACE("Call Terminated\n");
                    break;
                }
            }
            else if( call_data->active_call_present == 1)
            {
                WICED_BT_TRACE("Call: Active; Call-setup: DONE\n");
            }
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:
            WICED_BT_TRACE("Call(outgoing) setting-up\n");
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING:
            WICED_BT_TRACE("Remote(outgoing) ringing\n");
            break;

        default:
            break;
    }
    handsfree_ctxt_data.call_active = call_data->active_call_present;
    handsfree_ctxt_data.call_setup  = call_data->setup_state;
    handsfree_ctxt_data.call_held   = call_data->held_call_present;
}

static void handsfree_event_callback( wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data)
{
    hci_control_hfp_hf_event_t     p_val;
    int res = 0;

    memset(&p_val,0,sizeof(hci_control_hfp_hf_event_t));

    switch(event)
    {
        case WICED_BT_HFP_HF_CONNECTION_STATE_EVT:
            handsfree_connection_event_handler(p_data);
            break;

        case WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT:
            res = HCI_CONTROL_HF_EVENT_CONNECTED;
            p_val.conn.peer_features = p_data->ag_feature_flags;

            if(p_data->ag_feature_flags & WICED_BT_HFP_AG_FEATURE_INBAND_RING_TONE_CAPABILITY)
            {
                handsfree_ctxt_data.inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_ENABLED;
            }
            else
            {
                handsfree_ctxt_data.inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_DISABLED;
            }
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
            {
                wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->handle);
                if( (p_data->ag_feature_flags & WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION) &&
                        (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION) )
                {
                    handsfree_esco_params.use_wbs = WICED_TRUE;
                }
                else
                {
                    handsfree_esco_params.use_wbs = WICED_FALSE;
                }
            }
#endif
            break;

        case WICED_BT_HFP_HF_SERVICE_STATE_EVT:
            handsfree_send_ciev_cmd (p_data->handle,WICED_BT_HFP_HF_SERVICE_IND,p_data->service_state,&p_val.val);
            break;

        case WICED_BT_HFP_HF_CALL_SETUP_EVT:
        {
            if (handsfree_ctxt_data.call_active != p_data->call_data.active_call_present)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_IND,p_data->call_data.active_call_present,&p_val.val);

            if (handsfree_ctxt_data.call_held != p_data->call_data.held_call_present)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_HELD_IND,p_data->call_data.held_call_present,&p_val.val);

            if (handsfree_ctxt_data.call_setup != p_data->call_data.setup_state)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_SETUP_IND,p_data->call_data.setup_state,&p_val.val);

            handsfree_call_setup_event_handler(&p_data->call_data);
        }
            break;

        case WICED_BT_HFP_HF_RSSI_IND_EVT:
            handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_SIGNAL_IND,p_data->rssi,&p_val.val);
            break;

        case WICED_BT_HFP_HF_SERVICE_TYPE_EVT:
            handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_ROAM_IND,p_data->service_type,&p_val.val);
            break;

        case WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT:
            handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_BATTERY_IND,p_data->battery_level,&p_val.val);
            break;

        case WICED_BT_HFP_HF_RING_EVT:
            WICED_BT_TRACE("%s: RING \n", __func__);
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_RING;
            break;

        case WICED_BT_HFP_HF_INBAND_RING_STATE_EVT:
            handsfree_ctxt_data.inband_ring_status = p_data->inband_ring;
            break;

        case WICED_BT_HFP_HF_OK_EVT:
            WICED_BT_TRACE("%s: OK \n", __func__);
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_OK;
            break;

        case WICED_BT_HFP_HF_ERROR_EVT:
            WICED_BT_TRACE("%s: Error \n", __func__);
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_ERROR;
            break;

        case WICED_BT_HFP_HF_CME_ERROR_EVT:
            WICED_BT_TRACE("%s: CME Error \n", __func__);
            p_val.val.num = p_data->error_code;
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CMEE;
            break;

        case WICED_BT_HFP_HF_CLIP_IND_EVT:
            p_val.val.num = p_data->clip.type;
            strncpy( p_val.val.str, p_data->clip.caller_num, sizeof( p_val.val.str ) );
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CLIP;
            WICED_BT_TRACE("%s: CLIP - number %s, type %d\n", __func__, p_data->clip.caller_num, p_data->clip.type);
            break;

        case WICED_BT_HFP_HF_BINP_EVT:
            p_val.val.num = p_data->binp_data.type;
            strncpy( p_val.val.str, p_data->binp_data.caller_num, sizeof( p_val.val.str ) );
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BINP;
            WICED_BT_TRACE("%s: BINP - number %s, type %d\n", __func__, p_data->binp_data.caller_num, p_data->binp_data.type);
            break;

        case WICED_BT_HFP_HF_VOLUME_CHANGE_EVT:
            WICED_BT_TRACE("%s: %s VOLUME - %d \n", __func__, (p_data->volume.type == WICED_BT_HFP_HF_SPEAKER)?"SPK":"MIC",  p_data->volume.level);
            if (p_data->volume.type == WICED_BT_HFP_HF_MIC )
            {
                res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_VGM;
            }
            else
            {
                res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_VGS;
            }
            p_val.val.num = p_data->volume.level;
            break;

        case WICED_BT_HFP_HFP_CODEC_SET_EVT:
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BCS;
            if ( p_data->selected_codec == WICED_BT_HFP_HF_MSBC_CODEC )
                handsfree_esco_params.use_wbs = WICED_TRUE;
            else
                handsfree_esco_params.use_wbs = WICED_FALSE;
            p_val.val.num = p_data->selected_codec;


            if (handsfree_ctxt_data.init_sco_conn == WICED_TRUE)
            {
                /* timer started here to check if the sco has been created as an acceptor*/
                wiced_start_timer(&handsfree_app_states.hfp_timer,SCO_CONNECTION_WAIT_TIMEOUT);

                handsfree_ctxt_data.init_sco_conn = WICED_FALSE;
            }
#if defined(CYW20721B2) || defined(CYW43012C0)
            WICED_BT_TRACE("%s - CODEC_SET: %d\n", __func__, p_data->selected_codec);
            if ( p_data->selected_codec == WICED_BT_HFP_HF_MSBC_CODEC ) {
                handsfree_esco_params.use_wbs = WICED_TRUE;
                audio_config.sr = 16000;
            }
            else {
                handsfree_esco_params.use_wbs = WICED_FALSE;
                audio_config.sr = 8000;
            }

            audio_config.channels =  1;
            audio_config.bits_per_sample = DEFAULT_BITSPSAM;
            audio_config.volume = AM_VOL_LEVEL_HIGH-2;
            if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                stream_id = wiced_am_stream_open(HFP);
            }

            if( WICED_SUCCESS != wiced_am_stream_set_param(stream_id,AM_AUDIO_CONFIG, &audio_config))
                WICED_BT_TRACE("wiced_am_set_param failed\n");
#endif
            break;

        case WICED_BT_HFP_HFP_ACTIVE_CALL_EVT:
            handsfree_send_clcc_evt(p_data->handle,&p_data->active_call,&p_val.val);
            break;

        case WICED_BT_HFP_HF_CNUM_EVT:
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CNUM;
            memcpy(p_val.val.str, p_data->cnum_data, strlen(p_data->cnum_data));
            break;

        case WICED_BT_HFP_HF_BIND_EVT:
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BIND;
            p_val.val.str[0] = p_data->bind_data.ind_id + '0';
            p_val.val.str[1] = ',';
            p_val.val.str[2] = p_data->bind_data.ind_value + '0';
            break;

        default:
            break;
    }
    if ( res && (res <= (HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_MAX)) )
    {
        wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->handle);
        hci_control_send_hf_event( res, p_scb->rfcomm_handle, (hci_control_hfp_hf_event_t *)&p_val );
    }
}

static void handsfree_hfp_init(void)
{
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_hfp_hf_config_data_t config;

    handsfree_init_context_data();

    config.feature_mask     = BT_AUDIO_HFP_SUPPORTED_FEATURES;
    config.speaker_volume   = handsfree_ctxt_data.spkr_volume;
    config.mic_volume       = handsfree_ctxt_data.mic_volume;
#ifdef WICED_ENABLE_BT_HSP_PROFILE
    config.num_server       = 2;
#else
    config.num_server       = 1;
#endif
    config.scn[0]           = HANDS_FREE_SCN;
    config.uuid[0]          = UUID_SERVCLASS_HF_HANDSFREE;
#ifdef WICED_ENABLE_BT_HSP_PROFILE
    config.scn[1]           = HEADSET_SCN;
    config.uuid[1]          = UUID_SERVCLASS_HEADSET;
#endif

    result = wiced_bt_hfp_hf_init(&config, handsfree_event_callback);
    WICED_BT_TRACE("[%s] SCO Setting up voice path = %d\n",__func__, result);
}

/*
 * Handsfree Unit init
 */
void hci_control_hf_init(void)
{
    handsfree_app_states.pairing_allowed = WICED_FALSE;
    wiced_init_timer(&handsfree_app_states.hfp_timer, hfp_timer_expiry_handler, 0, WICED_MILLI_SECONDS_TIMER);
    handsfree_hfp_init();
}

void hci_control_hf_send_at_cmd (uint16_t handle,char *cmd, uint8_t arg_type, uint8_t arg_format, const char *p_arg, int16_t int_arg)
{
    char    buf[WICED_BT_HFP_HF_AT_MAX_LEN + 16];
    char    *p = buf;

    memset (buf, 0, (WICED_BT_HFP_HF_AT_MAX_LEN+16));

    *p++ = 'A';
    *p++ = 'T';

    /* copy result code string */
    memcpy(p,cmd, strlen(cmd));
    p += strlen(cmd);

    if(arg_type == WICED_BT_HFP_HF_AT_SET)
    {
        *p++ = '=';

    }
    else if(arg_type == WICED_BT_HFP_HF_AT_READ)
    {
        *p++ = '?';

    }
    else if(arg_type == WICED_BT_HFP_HF_AT_TEST)
    {
        *p++ = '=';
        *p++ = '?';

    }

    /* copy argument if any */
    if (arg_format == WICED_BT_HFP_HF_AT_FMT_INT)
    {
        p += utl_itoa((uint16_t) int_arg, p);
    }
    else if (arg_format == WICED_BT_HFP_HF_AT_FMT_STR)
    {
        utl_strcpy(p, (char *)p_arg);
        p += strlen(p_arg);
    }

    /* finish with \r*/
    *p++ = '\r';

    wiced_bt_hfp_hf_send_at_cmd(handle,buf);
}

void hci_control_hf_at_command (uint16_t handle, uint8_t command, int num, uint8_t* p_data)
{
    switch ( command )
    {
        case HCI_CONTROL_HF_AT_COMMAND_SPK:
            wiced_bt_hfp_hf_notify_volume (handle,
                    WICED_BT_HFP_HF_SPEAKER, num);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_MIC:
            wiced_bt_hfp_hf_notify_volume (handle,
                    WICED_BT_HFP_HF_MIC, num);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_BINP:
            hci_control_hf_send_at_cmd( handle, "+BINP",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 1 );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CHLD:
            wiced_bt_hfp_hf_perform_call_action(handle,
                    WICED_BT_HFP_HF_CALL_ACTION_HOLD_0 + num,(char *)p_data);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_BVRA:
            hci_control_hf_send_at_cmd( handle, "+BVRA",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CMEE:
            hci_control_hf_send_at_cmd( handle, "+CMEE",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_A:
            wiced_bt_hfp_hf_perform_call_action(handle,
                        WICED_BT_HFP_HF_CALL_ACTION_ANSWER,(char *)p_data);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CHUP:
            wiced_bt_hfp_hf_perform_call_action(handle,
                        WICED_BT_HFP_HF_CALL_ACTION_HANGUP,(char *)p_data);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CNUM:
            hci_control_hf_send_at_cmd(handle, "+CNUM",
                        WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CLCC:
            hci_control_hf_send_at_cmd(handle, "+CLCC",
                    WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CIND:
            hci_control_hf_send_at_cmd(handle, "+CIND",
                    WICED_BT_HFP_HF_AT_READ, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0 );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_D:
        case HCI_CONTROL_HF_AT_COMMAND_BLDN:
            wiced_bt_hfp_hf_perform_call_action (handle ,
                                        WICED_BT_HFP_HF_CALL_ACTION_DIAL ,(char *)p_data);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_NREC:
            hci_control_hf_send_at_cmd( handle, "+NREC",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 0 );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_VTS:
            hci_control_hf_send_at_cmd( handle, "+VTS",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0 );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_BTRH:
            hci_control_hf_send_at_cmd(handle, "+BTRH",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_BIEV:
            hci_control_hf_send_at_cmd(handle, "+BIEV",
                            WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;
        case HCI_CONTROL_HF_AT_COMMAND_BIA:
            hci_control_hf_send_at_cmd(handle, "+BIA",
                            WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;
    }
}

/*
 * Handle Handsfree commands received over UART.
 */
void hci_control_hf_handle_command(uint16_t opcode, uint8_t* p_data, uint32_t length)
{
    uint16_t                  handle;
    uint8_t                   hs_cmd;
    int                       num;
    uint8_t                  *p = (uint8_t *)p_data;
    BD_ADDR bd_addr;
    wiced_bt_hfp_hf_scb_t    *p_scb = NULL;

    switch (opcode)
    {
    case HCI_CONTROL_HF_COMMAND_CONNECT:
        STREAM_TO_BDADDR(bd_addr,p);
        hci_control_switch_hfp_role(HFP_HANDSFREE_UNIT_ROLE);
        wiced_bt_hfp_hf_connect(bd_addr);
        break;

    case HCI_CONTROL_HF_COMMAND_DISCONNECT:
        handle = p[0] | (p[1] << 8);
        wiced_bt_hfp_hf_disconnect(handle);
        break;

    case HCI_CONTROL_HF_COMMAND_OPEN_AUDIO:
        handle = p[0] | (p[1] << 8);
        p_scb = wiced_bt_hfp_hf_get_scb_by_handle (handle);
        if( p_scb )
        {
            // For all HF initiated audio connection establishments for which both sides support the Codec Negotiation feature,
            // the HF shall trigger the AG to establish a Codec Connection. ( Ref HFP Spec 1.7 : Section 4.11.2 )
            if ( (p_scb->peer_feature_mask & WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION) &&
                    (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION) )
            {
                wiced_bt_hfp_hf_at_send_cmd( p_scb, WICED_BT_HFP_HF_CMD_BCC,
                        WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0 );

                // As per spec on receiving AT+BCC command, AG will respond with OK and initiate Codec Connection Setup procedure.
                // While responding AT+BCS=<codec_id> to AG, from profile we will get "WICED_BT_HFP_HFP_CODEC_SET_EVT" event,
                // and based on init_sco_conn flag we will initiate SCO connection request.
                handsfree_ctxt_data.init_sco_conn = WICED_TRUE;
            }
            else
            {
                wiced_bt_sco_remove( handsfree_ctxt_data.sco_index );
                wiced_bt_sco_create_as_initiator( p_scb->peer_addr, &handsfree_ctxt_data.sco_index, (wiced_bt_sco_params_t *) &handsfree_esco_params );
            }
        }
        break;

    case HCI_CONTROL_HF_COMMAND_CLOSE_AUDIO:
        handle = p[0] | (p[1] << 8);
        //handle is not used
        wiced_bt_sco_remove( handsfree_ctxt_data.sco_index );
        break;

    case HCI_CONTROL_HF_COMMAND_TURN_OFF_PCM_CLK:
        wiced_bt_sco_turn_off_pcm_clock();
        break;

    case HCI_CONTROL_HF_COMMAND_BUTTON_PRESS:
        /* send a corresponding AT command */
#ifdef WICED_ENABLE_BT_HSP_PROFILE
        WICED_BT_TRACE("Send AT+CKPD=200\n");
        hci_control_hf_send_at_cmd(handsfree_ctxt_data.rfcomm_handle,"+CKPD",
                WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 200);
#endif
        break;
    case HCI_CONTROL_HF_COMMAND_LONG_BUTTON_PRESS:
        /* send a corresponding AT command */
        WICED_BT_TRACE("Send AT+BVRA=2\n");
        hci_control_hf_send_at_cmd(handsfree_ctxt_data.rfcomm_handle,"+BVRA",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 2);
        break;

    default:
        {
            uint8_t *data_ptr = (uint8_t *) wiced_bt_get_buffer(length+1);
            hs_cmd = opcode - HCI_CONTROL_HF_AT_COMMAND_BASE;

            memcpy (data_ptr, p, length);
            data_ptr[length] = 0;                      /* NULL terminate the AT string */
            handle = data_ptr[0] | (data_ptr[1] << 8);
            num = data_ptr[2] | (data_ptr[3] << 8);
            hci_control_hf_at_command (handle,hs_cmd, num, data_ptr+4);
            wiced_bt_free_buffer((void *)data_ptr);
        }
        break;
    }
}

#if defined(CYW20721B2) || defined(CYW43012C0)
static int32_t handsfree_utils_hfp_volume_to_am_volume(int32_t vol)
{
    uint32_t remainder;
    int32_t am_level;

    am_level    = (vol * AM_VOL_LEVEL_HIGH) / HFP_VOLUME_HIGH;
    remainder   = (vol * AM_VOL_LEVEL_HIGH) % HFP_VOLUME_HIGH;

    if (remainder >= AM_VOL_LEVEL_HIGH)
    {
        am_level++;
    }

    return am_level;
}
#endif

/*
 * Process SCO management callback
 */
void hci_control_hf_sco_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_hfp_hf_scb_t *p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr (handsfree_ctxt_data.peer_bd_addr);
    int status;

    WICED_BT_TRACE("hf_sco_management_callback: event=%d\n", event);

    switch ( event )
    {
        case BTM_SCO_CONNECTED_EVT:             /**< SCO connected event. Event data: #wiced_bt_sco_connected_t */
#if defined(CYW20721B2) || defined(CYW43012C0)
            /* setup audio path */
            if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                stream_id = wiced_am_stream_open(HFP);
                WICED_BT_TRACE("wiced_am_stream_open completed stream_id: %d\n", stream_id);
            }

            /* Set sample rate. */
            if (handsfree_esco_params.use_wbs == WICED_TRUE)
            {
                audio_config.sr = AM_PLAYBACK_SR_16K;
            }
            else
            {
                audio_config.sr = AM_PLAYBACK_SR_8K;
            }

            audio_config.volume = handsfree_utils_hfp_volume_to_am_volume(AM_VOL_LEVEL_HIGH - 2);
            audio_config.mic_gain = handsfree_utils_hfp_volume_to_am_volume(AM_VOL_LEVEL_HIGH - 2);

            if( WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_AUDIO_CONFIG, &audio_config))
                WICED_BT_TRACE("wiced_am_set_param failed\n");

            if( WICED_SUCCESS != wiced_am_stream_start(stream_id))
                WICED_BT_TRACE("wiced_am_stream_start failed stream_id : %d \n", stream_id);

            /* Set speaker volume and MIC gain to make the volume consistency between call
             * sessions. */
            if (WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_SPEAKER_VOL_LEVEL, (void *) &audio_config.volume))
                WICED_BT_TRACE("wiced_am_set_param failed\n");

            if (WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_MIC_GAIN_LEVEL, (void *) &audio_config.mic_gain))
                WICED_BT_TRACE("wiced_am_set_param failed\n");
#endif /* CYW20721B2 || CYW43012C0 */

            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_AUDIO_OPEN, p_scb->rfcomm_handle, NULL );
            WICED_BT_TRACE("%s: SCO Audio connected, sco_index = %d [in context sco index=%d]\n", __func__, p_event_data->sco_connected.sco_index, handsfree_ctxt_data.sco_index);
            handsfree_ctxt_data.is_sco_connected = WICED_TRUE;

            break;

        case BTM_SCO_DISCONNECTED_EVT:          /**< SCO disconnected event. Event data: #wiced_bt_sco_disconnected_t */
#if defined(CYW20721B2) || defined(CYW43012C0)
            if (stream_id != WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                if( WICED_SUCCESS != wiced_am_stream_stop(stream_id))
                    WICED_BT_TRACE("wiced_am_stream_stop failed stream_id : %d \n", stream_id);

                if( WICED_SUCCESS != wiced_am_stream_close(stream_id))
                    WICED_BT_TRACE("wiced_am_stream_close failed stream_id : %d \n", stream_id);

                stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
            }
#endif
            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_AUDIO_CLOSE, p_scb->rfcomm_handle, NULL );
            WICED_BT_TRACE("%s: SCO disconnection change event handler\n", __func__);

            status = wiced_bt_sco_create_as_acceptor(&handsfree_ctxt_data.sco_index);
            WICED_BT_TRACE("%s: status [%d] SCO INDEX [%d] \n", __func__, status, handsfree_ctxt_data.sco_index);
            handsfree_ctxt_data.is_sco_connected = WICED_FALSE;
            break;

        case BTM_SCO_CONNECTION_REQUEST_EVT:    /**< SCO connection request event. Event data: #wiced_bt_sco_connection_request_t */
            WICED_BT_TRACE("%s: SCO connection request event handler \n", __func__);

            if( wiced_is_timer_in_use(&handsfree_app_states.hfp_timer) )
            {
                wiced_stop_timer(&handsfree_app_states.hfp_timer);
            }

            a2dp_app_hci_control_audio_stop();
            if(handsfree_app_states.connect.profile_selected == WICED_BT_HFP_PROFILE)
            {
                wiced_bt_sco_accept_connection(p_event_data->sco_connection_request.sco_index, HCI_SUCCESS, (wiced_bt_sco_params_t *) &handsfree_esco_params);
            }
#ifdef WICED_ENABLE_BT_HSP_PROFILE
            else
            {
                wiced_bt_sco_accept_connection(p_event_data->sco_connection_request.sco_index, HCI_SUCCESS, (wiced_bt_sco_params_t *) &headset_sco_params);
            }
#endif
            break;

        case BTM_SCO_CONNECTION_CHANGE_EVT:     /**< SCO connection change event. Event data: #wiced_bt_sco_connection_change_t */
            WICED_BT_TRACE("%s: SCO connection change event handler\n", __func__);
            break;
    }
    UNUSED_VARIABLE(status);
}

#endif  // WICED_APP_HFP_HF_INCLUDED
