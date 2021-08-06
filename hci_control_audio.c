/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
 * This file implements audio application controlled over HCI.
 *
 */
#include "wiced_bt_trace.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_audio.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_app_cfg.h"
#include "wiced_result.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "hci_control.h"
#include "hci_control_audio.h"
#include "hci_control_rc_target.h"
#include "wiced_timer.h"
#include "string.h"
#include "wiced_memory.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_bt_a2d.h"
#include "wiced_transport.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW43012C0) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20819A1) || BTSTACK_VER >= 0x01020000 )
#include "wiced_bt_event.h"
#endif
#ifdef CYW9BT_AUDIO
#include "wiced_audio_manager.h"
#endif

#ifdef MP3_DECODER_INCLUDED
#include "wiced_hal_cpu_clk.h"
#include "wiced_bt_mp3_decoder.h"
#endif // MP3_DECODER_INCLUDED

/******************************************************************************
 *                          Constants
 ******************************************************************************/
#define AV_SBC_MAX_BITPOOL          53
#define SDP_DB_LEN                  400

#define DEFAULT_SAMPLE_FREQUENCY    AUDIO_SF_48K
#define DEFAULT_CHANNEL_CONFIG      AUDIO_CHCFG_STEREO

#ifdef MP3_DECODER_INCLUDED

#define HCI_CONTROL_AUDIO_MP3_DATA_DEFAULT_LEN                  512
#define HCI_CONTROL_AUDIO_MP3_DECODER_SOURCE_DATA_BUFFER_DEGREE 10
#define HCI_CONTROL_AUDIO_MP3_DECODER_SOURCE_DATA_BUFFER_SIZE   (HCI_CONTROL_AUDIO_MP3_DATA_DEFAULT_LEN * \
                                                                 HCI_CONTROL_AUDIO_MP3_DECODER_SOURCE_DATA_BUFFER_DEGREE)
#define HCI_CONTROL_AUDIO_MP3_DATA_RX_TOLERATE_DEGREE           4   // used to tolerate the race condition of UART communication
#endif // MP3_DECODER_INCLUDED

/******************************************************************************
 *                         Variable Definitions
 ******************************************************************************/
static uint8_t supported_av_codecs[] = {A2D_MEDIA_CT_SBC, A2D_MEDIA_CT_SBC};
#define MAX_SUPPORTED_CODECS (sizeof(supported_av_codecs))

typedef struct
{
    uint8_t seid;
    wiced_bt_avdt_cfg_t peer_caps;
    wiced_bool_t caps_already_updated;
} tAV_SEP_INFO;

typedef struct
{
    uint8_t             audio_route;        /* Audio Route, I2S, UART or Sine */
    uint8_t             audio_sf;           /* Audio sampling frequency */
    uint8_t             audio_chcfg;        /* Audio Channel Config (Mono or Stereo) */

    uint8_t             reconfig_sf;       /* Audio sampling frequency requested for reconfiguration */
    uint8_t             reconfig_chcfg;    /* Audio Channel Config (Mono or Stereo) requested for reconfiguration */

    BD_ADDR             peer_bda;
    wiced_bt_sdp_discovery_db_t*  p_sdp_db;

    AV_STATE            state;              /* AVDT State machine state */
    AV_STREAM_STATE     audio_stream_state; /* Audio Streaming to host state */

    // AVDT Params
    wiced_bt_avdt_reg_t avdt_register;       /* AVDT registration */
    wiced_bt_avdt_cs_t  stream_cb;           /* Stream control block */

    wiced_bt_a2d_sbc_cie_t sbc_caps_configured;        /* SBC capabilities configured */

    uint16_t            peer_avdt_version;
    uint8_t             avdt_handle;

    wiced_bool_t        reconfigure;
    wiced_bool_t        is_accepter;
    wiced_bool_t        is_host_streaming;
    wiced_bool_t        is_start_cmd_pending;
    wiced_bool_t        is_interrupted; /* TRUE: reconfigure process is triggered in the AVDT streaming state.
                                           False: reconfigure process is triggered in the AVDT open state */

    // Peer SEP Info
    uint8_t             peer_num_seps;
    uint8_t             sep_info_idx;
    uint8_t             sep_configured_for_streaming;

    wiced_bt_avdt_sep_info_t *sep_info;      /* stream discovery results */
    wiced_bt_avdt_cfg_t  *avdt_sep_config;
    tAV_SEP_INFO        av_sep_info[sizeof(supported_av_codecs)];

#ifdef MP3_DECODER_INCLUDED
    uint8_t             audio_format;
#endif // MP3_DECODER_INCLUDED
} tAV_APP_CB;

/* A2DP module control block */
static tAV_APP_CB av_app_cb;

/* SBC codec capabilities */
static const wiced_bt_a2d_sbc_cie_t av_sbc_caps =
{
/* samp_freq    */    (A2D_SBC_IE_SAMP_FREQ_16|A2D_SBC_IE_SAMP_FREQ_32|A2D_SBC_IE_SAMP_FREQ_44|A2D_SBC_IE_SAMP_FREQ_48),
/* ch_mode      */    (A2D_SBC_IE_CH_MD_MONO | A2D_SBC_IE_CH_MD_JOINT), // A2D_SBC_IE_CH_MD_DUAL | A2D_SBC_IE_CH_MD_STEREO
/* block_len    */    (A2D_SBC_IE_BLOCKS_16),     //A2D_SBC_IE_BLOCKS_8|A2D_SBC_IE_BLOCKS_12|A2D_SBC_IE_BLOCKS_16|
/* num_subbands */    (A2D_SBC_IE_SUBBAND_8),     //A2D_SBC_IE_SUBBAND_4|A2D_SBC_IE_SUBBAND_8
/* alloc_mthd   */    (A2D_SBC_IE_ALLOC_MD_S),    //A2D_SBC_IE_ALLOC_MD_S|A2D_SBC_IE_ALLOC_MD_L
/* max_bitpool  */    (AV_SBC_MAX_BITPOOL),       //A2D_SBC_IE_MAX_BITPOOL
/* min_bitpool  */    (A2D_SBC_IE_MIN_BITPOOL)    //A2D_SBC_IE_MIN_BITPOOL
};

/* L2CAP Connection Id and peer MTU */
uint16_t lcid = 0;
uint16_t peer_mtu;

#ifdef HCI_CONTROL_AUDIO_ROUTE_SINWAVE

/* data used to create a sine wave when loaded into the controller
 * Note: It is mandatory for this array to be 4 byte aligned.*/
short sinwave[] = {
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
};
#ifndef WICEDX
short single_tone_audio_buf[256]__attribute__((aligned(4)));
#else
short single_tone_audio_buf[256];
#endif
#endif

#ifdef CYW9BT_AUDIO
static int32_t stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
#endif

#ifdef MP3_DECODER_INCLUDED
/* Data format for HCI event, HCI_CONTROL_AUDIO_EVENT_REQUEST_DATA. */
typedef struct __attribute__((packed))
{
    uint16_t    packet_audio_size; // in bytes
    uint8_t     packet_num;
    uint16_t    req_packet_count;
    uint16_t    rx_packet_count;
} hci_audio_data_req_t;
#endif

/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
static wiced_bool_t   av_app_next_getcap( wiced_bt_avdt_sep_info_t *p_sep_info,
                                          wiced_bt_avdt_cfg_t *avdt_sep_config );

static wiced_result_t av_app_send_discover_req( void );
static wiced_result_t av_app_send_start_req( void );
static wiced_result_t av_app_disconnect_connection( void );

static wiced_result_t av_app_reconfigure_req( uint8_t new_sf, uint8_t new_chcfg );
static wiced_result_t av_app_send_close_req(void);

static void av_app_idle_suspend_timeout ( uint32_t arg );
static wiced_result_t av_app_start_audio( void );
static wiced_bool_t av_app_set_audio_streaming(wiced_bool_t start_audio);
static wiced_result_t av_app_send_suspend_req(void);

static uint16_t a2dp_app_create_sep( void );
static void av_app_send_setconfiguration( uint32_t cb_params );
uint8_t BTM_SetPacketTypes (BD_ADDR remote_bda, uint16_t pkt_types);
wiced_result_t av_app_initiate_sdp( BD_ADDR bda );
#ifdef CYW9BT_AUDIO
static void av_app_am_audio_start(void);
static void av_app_am_audio_stop(void);
#endif

#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
extern wiced_result_t wiced_bt_avrc_ct_cleanup( void );
#endif

wiced_timer_t hci_control_audio_conn_idle_timer;
wiced_timer_t hci_control_audio_set_cfg_timer;

#ifdef MP3_DECODER_INCLUDED
static void     hci_control_audio_data_request_handler(uint8_t num_packets, uint32_t packet_audio_size, uint32_t req_pkt_count, uint32_t rx_pkt_count);
static void     hci_control_audio_hclk_change(wiced_bool_t enable, wiced_cpu_clk_freq_t freq);
static void     hci_control_audio_mp3_audio_info_update_handler(wiced_bt_mp3_decoder_audio_frame_into_t *p_audio_info);
static int      hci_control_audio_mp3_data_decode(void *p_data);
static void     hci_control_audio_mp3_data_request(void);
static uint8_t  hci_control_audio_mp3_utils_channel_to_a2d_format(wiced_bt_mp3_channel_t channel);
static uint8_t  hci_control_audio_mp3_utils_sampling_rate_to_a2d_format(wiced_bt_mp3_sampling_rate_t sampling_rate);
#endif // MP3_DECODER_INCLUDED

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
/*
 * hci_control_audio_init
 */
void hci_control_audio_init ( void )
{
#ifdef MP3_DECODER_INCLUDED
    wiced_bt_mp3_decoder_config_t config = {0};
#endif // MP3_DECODER_INCLUDED

    memset (&av_app_cb, 0, sizeof (av_app_cb));

    wiced_init_timer(&hci_control_audio_conn_idle_timer, av_app_idle_suspend_timeout, 0, WICED_SECONDS_TIMER);
    wiced_init_timer(&hci_control_audio_set_cfg_timer, av_app_send_setconfiguration, 0, WICED_SECONDS_TIMER);

#ifdef MP3_DECODER_INCLUDED
    /* Initialize the MP3 Decoder Library. */
    config.buf_len_mp3_data         = HCI_CONTROL_AUDIO_MP3_DECODER_SOURCE_DATA_BUFFER_SIZE;
    config.p_audio_info_update_cb   = &hci_control_audio_mp3_audio_info_update_handler;

    if (!wiced_bt_mp3_decoder_init(&config))
    {
        WICED_BT_TRACE("Err: wiced_bt_mp3_decoder_init fail\n");
    }
#endif // MP3_DECODER_INCLUDED
}

/*
 * Handles the audio commands received from UART
 */
uint8_t hci_control_audio_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t len )
{
    uint8_t  status      = HCI_CONTROL_STATUS_FAILED;
    uint32_t send_status = 1;

    switch ( cmd_opcode )
    {
    case HCI_CONTROL_AUDIO_COMMAND_CONNECT:
        status = a2dp_app_hci_control_connect( p_data, len ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AUDIO_COMMAND_DISCONNECT:
        status = a2dp_app_hci_control_disconnect( p_data, len ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AUDIO_PACKET_COUNT:
        send_status = 0;
        break;

    case HCI_CONTROL_AUDIO_START:
        // When streaming audio to a headset, we will not use 3mbps modulation to improve
        // range.  The 0xcc18 allows all packets types.  Adding 0x2204 tell controller
        // not to use 3DH1, 3DH3 and 3DH5 packets.
        BTM_SetPacketTypes(av_app_cb.peer_bda,
                HCI_PKT_TYPES_MASK_DM5 | HCI_PKT_TYPES_MASK_DH5 | /* Use 1 mbps 5 slot packets */
                HCI_PKT_TYPES_MASK_DH3 | HCI_PKT_TYPES_MASK_DM3 | /* Use 1 mbps 3 slot packets */
                HCI_PKT_TYPES_MASK_DH1 | HCI_PKT_TYPES_MASK_DM1 | /* Use 1 mbps 1 slot packets */
                HCI_PKT_TYPES_MASK_NO_3_DH1 |               /* Don't use 3 mbps 1 slot packets */
                HCI_PKT_TYPES_MASK_NO_3_DH3 |               /* Don't use 3 mbps 3 slot packets */
                HCI_PKT_TYPES_MASK_NO_3_DH5);               /* Don't use 3 mbps 5 slot packets */

        status = a2dp_app_hci_control_start( p_data, len ) == WICED_SUCCESS ?
                HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AUDIO_STOP:
        // When stopping audio to a headset, we will allow back 3mbps modulation packets
        // 2mbps and 3mbps packets are implicitly enabled (negative logic)
        BTM_SetPacketTypes(av_app_cb.peer_bda,
                HCI_PKT_TYPES_MASK_DM5 | HCI_PKT_TYPES_MASK_DH5 | /* Use 1 mbps 5 slot packets */
                HCI_PKT_TYPES_MASK_DH3 | HCI_PKT_TYPES_MASK_DM3 | /* Use 1 mbps 3 slot packets */
                HCI_PKT_TYPES_MASK_DH1 | HCI_PKT_TYPES_MASK_DM1); /* Use 1 mbps 1 slot packets */

        status = a2dp_app_hci_control_stop(p_data, len) == WICED_SUCCESS ?
                HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;
#if ( defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW20721B1) || defined(CYW20721B2) || (CYW20819A1) )
    case HCI_CONTROL_AUDIO_READ_STATISTICS:
        {
            wiced_bt_a2dp_statistics_t a2dp_stat;
            int index = 0;
            wiced_audio_get_statistics (&a2dp_stat);
            WICED_BT_TRACE (" Total Streaming time %d msec \n", a2dp_stat.duration);
            for ( index = 0;index < 10; index ++)
            {
                WICED_BT_TRACE (" Delay [%d - %d ]msec ==> Pkts : %d \n",index*10,((index+1)*10)-1,a2dp_stat.packet_ack_stats_table[index]);
            }
            WICED_BT_TRACE (" Delay > 100msec ==> Pkts : %d \n",a2dp_stat.packet_ack_stats_table[index]);

            // Return the Audio statistics via WICED-HCI
            wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_STATISTICS, (uint8_t*)&a2dp_stat, sizeof( a2dp_stat ) );
        }
        break;
#endif

#ifdef MP3_DECODER_INCLUDED
    /* The received MP3 data */
    case HCI_CONTROL_AUDIO_DATA_MP3:
        //WICED_BT_TRACE("DBG: Receive %d bytes MP3 AUDIO DATA\n", len);
        if (wiced_bt_mp3_decoder_source_data_add(p_data, len) != len)
        {
            WICED_BT_TRACE("Warning: len is different.\n");
        }
        wiced_app_event_serialize(&hci_control_audio_mp3_data_decode, NULL);
        break;

    /* Audio file format */
    case HCI_CONTROL_AUDIO_DATA_FORMAT:
        {
            /* Save the audio format. */
            av_app_cb.audio_format = p_data[0];

            if (av_app_cb.audio_format == AUDIO_SRC_AUDIO_DATA_FORMAT_MP3)
            {
                WICED_BT_TRACE("DBG: audio data is MP3 format\n");
                /* Informed by Host device that the following streaming source is formated
                 * as MP3. */
                /* Reset the MP3 Decoder module. */
                wiced_bt_mp3_decoder_reset();

                /* Register the Audio Frame request handler to insert audio frame data.
                 * The audio frame data is decoded from the MP3 format. */
                wiced_audio_samples_route_init(&hci_control_audio_data_request_handler);

                /* Ask MP3 data from host device. */
                hci_control_audio_mp3_data_request();
            }
            else
            {
                WICED_BT_TRACE("DBG: audio data is PCM format\n");

                /* De-register the audio frame request handler. */
                wiced_audio_samples_route_init(NULL);
            }
        }
        break;
#endif // MP3_DECODER_INCLUDED
    default:
        status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
        break;
    }

    if ( send_status )
        hci_control_send_command_status_evt( HCI_CONTROL_AUDIO_EVENT_COMMAND_STATUS, status );

    return status;
}

void hci_control_audio_support_features_send(void)
{
    uint8_t features = 0;

#ifdef CYW9BT_AUDIO
    features |= AUDIO_SRC_FEATURE_I2S_INPUT;
#endif

#ifdef MP3_DECODER_INCLUDED
    /* Enable MP3 Format. */
    features |= AUDIO_SRC_FEATURE_MP3_FORMAT;
#endif // MP3_DECODER_INCLUDED

    wiced_transport_send_data(HCI_CONTROL_AUDIO_EVENT_SUPPORT_FEATURES,
            &features, sizeof(features));
}

#ifdef WICED_BT_TRACE_ENABLE
const char *dump_state_name(AV_STATE state)
{
    switch((int)state)
    {
        CASE_RETURN_STR(AV_STATE_IDLE)              /* Initial state (channel is unused) */
        CASE_RETURN_STR(AV_STATE_SDP_IN_PROGRESS)   /* SDP in Progress */
        CASE_RETURN_STR(AV_STATE_SDP_DONE)          /* SDP Complete */
        CASE_RETURN_STR(AV_STATE_CONNECTING)        /* Connecting */
        CASE_RETURN_STR(AV_STATE_CONNECTED)         /* Signaling Channel is connected and active */
        CASE_RETURN_STR(AV_STATE_CONFIGURE)         /* Peer device has sent a config request */
        CASE_RETURN_STR(AV_STATE_OPEN)              /* Data channel connected but not streaming */
        CASE_RETURN_STR(AV_STATE_STARTING)          /* Attempting Data streaming */
        CASE_RETURN_STR(AV_STATE_STARTED)           /* Data streaming */
        CASE_RETURN_STR(AV_STATE_RECONFIG)          /* Reconfiguring stream */
        CASE_RETURN_STR(AV_STATE_DISCONNECTING)     /* Disconnecting */
    }

    return NULL;
}

const char *dump_stream_state_name(AV_STREAM_STATE state)
{
    switch((int)state)
    {
        CASE_RETURN_STR(AV_STREAM_STATE_STOPPED)
        CASE_RETURN_STR(AV_STREAM_STATE_STARTING)
        CASE_RETURN_STR(AV_STREAM_STATE_STARTED)
        CASE_RETURN_STR(AV_STREAM_STATE_STOPPING)
    }

    return NULL;
}

const char *dump_avdt_event_name(int event)
{
    switch(event)
    {
        CASE_RETURN_STR(AVDT_DISCOVER_CFM_EVT)       /* 0    Discover confirm */
        CASE_RETURN_STR(AVDT_GETCAP_CFM_EVT)         /* 1    Get capabilities confirm */
        CASE_RETURN_STR(AVDT_OPEN_CFM_EVT)           /* 2    Open confirm */
        CASE_RETURN_STR(AVDT_OPEN_IND_EVT)           /* 3    Open indication */
        CASE_RETURN_STR(AVDT_CONFIG_IND_EVT)         /* 4    Configuration indication */
        CASE_RETURN_STR(AVDT_START_CFM_EVT)          /* 5    Start confirm */
        CASE_RETURN_STR(AVDT_START_IND_EVT)          /* 6    Start indication */
        CASE_RETURN_STR(AVDT_SUSPEND_CFM_EVT)        /* 7    Suspend confirm */
        CASE_RETURN_STR(AVDT_SUSPEND_IND_EVT)        /* 8    Suspend indication */
        CASE_RETURN_STR(AVDT_CLOSE_CFM_EVT)          /* 9    Close confirm */
        CASE_RETURN_STR(AVDT_CLOSE_IND_EVT)          /* 10   Close indication */
        CASE_RETURN_STR(AVDT_RECONFIG_CFM_EVT)       /* 11   Reconfiguration confirm */
        CASE_RETURN_STR(AVDT_RECONFIG_IND_EVT)       /* 12   Reconfiguration indication */
        CASE_RETURN_STR(AVDT_SECURITY_CFM_EVT)       /* 13   Security confirm */
        CASE_RETURN_STR(AVDT_SECURITY_IND_EVT)       /* 14   Security indication */
        CASE_RETURN_STR(AVDT_WRITE_CFM_EVT)          /* 15   Write confirm */
        CASE_RETURN_STR(AVDT_CONNECT_IND_EVT)        /* 16   Signaling channel connected */
        CASE_RETURN_STR(AVDT_DISCONNECT_IND_EVT)     /* 17   Signaling channel disconnected */
        CASE_RETURN_STR(AVDT_REPORT_CONN_EVT)        /* 18   Reporting channel connected */
        CASE_RETURN_STR(AVDT_REPORT_DISCONN_EVT)     /* 19   Reporting channel disconnected */
        CASE_RETURN_STR(AVDT_DELAY_REPORT_EVT)       /* 20   Delay report received */
        CASE_RETURN_STR(AVDT_DELAY_REPORT_CFM_EVT)   /* 21   Delay report response received */
    }

    return NULL;
}
#endif

static wiced_bt_avdt_cfg_t *peercaps_from_seid(uint8_t seid)
{
    wiced_bt_avdt_cfg_t *peer_caps = NULL;

    int i;
    for (i=0; i<MAX_SUPPORTED_CODECS;i++)
    {
        if (av_app_cb.av_sep_info[i].seid == seid)
        {
            peer_caps = &av_app_cb.av_sep_info[i].peer_caps;
            break;
        }
    }

    return peer_caps;
}


/*
 * Capabilities format checker for SBC.
 */
static wiced_bool_t av_app_sbc_format_check( uint8_t *peer_codec_info, wiced_bt_a2d_sbc_cie_t *sbc_caps )
{
    wiced_bt_a2d_sbc_cie_t codec_info;
    wiced_bool_t           ret_val = WICED_FALSE;
    wiced_bt_a2d_status_t            a2d_status;

    /*
     * By specification all sinks must support the full set of SBC codec parameters.
     * Still it is a good idea to check the capabilities of the endpoint just to be sure.
     */

    /* copy the default SBC codec parameters */
    memcpy( sbc_caps, &av_sbc_caps, sizeof(wiced_bt_a2d_sbc_cie_t) );

    a2d_status = wiced_bt_a2d_pars_sbc_info( &codec_info, peer_codec_info, WICED_TRUE );

    WICED_BT_TRACE( "[%s]: wiced_bt_a2d_pars_sbc_info status = %d codec type: %d role: %s\n", __FUNCTION__,
                    a2d_status, peer_codec_info[2],
                    av_app_cb.is_accepter ? "ACCEPTOR" : "INITIATOR");

    /* make sure the endpoint is at least SBC */
    if ( ( a2d_status == A2D_SUCCESS ) && ( peer_codec_info[2] == A2D_MEDIA_CT_SBC ) )
    {
        if (!av_app_cb.is_accepter)
        {
            uint8_t sample_freq = (av_app_cb.reconfigure) ? av_app_cb.reconfig_sf : av_app_cb.audio_sf;

            /* Check if the sample rate we want is available */
            /* TODO: Non-match should be a failure */
            sbc_caps->samp_freq = ( sample_freq == AUDIO_SF_16K )   ? A2D_SBC_IE_SAMP_FREQ_16 :
                                  ( sample_freq == AUDIO_SF_32K )   ? A2D_SBC_IE_SAMP_FREQ_32 :
                                  ( sample_freq == AUDIO_SF_44_1K ) ? A2D_SBC_IE_SAMP_FREQ_44 : A2D_SBC_IE_SAMP_FREQ_48;
        }
        WICED_BT_TRACE( "[%s]: samp_freq: 0x%x vs 0x%x\r\n", __FUNCTION__, sbc_caps->samp_freq, codec_info.samp_freq);
        ret_val = ( sbc_caps->samp_freq & codec_info.samp_freq ) ? WICED_TRUE : WICED_FALSE;
    }

    if ( ret_val )
    {
        if (!av_app_cb.is_accepter)
        {
            uint8_t ch_config = (av_app_cb.reconfigure) ? av_app_cb.reconfig_chcfg : av_app_cb.audio_chcfg;

            /* Check if the channel config we want is available */
            sbc_caps->ch_mode = ( ch_config == AUDIO_CHCFG_MONO )   ? A2D_SBC_IE_CH_MD_MONO : A2D_SBC_IE_CH_MD_JOINT;
        }
        WICED_BT_TRACE( "[%s]: ch_mode: 0x%x vs 0x%x\r\n", __FUNCTION__, sbc_caps->ch_mode, codec_info.ch_mode);
        ret_val = ( sbc_caps->ch_mode & codec_info.ch_mode ) ? WICED_TRUE : WICED_FALSE;
    }

    if ( ret_val )
    {
        /* make sure there is an overlap in the bitpool values */
        if ( codec_info.min_bitpool > sbc_caps->min_bitpool )
        {
            sbc_caps->min_bitpool = codec_info.min_bitpool;
        }
        if ( codec_info.max_bitpool < sbc_caps->max_bitpool )
        {
            sbc_caps->max_bitpool = codec_info.max_bitpool;
        }
        /* Sanity check */
        if ( sbc_caps->min_bitpool > sbc_caps->max_bitpool )
        {
            ret_val = WICED_FALSE;
        }
    }

    WICED_BT_TRACE( "[%s]: Exit Format Match: %s\n", __FUNCTION__, ret_val ? "TRUE" : "FALSE");
    return ret_val;
}

static void av_app_send_setconfiguration( uint32_t cb_params )
{
    wiced_bt_avdt_cfg_t *peercaps = NULL;
    int i;

    /* Either something failed or the remote continued the connection */
    if  (av_app_cb.state != AV_STATE_CONNECTED)
    {
        WICED_BT_TRACE("[%s] not connected state: %d bailing.\n", __FUNCTION__, av_app_cb.state);
        return;
    }

    /* if we are setting the config we are no longer the acceptor */
    av_app_cb.is_accepter = WICED_FALSE;

    /* determine if one of the sinks found will match our format */
    for ( i = 0; i < av_app_cb.peer_num_seps; i++ )
    {
        /* find a stream that is a sink, is not in use, and is audio */
        if ( ( av_app_cb.sep_info[i].tsep == AVDT_TSEP_SNK ) &&
             ( av_app_cb.sep_info[i].in_use == WICED_FALSE) &&
             ( av_app_cb.sep_info[i].media_type == av_app_cb.stream_cb.media_type ) )
        {
            peercaps = peercaps_from_seid(av_app_cb.sep_info[i].seid);
            WICED_BT_TRACE("[%s] check sep: %d seid: %d\n", __FUNCTION__, i, av_app_cb.sep_info[i].seid);

            if (peercaps != NULL)
            {
                /* we will use the first SBC SEP that we find. */
                if ( av_app_sbc_format_check( peercaps->codec_info, &av_app_cb.sbc_caps_configured ) )
                {
                    av_app_cb.sep_configured_for_streaming = i;
                    break;
                }
            }
            else
            {
                WICED_BT_TRACE("\t\t peercaps not found for seid %d!\n", av_app_cb.sep_info[i].seid);
            }
        }
    }

    if ( peercaps && (i < av_app_cb.peer_num_seps) )
    {
        wiced_bt_avdt_cfg_t av_sbc_cfg;

        WICED_BT_TRACE( "[%s]: found SBC SEP on peer seid: %d... \n", __FUNCTION__, av_app_cb.sep_info[i].seid );

        /* Build the configuration used to open the AVDT/A2DP connection */

        /* Copy the configuration exposed on initialization */
        memcpy( &av_sbc_cfg, &av_app_cb.stream_cb.cfg, sizeof(wiced_bt_avdt_cfg_t) );

        // Based on peer and our config reset AVDT_PSC_DELAY_RPT
        if ( ( av_app_cb.peer_avdt_version < AVDT_VERSION_1_3 ) || ( AVDT_VERSION < AVDT_VERSION_1_3 ) ||
                (!(av_sbc_cfg.psc_mask & AVDT_PSC_DELAY_RPT)) || (!(peercaps->psc_mask & AVDT_PSC_DELAY_RPT)) )
        {
            av_sbc_cfg.psc_mask &= ( (~AVDT_PSC_DELAY_RPT & 0xFFFF));
        }

        /* Build the config bytes from the configuration */
        wiced_bt_a2d_bld_sbc_info( AVDT_MEDIA_AUDIO, &av_app_cb.sbc_caps_configured, av_sbc_cfg.codec_info );

        WICED_BT_TRACE_ARRAY( (uint8_t *)&av_sbc_cfg, sizeof(av_sbc_cfg.codec_info) , "built ");

        wiced_bt_avdt_open_req( av_app_cb.avdt_handle, av_app_cb.peer_bda, av_app_cb.sep_info[av_app_cb.sep_configured_for_streaming].seid, &av_sbc_cfg );
    }
    else
    {
        WICED_BT_TRACE( "[%s]: ERROR: No peer format match found", __FUNCTION__ );
        av_app_disconnect_connection();
    }
}

/*
 * Handle the AVDTP get capabilities results.  Check the codec capabilities
 * of the next stream, if any.
 */
static void av_app_getcap_cmpl( uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data )
{
    uint8_t i;
    uint8_t *peer_codec_info = p_data->getcap_cfm.p_cfg->codec_info;

    WICED_BT_TRACE( "[%s]: Enter: %s", __FUNCTION__, dump_state_name(av_app_cb.state) );

    /* check the getcap complete response status */
    if ( p_data->getcap_cfm.hdr.err_code != AVDT_SUCCESS )
    {
        wiced_bt_free_buffer(av_app_cb.avdt_sep_config);
        av_app_cb.avdt_sep_config = NULL;
        WICED_BT_TRACE( "[%s]: ERROR: getcap status = %d", __FUNCTION__, p_data->getcap_cfm.hdr.err_code );
        /* TODO: getcap failed, report error to someone? Disconnect? */
        return;
    }

    /* Determine if this is one of the codecs we are interested in */
    for (i=0; i<MAX_SUPPORTED_CODECS; i++)
    {
        if ( ( peer_codec_info[2] == supported_av_codecs[i] ) &&
             ( av_app_cb.av_sep_info[i].caps_already_updated != WICED_TRUE ) )
        {
            uint8_t seid = av_app_cb.sep_info[av_app_cb.sep_info_idx].seid;

            WICED_BT_TRACE( "[%s]: Saving SEID %d information for codec id: %d",
                            __FUNCTION__, seid, supported_av_codecs[i]);

            av_app_cb.av_sep_info[i].seid = seid;
            av_app_cb.av_sep_info[i].caps_already_updated = WICED_TRUE;

            /* Save the codec information to the control block */
            memcpy(&av_app_cb.av_sep_info[i].peer_caps,
                   p_data->getcap_cfm.p_cfg,
                   sizeof(wiced_bt_avdt_cfg_t));

            break;
        }
        else
        {
            WICED_BT_TRACE( "[%s]: Codec not saved", __FUNCTION__);
        }
    }

    /* update the index for the next getcap call if any */
    av_app_cb.sep_info_idx += 1;

    if ( av_app_cb.sep_info_idx < av_app_cb.peer_num_seps )
    {
        if ( !av_app_next_getcap( av_app_cb.sep_info, p_data->getcap_cfm.p_cfg ) )
        {
            /* getcap requests are not done yet. wait for next call */
            return;
        }
    }

    /* Free allocation made for the the getcap call */
    wiced_bt_free_buffer(p_data->getcap_cfm.p_cfg);
    av_app_cb.avdt_sep_config = NULL;
#ifdef WICED_BT_TRACE_ENABLE
    /* dump the SEP information for all indices */
    WICED_BT_TRACE( "[%s]: getcap complete info... state: %s", __FUNCTION__, dump_state_name(av_app_cb.state) );
    for ( i = 0; i < MAX_SUPPORTED_CODECS; i++ )
    {
        WICED_BT_TRACE( "    GETCAP info for SEID: %d", av_app_cb.av_sep_info[i].seid);
        WICED_BT_TRACE_ARRAY( (uint8_t *)av_app_cb.av_sep_info[i].peer_caps.codec_info, AVDT_CODEC_SIZE, "codec_info" );
    }
#endif

    /* All getcaps complete at this point. */
    if (av_app_cb.state == AV_STATE_CONNECTED)
    {
        /* Start delay timer... */
        WICED_BT_TRACE( "[%s]: Delaying setconfig 2 seconds as acceptor...\n\r", __FUNCTION__);
        wiced_start_timer(&hci_control_audio_set_cfg_timer,2);
    }
}

/*
 * Send request to get the capabilities of the next available
 * stream found in the discovery results.
 */
static wiced_bool_t av_app_next_getcap( wiced_bt_avdt_sep_info_t *p_sep_info,
                                        wiced_bt_avdt_cfg_t *avdt_sep_config )
{
    wiced_bool_t last_getcap = WICED_FALSE;
    uint8_t i;

    WICED_BT_TRACE( "\n[%s]: sep_info_idx = %d\n\r", __FUNCTION__, av_app_cb.sep_info_idx);

    /* walk the discovery results looking for the next audio sink. */
    for ( i = av_app_cb.sep_info_idx; i < av_app_cb.peer_num_seps; i++ )
    {
        /* find a stream that is a sink, and is audio */
        if ( ( p_sep_info[i].tsep == AVDT_TSEP_SNK ) &&
             ( p_sep_info[i].media_type == av_app_cb.stream_cb.media_type ) )
        {
            break;
        }
    }

    /* set current index into discovery results */
    av_app_cb.sep_info_idx = i;

    /* if the sink is available find out its capabilities */
    if ( av_app_cb.sep_info_idx < av_app_cb.peer_num_seps )
    {
        wiced_bt_avdt_getcap_req_t *p_req;
        uint16_t getcap_status;

        /* attempt GetAllCapReq only if both local and peer version are
         * greater than or equal to 1.3
         */
        if ( ( av_app_cb.peer_avdt_version >= AVDT_VERSION_1_3 ) && ( AVDT_VERSION >= AVDT_VERSION_1_3 ) )
        {
            p_req = wiced_bt_avdt_get_all_cap_req;
        }
        else
        {
            p_req = wiced_bt_avdt_get_cap_req;
        }

        /* make the call to send the getcap request to the remote */
        getcap_status = (*p_req)( av_app_cb.peer_bda,
                                  p_sep_info[av_app_cb.sep_info_idx].seid,
                                  avdt_sep_config,
                                  av_app_getcap_cmpl );

        /* if getcap call returns an error , bail out */
        if ( getcap_status != AVDT_SUCCESS )
        {
            WICED_BT_TRACE( "[%s]: ERROR: getcap call status = %d", __FUNCTION__, getcap_status );
            last_getcap = WICED_TRUE;
        }
    }
    else
    {
        last_getcap = WICED_TRUE;
    }

    return last_getcap;
}

/*
 * Handle the AVDTP discover results.  Search through the results and find the
 * first available stream, and get its capabilities.
 */
static void av_app_disc_results (uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
    wiced_bt_avdt_discover_t *p_discover = (wiced_bt_avdt_discover_t *)p_data;
    wiced_bool_t last_getcap;
    uint8_t sink_cnt = 0;
    uint8_t i;

    wiced_bt_avdt_cfg_t *avdt_sep_config;

    /* first check of the discover was successful */
    if (p_discover->hdr.err_code != AVDT_SUCCESS)
    {
        WICED_BT_TRACE( "[%s]: ERROR: discover status = %d", __FUNCTION__, p_discover->hdr.err_code);
        wiced_bt_free_buffer(av_app_cb.sep_info);
        av_app_cb.sep_info = NULL;
        av_app_disconnect_connection();
        return;
    }

#if 0

    /* TODO: Add total_seps to response to be able to determine if all SEPs are discovered. */
    WICED_BT_TRACE( "[%s]: num_seps: %d total_seps: %d", __FUNCTION__,
                    p_discover->num_seps, p_discover->total_seps);

    /*
     * check if the total number of SEPs in the return is less than the total SEPs on the peer.
     * if it is, then we need to reallocate and try again to discover all of the SEPs
     */
    if (p_discover->num_seps < p_discover->total_seps)
    {
        uint8_t max_seps = p_discover->total_seps;
        wiced_bt_avdt_sep_info_t *p_sep_info;

        wiced_bt_free_buffer(p_discover->p_sep_info);

        p_sep_info = (wiced_bt_avdt_sep_info_t *)wiced_bt_get_buffer(max_seps * sizeof(wiced_bt_avdt_sep_info_t));
        max_seps = wiced_bt_get_buffer_size(p_sep_info) / sizeof(wiced_bt_avdt_sep_info_t);
        if ( AVDT_SUCCESS != wiced_bt_avdt_discover_req( av_app_cb.peer_bda, p_sep_info, max_seps, av_app_disc_results ) )
        {
            WICED_BT_TRACE( "[%s] failed!!\n\r", __FUNCTION__ );
            wiced_bt_free_buffer(p_sep_info);
        }
        return;
    }
#endif

    /* reset the current SEP index before processing the SEP list */
    av_app_cb.sep_info_idx = 0;

    /* store number of stream endpoints returned */
    av_app_cb.peer_num_seps = p_discover->num_seps;

    /* trace all discovered SEPs */
    WICED_BT_TRACE( "[%s]: Peer Num SEPs: = %d", __FUNCTION__, p_discover->num_seps );
    for (i = 0; i < p_discover->num_seps; i++)
    {
        WICED_BT_TRACE( "[%s] sep_info seid: %d media_type: %d tsep: %d in_use: %d\n\r", __FUNCTION__,
                p_discover->p_sep_info[i].seid,
                p_discover->p_sep_info[i].media_type,
                p_discover->p_sep_info[i].tsep,
                p_discover->p_sep_info[i].in_use);

        /* find a stream that is a sink, and is audio */
        if ((p_discover->p_sep_info[i].tsep       == AVDT_TSEP_SNK) &&
            (p_discover->p_sep_info[i].media_type == av_app_cb.stream_cb.media_type))
        {
            sink_cnt++;
        }
    }


    WICED_BT_TRACE( "[%s]: sink_cnt: = %d", __FUNCTION__, sink_cnt );

    if (sink_cnt != 0)
    {
        /* Allocate space for the sink SEP capabilities. */
        avdt_sep_config = (wiced_bt_avdt_cfg_t *)wiced_bt_get_buffer(sizeof(wiced_bt_avdt_cfg_t));

        /* Free Previous allocated buffer if not freed*/
        if (av_app_cb.avdt_sep_config != NULL)
        {
            wiced_bt_free_buffer(av_app_cb.avdt_sep_config);
        }

        /* Save allocated buffer pointer */
        av_app_cb.avdt_sep_config = avdt_sep_config;

        if (avdt_sep_config != NULL)
        {
            /* clear the audio SEP */
            memset( av_app_cb.av_sep_info, 0, sizeof(av_app_cb.av_sep_info) );

            /* clear the getcap results array */
            memset(avdt_sep_config, 0, sizeof(wiced_bt_avdt_cfg_t) );

            WICED_BT_TRACE( "[%s]: calling av_app_next_getcap for first SEID: = %d", __FUNCTION__,
                            p_discover->p_sep_info->seid);

            /* perform getcaps on discovered SEPs */
            last_getcap = av_app_next_getcap(p_discover->p_sep_info, avdt_sep_config);
            if (last_getcap == WICED_TRUE)
            {
                WICED_BT_TRACE( "[%s] av_app_next_getcap failed", __FUNCTION__ );

                wiced_bt_free_buffer(avdt_sep_config);
            }
        }
    }
    else
    {
        WICED_BT_TRACE( "[%s] WARNING No Available Sink SEPs on remote", __FUNCTION__ );
        /* TODO: Need to disconnect? */
    }
}

/*
 * Handle the a connection Indication or Confirmation. Treat
 * both the same. Initiate SEP discovery.
 */
static void av_app_connect_event_hdlr(uint8_t handle, BD_ADDR bd_addr)
{
    WICED_BT_TRACE( "[%s]: Handle: 0x%04x", __FUNCTION__, handle);

    av_app_cb.state = AV_STATE_CONNECTED;
    /* Restore default connection parameters as they may have been reset by previous connection. */
    av_app_cb.audio_sf      = DEFAULT_SAMPLE_FREQUENCY;
    av_app_cb.audio_chcfg   = DEFAULT_CHANNEL_CONFIG;

    /* Updating the peer BD Address before doing the SEP discovery*/
    memcpy(av_app_cb.peer_bda, bd_addr, BD_ADDR_LEN);

    /* Clear current configuration */
    memset(&av_app_cb.sbc_caps_configured, 0, sizeof(av_app_cb.sbc_caps_configured));

    /* Perform SEP discovery */
    av_app_send_discover_req();
}

/*
 * Handle the a disconnect Indication or Confirmation.  Treat both the same.
 */
static void av_app_disconnect_event_hdlr( uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data )
{
    av_app_cb.state = AV_STATE_IDLE;

    WICED_BT_TRACE( "[%s]: handle:%04x\n\r", __FUNCTION__, av_app_cb.avdt_handle );

    /* If SDP in progress, stop and deallocate memory */
    if (av_app_cb.p_sdp_db != NULL)
    {
        wiced_bt_sdp_cancel_service_search(av_app_cb.p_sdp_db);
        wiced_bt_free_buffer(av_app_cb.p_sdp_db);
        av_app_cb.p_sdp_db = NULL;
    }

    /* Check and free all allocated buffers */
    if (av_app_cb.sep_info != NULL)
    {
        wiced_bt_free_buffer(av_app_cb.sep_info);
        av_app_cb.sep_info = NULL;
    }

    if (av_app_cb.avdt_sep_config != NULL)
    {
        wiced_bt_free_buffer(av_app_cb.avdt_sep_config);
        av_app_cb.avdt_sep_config = NULL;
    }

    /* Reset the stream configured flag */
    av_app_cb.is_host_streaming = WICED_FALSE;

    av_app_set_audio_streaming(WICED_FALSE);

    hci_control_audio_send_disconnect_complete( av_app_cb.avdt_handle, p_data->disconnect_ind.err_code, p_data->disconnect_ind.err_param );
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
    /* Disconnect AVRCP */
    wiced_bt_avrc_tg_initiate_close();
#endif

#ifdef WICED_APP_AUDIO_SNK_INCLUDED
    /* Save audio route */
    hci_control_a2dp_source_audio_route_update(bd_addr, av_app_cb.audio_route);
#endif

#ifdef WICED_APP_AUDIO_ROLE_SERVICE_SWITCH_WITH_SNK
    /* Switch back to A2DP_Sink + AVRCP Controller */
    if (hci_control_rc_target_is_connected() == WICED_FALSE)
    {
        hci_control_switch_avrcp_role(AVRCP_CONTROLLER_ROLE);
    }
    hci_control_switch_a2dp_role(A2DP_SINK_ROLE);
#endif
}

/*
 * Handle the a open Confirmation event.
 */
static void av_app_open_confirm_event_hdlr(uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
    WICED_BT_TRACE( "[%s]: <%B> handle: %d error_code: %d\n\r", __FUNCTION__,
                    bd_addr, handle, p_data->open_cfm.hdr.err_code);

    if (AVDT_SUCCESS == p_data->open_cfm.hdr.err_code)
    {
        av_app_cb.state = AV_STATE_OPEN;

        WICED_BT_TRACE( "AVDT open confirm success %d\n\r", p_data->open_cfm.lcid);
        lcid = p_data->open_cfm.lcid;
        peer_mtu = p_data->open_cfm.peer_mtu;

        hci_control_audio_send_connect_complete( bd_addr, AVDT_SUCCESS, av_app_cb.avdt_handle );

#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
        /*
         * Attempt AVRCP SDP and attempt connection
         */
        if (hci_control_rc_target_is_connected() == WICED_FALSE)
        {
            wiced_bt_avrc_tg_initiate_open(av_app_cb.peer_bda);
        }
#endif
        /* In the midst of a reconfigure restart... */
        if (av_app_cb.reconfigure == WICED_TRUE)
        {
            /* Attempt a start... */
            av_app_start_audio( );
        }

#ifdef WICED_APP_AUDIO_SNK_INCLUDED
        /* save a2dp role to device database */
        hci_control_a2dp_role_update(bd_addr, A2DP_SOURCE_ROLE);
#endif
    }
    else
    {
        av_app_cb.reconfigure = WICED_FALSE;
        av_app_cb.is_interrupted = WICED_FALSE;
    }
}

/*
 * Handle the a start Confirmation event.
 */
static void av_app_start_confirm_event_hdlr( uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data )
{
    /* AVDT_START_CFM_EVT - start audio */
    WICED_BT_TRACE( "[%s]: handle: %d, lcid %d err_code: %d", __FUNCTION__, handle, lcid, p_data->start_cfm.err_code );

    if ( AVDT_SUCCESS == p_data->start_cfm.err_code )
    {
        av_app_set_audio_streaming(WICED_TRUE);

        av_app_cb.state = AV_STATE_STARTED;

        if (av_app_cb.reconfigure == WICED_TRUE)
        {
            av_app_cb.reconfigure = WICED_FALSE;
            av_app_cb.is_interrupted = WICED_FALSE;

            av_app_cb.audio_sf    = av_app_cb.reconfig_sf;
            av_app_cb.audio_chcfg = av_app_cb.reconfig_chcfg;
        }

    }
    else
    {
        av_app_cb.state = AV_STATE_OPEN;

        av_app_set_audio_streaming(WICED_FALSE);
    }
}

/*
 * Handle the start indication event.
 */
static void av_app_start_indication_event_hdlr( uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data )
{
    /* AVDT_START_IND_EVT - start audio */
    WICED_BT_TRACE( "[%s]: handle: %d, lcid %d", __FUNCTION__, handle, lcid );

    /* Send the start response */
    if ( !wiced_bt_avdt_start_resp( handle, p_data->start_ind.label, AVDT_SUCCESS ) )
    {
        av_app_cb.state = AV_STATE_STARTED;

        /* Check the flag that indicates that the host is streaming */
        if ( av_app_cb.is_host_streaming == WICED_TRUE )
        {
            /* Inform the application that the start was successful */
            av_app_set_audio_streaming( WICED_TRUE );
        }
        else
        {
            /* if we are not streaming, start a timer to suspend the stream */
            wiced_start_timer(&hci_control_audio_conn_idle_timer, AUDIO_IDLE_SUSPEND_TIMEOUT_IN_SECONDS);
        }
    }
}


/*
 * Handle the a suspend indication from peer event.
 */
static void av_app_suspend_indication_event_hdlr( uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data )
{
    WICED_BT_TRACE( "[%s]: <%B> error_code: %d\n\r", __FUNCTION__, bd_addr, p_data->reconfig_cfm.hdr.err_code);

    if ( av_app_cb.state == AV_STATE_STARTED )
    {
        av_app_set_audio_streaming(WICED_FALSE);

        av_app_cb.state = AV_STATE_OPEN;
    }
}

/*
 * Handle the a suspend indication from peer event.
 */
static void av_app_suspend_confirm_event_hdlr( uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data )
{
    WICED_BT_TRACE( "[%s]: <%B> error_code: %d\n\r", __FUNCTION__, bd_addr, p_data->reconfig_cfm.hdr.err_code);

    if ( av_app_cb.state == AV_STATE_STARTED )
    {
        av_app_set_audio_streaming(WICED_FALSE);

        av_app_cb.state = AV_STATE_OPEN;

        if (av_app_cb.reconfigure)
        {
            av_app_reconfigure_req( av_app_cb.reconfig_sf, av_app_cb.reconfig_chcfg );
        }
    }
}

/*
 * Handle the a reconfigure Confirmation event.
 */
static void av_app_reconfig_confirm_event_hdlr(uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
    WICED_BT_TRACE( "[%s]: <%B> error_code: %d\n\r", __FUNCTION__, bd_addr, p_data->reconfig_cfm.hdr.err_code);

    if (AVDT_SUCCESS == p_data->reconfig_cfm.hdr.err_code)
    {
        av_app_cb.reconfigure = WICED_FALSE;

        av_app_cb.audio_sf    = av_app_cb.reconfig_sf;
        av_app_cb.audio_chcfg = av_app_cb.reconfig_chcfg;

        /* Reconfig was successful. restart */
        if (av_app_cb.is_interrupted)
        {
            av_app_send_start_req();

            av_app_cb.is_interrupted = WICED_FALSE;
        }
    }
    else
    {
        // Reconfig failed. Try closing and re-starting
        av_app_send_close_req();
    }
}

/*
 * Handle the stream close confirm event.
 */
static void av_app_close_confirm_event_hdlr(uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
    WICED_BT_TRACE( "[%s]: <%B> error_code: %d\n\r", __FUNCTION__, bd_addr, p_data->close_cfm.err_code);

    if (av_app_cb.reconfigure == WICED_TRUE)
    {
        av_app_cb.state = AV_STATE_CONNECTED; /* We are still connected at the signaling channel level */

        memset(&av_app_cb.sbc_caps_configured, 0, sizeof(av_app_cb.sbc_caps_configured));

        /* Assuming the peer device does not disconnect when close happens, re-perform SEP discovery */
        av_app_send_discover_req();
    }
    else
    {
        /* we sent the close request. attempt a disconnect */
        av_app_disconnect_connection();
    }
}

/*
 * Handle the stream close confirm event.
 */
static void av_app_close_indication_event_hdlr(uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
    WICED_BT_TRACE( "[%s]: <%B> error_code: %d\n\r", __FUNCTION__, bd_addr, p_data->close_cfm.err_code);

    /* we sent the close request. attempt a disconnect */
    av_app_disconnect_connection();
}

/*
 * Handle the stream configuration indication event.
 */
static void av_app_config_indication_event_hdlr(uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
    WICED_BT_TRACE( "[%s]: <%B> SEID: %d error_code: %d \n\r", __FUNCTION__, bd_addr, p_data->config_ind.int_seid, p_data->config_ind.hdr.err_code);

    if (AVDT_SUCCESS == p_data->config_ind.hdr.err_code)
    {
        wiced_bt_a2d_sbc_cie_t sbc_caps;
        wiced_bt_a2d_status_t a2d_status;
        uint16_t rsp_status;

        uint8_t err_code = AVDT_SUCCESS;
        int i;

        av_app_cb.state = AV_STATE_CONFIGURE;

        /* if we are receiving the config we are the acceptor now */
        av_app_cb.is_accepter = WICED_TRUE;

        /* Find the index of the SEID in the sep info array. It will be used for reconfigure if necessary */
        av_app_cb.sep_info_idx = 0;
        for (i=0; i<av_app_cb.peer_num_seps; i++)
        {
            if (av_app_cb.sep_info[i].seid == p_data->config_ind.int_seid)
            {
                WICED_BT_TRACE( "[%s]:  SEID: %d found at index: %d\n", __FUNCTION__,
                                p_data->config_ind.int_seid, i);

                /* TODO: If not found should be an error Possible timing issue */
                av_app_cb.sep_configured_for_streaming = i;
                break;
            }
        }

        /* Got the indication of the setconfig. Check the format. */

        a2d_status = wiced_bt_a2d_pars_sbc_info( &av_app_cb.sbc_caps_configured,
                                      p_data->config_ind.p_cfg->codec_info,
                                      WICED_FALSE ); /* Set flag to check for single bit set. */
        if (A2D_SUCCESS == a2d_status)
        {
            if ( !av_app_sbc_format_check( p_data->config_ind.p_cfg->codec_info, &sbc_caps ) )
            {
                /* Incompatible format from remote. */
                WICED_BT_TRACE( "[%s]: WARNING!!! Incompatible format. Failing request. SEID: %d \n\r", __FUNCTION__,
                                p_data->config_ind.int_seid);

                err_code = AVDT_ERR_UNSUP_CFG;
                av_app_cb.state = AV_STATE_CONNECTED;
            }
        }
        else
        {
            /* Incompatible format from remote. */
            WICED_BT_TRACE( "[%s]: WARNING!!! Incompatible format. Failing request. SEID: %d \n\r", __FUNCTION__,
                            p_data->config_ind.int_seid);

            err_code = AVDT_ERR_UNSUP_CFG;
            av_app_cb.state = AV_STATE_CONNECTED;
        }

        /* Need to respond... */
        rsp_status = wiced_bt_avdt_config_rsp(handle, p_data->config_ind.hdr.label, err_code, AVDT_ASC_CODEC);
        if (rsp_status != AVDT_SUCCESS)
        {
            /* Response is broken... */
            av_app_cb.state = AV_STATE_CONNECTED;
        }
        else
        {
            /* stop the conn idle timer on config indication. wait for the play from the remote side */
            wiced_stop_timer(&hci_control_audio_set_cfg_timer);
        }
    }
}

/*
 * Handle the stream open indication event.
 */
static void av_app_open_indication_event_hdlr(uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data)
{
    WICED_BT_TRACE( "[%s]: <%B> error_code: %d\n\r", __FUNCTION__, bd_addr, p_data->open_ind.hdr.err_code);

    /* Need to set the proper state. */
    av_app_cb.state = AV_STATE_OPEN;

    /* Once open doesn't really matter who was the initiator*/
    av_app_cb.is_accepter = WICED_FALSE;

    /* Save off the lcid for the connection. */
    lcid = p_data->open_ind.lcid;
    peer_mtu = p_data->open_ind.peer_mtu;

    hci_control_audio_send_connect_complete( bd_addr, 0, av_app_cb.avdt_handle );
}

/*
 * Event handler for stream events.
 */
static void av_app_proc_stream_evt( uint8_t handle, BD_ADDR bd_addr, uint8_t event, wiced_bt_avdt_ctrl_t *p_data )
{
    if (event != AVDT_WRITE_CFM_EVT)
    {
        WICED_BT_TRACE ("[%s]: Handle: %d <%B> Event: %d (%s) State: %d (%s)\n\r", __FUNCTION__,
            handle, bd_addr, event, dump_avdt_event_name (event), av_app_cb.state, dump_state_name (av_app_cb.state));
    }

    switch ( event )
    {
    case AVDT_DISCONNECT_IND_EVT:
        av_app_disconnect_event_hdlr( handle, bd_addr, event, p_data );
        break;

    case AVDT_CONNECT_IND_EVT:
        if (p_data->connect_ind.err_code == AVDT_SUCCESS)
        {
            /* If the connect indication comes through here, the connect is initiated from the remote */
            av_app_cb.is_accepter = WICED_TRUE;

#ifdef WICED_APP_AUDIO_SNK_INCLUDED
            /* Set audio route for reconnection device */
            av_app_cb.audio_route = hci_control_a2dp_source_audio_route_get(bd_addr);
            WICED_BT_TRACE("Audio Route:%d\n", av_app_cb.audio_route);
#endif

            /* Do SDP to get peer version info */
            if ( av_app_initiate_sdp(bd_addr) != WICED_SUCCESS)
            {
                // Peer SDP failed so assume that peer SDP version is 1.2
                av_app_cb.peer_avdt_version = AVDT_VERSION_1_2;
                av_app_connect_event_hdlr( av_app_cb.avdt_handle, av_app_cb.peer_bda);
            }
        }
        break;

    case AVDT_OPEN_CFM_EVT:
        av_app_open_confirm_event_hdlr( handle, bd_addr, event, p_data );
        break;

    case AVDT_START_CFM_EVT:
        av_app_start_confirm_event_hdlr( handle, bd_addr, event, p_data );
        break;

    case AVDT_SUSPEND_IND_EVT:
        av_app_suspend_indication_event_hdlr( handle, bd_addr, event, p_data );
        break;

    case AVDT_SUSPEND_CFM_EVT:
        av_app_suspend_confirm_event_hdlr( handle, bd_addr, event, p_data );
        break;

    case AVDT_RECONFIG_CFM_EVT:
        av_app_reconfig_confirm_event_hdlr( handle, bd_addr, event, p_data );
        break;

    case AVDT_CLOSE_IND_EVT:
        av_app_close_indication_event_hdlr( handle, bd_addr, event, p_data );
        break;

    case AVDT_CLOSE_CFM_EVT:
        av_app_close_confirm_event_hdlr( handle, bd_addr, event, p_data );
        break;

    /* Things that shouldn't happen but usually do... */
    case AVDT_CONFIG_IND_EVT:
        av_app_config_indication_event_hdlr( handle, bd_addr, event, p_data );
        break;

    case AVDT_OPEN_IND_EVT:
        av_app_open_indication_event_hdlr( handle, bd_addr, event, p_data );
        break;

    case AVDT_START_IND_EVT:
        av_app_start_indication_event_hdlr( handle, bd_addr, event, p_data );
        break;

    /* Unhandled events... */
    case AVDT_WRITE_CFM_EVT:
        break;

    case AVDT_SECURITY_IND_EVT:
        break;

    case AVDT_SECURITY_CFM_EVT:
        break;

    default:
        break;
    }
}

static void av_app_connect_request_cback(uint8_t handle, wiced_bt_device_address_t bd_addr, uint8_t event,
                                          wiced_bt_avdt_ctrl_t *p_data)
{
    WICED_BT_TRACE( "[%s] event: %d\n\r", __FUNCTION__, event );

    switch ( event )
    {
        case AVDT_CONNECT_IND_EVT:
            av_app_connect_event_hdlr( handle, bd_addr );
            break;

        case AVDT_DISCONNECT_IND_EVT:
            av_app_disconnect_event_hdlr( handle, bd_addr, event, p_data );
            break;

        default:
            WICED_BT_TRACE( "[%s] ERROR! Invalid event.\n\r", __FUNCTION__ );
            break;
    }
}

/*******************************************************************************
**
** Function         av_app_create_connection
**
** Description      Attempts to create the connection to the peer. Note that success
**                  status does not mean that the connection is made. That is
**                  determined by the callback
**
** Returns          wiced_result_t:
**                             WICED_SUCCESS = connection request sent successfully
**                             WICED_NOT_CONNECTED = connection request unsuccessful
**
*******************************************************************************/
static wiced_result_t av_app_create_connection(void)
{
    uint16_t avdt_status;

    WICED_BT_TRACE( "[%s] \n\r", __FUNCTION__ );

    av_app_cb.is_accepter = WICED_FALSE;

#ifndef WICED_APP_AUDIO_ROLE_SERVICE_SWITCH_WITH_SNK
    /* Make sure the AVRCP Profile role is correct */
    hci_control_switch_avrcp_role(AVRCP_TARGET_ROLE);
#endif

    avdt_status =
            wiced_bt_avdt_connect_req(av_app_cb.peer_bda,
                                      av_app_cb.avdt_register.sec_mask, av_app_connect_request_cback);
    if (avdt_status != AVDT_SUCCESS)
    {
        WICED_BT_TRACE( "[%s] Failed!! avdt_status = %d\n\r", __FUNCTION__, avdt_status );
        return WICED_NOT_CONNECTED;
    }

    av_app_cb.state = AV_STATE_CONNECTING;
    return WICED_SUCCESS;
}

/*******************************************************************************
**
** Function         av_app_disconnect_connection
**
** Description      Attempts to disconnect a previously created connection.
**                     Note that success status does not mean that the disconnect
**                     was successful. That is determined by the callback.
**
** Returns          wiced_result_t:
**                             WICED_SUCCESS = disconnect request sent successfully
**                             WICED_ERROR   = disconnect request unsuccessful
**
*******************************************************************************/
static wiced_result_t av_app_disconnect_connection(void)
{
    wiced_result_t status = WICED_SUCCESS;

    WICED_BT_TRACE( "[%s] \n\r", __FUNCTION__ );

    if ( AVDT_SUCCESS != wiced_bt_avdt_disconnect_req( av_app_cb.peer_bda, av_app_disconnect_event_hdlr ) )
    {
        WICED_BT_TRACE( "[%s] failed!!\n\r", __FUNCTION__ );
        status = WICED_ERROR;
    }
    else
    {
        av_app_cb.state = AV_STATE_DISCONNECTING;
        if (avrcp_profile_role == AVRCP_TARGET_ROLE)
        {
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
            wiced_bt_avrc_tg_initiate_close();
#endif
        }
        else
        {
            /* unexpected AVRCP role with A2DP source, but we still need to disconnect
             * AVRC links for a cleanup environment */
            WICED_BT_TRACE("Unexpected AVRCP role:%d\n", avrcp_profile_role);
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
            wiced_bt_avrc_ct_cleanup();
#endif
        }
    }

    return status;
}

/*
 * Sends request to discover SEPs on a peer device.
 * Note that success status does not mean that the discover
 * was successful. That is determined by the callback.
 */
static wiced_result_t av_app_send_discover_req( void )
{
    wiced_result_t status = WICED_OUT_OF_HEAP_SPACE;

    wiced_bt_avdt_sep_info_t *p_sep_info;
    uint16_t discover_size = sizeof(wiced_bt_avdt_sep_info_t) * AV_NUM_SEPS;
    uint8_t max_seps = 0;

    WICED_BT_TRACE( "[%s] state: %d\n\r", __FUNCTION__, av_app_cb.state );

    if (av_app_cb.sep_info != NULL)
    {
        wiced_bt_free_buffer(av_app_cb.sep_info);
    }
    /* Allocate space for the discovery results */
    p_sep_info = (wiced_bt_avdt_sep_info_t *)wiced_bt_get_buffer(discover_size);

    /* Store the allocated pointer */
    av_app_cb.sep_info = p_sep_info;
    if (p_sep_info != NULL)
    {
        status = WICED_SUCCESS;

        max_seps = wiced_bt_get_buffer_size(p_sep_info) / sizeof(wiced_bt_avdt_sep_info_t);
        if ( AVDT_SUCCESS != wiced_bt_avdt_discover_req( av_app_cb.peer_bda, p_sep_info, max_seps, av_app_disc_results ) )
        {
            WICED_BT_TRACE( "[%s] failed!!\n\r", __FUNCTION__ );
            wiced_bt_free_buffer(p_sep_info);
            status = WICED_ERROR;
        }
    }

    return status;
}

/*
 * Sends request to start a stream to the peer device.
 * Note that success status does not mean that the start
 * was successful. That is determined by the callback.
 */
static wiced_result_t av_app_send_start_req( void )
{
    wiced_result_t status = WICED_SUCCESS;

    WICED_BT_TRACE( "[%s] \n\r", __FUNCTION__ );

    if (av_app_cb.state == AV_STATE_OPEN)
    {
        if ( AVDT_SUCCESS != wiced_bt_avdt_start_req( &av_app_cb.avdt_handle, 1 ) )
        {
            WICED_BT_TRACE( "[%s] failed!! hdl: %d \n\r", __FUNCTION__, av_app_cb.avdt_handle );
            status = WICED_ERROR;
        }
        else
        {
            av_app_cb.state = AV_STATE_STARTING;
        }
    }

    return status;
}

/*
 * Sends request to suspend a started stream to a peer device.
 * Note that success status does not mean that the suspend
 * was successful. That is determined by the callback.
 */
static wiced_result_t av_app_send_suspend_req(void)
{
    wiced_result_t status = WICED_SUCCESS;

    uint16_t avdt_status = wiced_bt_avdt_suspend_req( &av_app_cb.avdt_handle, 1 );
    if (AVDT_SUCCESS != avdt_status)
    {
        WICED_BT_TRACE( "wiced_bt_avdt_suspend_req failed hdl: %d\n\r", av_app_cb.avdt_handle );
        status = WICED_ERROR;
    }

    WICED_BT_TRACE( "[%s] exit status: %d\n", __FUNCTION__, avdt_status );

    return status;
}

/*
 * Sends request to close an avdtp connection to a peer device.
 * Note that success status does not mean that the close
 * was successful. That is determined by the callback.
 */
static wiced_result_t av_app_send_close_req(void)
{
    wiced_result_t status = WICED_SUCCESS;
    uint16_t avdt_status;

    WICED_BT_TRACE( "[%s] handle: %d enter state: %s %d\n", __FUNCTION__,
                    av_app_cb.avdt_handle, dump_state_name(av_app_cb.state), av_app_cb.state );

    /* if we are currently streaming, stop streaming */
    if ( av_app_cb.state == AV_STATE_STARTED )
    {
        av_app_set_audio_streaming(WICED_FALSE);
    }

    avdt_status = wiced_bt_avdt_close_req( av_app_cb.avdt_handle );
    if ( AVDT_SUCCESS != avdt_status )
    {
        WICED_BT_TRACE( "wiced_bt_avdt_close_req failed hdl: %d\n\r", av_app_cb.avdt_handle );
        status = WICED_ERROR;
    }

    WICED_BT_TRACE( "[%s] exit status: %d\n", __FUNCTION__, avdt_status );

    return status;
}

/*
 * Reconfigures stream at the request of the application. Must
 * be in a suspended state to do this.
 */
static wiced_result_t av_app_reconfigure_req(uint8_t new_sf, uint8_t new_chcfg)
{
    wiced_result_t status = WICED_SUCCESS;
    uint16_t avdt_status;

    av_app_cb.sep_info_idx = av_app_cb.sep_configured_for_streaming;

    WICED_BT_TRACE( "[%s] enter state: %d seid %d  av_app_cb.sep_info_idx:%d \n", __FUNCTION__,
                    av_app_cb.state ,av_app_cb.sep_info[av_app_cb.sep_configured_for_streaming].seid, av_app_cb.sep_info_idx);

    av_app_cb.reconfigure = WICED_TRUE;

    av_app_cb.reconfig_sf    = new_sf;
    av_app_cb.reconfig_chcfg = new_chcfg;

    if (av_app_cb.state == AV_STATE_STARTED)
    {
        /*
         * Need to send a suspend to the peer before we can reconfigure.
         * Flag is set so we will come back here when suspended.
         */
        status = av_app_send_suspend_req( );

        av_app_cb.is_interrupted = WICED_TRUE;
    }
    else
    {
        wiced_bt_a2d_sbc_cie_t sbc_caps;

        wiced_bt_avdt_cfg_t *peer_caps = peercaps_from_seid(av_app_cb.sep_info[av_app_cb.sep_configured_for_streaming].seid);

        /* we will use the first SBC SEP that we find. */
        if ( av_app_sbc_format_check( peer_caps->codec_info, &sbc_caps ) )
        {
            wiced_bt_avdt_cfg_t av_sbc_cfg;

            memcpy( &av_app_cb.sbc_caps_configured, &sbc_caps, sizeof(sbc_caps));

            /* build the configuration used to reopen the AVDT/A2DP connection */
            memcpy( &av_sbc_cfg, &av_app_cb.stream_cb.cfg, sizeof(wiced_bt_avdt_cfg_t) );

            wiced_bt_a2d_bld_sbc_info( AVDT_MEDIA_AUDIO, &av_app_cb.sbc_caps_configured, av_sbc_cfg.codec_info );

            WICED_BT_TRACE_ARRAY( (uint8_t *)&av_sbc_cfg, sizeof(av_sbc_cfg.codec_info) , "built ");

            avdt_status = wiced_bt_avdt_reconfig_req( av_app_cb.avdt_handle, &av_sbc_cfg );
            if ( AVDT_SUCCESS != avdt_status )
            {
                av_app_cb.reconfigure = WICED_FALSE;

                WICED_BT_TRACE( "wiced_bt_avdt_suspend_req failed hdl: %d\n\r", av_app_cb.avdt_handle );
                status = WICED_ERROR;
            }

            av_app_cb.is_interrupted = WICED_FALSE;
        }
        else
        {
            av_app_cb.reconfigure = WICED_FALSE;

            /* could not match up the format with the current SEP capabilities */
            status = WICED_ERROR;
        }
    }

    WICED_BT_TRACE( "[%s] exit status: %d\n", __FUNCTION__, status );

    return status;
}

/*
* Sends request to suspend a started stream to a peer device when a (3 second)
* timer expires. Initiated from av_app_stop_audio.
*/
static void av_app_idle_suspend_timeout ( uint32_t arg )
{
    WICED_BT_TRACE( "[%s] \n", __FUNCTION__ );

    /* stop routing data */
    av_app_set_audio_streaming(WICED_FALSE);

    /* Send a suspend if the audio has been paused for more than the timeout. */
    av_app_send_suspend_req( );
}

#ifdef HCI_CONTROL_AUDIO_ROUTE_SINWAVE
static void av_app_configure_sinewave( short* p_data, uint32_t samples, short* p_audio_data )
{
    int i;
    WICED_BT_TRACE( "[%s] \n", __FUNCTION__ );
    for (i = 0; i < samples; i++)
    {
         p_audio_data[i*2] = p_data[i%32];
         p_audio_data[i*2 + 1] = p_data[i%32];
    }
}
#endif

#ifndef WICEDX
/*
 * Start audio routing as configured
 */
static void av_app_start_audio_stream( void )
{
    WICED_BT_TRACE( "[%s]: route:%d sample rate:%d lcid:%04x\n\r", __FUNCTION__, av_app_cb.audio_route, av_app_cb.audio_sf, lcid);

    av_app_cb.audio_stream_state = AV_STREAM_STATE_STARTED;

#ifdef HCI_CONTROL_AUDIO_ROUTE_SINWAVE
    /* if using sine wave output, start it up*/
    if ( av_app_cb.audio_route == AUDIO_ROUTE_SINE )
    {
        av_app_configure_sinewave( sinwave, sizeof( sinwave ) / 4, single_tone_audio_buf);
        wiced_audio_set_sinwave( single_tone_audio_buf );
    }
#endif

    /* Notify the app that the data is about to start */
    hci_control_audio_send_started_stopped ( av_app_cb.avdt_handle, WICED_TRUE );

    if (av_app_cb.audio_route == AUDIO_ROUTE_I2S)
    {
        /* Turn off SW interrupt timing */
        #ifndef CYW43012C0
        wiced_audio_use_sw_timing(0);
        #endif

        /* Start A2DP Source in FW */
        wiced_audio_start(WICED_TRUE, av_app_cb.audio_route, lcid, &av_app_cb.sbc_caps_configured);

#ifdef CYW9BT_AUDIO
        /* Start External Audio Codec */
        av_app_am_audio_start();
#endif
    }
    else
    {
        /* For timing use SW interrupt instead of I2S.
         * To use I2S interrupt please remove wiced_audio_use_sw_timing API call and make sure that I2S interface is configured. */
        #ifndef CYW43012C0
        wiced_audio_use_sw_timing(1);
        #endif

        /* Start request for audio samples over uart */
        wiced_audio_start(WICED_TRUE, av_app_cb.audio_route, lcid, &av_app_cb.sbc_caps_configured);
    }
}
#endif

/*
 * Request to suspend audio stream has been completed
 */
void av_app_audio_stopped_callback( void )
{
    WICED_BT_TRACE( "[%s] state: %s %d\n", __FUNCTION__, dump_state_name(av_app_cb.state), av_app_cb.state );

    av_app_cb.audio_stream_state = AV_STREAM_STATE_STOPPED;

    hci_control_audio_send_started_stopped ( av_app_cb.avdt_handle, WICED_FALSE );

    /* received a request to start during stop queue delay */
    if (av_app_cb.audio_stream_state == AV_STREAM_STATE_STARTING)
    {
        av_app_set_audio_streaming(WICED_TRUE);
    }
}

#ifndef WICEDX
static void av_app_stop_audio_stream( void )
{
    av_app_cb.audio_stream_state = AV_STREAM_STATE_STOPPING;

    wiced_audio_suspend ( lcid, &av_app_audio_stopped_callback );

#ifdef CYW9BT_AUDIO
    if (av_app_cb.audio_route == AUDIO_ROUTE_I2S)
    {
        /* Stop External Audio Codec */
        av_app_am_audio_stop();
    }
#endif
}

static int av_app_set_audio_stream_state_serialized( void *data )
{
    wiced_bool_t start_audio = *(wiced_bool_t *)data;

    WICED_BT_TRACE( "[%s] stream state: %s %d\n", __FUNCTION__, dump_stream_state_name(av_app_cb.audio_stream_state), av_app_cb.audio_stream_state );

    if (start_audio)
    {
        switch(av_app_cb.audio_stream_state)
        {
            case AV_STREAM_STATE_STOPPING:
                av_app_cb.audio_stream_state = AV_STREAM_STATE_STARTING;
                break;

            case AV_STREAM_STATE_STOPPED:
                av_app_start_audio_stream();
                break;

            default:
                break;
        }
    }
    else
    {
        switch(av_app_cb.audio_stream_state)
        {
            case AV_STREAM_STATE_STARTING:
            case AV_STREAM_STATE_STARTED:
                av_app_stop_audio_stream();
                break;

            case AV_STREAM_STATE_STOPPED:
            case AV_STREAM_STATE_STOPPING:
                // Nothing to be done as stream is already stopped or in stopping state.
                break;
        }
    }

    return 0;
}

static wiced_bool_t av_app_set_audio_streaming(wiced_bool_t start_audio)
{
    static wiced_bool_t st_start_audio;

    st_start_audio = start_audio;

    WICED_BT_TRACE( "[%s] stream state: %s %d\n", __FUNCTION__, dump_stream_state_name(av_app_cb.audio_stream_state), av_app_cb.audio_stream_state );

    wiced_app_event_serialize( av_app_set_audio_stream_state_serialized, &st_start_audio );

    return start_audio;
}

#else

static wiced_bool_t av_app_set_audio_streaming (wiced_bool_t start_audio)
{
    WICED_BT_TRACE( "[%s] stream state: %s %d\n", __FUNCTION__, dump_stream_state_name(av_app_cb.audio_stream_state), av_app_cb.audio_stream_state );

    if (start_audio)
    {
        wiced_bt_dev_cancel_sniff_mode (av_app_cb.peer_bda);
        wiced_a2dp_start_streaming (WICED_TRUE, av_app_cb.avdt_handle, &av_app_cb.sbc_caps_configured, peer_mtu);
    }
    else
        wiced_a2dp_start_streaming (WICED_FALSE, av_app_cb.avdt_handle, NULL, 0);

    hci_control_audio_send_started_stopped (av_app_cb.avdt_handle, start_audio);

    return start_audio;
}
#endif // WICEDX

/*
 * Stops streaming at the request of the application. Starts an idle timer that
 * will result in a SUSPEND being sent to the peer on timeout.
 */
static wiced_result_t av_app_stop_audio( wiced_bool_t use_timer )
{
    wiced_result_t status = WICED_SUCCESS;

    WICED_BT_TRACE( "[%s] state: %s %d\n", __FUNCTION__, dump_state_name(av_app_cb.state), av_app_cb.state );

    if ( use_timer )
    {
        /* if we are streaming, suspend the stream */
        wiced_start_timer(&hci_control_audio_conn_idle_timer, AUDIO_IDLE_SUSPEND_TIMEOUT_IN_SECONDS);
    }
    else
    {
        /* stop timer in case it was running (race condition) */
        wiced_stop_timer (&hci_control_audio_conn_idle_timer);

        /* stop routing data */
        av_app_set_audio_streaming(WICED_FALSE);

        status = av_app_send_suspend_req();
    }

    return status;
}

/*
 * Starts streaming at the request of the application. If suspended
 * it will send the request to START the stream with the peer.
 */
static wiced_result_t av_app_start_audio( void )
{
    wiced_result_t status = WICED_SUCCESS;

    WICED_BT_TRACE( "[%s] state: %s \n\r", __FUNCTION__, dump_state_name(av_app_cb.state));

    switch(av_app_cb.state)
    {
        case AV_STATE_OPEN:
            /* send play command to the audio application */
            status = av_app_send_start_req( );
            break;

        case AV_STATE_STARTED:
            /* Already started, start music  */
            av_app_set_audio_streaming(WICED_TRUE);
            break;

        default:
            break;
    }

    /* TODO: Need to check if connected for this. Send a start request if connected */
    return status;
}

/*
 * This routine is called upon the completion of a search service attribute request.
 */
void av_app_sdp_cback( uint16_t sdp_result )
{
    wiced_result_t status = WICED_ERROR;
    wiced_bt_sdp_discovery_record_t *p_rec = NULL;
    wiced_bt_sdp_protocol_elem_t     elem;

    if ( sdp_result == WICED_BT_SDP_SUCCESS )
    {
        // Search Sink record and find AVDTP version
        if ( (p_rec = wiced_bt_sdp_find_service_in_db (av_app_cb.p_sdp_db,
                      UUID_SERVCLASS_AUDIO_SINK, p_rec)) != NULL)
        {
            /* Get AVDTP version */
            if (wiced_bt_sdp_find_protocol_list_elem_in_rec (p_rec,
                          UUID_PROTOCOL_AVDTP, &elem) == WICED_TRUE)
            {
                /* Found the A2DP Sink service. attempt to connect to it */
                av_app_cb.state = AV_STATE_SDP_DONE;

                av_app_cb.peer_avdt_version = elem.params[0];
                WICED_BT_TRACE( "Service is found in the remote device %x\n\r", av_app_cb.peer_avdt_version );
            }
        }
    }

    /* Free the SDP db */
    if ( av_app_cb.p_sdp_db )
    {
        wiced_bt_free_buffer( av_app_cb.p_sdp_db );
        av_app_cb.p_sdp_db = NULL;
    }

    if (av_app_cb.state == AV_STATE_SDP_DONE)
    {
        if (av_app_cb.is_accepter == WICED_FALSE)
        {
            /* Establish L2CAP connection to the AVDTP service */
            status = av_app_create_connection( );
            if ( WICED_SUCCESS == status )
            {
                av_app_cb.state = AV_STATE_CONNECTING;
            }

            WICED_BT_TRACE( "[%s] av_app_create_connection status: %d\n\r", __FUNCTION__, status );
        }
        else
        {
            av_app_connect_event_hdlr( av_app_cb.avdt_handle, av_app_cb.peer_bda);
            return;
        }
    }

    if ( WICED_SUCCESS != status )
    {
        if (av_app_cb.is_accepter == WICED_FALSE)
        {
             av_app_cb.state = AV_STATE_IDLE;
             hci_control_audio_send_connect_complete( av_app_cb.peer_bda, status, 0 );

#ifdef WICED_APP_AUDIO_ROLE_SERVICE_SWITCH_WITH_SNK
             /* Switch back to A2DP_Sink + AVRCP Controller */
             hci_control_switch_avrcp_role(AVRCP_CONTROLLER_ROLE);
             hci_control_switch_a2dp_role(A2DP_SINK_ROLE);
#endif
        }
        else
        {
            // Peer SDP failed so assume that peer SDP version is 1.2
            av_app_cb.peer_avdt_version = AVDT_VERSION_1_2;
            av_app_connect_event_hdlr( av_app_cb.avdt_handle, av_app_cb.peer_bda);
        }
    }
}

/*
 * Send SDP search request
 */
wiced_result_t av_app_initiate_sdp( BD_ADDR bda )
{
    wiced_bt_uuid_t             uuid_list;
    uint16_t attr_list[] = { ATTR_ID_SERVICE_CLASS_ID_LIST, ATTR_ID_PROTOCOL_DESC_LIST, ATTR_ID_BT_PROFILE_DESC_LIST };
    wiced_bool_t                result = WICED_FALSE;

    memcpy( av_app_cb.peer_bda, bda, BD_ADDR_LEN );

    if ( !av_app_cb.p_sdp_db )
    {
        /* Allocate memory for the discovery database */
        av_app_cb.p_sdp_db = ( wiced_bt_sdp_discovery_db_t * ) wiced_bt_get_buffer( SDP_DB_LEN );
        if ( av_app_cb.p_sdp_db == NULL )
        {
            return WICED_ERROR;
        }
    }

    WICED_BT_TRACE( "av_app_initiate_sdp %x\n\r", (uint32_t)av_app_cb.p_sdp_db );

    uuid_list.len       = LEN_UUID_16;
    uuid_list.uu.uuid16 = UUID_SERVCLASS_AUDIO_SINK;

    /* Search the contents of the database for the A2DP Sink service */
    result = wiced_bt_sdp_init_discovery_db (av_app_cb.p_sdp_db, SDP_DB_LEN,
                                             1,&uuid_list,
                                             sizeof(attr_list)/sizeof(attr_list[0]), attr_list);

    if (result == WICED_TRUE)
    {
        if ( wiced_bt_sdp_service_search_attribute_request( av_app_cb.peer_bda, av_app_cb.p_sdp_db, av_app_sdp_cback) )
        {
            av_app_cb.state = AV_STATE_SDP_IN_PROGRESS;
            return WICED_SUCCESS;
        }
        else
        {
            WICED_BT_TRACE("[%s] wiced_bt_sdp_service_search_attribute_request fail \n", __func__);
        }
    }
    else
    {
        WICED_BT_TRACE("[%s] wiced_bt_sdp_init_discovery_db fail \n", __func__);
    }

    wiced_bt_free_buffer(av_app_cb.p_sdp_db);
    av_app_cb.p_sdp_db = NULL;
    return WICED_ERROR;
}

/*******************************************************************************
 * A2DP Application HCI Control handlers
*******************************************************************************/
/*
 * Handles the audio connect command
 */
wiced_result_t a2dp_app_hci_control_connect(uint8_t* p_data, uint32_t len)
{
    wiced_result_t status = WICED_ERROR;

    WICED_BT_TRACE( "[%s] state: %s \n\r", __FUNCTION__, dump_state_name(av_app_cb.state));

#ifdef WICED_APP_AUDIO_ROLE_SERVICE_SWITCH_WITH_SNK
    /* Switch Role */
    hci_control_switch_avrcp_role(AVRCP_TARGET_ROLE);
    hci_control_switch_a2dp_role(A2DP_SOURCE_ROLE);
#endif

    /* First make sure that there is not already a connection */
    if (av_app_cb.state == AV_STATE_IDLE)
    {
        wiced_bt_device_address_t bd_addr;

        STREAM_TO_BDADDR(bd_addr,p_data);

        /* store routing parameter */
        av_app_cb.audio_route = *p_data;

        WICED_BT_TRACE( "[%s]: <%B> Route: %d, Sample Rate: %d, Channel Config: %d, \n",
                __FUNCTION__, bd_addr, av_app_cb.audio_route, av_app_cb.audio_sf, av_app_cb.audio_chcfg );

        status = av_app_initiate_sdp(bd_addr);
    }
    else
    {
        WICED_BT_TRACE( "[%s]: Bad state: %d", __FUNCTION__, av_app_cb.state);
    }

    return status;
}

/*
 * Handles the audio disconnect
 */
wiced_result_t a2dp_app_hci_control_disconnect( uint8_t* p_data, uint32_t len )
{
    uint16_t       handle = p_data[0] + (p_data[1] << 8);
    wiced_result_t status = WICED_ERROR;

    WICED_BT_TRACE( "[%s] state: %s Handle %d\n\r", __FUNCTION__, dump_state_name(av_app_cb.state), handle );

    /* if already idle there is no connection to disconnect */
    /* TODO: Bad use of an enumerated type as an integer. Fix later */
    if ( av_app_cb.state <= AV_STATE_CONNECTED )
    {
        /* attempt to disconnect directly */
        status = av_app_disconnect_connection();
    }
    else
    {
        status = av_app_send_close_req( );
    }

    return status;
}

wiced_bool_t av_app_check_configured_settings ( wiced_bt_a2d_sbc_cie_t * p_sbc_caps, int new_sf, int new_chcfg )
{
    uint32_t samp_freq, ch_mode;

    /* TODO: Is setting a default value valid here? If the sf or chcfg do not match one of the cases it should be an error! */

    switch(new_sf)
    {
        case AUDIO_SF_16K   : samp_freq = A2D_SBC_IE_SAMP_FREQ_16; break;
        case AUDIO_SF_32K   : samp_freq = A2D_SBC_IE_SAMP_FREQ_32; break;
        case AUDIO_SF_44_1K : samp_freq = A2D_SBC_IE_SAMP_FREQ_44; break;
        case AUDIO_SF_48K   : samp_freq = A2D_SBC_IE_SAMP_FREQ_48; break;
        default:
            samp_freq = A2D_SBC_IE_SAMP_FREQ_48;
            break;
    }

    switch(new_chcfg)
    {
        case AUDIO_CHCFG_MONO:   ch_mode = A2D_SBC_IE_CH_MD_MONO; break;
        case AUDIO_CHCFG_STEREO: ch_mode = A2D_SBC_IE_CH_MD_JOINT; break;
        default:
            ch_mode = A2D_SBC_IE_CH_MD_JOINT;
            break;
    }

    WICED_BT_TRACE("\t[%s] samp freq (%d %x %x) ch mode (%d %x %x) \n", __FUNCTION__,
                   new_sf, samp_freq, p_sbc_caps->samp_freq,
                   new_chcfg, ch_mode, p_sbc_caps->ch_mode);

    return ((samp_freq == p_sbc_caps->samp_freq) && (ch_mode == p_sbc_caps->ch_mode));
}

/*
 * Handles the audio start
 */
wiced_result_t a2dp_app_hci_control_start( uint8_t* p_data, uint32_t len )
{
    uint16_t        handle    = p_data[0] + (p_data[1] << 8);
    wiced_result_t  status    = WICED_ERROR;
    uint8_t         new_sf    = p_data[2];
    uint8_t         new_chcfg = p_data[3];
#ifdef MP3_DECODER_INCLUDED
    wiced_bt_mp3_decoder_audio_frame_into_t *p_audio_frame_info = NULL;
#endif

    WICED_BT_TRACE( "[%s] state: %s Handle %d reconfigure:%d sf:%d chcfg:%d\n\r", __FUNCTION__,
                    dump_state_name(av_app_cb.state), handle, av_app_cb.reconfigure, new_sf, new_chcfg );

    /* stop timer if running (possible suspend in progress) */
    wiced_stop_timer(&hci_control_audio_conn_idle_timer);


    switch(av_app_cb.state)
    {
        case AV_STATE_OPEN:
        case AV_STATE_STARTED:
#ifdef MP3_DECODER_INCLUDED
            /* Check current streaming mode. */
            if (av_app_cb.audio_format == AUDIO_SRC_AUDIO_DATA_FORMAT_MP3)
            {
                /* Get current audio frame info. */
                p_audio_frame_info = wiced_bt_mp3_audio_frame_info_get();
                new_sf = hci_control_audio_mp3_utils_sampling_rate_to_a2d_format(p_audio_frame_info->sampling_rate);
                new_chcfg = hci_control_audio_mp3_utils_channel_to_a2d_format(p_audio_frame_info->channel);
            }
#endif

          if (av_app_check_configured_settings(&av_app_cb.sbc_caps_configured, new_sf, new_chcfg) == WICED_FALSE)
          {
              // Need to reconfigure before we start the stream.
              status = av_app_reconfigure_req(new_sf, new_chcfg);
              if (status == WICED_SUCCESS)
              {
                  /* Set flag to start streaming after AVDT RECONFIG process is finished. */
                  av_app_cb.is_interrupted = WICED_TRUE;
              }
          }
          else
          {
              // Play on...
              status = av_app_start_audio( );
          }
          break;

        default:
            WICED_BT_TRACE( "[%s] ERROR! Bad state: %s", __FUNCTION__, dump_state_name(av_app_cb.state));
            break;
    }

    /* If start successful note that the host requested the stream */
    av_app_cb.is_host_streaming = (status == WICED_SUCCESS) ? WICED_TRUE : WICED_FALSE;

    return status;
}

/*
 * Handles the audio stop
 */
wiced_result_t a2dp_app_hci_control_stop(uint8_t* p_data, uint32_t len)
{
    uint16_t handle = p_data[0] + (p_data[1] << 8);

    WICED_BT_TRACE( "[%s] state: %s Handle %d\n\r", __FUNCTION__, dump_state_name(av_app_cb.state), handle);

    /* Note that the host requested to stop streaming. */
    av_app_cb.is_host_streaming = WICED_FALSE;

    /* If in the middle of a reconfigure cycle, stop it */
    av_app_cb.reconfigure = WICED_FALSE;
    av_app_cb.is_interrupted = WICED_FALSE;

    if (av_app_cb.state == AV_STATE_STARTED)
    {
        return av_app_stop_audio( WICED_FALSE );
    }

    return WICED_ERROR;
}

static uint16_t a2dp_app_create_sep( void )
{
    uint16_t avdt_status;
    wiced_bt_a2d_sbc_cie_t sbc_caps;

    memcpy(&sbc_caps, &av_sbc_caps, sizeof(wiced_bt_a2d_sbc_cie_t));

    avdt_status = wiced_bt_a2d_bld_sbc_info( AVDT_MEDIA_AUDIO, &sbc_caps, av_app_cb.stream_cb.cfg.codec_info );
    if ( A2D_SUCCESS !=  avdt_status)
    {
        WICED_BT_TRACE( "[%s]: a2d bld sbc info failed\n. status: 0x%x", __FUNCTION__, avdt_status );
    }
    else
    {
        avdt_status =  wiced_bt_avdt_create_stream( &av_app_cb.avdt_handle, &av_app_cb.stream_cb );
        if ( AVDT_SUCCESS != avdt_status )
        {
            WICED_BT_TRACE( "[%s]: wiced_bt_avdt_create_stream failed. status: 0x%x\n", __FUNCTION__, avdt_status );
        }
    }

    return avdt_status;
}

/*
 * Register with AVDT and create stream
 */
void avdt_app_register()
{
    // Register the AVDT callback
    wiced_bt_avdt_register( &av_app_cb.avdt_register, av_app_proc_stream_evt );

    if ( A2D_SUCCESS != a2dp_app_create_sep() )
    {
        WICED_BT_TRACE( "[%s]: a2dp_app_create_sep failed\n", __FUNCTION__ );
    }
}

/*
 * Initialization of AVTD registration parameters and
 * Stream connection parameters
 *     reg.ctrl_mtu  = L2CAP MTU of the AVDTP signaling channel
 *     reg.ret_tout  = AVDTP signaling retransmission timeout (default : 4s)
 *     reg.sig_tout  = AVDTP signaling message timeout (default : 4s)
 *     reg.idle_tout = AVDTP idle signaling channel timeout (default : 10s)
 *     reg.sec_mask  = Security mask (default : None)
 *
 *     cs.cfg.num_codec   = Number of media codec information elements
 *     cs.cfg.num_protect = Number of content protection information elements
 *     cs.tsep            = AVDT_TSEP_SRC: Source SEP, AVDT_TSEP_SNK : : Sink SEP
 *     cs.nsc_mask        = Nonsupported protocol command messages
 *     cs.cfg.psc_mask    = Protocol service capabilities
 *     cs.media_type      = Media type
 *     cs.mtu             = The L2CAP MTU of the transport channel
 *     cs.flush_to        = The L2CAP flush timeout of the transport channel
 */
void avdt_init( )
{
    WICED_BT_TRACE( "[%s] \n\r", __FUNCTION__ );

    /* Initialize the AVDT specific values parameters which are set at registration. */
    av_app_cb.avdt_register.ctrl_mtu  = wiced_bt_avrc_get_ctrl_mtu();        /* AV_CTRL_MTU = 1691 */
    av_app_cb.avdt_register.ret_tout  = AV_RET_TOUT;        /* AV_RET_TOUT = 4 */
    av_app_cb.avdt_register.sig_tout  = AV_SIG_TOUT;        /* AV_SIG_TOUT = 4 */
    av_app_cb.avdt_register.idle_tout = AV_IDLE_TOUT;       /* AV_IDLE_TOUT = 10 */
    /* Security mask, AV_SEC_MASK = none */
#if BTSTACK_VER > 0x01020000
    av_app_cb.avdt_register.sec_mask  = wiced_bt_cfg_settings.security_required;
#else
    av_app_cb.avdt_register.sec_mask  = wiced_bt_cfg_settings.security_requirement_mask;
#endif

    /* Stream connection parameters */
    memset(&av_app_cb.stream_cb, 0, sizeof(wiced_bt_avdt_cs_t));

    av_app_cb.stream_cb.cfg.num_codec   = 1;
    av_app_cb.stream_cb.cfg.num_protect = 0;
    av_app_cb.stream_cb.tsep            = AVDT_TSEP_SRC;          /* AVDT_TSEP_SRC: Source SEP, AVDT_TSEP_SNK : : Sink SEP */
    av_app_cb.stream_cb.nsc_mask        = AVDT_NSC_RECONFIG;      /* Reconfigure command not supported */
#if BTSTACK_VER >= 0x01020000
    av_app_cb.stream_cb.p_avdt_ctrl_cback    = av_app_proc_stream_evt; /* AVDT event callback */
#else
    av_app_cb.stream_cb.p_ctrl_cback    = av_app_proc_stream_evt; /* AVDT event callback */
#endif
    av_app_cb.stream_cb.cfg.psc_mask    = AVDT_PSC_TRANS|AVDT_PSC_DELAY_RPT;         /* Protocol service capabilities = Media transport and Delay Report*/
    av_app_cb.stream_cb.media_type      = AVDT_MEDIA_AUDIO;       /* AVDT_MEDIA_AUDIO, AVDT_MEDIA_VIDEO, AVDT_MEDIA_MULTI */
    //av_app_cb.stream_cb.mtu             = L2CAP_DEFAULT_MTU;      /* AV_DATA_MTU; */
    //av_app_cb.stream_cb.flush_to        = L2CAP_NO_AUTOMATIC_FLUSH;

    /* Initialize state flags */
    av_app_cb.is_accepter        = WICED_FALSE;
}


/*
 * AVDTP and AVCTP register and starts inquiry, if device is already paired
 * reconnects. If reconnect fails on SDP failure then inquiry starts
 * and replaces the old device entry in NVRAM with new device paired.
 */
void av_app_start (void)
{
    WICED_BT_TRACE( "[%s] Application start\n\r", __FUNCTION__ );

#ifdef A2DP_DEBUG_OTA
    // enable simple pairing to OTA traces
    btsnd_hcic_write_simple_pairing_mode( HCI_SP_MODE_UNDEFINED );
#endif

    /* Register with AVDT */
    avdt_app_register();
}

/*
 * Initializes the A2DP application memory control block.
 * If needed, the memory is allocated.
 * NOTE: This should be called first before invoking any other A2DP API.
 */
wiced_bool_t av_app_memInit(void)
{
#if A2DP_APP_DYNAMIC_MEMORY == TRUE
    a2dp_app_cb_ptr = (tAV_APP_CB *)mpaf_mm_sbrk(sizeof(av_app_cb));

    if (!a2dp_app_cb_ptr)
    {
        return WICED_FALSE;
    }

    BT_MEMSET(&av_app_cb, 0, sizeof(av_app_cb));
#else
    /* When the *_DYNAMIC_MEMORY is not defined, the MPAF area is expected to
     * be a zero initialized BSS region and hence we don't need to
     * re-initialize this structure again.
     */
#endif

    return WICED_TRUE;
}

/*
void debug_gpio( )
{
    int     out_enable = 1;    // 0 : output, 1: input
    uint8_t gpio_num;

    WICED_BT_TRACE( "Debug GPIO\n\r" );
    for ( gpio_num = 0; gpio_num < 8; gpio_num++ )
    {
        gpio_set_direction( gpio_num, out_enable );
        gpio_set( gpio_num, 0 );
    }
}
*/
/*
 * AV application initialization
 */
void av_app_init( void )
{
    //mia_enableLhlInterrupt( 0 ); //Todo wiced check. Remove this if not needed.

    hci_control_audio_init();

    /* Application control block memory init*/
    if ( av_app_memInit( ) )
    {
        avdt_init( );
        av_app_start( ); /* start the application */
    }

    /* audio route UART by default */
    av_app_cb.audio_route = AUDIO_ROUTE_UART;

    /* Store the current default so we can compare on disconnect */
    av_app_cb.audio_sf = DEFAULT_SAMPLE_FREQUENCY;
    av_app_cb.audio_chcfg = DEFAULT_CHANNEL_CONFIG;

    av_app_cb.audio_stream_state = AV_STREAM_STATE_STOPPED;
    av_app_cb.is_host_streaming = WICED_FALSE;

    WICED_BT_TRACE ("[%s] exit\n", __FUNCTION__ );
}

/*
 * AV application deinitialization
 * This function shall be called if no connection
 */
wiced_result_t av_app_deinit(void)
{
    WICED_BT_TRACE("[%s]\n", __FUNCTION__);

    if (av_app_cb.state != AV_STATE_IDLE)
    {
        WICED_BT_TRACE("Error: a2dp_source is not in IDLE mode\n");
        return WICED_ERROR;
    }

    /* remove AVDT stream */
    wiced_bt_avdt_remove_stream(av_app_cb.avdt_handle);
    av_app_cb.avdt_handle = 0;

    /* stop timer */
    if (wiced_is_timer_in_use(&hci_control_audio_conn_idle_timer))
    {
        wiced_stop_timer(&hci_control_audio_conn_idle_timer);
    }
    if (wiced_is_timer_in_use(&hci_control_audio_set_cfg_timer))
    {
        wiced_stop_timer(&hci_control_audio_set_cfg_timer);
    }

    return WICED_SUCCESS;
}

wiced_bool_t hci_control_audio_is_connected(void)
{
    return (av_app_cb.state >= AV_STATE_CONNECTED) && (av_app_cb.state != AV_STATE_DISCONNECTING);
}

#ifdef CYW9BT_AUDIO
/*
 * av_app_am_audio_start
 * Start AudioManager for A2DP Source Stream
 */
static void av_app_am_audio_start(void)
{
    audio_config_t audio_config;
    wiced_result_t status;

    /* If Codec Stream not yet opened, opee it */
    if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        stream_id = wiced_am_stream_open(CAPTURE);
        if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
        {
            WICED_BT_TRACE("ERR: wiced_am_stream_open failed\n");
            return;
        }
    }

    switch(av_app_cb.audio_sf)
    {
        case AUDIO_SF_16K:
            audio_config.sr = 16000;
            break;

        case AUDIO_SF_32K:
            audio_config.sr = 32000;
            break;

        case AUDIO_SF_44_1K:
            audio_config.sr = 44100;
            break;

        case AUDIO_SF_48K:
            audio_config.sr = 48000;
        break;

        default:
            audio_config.sr = 44100;
        break;
    }

    audio_config.channels =  2;
    audio_config.bits_per_sample = DEFAULT_BITSPSAM;
    audio_config.volume = AM_VOL_LEVEL_HIGH - 2;
    audio_config.mic_gain = AM_VOL_LEVEL_HIGH - 2;
    audio_config.sink = AM_HEADPHONES;

    /* Configure the Codec */
    status = wiced_am_stream_set_param(stream_id, AM_AUDIO_CONFIG, &audio_config);
    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("ERR: wiced_am_stream_set_param failed\n");
        return;
    }

    /* Start the Codec */
    status = wiced_am_stream_start(stream_id);
    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("ERR: wiced_am_stream_start failed\n");
        return;
    }

    /* Set Codec Gain */
    status= wiced_am_stream_set_param(stream_id, AM_MIC_GAIN_LEVEL, &audio_config.mic_gain);
    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("ERR: wiced_am_stream_set_param failed\n");
        return;
    }
    WICED_BT_TRACE("External Codec Started\n");
}

/*
 * av_app_am_audio_stop
 * Stop AudioManager for A2DP Source Stream
 */
static void av_app_am_audio_stop(void)
{
    wiced_result_t status;

    if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        WICED_BT_TRACE("ERR: av_app_am_audio_stop Codec not opened\n");
        return;
    }

    status = wiced_am_stream_stop(stream_id);
    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("ERR: wiced_am_stream_stop failed\n");
        return;
    }

    status = wiced_am_stream_close(stream_id);
    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("ERR: wiced_am_stream_close failed\n");
        return;
    }

    stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
}
#endif

#ifdef MP3_DECODER_INCLUDED

static void hci_control_audio_data_request_handler(uint8_t num_packets, uint32_t packet_audio_size, uint32_t req_pkt_count, uint32_t rx_pkt_count)
{
    uint8_t pcm_data[WICED_BT_AUDIO_PCM_DATA_SIZE_IN_AN_AUDIO_FRAME];
    uint32_t available_pcm_data_size;
    uint8_t requested_packet_num = num_packets;

    /* Check parameter. */
    if (packet_audio_size > WICED_BT_AUDIO_PCM_DATA_SIZE_IN_AN_AUDIO_FRAME)
    {
        WICED_BT_TRACE("Err: The requested packet audio size is incorrect (%d).\n", packet_audio_size);
        return;
    }

    /* Set PCM data to litehost. */
    while (requested_packet_num)
    {
        /* Get the PCM data from the MP3 decoder module. */
        available_pcm_data_size = wiced_bt_mp3_decoder_pcm_data_get(packet_audio_size, &pcm_data[0]);

        if (packet_audio_size != available_pcm_data_size)
        {
            WICED_BT_TRACE("Err: Decoded PCM data is not enough (%d, %d)\n",
                           packet_audio_size,
                           available_pcm_data_size);

            /* Append 0's */
            memset((void *) &pcm_data[available_pcm_data_size],
                   0,
                   packet_audio_size - available_pcm_data_size);
        }

        /* Set PCM data to litehost. */
        if (!wiced_audio_samples_set(&pcm_data[0], packet_audio_size))
        {
            WICED_BT_TRACE("Err: wiced_audio_samples_set fails.\n");
        }

        requested_packet_num--;
    }

    /* Set an event to decode the MP3 data. */
    wiced_app_event_serialize(&hci_control_audio_mp3_data_decode, NULL);
}

static int hci_control_audio_mp3_data_decode(void *p_data)
{
    (void) p_data;

    //WICED_BT_TRACE("hci_control_audio_mp3_data_decode\n");

    hci_control_audio_hclk_change(WICED_TRUE, WICED_CPU_CLK_96MHZ);

    wiced_bt_mp3_decoder_pcm_samples_generate();

    hci_control_audio_hclk_change(WICED_FALSE, WICED_CPU_CLK_96MHZ);

    hci_control_audio_mp3_data_request();

    return 0;
}

/*
 * hci_control_audio_mp3_data_request
 *
 * Request MP3 data from host device.
 */
static void hci_control_audio_mp3_data_request(void)
{
    uint32_t len;
    hci_audio_data_req_t audio_req;

    /* Calculate the length of MP3 data to be requested. */
    len = wiced_bt_mp3_decoder_source_data_available_space_get();

    if (len < (HCI_CONTROL_AUDIO_MP3_DATA_DEFAULT_LEN * HCI_CONTROL_AUDIO_MP3_DATA_RX_TOLERATE_DEGREE))
    {
        return;
    }

    /* Request MP3 data from host device. */
    audio_req.packet_audio_size = HCI_CONTROL_AUDIO_MP3_DATA_DEFAULT_LEN;
    audio_req.packet_num        = 1;
    audio_req.req_packet_count  = 1;
    audio_req.rx_packet_count   = 0;

    wiced_transport_send_data(HCI_CONTROL_AUDIO_EVENT_REQUEST_DATA, (uint8_t *) &audio_req, sizeof(audio_req));
}

static void hci_control_audio_hclk_change(wiced_bool_t enable, wiced_cpu_clk_freq_t freq)
{
    wiced_transport_uart_interrupt_disable();

    /* Pause UART Rx. */
    wiced_transport_uart_rx_pause();

    /* Set HCLK. */
    wiced_update_cpu_clock(enable, freq);

    /* Resume UART Rx. */
    wiced_transport_uart_rx_resume();

    wiced_transport_uart_interrupt_enable();
}

static void hci_control_audio_mp3_audio_info_update_handler(wiced_bt_mp3_decoder_audio_frame_into_t *p_audio_info)
{
    uint8_t new_sf;
    uint8_t new_chcfg;

    WICED_BT_TRACE("hci_control_audio_mp3_audio_info_update_handler (%s) (%d, %s)\n",
                   dump_state_name(av_app_cb.state),
                   p_audio_info->sampling_rate == WICED_BT_MP3_SAMPLING_RATE_44100 ? 44100 :
                   p_audio_info->sampling_rate == WICED_BT_MP3_SAMPLING_RATE_48000 ? 48000 :
                   p_audio_info->sampling_rate == WICED_BT_MP3_SAMPLING_RATE_32000 ? 32000 : 0,
                   p_audio_info->channel == WICED_BT_MP3_CHANNEL_STEREO ? "Stereo" :
                   p_audio_info->channel == WICED_BT_MP3_CHANNEL_JOINT_STEREO ? "Joint Stereo" :
                   p_audio_info->channel == WICED_BT_MP3_CHANNEL_DUAL ? "Dual" :
                   p_audio_info->channel == WICED_BT_MP3_CHANNEL_MONO ? "Mono" : "x");

    /* Convert sampling rate to A2DP format. */
    new_sf = hci_control_audio_mp3_utils_sampling_rate_to_a2d_format(p_audio_info->sampling_rate);

    /* Convert channel configuration to A2DP format. */
    new_chcfg = hci_control_audio_mp3_utils_channel_to_a2d_format(p_audio_info->channel);

    /* Send AVDTP_RECONFIGURE command to the sink device. */
    av_app_reconfigure_req(new_sf, new_chcfg);
}

/*
 * Convert channel configuration to A2DP format
 *
 * Default is AUDIO_CHCFG_STEREO
 */
static uint8_t hci_control_audio_mp3_utils_channel_to_a2d_format(wiced_bt_mp3_channel_t channel)
{
    switch (channel)
    {
    case WICED_BT_MP3_CHANNEL_STEREO:
    case WICED_BT_MP3_CHANNEL_JOINT_STEREO:
    case WICED_BT_MP3_CHANNEL_DUAL:
        return AUDIO_CHCFG_STEREO;

    case WICED_BT_MP3_CHANNEL_MONO:
        return AUDIO_CHCFG_MONO;

    default:
        break;
    }

    return AUDIO_CHCFG_STEREO;
}

/*
 * Convert sampling rate to A2DP format
 *
 * Default is AUDIO_SF_48K
 */
static uint8_t hci_control_audio_mp3_utils_sampling_rate_to_a2d_format(wiced_bt_mp3_sampling_rate_t sampling_rate)
{
    switch (sampling_rate)
    {
    case WICED_BT_MP3_SAMPLING_RATE_44100:
        return AUDIO_SF_44_1K;

    case WICED_BT_MP3_SAMPLING_RATE_48000:
        return AUDIO_SF_48K;

    case WICED_BT_MP3_SAMPLING_RATE_32000:
        return AUDIO_SF_32K;

    default:
        break;
    }

    return AUDIO_SF_48K;
}

#endif // MP3_DECODER_INCLUDED
