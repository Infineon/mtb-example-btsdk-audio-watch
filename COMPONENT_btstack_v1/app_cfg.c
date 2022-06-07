/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Runtime Bluetooth stack configuration parameters
 *
 */

#include "wiced_app_cfg.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_avrc.h"
//#include "wiced_bt_hid_defs.h"

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_app.h"
#include "wiced_bt_avrc_tg.h"
#include "wiced_bt_audio.h"
#include "wiced_bt_avdt.h"
#include "cycfg_gatt_db.h"

#ifdef WICED_APP_HFP_HF_INCLUDED
#include "hci_control_hfp_hf.h"
#endif
#ifdef WICED_APP_HFP_AG_INCLUDED
#include "hci_control_hfp_ag.h"
#endif

/* If APP_AVRC_TRACK_INFO_SUPPORTED, APP_AVRC_PLAY_STATUS_SUPPORTED or APP_AVRC_SETTING_CHANGE_SUPPORTED are supported, set the AVRC profile
 version as 1.3, else set it to 1.0 */
#if (defined(APP_AVRC_TRACK_INFO_SUPPORTED) || defined(APP_AVRC_PLAY_STATUS_SUPPORTED) || defined(APP_AVRC_SETTING_CHANGE_SUPPORTED))
#define AVRC_PROFILE_VER  AVRC_REV_1_6
#else
#define AVRC_PROFILE_VER  AVRC_REV_1_0
#endif

/*
 * Definitions
 */
// SDP Record handle for AVDT Source
#define HANDLE_AVDT_SOURCE                      0x10001
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_TARGET                      0x10002
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_CONTROLLER                  0x10003
// SDP Record handle for HFP Audio Gateway
#define HANDLE_HFP_AG                           0x10004
// SDP Record handle for HFP Hands-Free Unit
#define HANDLE_HFP_HF                           0x10005

#define HANDLE_PAN_PANU                         0x10006
#define HANDLE_PAN_PANNAP                       0x10007

// SDP Record handle for PNP (Device Information)
#define HANDLE_PNP                              0x10008

#define WICED_DEVICE_NAME                       "Watch"

#define AV_SBC_MAX_BITPOOL          53

#if defined(CYW43012C0) || defined(CYW20819A1)
#define AUDIO_TX_BUFFER_SIZE        0x2000
#else
#define AUDIO_TX_BUFFER_SIZE        11000
#endif

/* It needs 15K bytes for HFP(mSBC use mainly) */
#if defined(WICED_APP_HFP_HF_INCLUDED)
#define AUDIO_CODEC_BUFFER_SIZE     (16 * 1024) //0x4000
#elif defined(WICED_APP_HFP_AG_INCLUDED)
#define AUDIO_BUF_SIZE_MAIN_HFP     (15 * 1024) //0x3C00
#define AUDIO_CODEC_BUFFER_SIZE     AUDIO_BUF_SIZE_MAIN_HFP
#else
#define AUDIO_CODEC_BUFFER_SIZE     0x2000
#endif

#if defined(CYW20819A1) || defined(CYW43012C0)
#define WICED_MAX_LE_CLIENT_CONN    1
#else
#define WICED_MAX_LE_CLIENT_CONN    3
#endif


const uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name                         = (uint8_t*)app_gap_device_name,                               /**< Local device name (NULL terminated). Use same as configurator generated string.*/
    .device_class                        = {0x20, 0x07, 0x04},                                         /**< Local device class */
    .security_requirement_mask           = (  BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT ), /**< Security requirements mask (BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT (see #wiced_bt_sec_level_e)) */

    .max_simultaneous_links              = 3,                                                          /**< Maximum number simultaneous links to different devices */

    .br_edr_scan_cfg =                                              /* BR/EDR scan config */
    {
        .inquiry_scan_type               = BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .inquiry_scan_interval           = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  (0 to use default) */
        .inquiry_scan_window             = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window (0 to use default) */

        .page_scan_type                  = BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .page_scan_interval              = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  (0 to use default) */
        .page_scan_window                = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window (0 to use default) */
    },

    .ble_scan_cfg =                                                 /* BLE scan settings  */
    {
        .scan_mode                       = BTM_BLE_SCAN_MODE_ACTIVE,                                   /**< BLE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

        /* Advertisement scan configuration */
        .high_duty_scan_interval         = 96,                                                         /**< High duty scan interval */
        .high_duty_scan_window           = 48,                                                         /**< High duty scan window */
        .high_duty_scan_duration         = 30,                                                         /**< High duty scan duration in seconds (0 for infinite) */

        .low_duty_scan_interval          = 2048,                                                       /**< Low duty scan interval  */
        .low_duty_scan_window            = 48,                                                         /**< Low duty scan window */
        .low_duty_scan_duration          = 60,                                                         /**< Low duty scan duration in seconds (0 for infinite) */

        /* Connection scan configuration */
        .high_duty_conn_scan_interval    = 96,                                                         /**< High duty cycle connection scan interval */
        .high_duty_conn_scan_window      = 48,                                                         /**< High duty cycle connection scan window */
        .high_duty_conn_duration         = 30,                                                         /**< High duty cycle connection duration in seconds (0 for infinite) */

        .low_duty_conn_scan_interval     = 2048,                                                       /**< Low duty cycle connection scan interval */
        .low_duty_conn_scan_window       = 48,                                                         /**< Low duty cycle connection scan window */
        .low_duty_conn_duration          = 30,                                                         /**< Low duty cycle connection duration in seconds (0 for infinite) */

        /* Connection configuration */
        .conn_min_interval               = 112,                                                        /**< Minimum connection interval */
        .conn_max_interval               = 128,                                                        /**< Maximum connection interval */
        .conn_latency                    = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection latency */
        .conn_supervision_timeout        = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection link supervision timeout */
    },

    .ble_advert_cfg =                                               /* BLE advertisement settings */
    {
        .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */
                                           BTM_BLE_ADVERT_CHNL_38 |
                                           BTM_BLE_ADVERT_CHNL_39,

        .high_duty_min_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,            /**< High duty undirected connectable minimum advertising interval */
        .high_duty_max_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,            /**< High duty undirected connectable maximum advertising interval */
        .high_duty_duration              = 30,                                                         /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */

        .low_duty_min_interval           = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,             /**< Low duty undirected connectable minimum advertising interval */
        .low_duty_max_interval           = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MAX_INTERVAL,             /**< Low duty undirected connectable maximum advertising interval */
        .low_duty_duration               = 60,                                                         /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */

        .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High duty directed connectable minimum advertising interval */
        .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High duty directed connectable maximum advertising interval */

        .low_duty_directed_min_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low duty directed connectable minimum advertising interval */
        .low_duty_directed_max_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low duty directed connectable maximum advertising interval */
        .low_duty_directed_duration      = 30,                                                         /**< Low duty directed connectable advertising duration in seconds (0 for infinite) */

        .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        .high_duty_nonconn_duration      = 30,                                                         /**< High duty non-connectable advertising duration in seconds (0 for infinite) */

        .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        .low_duty_nonconn_duration       = 0                                                           /**< Low duty non-connectable advertising duration in seconds (0 for infinite) */
    },

    .gatt_cfg =                                                     /* GATT configuration */
    {
        .appearance                     = APPEARANCE_GENERIC_TAG,                                      /**< GATT appearance (see gatt_appearance_e) */
        .client_max_links               = WICED_MAX_LE_CLIENT_CONN,                                    /**< Client config: maximum number of servers that local client can connect to  */
        .server_max_links               = 1,                                                           /**< Server config: maximum number of remote clients connections allowed by the local */
        .max_attr_len                   = 360,                                                         /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
#if !defined(CYW20706A2)
        .max_mtu_size                   = 365,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
#endif
    },

    .rfcomm_cfg =                                                   /* RFCOMM configuration */
    {
        .max_links                      = WICED_BT_RFCOMM_MAX_CONN,                                    /**< Maximum number of simultaneous connected remote devices*/
        .max_ports                      = WICED_BT_RFCOMM_MAX_CONN                                     /**< Maximum number of simultaneous RFCOMM ports */
    },

    .l2cap_application =                                            /* Application managed l2cap protocol configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        .max_psm                        = 0,                                                           /**< Maximum number of application-managed BR/EDR PSMs */
        .max_channels                   = 0,                                                           /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        .max_le_psm                     = 0,                                                           /**< Maximum number of application-managed LE PSMs */
        .max_le_channels                = 0,                                                           /**< Maximum number of application-managed LE channels */
#if !defined(CYW20706A2)
        /* LE L2cap fixed channel configuration */
        .max_le_l2cap_fixed_channels    = 1,                                                            /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */
#endif
    },

    .avdt_cfg =
    /* Audio/Video Distribution configuration */
    {
        .max_links                      = 1,                                                           /**< Maximum simultaneous audio/video links */
#if !defined(CYW20706A2)
        .max_seps                       = 3                                                            /**< Maximum number of stream end points */
#endif
    },

    .avrc_cfg =                                                     /* Audio/Video Remote Control configuration */
    {
        .roles                          = 1,                                                           /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
        .max_links                      = 1                                                            /**< Maximum simultaneous remote control links */
    },

    /* LE Address Resolution DB size  */
    .addr_resolution_db_size            = 5,                                                           /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/

#ifdef CYW20706A2
    .max_mtu_size                       = 365,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    .max_pwr_db_val                     = 12                                                           /**< Max. power level of the device */
#else
    /* Maximum number of buffer pools */
    .max_number_of_buffer_pools         = 8,                                                           /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */

    /* Interval of  random address refreshing */
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,            /**< Interval of  random address refreshing - secs */

    /* BLE Filter Accept List size */
    .ble_filter_accept_list_size                = 2,                                                           /**< Maximum number of Filter Accept List devices allowed. Cannot be more than 128 */
#endif

#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1) || defined (CYW20820A1)
    .default_ble_power_level            = 12,                                                           /**< Default LE power level, Refer lm_TxPwrTable table for the power range */
#endif
};

#ifdef WICED_APP_AUDIO_SRC_INCLUDED
#define SLEN_A2DP_SRC   (56 + 2)
#else
#define SLEN_A2DP_SRC   0
#endif
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
#define SLEN_AVRC_TG    (56 + 2)
#else
#define SLEN_AVRC_TG    0
#endif
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
#define SLEN_AVRC_CT    (59 + 2)
#else
#define SLEN_AVRC_CT    0
#endif
#ifdef WICED_APP_HFP_AG_INCLUDED
#define SLEN_HFP_AG     (60 + 2)
#else
#define SLEN_HFP_AG     0
#endif
#ifdef WICED_APP_HFP_HF_INCLUDED
#define SLEN_HFP_HF     (75 + 2)
#else
#define SLEN_HFP_HF     0
#endif
#ifdef WICED_APP_PANU_INCLUDED
#define SLEN_PAN_PANU   (108 + 2)
#else
#define SLEN_PAN_PANU   0
#endif
#ifdef WICED_APP_PANNAP_INCLUDED
#define SLEN_PAN_PANNAP (126 + 2)
#else
#define SLEN_PAN_PANNAP   0
#endif
#define SLEN_PNP        (69 + 2)

#define SLEN            (SLEN_A2DP_SRC + SLEN_AVRC_TG + SLEN_AVRC_CT + SLEN_HFP_AG + SLEN_HFP_HF + SLEN_PAN_PANU + SLEN_PAN_PANNAP + SLEN_PNP)
/*
 * This is the SDP database for the whole hci_control application
 */
const uint8_t wiced_app_cfg_sdp_record[] =
{
    // length is the sum of all records
    SDP_ATTR_SEQUENCE_2(SLEN),

#ifdef WICED_APP_AUDIO_SRC_INCLUDED
    // SDP Record for AVDT Source
    SDP_ATTR_SEQUENCE_1(56),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVDT_SOURCE),                         // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SOURCE),                     // 8 bytes
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),   // 3 + 2 bytes
            SDP_ATTR_SEQUENCE_1(6),                                         // 2 bytes
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),                       // 3 bytes
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),                         // 3 bytes
            SDP_ATTR_SEQUENCE_1(6),                                         // 2 bytes
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),                       // 3 bytes
                SDP_ATTR_VALUE_UINT2(0x103),                                // 3 bytes
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),  // 3 + 2 bytes
            SDP_ATTR_SEQUENCE_1(6),                                         // 2 bytes
                SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION),     // 3 bytes
                SDP_ATTR_VALUE_UINT2(0x103),                                // 3 bytes
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x0001),
#endif

#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
    // SDP Record for AVRC Target
    SDP_ATTR_SEQUENCE_1(56),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_TARGET),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_TARGET),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x100),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_PROFILE_VER),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_TG_CAT1),
#endif

#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
    // SDP Record for AVRC Controller
    SDP_ATTR_SEQUENCE_1(59),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_CONTROLLER),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_CONTROL),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_6),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_CT_CAT2),
#endif

#ifdef WICED_APP_HFP_AG_INCLUDED
    // SDP record for HFP AG ( total length of record: 62 )
    SDP_ATTR_SEQUENCE_1( 60 ),                                              // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE( HANDLE_HFP_AG ),                            // 8 byte ( handle=0x10004 )
        SDP_ATTR_ID( ATTR_ID_SERVICE_CLASS_ID_LIST ),                       // 3 bytes
            SDP_ATTR_SEQUENCE_1( 6 ),                                       // 2 bytes
                SDP_ATTR_UUID16( UUID_SERVCLASS_AG_HANDSFREE ),             // 3 bytes ServiceClass0 UUID_SERVCLASS_AG_HANDSFREE
                SDP_ATTR_UUID16( UUID_SERVCLASS_GENERIC_AUDIO ),            // 3 bytes ServiceClass1 UUID_SERVCLASS_GENERIC_AUDIO
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST( 1 ),                            // 17 bytes ( SCN=1 )
        SDP_ATTR_PROFILE_DESC_LIST( UUID_SERVCLASS_AG_HANDSFREE, 0x0108 ),  // 13 bytes UUID_SERVCLASS_HF_HANDSFREE, version 0x0105
		SDP_ATTR_UINT1(ATTR_ID_NETWORK, 0x00),                                  // 5 byte
		SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES,  AG_SUPPORTED_FEATURES_ATT), //6 bytes
#endif

#ifdef WICED_APP_HFP_HF_INCLUDED
    // SDP Record for Hands-Free Unit
    SDP_ATTR_SEQUENCE_1(75),
        SDP_ATTR_RECORD_HANDLE( HANDLE_HFP_HF ),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                SDP_ATTR_UUID16(UUID_SERVCLASS_GENERIC_AUDIO),
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST( HANDS_FREE_SCN ),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                SDP_ATTR_VALUE_UINT2(0x0108),
        SDP_ATTR_SERVICE_NAME(15),
            'W', 'I', 'C', 'E', 'D', ' ', 'H', 'F', ' ', 'D', 'E', 'V', 'I', 'C', 'E',
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, SUPPORTED_FEATURES_ATT),
#endif

#ifdef WICED_APP_PANU_INCLUDED
        SDP_ATTR_SEQUENCE_1(108),                                               // 2 byte
            SDP_ATTR_RECORD_HANDLE(HANDLE_PAN_PANU),                            // 8 byte
            SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),                         // 3 byte
                SDP_ATTR_SEQUENCE_1(3),                                         // 2 byte
                    SDP_ATTR_UUID16(UUID_SERVCLASS_PANU),                       // 3 byte
            SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(24),   // 3 + 2 byte
                SDP_ATTR_SEQUENCE_1(6),                                         // 2 byte
                    SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),                       // 3 byte
                    SDP_ATTR_VALUE_UINT2(BT_PSM_BNEP),                          // 3 byte
                SDP_ATTR_SEQUENCE_1(14),                                        // 2 byte
                    SDP_ATTR_UUID16(UUID_PROTOCOL_BNEP),                        // 3 byte
                    SDP_ATTR_VALUE_UINT2(0x0100),                               // 3 byte
                    SDP_ATTR_SEQUENCE_1(6),                                     // 2 byte
                        SDP_ATTR_VALUE_UINT2(0x0800),                           // 3 byte
                        SDP_ATTR_VALUE_UINT2(0x0806),                           // 3 byte
            SDP_ATTR_ID(ATTR_ID_LANGUAGE_BASE_ATTR_ID_LIST), SDP_ATTR_SEQUENCE_1(9),      // 3 + 2
                SDP_ATTR_VALUE_UINT2(LANG_ID_CODE_ENGLISH),                     // 3 byte
                SDP_ATTR_VALUE_UINT2(LANG_ID_CHAR_ENCODE_UTF8),                 // 3 byte
                SDP_ATTR_VALUE_UINT2(LANGUAGE_BASE_ID),                         // 3 byte
            SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),  // 3 + 2 byte
                SDP_ATTR_SEQUENCE_1(6),                                         // 2 byte
                    SDP_ATTR_UUID16(UUID_SERVCLASS_PANU),                       // 3 byte
                    SDP_ATTR_VALUE_UINT2(0x0100),                               // 3 byte
            SDP_ATTR_SERVICE_NAME(10),                                          // 5 byte
                'W', 'I', 'C', 'E', 'D', ' ', 'P', 'A', 'N', 'U',               // 10 byte
            SDP_ATTR_SERVICE_DESCRIPTION(10),                                   // 5 byte
                'W', 'I', 'C', 'E', 'D', ' ', 'P', 'A', 'N', 'U',               // 10 byte
            SDP_ATTR_UINT2(ATTR_ID_SECURITY_DESCRIPTION, 0x0001),               // 6 byte
#endif
#ifdef WICED_APP_PANNAP_INCLUDED
            SDP_ATTR_SEQUENCE_1(126),                                               // 2 byte
                SDP_ATTR_RECORD_HANDLE(HANDLE_PAN_PANNAP),                          // 8 byte
                SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),                         // 3 byte
                    SDP_ATTR_SEQUENCE_1(3),                                         // 2 byte
                        SDP_ATTR_UUID16(UUID_SERVCLASS_NAP),                        // 3 byte
                SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(24),   // 3 + 2 byte
                    SDP_ATTR_SEQUENCE_1(6),                                         // 2 byte
                        SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),                       // 3 byte
                        SDP_ATTR_VALUE_UINT2(BT_PSM_BNEP),                          // 3 byte
                    SDP_ATTR_SEQUENCE_1(14),                                        // 2 byte
                        SDP_ATTR_UUID16(UUID_PROTOCOL_BNEP),                        // 3 byte
                        SDP_ATTR_VALUE_UINT2(0x0100),                               // 3 byte
                        SDP_ATTR_SEQUENCE_1(6),                                     // 2 byte
                            SDP_ATTR_VALUE_UINT2(0x0800),                           // 3 byte
                            SDP_ATTR_VALUE_UINT2(0x0806),                           // 3 byte
                SDP_ATTR_ID(ATTR_ID_LANGUAGE_BASE_ATTR_ID_LIST), SDP_ATTR_SEQUENCE_1(9),      // 3 + 2
                    SDP_ATTR_VALUE_UINT2(LANG_ID_CODE_ENGLISH),                     // 3 byte
                    SDP_ATTR_VALUE_UINT2(LANG_ID_CHAR_ENCODE_UTF8),                 // 3 byte
                    SDP_ATTR_VALUE_UINT2(LANGUAGE_BASE_ID),                         // 3 byte
                SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),  // 3 + 2 byte
                    SDP_ATTR_SEQUENCE_1(6),                                         // 2 byte
                        SDP_ATTR_UUID16(UUID_SERVCLASS_NAP),                        // 3 byte
                        SDP_ATTR_VALUE_UINT2(0x0100),                               // 3 byte
                SDP_ATTR_SERVICE_NAME(12),                                          // 5 byte
                    'W', 'I', 'C', 'E', 'D', ' ', 'P', 'A', 'N', 'N', 'A', 'P',     // 12 byte
                SDP_ATTR_SERVICE_DESCRIPTION(12),                                   // 5 byte
                    'W', 'I', 'C', 'E', 'D', ' ', 'P', 'A', 'N', 'N', 'A', 'P',     // 12 byte
                SDP_ATTR_UINT2(ATTR_ID_SECURITY_DESCRIPTION, 0x0001),               // 6 byte
                SDP_ATTR_UINT2(ATTR_ID_NET_ACCESS_TYPE, 0x0005),                    // 6 byte
                SDP_ATTR_UINT4(ATTR_ID_MAX_NET_ACCESS_RATE, 0x001312D0),            // 8 byte
#endif
    // SDP record Device ID (total = 69 + 2 = 71)
    SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE(HANDLE_PNP),                                 // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),                  // 8
        SDP_ATTR_PROTOCOL_DESC_LIST(1),                                     // 18
        SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),                    // 6
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x0f),                            // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x0201),                         // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),                    // 6
        SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),                     // 5
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG),// 6
};

/*****************************************************************************
 * wiced_bt  buffer pool configuration
 *
 * Configure buffer pools used by the stack  according to application's requirement
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count } */
    { 64,       16   },      /* Small Buffer Pool */
    { 360,      4   },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
#if defined(CYW20819A1)
    { 896,      9   },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,     5   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
#elif defined(WICED_APP_PANU_INCLUDED) || defined(WICED_APP_PANNAP_INCLUDED)
    { 1707,     9   },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,     5   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
#else
    { 1024,     9   },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,     5   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
#endif
};

/**  Audio buffer configuration configuration */
const wiced_bt_audio_config_buffer_t wiced_bt_audio_buf_config = {
    .role                             =   WICED_AUDIO_SOURCE_ROLE,
    .audio_tx_buffer_size             =   AUDIO_TX_BUFFER_SIZE,

    .audio_codec_buffer_size          =   AUDIO_CODEC_BUFFER_SIZE
#if defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1)
    ,.audio_tx_buffer_watermark_level =   50
#endif
};

/*
 * wiced_app_cfg_get_settings
 */
const wiced_bt_cfg_settings_t *wiced_app_cfg_get_settings(void)
{
    return &wiced_bt_cfg_settings;
}

/*
 * wiced_app_cfg_buf_pools_get_num
 */
int wiced_app_cfg_buf_pools_get_num(void)
{
    return (int)sizeof(wiced_app_cfg_buf_pools)/sizeof(wiced_app_cfg_buf_pools[0]);
}

/*
 * wiced_app_cfg_buf_pools_get
 */
const wiced_bt_cfg_buf_pool_t *wiced_app_cfg_buf_pools_get(void)
{
    return wiced_app_cfg_buf_pools;
}

/*
 * wiced_app_cfg_sdp_record_get
 */
uint8_t *wiced_app_cfg_sdp_record_get(void)
{
    return (uint8_t *)wiced_app_cfg_sdp_record;
}

/*
 * wiced_app_cfg_sdp_record_get_size
 */
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return (uint16_t)sizeof(wiced_app_cfg_sdp_record);
}
