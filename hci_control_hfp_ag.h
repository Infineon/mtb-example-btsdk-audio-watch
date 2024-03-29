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
 * Hands-free profile Audio Gateway support for watch application
 */

#ifndef __HCI_CONTROL_HFP_AG_H_
#define __HCI_CONTROL_HFP_AG_H_

#ifdef WICED_APP_HFP_AG_INCLUDED
#include "wiced_bt_hfp_ag.h"
#endif

#define HCI_CONTROL_AG_NUM_SCB          2           /* Max simultaneous connections to HFs */

/* AG Supported Feature */
#if (BTM_WBS_INCLUDED == TRUE )
#define AG_SUPPORTED_FEATURES_ATT          ( WICED_BT_HFP_AG_SDP_FEATURE_VRECG | \
                                             WICED_BT_HFP_AG_SDP_FEATURE_WIDEBAND_SPEECH )
#else
#define AG_SUPPORTED_FEATURES_ATT          ( WICED_BT_HFP_AG_SDP_FEATURE_VRECG )

#endif

/*
 * Audio Gateway init
 */
void hci_control_ag_init( void );

/*
 * Handle Handsfree commands received over UART.
 */
void hci_control_ag_handle_command( uint16_t opcode, uint8_t* p_data, uint32_t length );

#endif /* __HCI_CONTROL_HFP_AG_H_ */
