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

#ifndef HCI_CONTROL_PANU_H
#define HCI_CONTROL_PANU_H

#include <stdint.h>

#ifdef WICED_APP_PANU_INCLUDED

typedef struct
{
    BD_ADDR             bd_addr;
} panu_open_t;

typedef struct
{
    BD_ADDR             bd_addr;
} panu_connect_t;

typedef union
{
    panu_open_t    open;
    panu_connect_t conn;
} panu_event_t;

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
extern void panu_hci_send_panu_event(uint16_t evt, uint16_t handle, panu_event_t *p_data);
extern void panu_connected(BD_ADDR bd_addr);
extern void panu_disconnected(uint16_t handle);
extern void panu_connect_failed(uint16_t handle);
extern void hci_control_panu_init( void );
extern void hci_control_panu_handle_command(uint16_t opcode, uint8_t* p_data, uint32_t length);
extern void hci_control_panu_set_pfilter(uint16_t handle);
extern void hci_control_panu_set_mfilter(uint16_t handle);
extern void testcase_pan_panu_ip_app_bv_05_i(uint16_t handle);

#endif

#endif // HCI_CONTROL_PANU_H
