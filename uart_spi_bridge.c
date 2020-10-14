/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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
 * UART-SPI bridge application
 *
 * This application implements the SPI Master and act as UART-SPI Bridge.
 * The uart_spi_bridge sample application demonstrates the signaling protocol
 * to be implemented on the customer MCU to communicate over SPI using the
 * WICED_HCI messages.
 * The bridge application implements the following functionalities
 *   1. Receives WICED_HCI Commands from MCU/Host over PUART and
 *      forwards the commands over SPI to the Slave device.
 *   2. Receives WICED_HCI Events from the connected WICED SPI device
 *      and forwards them over PUART to the connected host.
 *   3. Intercepts the HCI_CONTROL_AUDIO_DATA request from the WICED
 *      SPI device and responds with a stored sine wave to the WICED device.
 *
 * * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED Bluetooth ( 20706 ) evaluation board into your computer
 * 2. Build and download the uart_spi_bridge application to a 20706
 *    board(SPI Master).
 * 3. Build and download the hci_av_source application to another 20706
 *    board(SPI Slave).
 * 4. Connect the SPI signals between master and slave.
 *    -------------------------------------------------
 *    |             Master        |        Slave       |
 *    -------------------------------------------------
 *    | SPI CLK    P36(J19.6)     |     P3(J22.8)      |
 *    -------------------------------------------------
 *    | SPI MOSI   P0(J22.6)      |     P0(J22.6)      |
 *    -------------------------------------------------
 *    | SPI MISO   P25(J19.4)     |     P25(J19.4)     |
 *    -------------------------------------------------
 *    | SPI CS     P26(J22.4)     |     P2(J22.5)      |
 *    -------------------------------------------------
 *    | SLAVE READY P12(J22.7)    |     P15(J22.3)     |
 *    -------------------------------------------------
 *    | GND         J19.8         |     J19.8          |
 *    -------------------------------------------------
 * 5. Open the Client Control application. Select  the PUART port on the 20706
 *    running the uart_spi_bridge application at 115200 baud rate
 *    without flow control
 * 6. Reset the 20706 that runs the uart_spi_bridge application.
 * 7. Reset the 20706 that runs the watch application.
 * 6. Use ClientControl application to send various commands
 */
#include "sparcommon.h"
#include "wiced_bt_cfg.h"
#include "wiced_platform.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_trace.h"
#include "hci_control_api.h"
#include "wiced_bt_stack.h"
#include "bridge_spi.h"


/******************************************************
 *               Variable Definitions
 ******************************************************/

extern BOOL8   gpio_initialized;
extern uint32_t SPI_CLOCK_SPEED;

extern int hci_bridge_get_num_buf_pools();
extern const wiced_bt_cfg_settings_t hci_bridge_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t hci_bridge_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];

/******************************************************
 *               Function Declarations
 ******************************************************/

static void     hci_bridge_send_device_started_evt( void );
static wiced_result_t hci_bridge_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void     hci_bridge_spi_master_data_received( uint8_t* p_data );
static void     toggle_gpio ( int count );
uint8_t         hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );

extern uint8_t spi_master_handle_tx_and_rx( uint32_t event );
extern void hci_puart_init( void );

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 *  Application Start, ie, entry point to the application.
 */
APPLICATION_START( )
{
    wiced_bt_stack_init( hci_bridge_management_callback ,  &hci_bridge_cfg_settings, hci_bridge_cfg_buf_pools);
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t hci_bridge_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result = WICED_BT_SUCCESS;
    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            gpio_initialized = TRUE;
            SPI_CLOCK_SPEED = 8000000;
            spi_master_init();
            spi_register_tx_rx_handler(spi_master_handle_tx_and_rx);
            spi_master_register_data_received_cb(hci_bridge_spi_master_data_received);
            hci_puart_init();
            //hci_bridge_send_device_started_evt();
            break;

        case BTM_DISABLED_EVT:
            break;

        default:
            break;
    }
    return result;
}

/*
 * Handle received command over PUART. Please refer to the WICED HCI Control Protocol
 * Software User Manual (WICED-SWUM10x-R) for details on the
 * HCI UART control protocol.
*/
uint8_t hci_bridge_proc_rx_cmd( uint8_t *p_data, uint32_t length )
{
    //Expected minimum 4 byte as the wiced header
    if(( length < 4 ) || (!p_data))
    {
        WICED_BT_TRACE("invalid params\n");
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    spi_master_tx_data(p_data,length);
    return HCI_CONTROL_STATUS_SUCCESS;
}

/*
 *  Send Device Started event through PUART
 */
void hci_bridge_send_device_started_evt( void )
{
    uint8_t p_data[5];
    p_data[0] = 0x19;
    p_data[1] = ( HCI_CONTROL_EVENT_DEVICE_STARTED & 0xFF);
    p_data[2] = ( ( HCI_CONTROL_EVENT_DEVICE_STARTED >> 8 ) & 0xFF );
    p_data[3] = 0;
    p_data[4] = 0;
    wiced_hal_puart_synchronous_write( p_data,5);
    //wiced_trans_send_evt_over_puart( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
}

/*
 *  Handles the received data from the SPI
 */
void hci_bridge_spi_master_data_received( uint8_t* p_data )
{
    uint16_t length = p_data[3]|( p_data[4] << 8 );

    if( (p_data[1] == 0x06) && (p_data[2] == 0x05) )
    {
        toggle_gpio(p_data[7]);
        spi_master_tx_audio_data(p_data[7]);
    }
    else
    {
        wiced_hal_puart_synchronous_write(p_data,length+5);
    }
}

void toggle_gpio ( int count )
{
    while(count--)
    {
        wiced_hal_gpio_set_pin_output(3, 1);
        wiced_hal_gpio_set_pin_output(3, 0);
    }
}
