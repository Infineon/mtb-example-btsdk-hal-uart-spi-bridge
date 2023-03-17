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
#include "sparcommon.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_hal_mia.h"
#include "wiced_bt_app_common.h"
#include "string.h"

#define CMD_LEN 1024

typedef struct
{
    uint8_t event_rx_buffer[CMD_LEN];
    uint16_t uart_rx_used_20703;
    uint16_t uart_read_len_20703;
    uint8_t uart_stdio_rx_buffer[CMD_LEN];
    uint16_t uart_rx_used_len;
    uint16_t uart_read_len;
}uart_bridge_buffer_t;

typedef struct
{
    uint8_t  uart_state_20703;
    uint8_t  uart_state_stdio;
}bridge_state_t;

uart_bridge_buffer_t  uart_bridge_buffer;
bridge_state_t  state;

extern uint8_t  hci_bridge_proc_rx_cmd( uint8_t *p_data, uint32_t length );

void hci_puart_rx_interrupt_callback(void* unused)
{
    // There can be at most 16 bytes in the HW FIFO.
    uint8_t  readbyte;

    wiced_hal_puart_read( &readbyte );

    uart_bridge_buffer.uart_stdio_rx_buffer[uart_bridge_buffer.uart_rx_used_len++] = readbyte;


    WICED_BT_TRACE("puar_rx_interrupt_callback\n");

    switch(state.uart_state_stdio)
    {
    case 0: /* new packet */
        state.uart_state_stdio = 1;
        uart_bridge_buffer.uart_read_len = 0;

        if((uart_bridge_buffer.uart_stdio_rx_buffer[0] == 1))
        {
            uart_bridge_buffer.uart_read_len = 3 + 1;
        }else if((uart_bridge_buffer.uart_stdio_rx_buffer[0] == 2) || (uart_bridge_buffer.uart_stdio_rx_buffer[0] == 0x19))
        {
            uart_bridge_buffer.uart_read_len = 4 + 1;
        }
        else if (uart_bridge_buffer.uart_stdio_rx_buffer[0] == 4)
        {
            uart_bridge_buffer.uart_read_len = 2 + 1;
        }
        else
        {
            state.uart_state_stdio = 0;/* back to wait */
            uart_bridge_buffer.uart_rx_used_len = 0;
        }

        break;

    case 1:

        if(uart_bridge_buffer.uart_read_len ==  uart_bridge_buffer.uart_rx_used_len )
        {
            if((uart_bridge_buffer.uart_stdio_rx_buffer[0] == 1))
            {
                uart_bridge_buffer.uart_read_len = 4 + uart_bridge_buffer.uart_stdio_rx_buffer[3];
            }
            else if ((uart_bridge_buffer.uart_stdio_rx_buffer[0] == 2) || (uart_bridge_buffer.uart_stdio_rx_buffer[0] == 0x19))
            {
                uart_bridge_buffer.uart_read_len = 5 + ( uart_bridge_buffer.uart_stdio_rx_buffer[3] | (uart_bridge_buffer.uart_stdio_rx_buffer[4] << 8) );
            }
            else if (uart_bridge_buffer.uart_stdio_rx_buffer[0] == 4)
            {
                uart_bridge_buffer.uart_read_len = 3 + uart_bridge_buffer.uart_stdio_rx_buffer[2];
            }

            if(uart_bridge_buffer.uart_read_len ==  uart_bridge_buffer.uart_rx_used_len )
            {
              //  SendPacketToHciUart(uart_bridge_buffer.uart_stdio_rx_buffer,  uart_bridge_buffer.uart_rx_used_len );

                WICED_BT_TRACE_ARRAY(uart_bridge_buffer.uart_stdio_rx_buffer, uart_bridge_buffer.uart_rx_used_len, "PUART" );
                hci_bridge_proc_rx_cmd(&uart_bridge_buffer.uart_stdio_rx_buffer[0],  uart_bridge_buffer.uart_rx_used_len );

                state.uart_state_stdio  = 0;
                uart_bridge_buffer.uart_rx_used_len = 0;
            }else
            {
                state.uart_state_stdio = 2;
            }

        }
        break;
    case 2:

        if(uart_bridge_buffer.uart_read_len ==  uart_bridge_buffer.uart_rx_used_len )
        {

            //SendPacketToHciUart(uart_bridge_buffer.uart_stdio_rx_buffer,  uart_bridge_buffer.uart_rx_used_len );
            WICED_BT_TRACE_ARRAY(uart_bridge_buffer.uart_stdio_rx_buffer, uart_bridge_buffer.uart_rx_used_len, "PUART" );
            hci_bridge_proc_rx_cmd(&uart_bridge_buffer.uart_stdio_rx_buffer[0],  uart_bridge_buffer.uart_rx_used_len);
            state.uart_state_stdio  = 0;
            uart_bridge_buffer.uart_rx_used_len = 0;

        }
        break;
        }


    wiced_hal_puart_reset_puart_interrupt( );

}

/* Sample code to test puart driver. Initialises puart, selects puart pads,
 * turn off flow control, and enables Tx and Rx.
 * Echoes the input byte with increment by 1.
 */
void hci_puart_init( void )
{
    uint8_t read_5_bytes[5];

    wiced_bt_app_init();
    wiced_hal_puart_init();

    // Possible uart tx and rx combination.
    // Pin for Rx: p2, Pin for Tx: p0
    // Note that p2 and p0 might not be avaliable for use on your
    // specific hardware platform.
    // Please see the User Documentation to reference the valid pins.
    wiced_hal_puart_select_uart_pads( 33, 31, 0, 0);

    /* Turn off flow control */
    wiced_hal_puart_flow_off( );  // call wiced_hal_puart_flow_on(); to turn on flow control

    // BEGIN - puart interrupt
    wiced_hal_puart_register_interrupt(hci_puart_rx_interrupt_callback);

    /* Turn on Tx */
    wiced_hal_puart_enable_tx(); // call wiced_hal_puart_disable_tx to disable transmit capability.
   // wiced_hal_puart_print( "Hello World!\r\nType something! Keystrokes are echoed to the terminal ...\r\n");

    /* Enable to change puart baud rate. eg: 9600, 19200, 38200 */
    //wiced_hal_puart_set_baudrate( 115200 );

    memset(&uart_bridge_buffer,0,sizeof(uart_bridge_buffer));

}
