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
 * This file implements the SPI master. Also sends the stored sine data
 * to support audio streaming
 *
 */
#include "spiffydriver.h"
#include "gpiodriver.h"
#include "wiced.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_mia.h"
#include "wiced_hal_pspi.h"
#include "wiced_hal_gpio.h"
#include "bridge_spi.h"
#include "hci_control_api.h"
#include "string.h"
#include "wiced_bt_app_hal_common.h"
#include "wiced_bt_app_common.h"
#include "wiced_bt_event.h"

/**************************************************************
**           Macros
**************************************************************/

uint32_t SPI_CLOCK_SPEED = (2000000);

#define SPI_TX_BUFFER_SIZE                  1024
#define SPI_TX_BUFFER_COUNT                 5

#define SPI_RX_BUFFER_SIZE                  1024
#define SPI_RX_BUFFER_COUNT                 5

#define TRAN_PKT_HCI_CMD                    0x01
#define TRAN_PKT_HCI_ACL                    0x02
#define MPAF_TRAN_PKT_TYPE                  0x19
#define TRAN_PKT_HCI_EVENT                  0x04

#define PACKET_TYPE_VALID(packet_type ) ( ( packet_type == TRAN_PKT_HCI_CMD ) || \
                                          ( packet_type == TRAN_PKT_HCI_ACL ) || \
                                          ( packet_type == MPAF_TRAN_PKT_TYPE ) || \
                                          ( packet_type == TRAN_PKT_HCI_EVENT ))
#define HOST_BT_DEV_READY_GPIO              WICED_P12
#define SPI_CS_GPIO                         WICED_P26
#define MASTER_DEBUG_GPIO                   WICED_P03


/**************************************************************
**           Variable Definitons
**************************************************************/
spi_master_info_t spi_master_info;

//Audio Data Handling
#define AUDIO_DATA_LENGTH               1029
uint8_t spi_audio_data[AUDIO_DATA_LENGTH];

short sin_samples[] = {
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,

    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
    0,390.4,765.6,1111.2,1414.4,1663.2,1848,1961.6,2000,1961.6,1848,1663.2,1414.4,1111.2,765.6,390.4,0,-292.8,-574.2,-833.4,-1060.8,-1247.4,-1386,-1471.2,-1500,-1471.2,-1386,-1247.4,-1060.8,-833.4,-574.2,-292.8,
};

extern uint8_t hci_spi_transport; //Variable to indicate SPI transport is in use
extern spi_mode_t spi_mode;

/**************************************************************
**           Function Declarations
**************************************************************/

void spi_master_toggle_gpio( uint8_t count );

/**************************************************************
**           Function Definitions
**************************************************************/
wiced_result_t spi_master_buffer_init( uint32_t tx_buffer_size, uint32_t rx_buffer_size, uint32_t tx_buffer_count,uint32_t rx_buffer_count )
{
    wiced_result_t result = WICED_ERROR;

    spi_master_info.p_spi_tx_buff_pool = wiced_bt_create_pool( tx_buffer_size + sizeof(spi_master_data_t), tx_buffer_count );
    spi_master_info.p_spi_rx_buff_pool = wiced_bt_create_pool( rx_buffer_size + sizeof(spi_master_data_t), rx_buffer_count );

    if( ( spi_master_info.p_spi_tx_buff_pool ) && ( spi_master_info.p_spi_rx_buff_pool ) )
    {
        result = WICED_SUCCESS;
    }
    return result;
}

void spi_master_setup_audio_data( void )
{
    uint16_t bytes_per_packet = 1024;
    spi_audio_data[0]= 0x19;
    spi_audio_data[1] = HCI_CONTROL_AUDIO_DATA & 0xff;
    spi_audio_data[2] = (HCI_CONTROL_AUDIO_DATA >> 8) & 0xff;
    spi_audio_data[3]= (bytes_per_packet & 0xff);
    spi_audio_data[4] = ((bytes_per_packet >> 8) & 0xff );
    memcpy( &spi_audio_data[5], sin_samples, 1024);
}

void spi_master_dev_ready_interrupt_handler( void *data, uint8_t port_pin )
{
    if ( wiced_hal_gpio_get_pin_interrupt_status( HOST_BT_DEV_READY_GPIO ) )
    {
        wiced_hal_gpio_clear_pin_interrupt_status( HOST_BT_DEV_READY_GPIO );
    }
    spi_master_toggle_gpio(1);
    spi_set_event(SPI_DEV_READY_EVENT);
}

void spi_master_toggle_gpio( uint8_t count )
{
    uint8_t index;
    for ( index = 0; index < count; index++ )
    {
        wiced_hal_gpio_set_pin_output(MASTER_DEBUG_GPIO, 1);
        wiced_hal_gpio_set_pin_output(MASTER_DEBUG_GPIO, 0);
    }
}

void spi_master_assert_cs(void)
{
    wiced_hal_gpio_set_pin_output(SPI_CS_GPIO, 0);
}

void spi_master_deassert_cs(void)
{
    wiced_hal_gpio_set_pin_output(SPI_CS_GPIO, 1);
}

void spi_master_configure_gpio(void)
{
    wiced_bt_app_hal_init();

    wiced_hal_gpio_configure_pin( HOST_BT_DEV_READY_GPIO, ( GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_RISING_EDGE ), GPIO_PIN_OUTPUT_LOW );
    wiced_hal_gpio_register_pin_for_interrupt( HOST_BT_DEV_READY_GPIO, spi_master_dev_ready_interrupt_handler,NULL );

    wiced_hal_gpio_configure_pin(SPI_CS_GPIO, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH );//chip select
    wiced_hal_gpio_configure_pin(MASTER_DEBUG_GPIO, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW ); //debug purpose
}

void spi_master_init( void )
{
    INIT_SLIST_NODE(&spi_master_info.spi_tx_data_q);
    INIT_SLIST_NODE(&spi_master_info.spi_rx_data_q);
    wiced_hal_pspi_init( MASTER1_CONFIG,
                         INPUT_PIN_FLOATING,
                         MASTER1_P36_CLK_P00_MOSI_P25_MISO,
                         SPI_CLOCK_SPEED,
                         SPI_MSB_FIRST,
                         SPI_SS_ACTIVE_LOW,
                         SPI_MODE_0,
                         SPI_CS_GPIO);
    spi_master_configure_gpio();
    spi_master_buffer_init(SPI_TX_BUFFER_SIZE, SPI_RX_BUFFER_SIZE, SPI_TX_BUFFER_COUNT, SPI_RX_BUFFER_COUNT);
    spi_master_setup_audio_data(); //Testing audio
}


wiced_result_t spi_master_tx_data( uint8_t* p_data, uint32_t length )
{
    uint8_t* p_data_payload;
    spi_master_data_t* p_tx_data;

    p_tx_data = wiced_bt_get_buffer_from_pool( spi_master_info.p_spi_tx_buff_pool );
    if ( p_tx_data )
    {
        p_data_payload = (uint8_t*)p_tx_data;
        p_data_payload += sizeof(spi_master_data_t);
        if ( ( length + sizeof(spi_master_data_t) )<= wiced_bt_get_buffer_size( p_tx_data ) )
        {
            memcpy( p_data_payload, p_data, length );
            p_tx_data->p_data = p_data_payload;
            p_tx_data->length = length;
            slist_add_tail(&p_tx_data->node,&spi_master_info.spi_tx_data_q);
            spi_set_event(SPI_TX_DATA_EVENT);
            return WICED_SUCCESS;
        }
        else
        {
             wiced_bt_free_buffer( p_tx_data );
        }
    }
    return WICED_ERROR;
}

void spi_master_clear_tx_fifo(void)
{
    while( REG32(spiffy_TxFIFOLevel_adr) & SPIFFY_TX_FIFO_LEVEL_MASK )
    {
        REG32(spiffy_TxFIFO_adr);
    }
}

void spi_master_clear_rx_fifo( void )
{
    while( REG32(spiffy_RxFIFOLevel_adr) & SPIFFY_RX_FIFO_LEVEL_MASK )
    {
        REG32(spiffy_RxFIFO_adr);
    }
}

//Blocking read
void spi_master_start_tx( uint8_t* p_data, uint32_t length )
{
    spi_master_clear_tx_fifo();

    spi_master_assert_cs();
    spiffyd_txData(0,length , p_data);
    spi_master_deassert_cs();

    spi_master_toggle_gpio(2);

    spi_master_clear_rx_fifo();
    spi_set_event(SPI_TX_DMA_DONE_EVENT);
}

void spi_master_read_bytes( uint8_t* p_data, uint32_t length )
{
     //spi_master_assert_cs();
     spi_master_toggle_gpio(3);
     spiffyd_rxData(SPIFFYD_1, length, p_data );
     //spi_master_deassert_cs();
     spi_master_toggle_gpio(3);
}

void spi_master_register_data_received_cb( spi_master_data_rx_cb_t p_cb )
{
    spi_master_info.p_data_rx_cb = p_cb;
}

void spi_master_tx_audio_data( uint8_t packet_count )
{
    spi_master_info.audio_pending += packet_count;
    spi_master_toggle_gpio(packet_count);
    spi_set_event(SPI_TX_AUDIO_DATA_EVENT);
}

int spi_master_data_received_cback( void * data )
{
    slist_node_t* p_node;
    spi_master_data_t* p_rx_data;
    if ( !slist_empty( &spi_master_info.spi_rx_data_q ) )
    {
       // osapi_INIT_LOCK_CONTEXT

        //osapi_LOCK_CONTEXT
        p_node = slist_front( &spi_master_info.spi_rx_data_q );
        slist_del(p_node,&spi_master_info.spi_rx_data_q );
        //osapi_UNLOCK_CONTEXT

        p_rx_data = (spi_master_data_t*)p_node;
        if( spi_master_info.p_data_rx_cb )
        {
            spi_master_info.p_data_rx_cb( p_rx_data->p_data );
        }
        wiced_bt_free_buffer( p_node );
    }
    return 0;
}


uint8_t spi_master_handle_tx_and_rx( uint32_t event )
{
    spi_master_data_t* p_tx_data;
    spi_master_data_t* p_rx_data;
    slist_node_t* p_rx_node;
    uint8_t loop_continue = TRUE;

    while ( loop_continue )
    {
        loop_continue = FALSE;
        switch ( spi_master_info.spi_state )
        {
            case SPI_INIT_STATE:
                if ( ( event & SPI_DEV_READY_EVENT ) && !spi_master_info.audio_pending && slist_empty( &spi_master_info.spi_tx_data_q ) )
                {
                    uint8_t wiced_hci_rx_command[] = { 0x19, 0x00, 0x00, 0x00, 0x00 };
                    spi_master_toggle_gpio(4);
                    spi_master_info.spi_state = SPI_RX_WAIT_FOR_CMD_TRANSMIT_DONE_STATE;
                    spi_master_start_tx( wiced_hci_rx_command, WICED_HCI_HEADER_LENGTH );
                }
                else if ( ( event & SPI_DEV_READY_EVENT ) && ( spi_master_info.audio_pending || !slist_empty( &spi_master_info.spi_tx_data_q ) ) )
                {
                    spi_master_toggle_gpio(5);
                    spi_master_info.spi_state = SPI_TX_HEADER_STATE;
                    loop_continue = TRUE;
                }
                else if (  spi_master_info.audio_pending || !slist_empty( &spi_master_info.spi_tx_data_q ) )
                {
                    spi_master_toggle_gpio(6);
                    spi_master_assert_cs();
                    spi_master_info.spi_state = SPI_TX_HEADER_STATE;
                }
                break;

             case SPI_TX_HEADER_STATE:
                if ( event & SPI_DEV_READY_EVENT )
                {
                    if ( spi_master_info.audio_pending )
                    {
                        spi_master_info.p_tx_data = spi_audio_data;
                        spi_master_info.tx_packet_length = AUDIO_DATA_LENGTH;

                        spi_master_info.audio_pending--;
                        spi_master_info.spi_state = SPI_TX_WAIT_FOR_HEADER_TRANSMIT_DONE_STATE;
                        spi_master_start_tx( spi_master_info.p_tx_data, WICED_HCI_HEADER_LENGTH );
                    }
                    else if ( !slist_empty( &spi_master_info.spi_tx_data_q ) )
                    {
                        spi_master_info.p_tx_node = slist_front( &spi_master_info.spi_tx_data_q );
                        slist_del( spi_master_info.p_tx_node, &spi_master_info.spi_tx_data_q );
                        p_tx_data = (spi_master_data_t*)spi_master_info.p_tx_node;
                        spi_master_info.tx_packet_length = p_tx_data->length;
                        spi_master_info.p_tx_data = p_tx_data->p_data;

                        spi_master_info.spi_state = SPI_TX_WAIT_FOR_HEADER_TRANSMIT_DONE_STATE;
                        spi_master_start_tx( spi_master_info.p_tx_data, WICED_HCI_HEADER_LENGTH );
                    }
                }
                break;
            case SPI_TX_WAIT_FOR_HEADER_TRANSMIT_DONE_STATE:
                if ( event & SPI_TX_DMA_DONE_EVENT )
                {
                    //If payload available to be transmitted
                    if ( spi_master_info.tx_packet_length - WICED_HCI_HEADER_LENGTH )
                    {
                        spi_master_info.spi_state = SPI_TX_PAYLOAD_WAIT_FOR_DEV_READY_STATE;
                    }
                    else
                    {
                        event = SPI_TX_DMA_DONE_EVENT;
                        loop_continue = TRUE;
                        spi_master_info.spi_state = SPI_TX_WAIT_FOR_PAYLOAD_TRANSMIT_DONE_STATE;
                    }
                }
                break;

            case SPI_TX_PAYLOAD_WAIT_FOR_DEV_READY_STATE:
                if ( event & SPI_DEV_READY_EVENT )
                {
                    spi_master_info.spi_state = SPI_TX_WAIT_FOR_PAYLOAD_TRANSMIT_DONE_STATE;
                    spi_master_start_tx( spi_master_info.p_tx_data+WICED_HCI_HEADER_LENGTH, spi_master_info.tx_packet_length-WICED_HCI_HEADER_LENGTH );
                }
                break;

            case SPI_TX_WAIT_FOR_PAYLOAD_TRANSMIT_DONE_STATE:
                if ( event & SPI_TX_DMA_DONE_EVENT )
                {
                    if ( spi_master_info.p_tx_node )
                    {
                        wiced_bt_free_buffer(spi_master_info.p_tx_node);
                    }
                    spi_master_info.p_tx_node = NULL;
                    event = 0;
                    loop_continue = TRUE;
                    spi_master_info.spi_state = SPI_TX_DONE_STATE;
                }
                break;

            case SPI_RX_WAIT_FOR_CMD_TRANSMIT_DONE_STATE:
                if ( event & SPI_TX_DMA_DONE_EVENT )
                {
                    spi_master_info.spi_state = SPI_RX_WAIT_FOR_DATA_READ_START_STATE;
                }
                break;

            case SPI_RX_WAIT_FOR_DATA_READ_START_STATE:
                if ( event & SPI_DEV_READY_EVENT )
                {
                    //Read the header from the slave
                    uint8_t packet_header[5];
                    spi_master_clear_rx_fifo();
                    spi_master_assert_cs();
                    spi_master_read_bytes(packet_header,5);
                    if ( PACKET_TYPE_VALID( packet_header[0] ) )
                    {
                        spi_master_info.rx_packet_length =
                            packet_header[3] | ( packet_header[4] << 8 );
                        p_rx_node = wiced_bt_get_buffer_from_pool( spi_master_info.p_spi_rx_buff_pool );
                        spi_master_info.p_rx_buf = (uint8_t*)p_rx_node;
                        if ( spi_master_info.p_rx_buf )
                        {
                            if ( spi_master_info.rx_packet_length + 5 + sizeof(spi_master_data_t) <= wiced_bt_get_buffer_size( spi_master_info.p_rx_buf ) )
                            {
                                //osapi_INIT_LOCK_CONTEXT
                                spi_master_info.p_rx_buf += sizeof(spi_master_data_t);
                                spi_master_read_bytes(spi_master_info.p_rx_buf+5,spi_master_info.rx_packet_length);
                                spi_master_deassert_cs();
                                memcpy( spi_master_info.p_rx_buf,packet_header, 5 );

                                p_rx_data = (spi_master_data_t*)p_rx_node;
                                p_rx_data->p_data = spi_master_info.p_rx_buf;
                                p_rx_data->length = WICED_HCI_HEADER_LENGTH + spi_master_info.rx_packet_length;

                                //osapi_LOCK_CONTEXT
                                slist_add_tail(p_rx_node,&spi_master_info.spi_rx_data_q );
                                //osapi_UNLOCK_CONTEXT
                                wiced_app_event_serialize( spi_master_data_received_cback, NULL);
                            }
                            else
                            {
                                wiced_bt_free_buffer(p_rx_node);
                            }
                        }
                    }
                    spi_master_deassert_cs();
                    //Todo handle the failure scenarios
                    event = 0;
                    loop_continue = TRUE;
                    spi_master_info.spi_state = SPI_RX_DONE_STATE;
               }
               break;

            case SPI_TX_DONE_STATE:
            case SPI_RX_DONE_STATE:
                if ( !event )
                {
                    if ( spi_master_info.audio_pending )
                    {
                        event = SPI_TX_AUDIO_DATA_EVENT;
                        loop_continue = TRUE;
                    }
                    else if ( !slist_empty( &spi_master_info.spi_tx_data_q ) )
                    {
                        event = SPI_TX_DATA_EVENT;
                        loop_continue = TRUE;
                    }
                    spi_master_info.spi_state = SPI_INIT_STATE;
                    spi_master_toggle_gpio(10);
               }
               break;

            default:
                spi_master_toggle_gpio(15);
                break;
        }
    }
    return 0;
}
