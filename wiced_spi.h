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
 * WICED SPI Driver
 */

#define WICED_HCI_HEADER_LENGTH 5

typedef enum
{
    SPI_MASTER_MODE=1,
    SPI_SLAVE_MODE
}spi_mode_t;

typedef enum
{
    SPI_RX_DATA_EVENT        = ( 1 << 0 ),
    SPI_TX_DATA_EVENT        = ( 1 << 1 ),
    SPI_RX_DMA_DONE_EVENT    = ( 1 << 2 ),
    SPI_TX_DMA_DONE_EVENT    = ( 1 << 3 ),
    SPI_INIT_EVENT           = ( 1 << 4 ),
    SPI_RX_DATA_START_EVENT  = ( 1 << 5 ),
    SPI_TX_AUDIO_DATA_EVENT  = ( 1 << 6 ),
    SPI_DEV_READY_EVENT      = ( 1 << 7 ),
    SPI_WAKEUP_EVENT         = ( 1 << 8 ),
}spi_events_t;

typedef uint8_t (*spi_tx_rx_handler_t)( uint32_t event );
/**
  * SPI Data received callback
  *
  * @param p_data          : Received data
  *
  * @return void
  */
typedef void (*wiced_trans_spi_data_rx_cback_t)( uint8_t* p_data, uint32_t length );

void spi_register_tx_rx_handler( spi_tx_rx_handler_t p_handler );

void spi_set_event( uint32_t event );
