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
#include "wiced_memory.h"
#include "wiced_spi.h"
#include "slist.h"

#ifndef REG32
#define REG32(x)     *((volatile unsigned*)(x))
#endif

typedef enum
{
    SPI_INIT_STATE,
    SPI_TX_HEADER_STATE,
    SPI_TX_WAIT_FOR_HEADER_TRANSMIT_DONE_STATE,
    SPI_TX_PAYLOAD_WAIT_FOR_DEV_READY_STATE,
    SPI_TX_WAIT_FOR_PAYLOAD_TRANSMIT_DONE_STATE,
    SPI_RX_WAIT_FOR_CMD_TRANSMIT_DONE_STATE,
    SPI_RX_WAIT_FOR_DATA_READ_START_STATE,
    SPI_TX_DONE_STATE,
    SPI_RX_DONE_STATE,
}spi_master_states_t;

typedef struct
{
     slist_node_t node; /* slist node to handle the spi tx data q*/
     uint8_t* p_data;
     uint32_t length;
}spi_master_data_t;

typedef void (*spi_master_data_rx_cb_t)( uint8_t* p_data );

typedef struct
{
    uint8_t spi_state;
    uint8_t rx_pending;
    uint8_t audio_pending;
    uint16_t rx_packet_length;
    uint8_t* p_rx_buf;
    uint16_t tx_packet_length;
    uint8_t* p_tx_data;
    slist_node_t* p_tx_node;
    wiced_bt_buffer_pool_t* p_spi_rx_buff_pool;
    wiced_bt_buffer_pool_t* p_spi_tx_buff_pool;
    spi_master_data_rx_cb_t p_data_rx_cb;
    struct slist_node_t spi_tx_data_q;
    struct slist_node_t spi_rx_data_q;
}spi_master_info_t;

/* External Definitions */
extern void spi_master_init( void );
extern void spi_master_register_data_received_cb( spi_master_data_rx_cb_t p_cb );
extern wiced_result_t spi_master_tx_data( uint8_t* p_data, uint32_t length );
extern void spi_master_tx_audio_data( uint8_t packet_count );
