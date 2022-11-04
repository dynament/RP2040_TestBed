/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 Dynament Infrared Gas Sensors Ltd.
  * Copyright (c) 2022 Status Scientific Controls Ltd.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "hardware/spi.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t  SPI_ReadPointer       = 0;
uint8_t  SPI_WritePointer      = 0;
uint16_t UART_ReadPointer_PC   = 0;
uint16_t UART_WritePointer_PC  = 0;
uint16_t UART_ReadPointer_SEN  = 0;
uint16_t UART_WritePointer_SEN = 0;

uint8_t SPI_RxBuffer      [ SPI_BUFFER_LENGTH  ];
uint8_t UART_RxBuffer_PC  [ UART_BUFFER_LENGTH ];
uint8_t UART_TxBuffer_PC  [ UART_BUFFER_LENGTH ];
uint8_t UART_RxBuffer_SEN [ UART_BUFFER_LENGTH ];
uint8_t UART_TxBuffer_SEN [ UART_BUFFER_LENGTH ];

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
// UART_RX interrupt handler
void on_uart_rx ( )
{
    if ( uart_is_readable ( UART_PC ) )
    {
        UART_RxBuffer_PC [ UART_WritePointer_PC ] = uart_getc ( UART_PC );
        UART_WritePointer_PC++;

        if ( UART_WritePointer_PC == UART_BUFFER_LENGTH )
        {
            UART_WritePointer_PC = 0;
        }
    }

    else if ( uart_is_readable ( UART_SEN ) )
    {
        UART_RxBuffer_SEN [ UART_WritePointer_SEN ] = uart_getc ( UART_SEN );
        UART_WritePointer_SEN++;

        if ( UART_WritePointer_SEN == UART_BUFFER_LENGTH )
        {
            UART_WritePointer_SEN = 0;
        }
    }

    else
    {
        // Nothing to do
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */

int main ( void )
{
    uint8_t  GetSerial [ 7 ] = { 0x10 , 0x13 , 0x30 , 0x10 , 0x1F , 0x00 , 0x82 };
    uint8_t  LineEnd   [ 2 ] = "\r\n";
    uint8_t  Sensor  = 0;
    uint16_t Length  = 0;
    uint16_t Timeout = 0;

    // Useful information for picotool
    bi_decl ( bi_program_description ( "RP2040 Premier" ) );

    // Initialise standard stdio types
    stdio_init_all ( );

    // Initialize all configured peripherals
    // Set up GPIO
    gpio_init    ( A0_PIN         );
    gpio_init    ( A1_PIN         );
    gpio_init    ( A2_PIN         );
    gpio_init    ( EN1_PIN        );
    gpio_init    ( EN2_PIN        );
    gpio_init    ( EN3_PIN        );
    gpio_init    ( LED_PICO_PIN   );
    gpio_init    ( LED_RED_PIN    );
    gpio_init    ( LED_YELLOW_PIN );
    gpio_init    ( RELAY_PIN      );
    gpio_set_dir ( A0_PIN         , GPIO_OUT );
    gpio_set_dir ( A1_PIN         , GPIO_OUT );
    gpio_set_dir ( A2_PIN         , GPIO_OUT );
    gpio_set_dir ( EN1_PIN        , GPIO_OUT );
    gpio_set_dir ( EN2_PIN        , GPIO_OUT );
    gpio_set_dir ( EN3_PIN        , GPIO_OUT );
    gpio_set_dir ( LED_PICO_PIN   , GPIO_OUT );
    gpio_set_dir ( LED_RED_PIN    , GPIO_OUT );
    gpio_set_dir ( LED_YELLOW_PIN , GPIO_OUT );
    gpio_set_dir ( RELAY_PIN      , GPIO_OUT );
    A0_LOW;
    A1_LOW;
    A2_LOW;
    EN1_LOW;
    EN2_LOW;
    EN3_LOW;
    LED_PICO_OFF;
    LED_RED_OFF;
    LED_YELLOW_OFF;
    RELAY_OFF;

    // Set up SPI
    spi_init          ( SPI_ID   , SPI_BAUD_RATE );
    gpio_set_function ( SPI_CS   , GPIO_FUNC_SPI );
    gpio_set_function ( SPI_MISO , GPIO_FUNC_SPI );
    gpio_set_function ( SPI_MOSI , GPIO_FUNC_SPI );
    gpio_set_function ( SPI_SCK  , GPIO_FUNC_SPI );

    // Set up UART hardware
    uart_init         ( UART_PC         , UART_BAUD_RATE );
    gpio_set_function ( UART_PC_RX_PIN  , GPIO_FUNC_UART );
    gpio_set_function ( UART_PC_TX_PIN  , GPIO_FUNC_UART );
    uart_init         ( UART_SEN        , UART_BAUD_RATE );
    gpio_set_function ( UART_SEN_RX_PIN , GPIO_FUNC_UART );
    gpio_set_function ( UART_SEN_TX_PIN , GPIO_FUNC_UART );
    // Set UART parameters
    uart_set_hw_flow      ( UART_PC  , false     , false              );    // Disable CTS / RTS
    uart_set_format       ( UART_PC  , DATA_BITS , STOP_BITS , PARITY );    // Data format
    uart_set_fifo_enabled ( UART_PC  , false                          );    // Turn off FIFO ( handled character by character )
    uart_set_hw_flow      ( UART_SEN , false     , false              );    // Disable CTS / RTS
    uart_set_format       ( UART_SEN , DATA_BITS , STOP_BITS , PARITY );    // Data format
    uart_set_fifo_enabled ( UART_SEN , false                          );    // Turn off FIFO ( handled character by character )
    // Set up UART_RX interrupt
    irq_set_exclusive_handler ( UART1_IRQ , on_uart_rx   );
    irq_set_enabled           ( UART1_IRQ , true         );
    uart_set_irq_enables      ( UART_PC   , true , false );  // Enable UART interrupt ( RX only )
    irq_set_exclusive_handler ( UART0_IRQ , on_uart_rx   );
    irq_set_enabled           ( UART0_IRQ , true         );
    uart_set_irq_enables      ( UART_SEN  , true , false );  // Enable UART interrupt ( RX only )

    // Set power relay
    LED_RED_ON;
    RELAY_ON;
    sleep_ms ( 1000 );  // Wait 1 second for power supplies to settle
    LED_YELLOW_ON;

    // Infinite loop
    while ( 1 )
    {
        if ( UART_WritePointer_PC > 0 )
        {
            Length = sprintf ( UART_TxBuffer_PC , "%s\r\n" , UART_RxBuffer_PC );
            uart_write_blocking ( UART_PC , UART_TxBuffer_PC , Length );
            memset ( UART_RxBuffer_PC , 0 , sizeof ( UART_RxBuffer_PC ) );
            UART_WritePointer_PC = 0;
        }

        memset ( UART_TxBuffer_PC , 0 , sizeof ( UART_TxBuffer_PC ) );
        Length = sprintf ( UART_TxBuffer_PC , "Sensor get serial number string: " );
        memcpy ( UART_TxBuffer_PC + 33 , GetSerial , 7 );
        memcpy ( UART_TxBuffer_PC + 40 , LineEnd   , 2 );
        uart_write_blocking ( UART_PC , UART_TxBuffer_PC , ( Length + 9 ) );

        Sensor = 1;

        while ( Sensor < 25 )
        {
            // Set sensor position
            Set_MUX ( Sensor );

            // Send serial request
            memset ( UART_TxBuffer_SEN , 0 , sizeof ( UART_TxBuffer_SEN ) );
            memcpy ( UART_TxBuffer_SEN , GetSerial , 7 );
            uart_write_blocking ( UART_SEN , UART_TxBuffer_SEN , 7 );

            // Wait for response w/timeout
            Timeout = 0;
            while ( ( UART_WritePointer_SEN < 12 ) && ( Timeout < UART_TIMEOUT ) )
            {
                sleep_ms ( 10 );
                Timeout += 10;
            }

            // Send response to PC serial terminal
            if ( UART_WritePointer_SEN > 10 )
            {
                Length = sprintf ( UART_TxBuffer_PC , "Sensor %d serial number: %s\r\n" , Sensor , UART_RxBuffer_SEN );
                uart_write_blocking ( UART_PC , UART_TxBuffer_PC , Length );
                memset ( UART_RxBuffer_SEN , 0 , sizeof ( UART_RxBuffer_SEN ) );
                UART_WritePointer_SEN = 0;
            }
            else
            {
                Length = sprintf ( UART_TxBuffer_PC , "Sensor %d serial number: READ ERROR\r\n" , Sensor );
                uart_write_blocking ( UART_PC , UART_TxBuffer_PC , Length );
                memset ( UART_RxBuffer_SEN , 0 , sizeof ( UART_RxBuffer_SEN ) );
                UART_WritePointer_SEN = 0;
            }

            Sensor++;
        }

        sleep_ms ( 1000 );
    }
}

void Set_MUX ( uint8_t sensor )
{
    switch ( sensor )
    {
        case 1:
            A2_LOW;
            A1_LOW;
            A0_LOW;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 2:
            A2_LOW;
            A1_LOW;
            A0_HIGH;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 3:
            A2_LOW;
            A1_HIGH;
            A0_LOW;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 4:
            A2_LOW;
            A1_HIGH;
            A0_HIGH;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 5:
            A2_HIGH;
            A1_LOW;
            A0_LOW;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 6:
            A2_LOW;
            A1_LOW;
            A0_LOW;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 7:
            A2_HIGH;
            A1_LOW;
            A0_LOW;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 8:
            A2_HIGH;
            A1_LOW;
            A0_HIGH;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 9:
            A2_HIGH;
            A1_HIGH;
            A0_LOW;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 10:
            A2_HIGH;
            A1_HIGH;
            A0_HIGH;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 11:
            A2_HIGH;
            A1_LOW;
            A0_HIGH;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 12:
            A2_LOW;
            A1_LOW;
            A0_HIGH;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 13:
            A2_LOW;
            A1_LOW;
            A0_LOW;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 14:
            A2_LOW;
            A1_LOW;
            A0_HIGH;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 15:
            A2_LOW;
            A1_HIGH;
            A0_LOW;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 16:
            A2_LOW;
            A1_HIGH;
            A0_HIGH;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 17:
            A2_HIGH;
            A1_HIGH;
            A0_LOW;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 18:
            A2_LOW;
            A1_HIGH;
            A0_LOW;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 19:
            A2_HIGH;
            A1_LOW;
            A0_LOW;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 20:
            A2_HIGH;
            A1_LOW;
            A0_HIGH;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 21:
            A2_HIGH;
            A1_HIGH;
            A0_LOW;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 22:
            A2_HIGH;
            A1_HIGH;
            A0_HIGH;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 23:
            A2_HIGH;
            A1_HIGH;
            A0_HIGH;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 24:
            A2_LOW;
            A1_HIGH;
            A0_HIGH;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        default:
            // Invalid number
            A2_LOW;
            A1_LOW;
            A0_LOW;
            EN1_LOW;
            EN2_LOW;
            EN3_LOW;
        break;
    }
}

void SPI_Write ( uint8_t buffer [ ] , size_t len )
{
    int i;

    for ( i = 0 ; i < len ; ++i )
    {
        if ( i % 16 == 15 )
        {
            printf ( "%02x\n" , buffer [ i ] );
        }
        else
        {
            printf ( "%02x " , buffer [ i ] );
        }
    }

    // append trailing newline if there isn't one
    if ( i % 16 )
    {
        putchar ( '\n' );
    }
}

/* SPI master example code */
/*
int main ( void )
{
    uint8_t out_buf [ SPI_BUFFER_LENGTH ] , in_buf [ SPI_BUFFER_LENGTH ];

    // Initialize output buffer
    for ( size_t i = 0 ; i < BUF_LEN ; ++i )
    {
        out_buf [ i ] = i;
    }

    printf   ( "SPI master says: The following buffer will be written to MOSI endlessly:\n" );
    printbuf ( out_buf , SPI_BUFFER_LENGTH );

    for ( size_t i = 0 ; ; ++i )
    {
        // Write the output buffer to MOSI and at the same time read from MISO
        spi_write_read_blocking ( SPI_ID , out_buf , in_buf , SPI_BUFFER_LENGTH );

        // Write to stdio whatever came in on the MISO line
        printf   ( "SPI master says: read page %d from the MISO line:\n" , i );
        printbuf ( in_buf , SPI_BUFFER_LENGTH );

        // Sleep for ten seconds so you get a chance to read the output
        sleep_ms ( 10 * 1000 );
    }
}
*/
