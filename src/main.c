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

#include "main.h"

uint8_t  aucRxBufferMaster [ P2P_BUFFER_MASTER_SIZE ] __attribute__( ( aligned ( 16 ) ) );
uint8_t  aucRxBufferSlave  [ P2P_BUFFER_SLAVE_SIZE  ] __attribute__( ( aligned ( 16 ) ) );
uint16_t uiRxBufferMasterGet = 0;
uint16_t uiRxBufferMasterPut = 0;
uint16_t uiRxBufferSlaveGet  = 0;
uint16_t uiRxBufferSlavePut  = 0;

uint8_t  countPC                  = 0;
uint8_t  countSensor              = 0;
uint8_t  ucHighTemperatureFlag    = false;
uint8_t  ucPcCommsFlag            = 0;
uint8_t  ucSensorCommsFlag        = 0;
uint8_t  ucSerialNumberRequest    = false;
uint16_t EnclosureTemperature     = 0;
uint16_t uiCommsStatus            = 0;
uint16_t uiTemperatureCutoffPoint = 0;
uint16_t uiTemperatureResetPoint  = 0;

volatile uint16_t uiCommsTimeout = 0;

void getTemperatureStatus ( void );

// Timer interrupts
bool temperature_1000ms_callback ( struct repeating_timer *t )
{
    // getTemperatureStatus ( );
}

bool timer_10ms_callback ( struct repeating_timer *t )
{
    if ( ucHighTemperatureFlag == true )
    {
        LED_YELLOW_ON;  // PC Comms led on, high temperature
        LED_RED_ON;     // Sensor comms led on, high temperature
    }
    else
    {
        if ( ucPcCommsFlag == 1 )
        {
            LED_YELLOW_ON;  // PC comms data led on
        }
        else
        {
            // Nothing to do
        }
    
        if ( ucSensorCommsFlag == 1 )
        {
            LED_RED_ON; // Sensor comms data led on
        }
        else
        {
            // Nothing to do
        }

        countPC++;
        if ( countPC > 19 )
        {
            LED_YELLOW_OFF; // PC comms data led off
            ucPcCommsFlag = 0;
            countPC       = 0;
        }
        else
        {
            // Nothing to do
        }
    
        countSensor++;
        if ( countSensor > 19 )
        {
            LED_RED_OFF;    // Sensor comms data led off
            ucSensorCommsFlag = 0;
            countSensor       = 0;
        }
        else
        {
            // Nothing to do
        }
    }

    if ( uiCommsTimeout > 0 )
    {
        uiCommsTimeout--;
    }
    else
    {
        // Nothing to do
    }
}

bool watchdog_500ms_callback ( struct repeating_timer *t )
{
    if ( gpio_get ( LED_PICO_PIN ) )
    {
        LED_PICO_OFF;
    }
    else
    {
        LED_PICO_ON;
    }
}

// UART_RX interrupt handler
void on_uart_rx ( )
{
    if ( uart_is_readable ( UART_PC ) )
    {
        aucRxBufferMaster [ uiRxBufferMasterPut ] = uart_getc ( UART_PC );  // Store character in buffer
        uiRxBufferMasterPut++;

        if ( uiRxBufferMasterPut == P2P_BUFFER_MASTER_SIZE )
        {
            uiRxBufferMasterPut = 0; // At end wrap to beginning
        }

        if ( uiRxBufferMasterPut == uiRxBufferMasterGet )
        {
            // Increment get pointer on overflow
            if ( ++uiRxBufferMasterGet == P2P_BUFFER_MASTER_SIZE )
            {
                uiRxBufferMasterGet = 0;
            }
        }
        
        if ( UART_UARTRSR_FE_BITS == ( uart_get_hw ( UART_PC ) -> rsr ) )
        {
            // Clear framing error
            hw_clear_bits ( &uart_get_hw ( UART_PC ) -> rsr , UART_UARTRSR_FE_BITS );
        }

        if ( UART_UARTRSR_OE_BITS == ( uart_get_hw ( UART_PC ) -> rsr ) )
        {
            // Clear overrun error
            hw_clear_bits ( &uart_get_hw ( UART_PC ) -> rsr , UART_UARTRSR_OE_BITS );
        }

    }

    else if ( uart_is_readable ( UART_SEN ) )
    {
        aucRxBufferSlave [ uiRxBufferSlavePut++ ] = uart_getc ( UART_SEN ); // Store character in buffer

        if ( uiRxBufferSlavePut == P2P_BUFFER_SLAVE_SIZE )
        {
            uiRxBufferSlavePut = 0; // At end wrap to beginning
        }

        if ( uiRxBufferSlavePut == uiRxBufferSlaveGet )
        {
            // Increment get pointer on overflow
            if ( ++uiRxBufferSlaveGet == P2P_BUFFER_SLAVE_SIZE )
            {
                uiRxBufferSlaveGet = 0;
            }
        }

        if ( UART_UARTRSR_FE_BITS == ( uart_get_hw ( UART_SEN ) -> rsr ) )
        {
            // Clear framing error
            hw_clear_bits ( &uart_get_hw ( UART_SEN ) -> rsr , UART_UARTRSR_FE_BITS );
        }

        if ( UART_UARTRSR_OE_BITS == ( uart_get_hw ( UART_SEN ) -> rsr ) )
        {
            // Clear overrun error
            hw_clear_bits ( &uart_get_hw ( UART_SEN ) -> rsr , UART_UARTRSR_OE_BITS );
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
    struct repeating_timer timer_10ms;
    struct repeating_timer timer_getTemp;
    struct repeating_timer timer_watchdog;

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

    // Set up timer interrupts
    add_repeating_timer_ms ( 1000 , temperature_1000ms_callback , NULL , &timer_getTemp );
    add_repeating_timer_ms (   10 , timer_10ms_callback         , NULL , &timer_10ms    );
    add_repeating_timer_ms (  500 , watchdog_500ms_callback     , NULL , &timer_watchdog );

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

    // Set up watchdog
    // watchdog_enable ( 10000 , 1 );   // Watchdog must be updated within 10 s or chip will reboot

    ucHighTemperatureFlag = false;
    ucPassThroughMode     = false ;
    ucPcCommsFlag         = 0;
    ucSensorCommsFlag     = 0;
    ucSerialNumberRequest = false;

    // Set power relay
    LED_RED_OFF;
    LED_YELLOW_OFF;
    RELAY_ON;

    uiCommsTimeout = 500;   // 5 seconds

    while ( uiCommsTimeout )
    {
        watchdog ( );
    }

    // Clear comms buffer on spurious characters
    uiRxBufferMasterGet = 0;
    uiRxBufferMasterPut = 0;
    uiRxBufferSlaveGet  = 0;
    uiRxBufferSlavePut  = 0;

    // Get temperature limits
    uiTemperatureCutoffPoint = DEG90;
    uiTemperatureResetPoint  = DEG82;

    // Infinite loop
    while ( 1 )
    {
        watchdog ( );

        if ( ucHighTemperatureFlag == false )
        {
            RELAY_ON;   // Turn on power to sensors

            ucFlagJigCommand = false;
            uiCommsStatus    = p2pPollMaster ( );
            
            if ( uiCommsStatus )
            {
                if ( uiCommsMode == COMMS_WRITE )
                {
                    if ( ucPassThroughMode == true )
                    {
                        ucPassThroughMode = false;
                        uiCommsMode       = COMMS_WAIT;
                        uiCommsStatus     = p2pPollSlaveWritePassThrough ( );
                    }
                    else if ( ucSerialNumberRequest == true )
                    {
                        ucSerialNumberRequest = false;
                        uiCommsMode           = COMMS_WAIT;
                    }
                    else
                    {
                        uiCommsStatus = p2pPollSlaveWrite ( );
                    }
                }
                else
                {
                    if ( ucSerialNumberRequest == true )
                    {
                        // Nothing to do
                    }
                    else
                    {
                        uiCommsStatus = p2pPollSlaveRead ( );
                    }
                }

                if ( uiCommsStatus )
                {
                    // Nothing to do
                }
                else
                {
                    // Nothing to do
                }
            }
            else
            {
                // Nothing to do
            }
        }
        else
        {
            RELAY_OFF;  // Turn off power to sensors
        }
    }

    return 0;
}

// Get value for the temperature IC from SPI ADC
// Discard spurious readings
// Set flag if the maximum temperature has been reached
void getTemperatureStatus ( void )
{
    static uint16_t uiTemperaturePrevious = 0;

    uint16_t uiDiff               = 0;
    uint16_t uiTemperatureCurrent = 0;

    SPI_Read ( TEMPERATURE );
    sleep_ms ( 10 );

    uiTemperatureCurrent = EnclosureTemperature;

    if ( uiTemperatureCurrent > uiTemperaturePrevious )
    {
        uiDiff = uiTemperatureCurrent - uiTemperaturePrevious;
    }
    else
    {
        uiDiff = uiTemperaturePrevious - uiTemperatureCurrent;
    }

    if ( uiDiff < 10 )
    {
        if ( uiTemperatureCurrent < uiTemperatureCutoffPoint )
        {
            ucHighTemperatureFlag = true;   // Temperature above the cutoff point
        }
        else
        {
            if ( ucHighTemperatureFlag == true )
            {
                // Compensate for hysteresis
                if ( uiTemperatureCurrent > uiTemperatureResetPoint )
                {
                    // Temperature below the reset point
                    ucHighTemperatureFlag = false;
                }
            }
            else
            {
                // Nothing to do
            }
        }
    }
    else
    {
        // Discard sample if change is too large
    }

    uiTemperaturePrevious = uiTemperatureCurrent;
}

void Set_MUX ( uint8_t sensor )
{
    switch ( sensor )
    {
        case 0:
            A2_LOW;
            A1_LOW;
            A0_LOW;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 1:
            A2_LOW;
            A1_LOW;
            A0_HIGH;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 2:
            A2_LOW;
            A1_HIGH;
            A0_LOW;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 3:
            A2_LOW;
            A1_HIGH;
            A0_HIGH;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 4:
            A2_HIGH;
            A1_LOW;
            A0_LOW;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 5:
            A2_LOW;
            A1_LOW;
            A0_LOW;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 6:
            A2_HIGH;
            A1_LOW;
            A0_LOW;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 7:
            A2_HIGH;
            A1_LOW;
            A0_HIGH;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 8:
            A2_HIGH;
            A1_HIGH;
            A0_LOW;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 9:
            A2_HIGH;
            A1_HIGH;
            A0_HIGH;
            EN1_HIGH;
            EN2_LOW;
            EN3_LOW;
        break;

        case 10:
            A2_HIGH;
            A1_LOW;
            A0_HIGH;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 11:
            A2_LOW;
            A1_LOW;
            A0_HIGH;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 12:
            A2_LOW;
            A1_LOW;
            A0_LOW;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 13:
            A2_LOW;
            A1_LOW;
            A0_HIGH;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 14:
            A2_LOW;
            A1_HIGH;
            A0_LOW;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 15:
            A2_LOW;
            A1_HIGH;
            A0_HIGH;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 16:
            A2_HIGH;
            A1_HIGH;
            A0_LOW;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 17:
            A2_LOW;
            A1_HIGH;
            A0_LOW;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 18:
            A2_HIGH;
            A1_LOW;
            A0_LOW;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 19:
            A2_HIGH;
            A1_LOW;
            A0_HIGH;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 20:
            A2_HIGH;
            A1_HIGH;
            A0_LOW;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 21:
            A2_HIGH;
            A1_HIGH;
            A0_HIGH;
            EN1_LOW;
            EN2_HIGH;
            EN3_LOW;
        break;

        case 22:
            A2_HIGH;
            A1_HIGH;
            A0_HIGH;
            EN1_LOW;
            EN2_LOW;
            EN3_HIGH;
        break;

        case 23:
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
void SPI_Read ( uint8_t channel )
{
    if ( channel == TEMPERATURE )
    {
        EnclosureTemperature = 0;
    }
    else if ( channel == DAC )
    {

    }
    else
    {
        // Nothing to do
    }
}

void SPI_Write ( uint8_t buffer [ ] , size_t len )
{

}

void UpdateBaudRate ( uint16_t baudrate )
{
    uart_init ( UART_PC  , baudrate );
    uart_init ( UART_SEN , baudrate );
}

void watchdog ( void )
{
    watchdog_update ( );
}
