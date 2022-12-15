/*
*******************************************************************************
 *  Author:             Craig Hemingway                                       *
 *  Company:            Dynament Ltd.                                         *
 *                      Status Scientific Controls Ltd.                       *
 *  Project :           24-Way Premier IR Sensor Jig                          *
 *  Filename:   		main.c                                                *
 *  Date:		        30/11/2012                                            *
 *  File Version:   	4.0.0                                                 *
 *  Version history:    4.0.0 - 30/11/2022 - Craig Hemingway                  *
 *                          PIC code ported over to RP2040                    *
 *                          Additional DAC check & adjust features            *
 *                      3.0.0 - 27/01/2014 - Frank Kups                       *
 *                          Latest program for sensor jig Version 4           *
 *  Tools Used: Visual Studio Code -> 1.73.1                                  *
 *              Compiler           -> GCC 11.3.1 arm-none-eabi                *
 *                                                                            *
 ******************************************************************************
*/

#include <main.h>

#include <comms.h>
#include <string.h>
#include <hardware/spi.h>
#include <hardware/watchdog.h>
#include <pico/binary_info.h>

struct repeating_timer timer_10ms;
struct repeating_timer timer_getTemp;
struct repeating_timer timer_heartbeat;

// UART
uint8_t  aucRxBufferMaster [ P2P_BUFFER_MASTER_SIZE ]   __attribute__( ( aligned ( 16 ) ) );
uint8_t  aucRxBufferSlave  [ P2P_BUFFER_SLAVE_SIZE  ]   __attribute__( ( aligned ( 16 ) ) );
volatile uint16_t uiRxBufferMasterGet = 0;
volatile uint16_t uiRxBufferMasterPut = 0;
volatile uint16_t uiRxBufferSlaveGet  = 0;
volatile uint16_t uiRxBufferSlavePut  = 0;

// SPI
uint8_t  SPI_RxBuffer [ SPI_BUFFER_LENGTH ];
uint8_t  SPI_TxBuffer [ SPI_BUFFER_LENGTH ];

// DAC check
uint8_t  SensorPos = 0;
struct DAC_Reading_t
{
    uint16_t DAC_ADC_Initial [ 24 ];
    uint16_t DAC_mV_Initial  [ 24 ];
    uint16_t DAC_ADC_Zero    [ 24 ];
    uint16_t DAC_mV_Zero     [ 24 ];
    uint16_t DAC_ADC_FSD     [ 24 ];
    uint16_t DAC_mV_FSD      [ 24 ];
};

struct DAC_Reading_t DAC_Reading;

const uint8_t ACK_String [ ] = { 0x10 , 0x16 };
const uint8_t NAK_String [ ] = { 0x10 , 0x19 };
const uint8_t DAC_Change [ ] = { 0x10 , 0x15 , 0xE5 , 0xA2 , 0x0C , 0x10 , 0x1F , 0x01 , 0xE7 };
const uint8_t DAC_Zero   [ ] = { 0x10 , 0x1A , 0x01 , 0x01 , 0x10 , 0x1F , 0x00 , 0x5B };
const uint8_t DAC_FSD    [ ] = { 0x10 , 0x1A , 0x01 , 0x02 , 0x10 , 0x1F , 0x00 , 0x5C };
uint8_t Sensor_RxBuffer [ 100 ];
uint8_t Sensor_TxBuffer [ 100 ];
uint8_t Length = 0;
uint8_t RX_Status = 0;
const uint8_t RX_ACK = 1;
const uint8_t RX_NAK = 2;
const uint8_t RX_ERROR = 9;
uint8_t Check_Response ( void );

uint8_t  countPC                  = 0;
uint8_t  countSensor              = 0;
uint8_t  Flag_GetTemperature      = false;
uint8_t  ucHighTemperatureFlag    = false;
volatile uint8_t  ucPcCommsFlag            = 0;
volatile uint8_t  ucSensorCommsFlag        = 0;
uint8_t  ucSerialNumberRequest    = false;
uint16_t uiCommsStatus            = 0;
uint16_t uiTemperatureCutoffPoint = 0;
uint16_t uiTemperatureResetPoint  = 0;

volatile uint16_t uiCommsTimeout  = 0;

volatile uint8_t  ucFlagJigCommand  = false;
volatile uint8_t  ucPassThroughMode = false;
volatile uint16_t uiCommsMode       = COMMS_WAIT;

uint16_t SPI_Read             ( uint8_t channel );
void     DAC_Check            ( void );
void     getTemperatureStatus ( void );

// Timer interrupts
bool temperature_5000ms_callback ( struct repeating_timer *t )
{
    Flag_GetTemperature = true;
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

bool timer_heartbeat_500ms_callback ( struct repeating_timer *t )
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
        ucPcCommsFlag = 1;
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
        ucSensorCommsFlag = 1;
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
    // Useful information for picotool
    bi_decl ( bi_program_description ( "RP2040 Premier" ) );

    // Initialise standard stdio types
    stdio_init_all ( );

    // Set up watchdog
    // watchdog_enable ( WATCHDOG_MILLISECONDS , 1 );

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
    gpio_init    ( SPI_CS         );
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
    gpio_set_dir ( SPI_CS         , GPIO_OUT );

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
    SPI_CS_HIGH;

    // Set up timer interrupts
    add_repeating_timer_ms ( 5000 , temperature_5000ms_callback    , NULL , &timer_getTemp   );
    add_repeating_timer_ms (   10 , timer_10ms_callback            , NULL , &timer_10ms      );
    add_repeating_timer_ms (  500 , timer_heartbeat_500ms_callback , NULL , &timer_heartbeat );

    // Set up SPI
    spi_init          ( SPI_ID   , SPI_BAUD_RATE );
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

    ucHighTemperatureFlag = false;
    ucPassThroughMode     = false ;
    ucPcCommsFlag         = 0;
    ucSensorCommsFlag     = 0;
    ucSerialNumberRequest = false;

    // Turn on power to sensors
    RELAY_ON;

    uiCommsTimeout = 500;   // 5 seconds

    while ( uiCommsTimeout );

    // Clear comms buffer on spurious characters
    uiRxBufferMasterGet = 0;
    uiRxBufferMasterPut = 0;
    uiRxBufferSlaveGet  = 0;
    uiRxBufferSlavePut  = 0;

    // Get temperature limits
    uiTemperatureCutoffPoint = ( uint16_t ) DEG90;
    uiTemperatureResetPoint  = ( uint16_t ) DEG82;

    // TEST USE ONLY
    gpio_init    ( 28 );
    gpio_set_dir ( 28 , GPIO_IN  );

    // Infinite loop
    for ( ; ; )
    {
        watchdog ( );

        // if ( gpio_get ( 28 ) )
        // {
        //     DAC_Check ( );
        // }
        // else
        // {
        //     // Nothing to do
        // }

        if ( true == Flag_GetTemperature )
        {
            getTemperatureStatus ( );
            Flag_GetTemperature = false;
        }
        else
        {
            // Nothing to do
        }

        if ( false == ucHighTemperatureFlag )
        {
            RELAY_ON;   // Turn on power to sensors

            ucFlagJigCommand = false;
            uiCommsStatus    = p2pPollMaster ( );
            
            if ( uiCommsStatus )
            {
                if ( COMMS_WRITE == uiCommsMode )
                {
                    if ( true == ucPassThroughMode )
                    {
                        ucPassThroughMode = false;
                        uiCommsMode       = COMMS_WAIT;
                        uiCommsStatus     = p2pPollSlaveWritePassThrough ( );
                    }
                    else if ( true == ucSerialNumberRequest )
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
                    if ( true == ucSerialNumberRequest )
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

    uiTemperatureCurrent = SPI_Read ( TEMPERATURE );

    if ( uiTemperaturePrevious < uiTemperatureCurrent )
    {
        uiDiff = uiTemperatureCurrent - uiTemperaturePrevious;
    }
    else
    {
        uiDiff = uiTemperaturePrevious - uiTemperatureCurrent;
    }

    if ( 10 > uiDiff )
    {
        if ( uiTemperatureCutoffPoint > uiTemperatureCurrent )
        {
            ucHighTemperatureFlag = true;   // Temperature above the cutoff point
        }
        else
        {
            if ( true == ucHighTemperatureFlag )
            {
                // Compensate for hysteresis
                if ( uiTemperatureResetPoint < uiTemperatureCurrent )
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

uint8_t Check_Response ( void )
{
    static uint16_t uiRxBufferSlavePrevious = 0;
    uint8_t result = 0;

    for ( uiRxBufferSlaveGet = uiRxBufferSlavePrevious ; uiRxBufferSlaveGet < uiRxBufferSlavePut ; uiRxBufferSlaveGet++ )
    {
        if ( 0x10 == ( aucRxBufferSlave [ uiRxBufferSlaveGet ] ) )
        {
            break;
        }
        else
        {
            // Keep looking
        }
    }

    if ( 0 == ( memcmp ( aucRxBufferSlave + uiRxBufferSlaveGet , ACK_String , 2 ) ) )
    {
        uiRxBufferSlaveGet = uiRxBufferSlavePut;
        result = RX_ACK;
    }
    else if ( 0 == ( memcmp ( aucRxBufferSlave + uiRxBufferSlaveGet , NAK_String , 2 ) ) )
    {
        uiRxBufferSlaveGet = uiRxBufferSlavePut;
        result = RX_NAK;
    }
    else
    {
        result = RX_ERROR;   // Unexpected or no response
    }

    uiRxBufferSlavePrevious = uiRxBufferSlavePut;
    return result;
}

void DAC_Check ( void )
{
    uiRxBufferSlaveGet = 0;
    uiRxBufferSlavePut = 0;

    for ( SensorPos = 0 ; SensorPos < 24 ; SensorPos++ )
    {
        watchdog ( );
        uiCommsTimeout = 500;
        RX_Status = 0;
        Set_MUX ( SensorPos );
        sleep_ms ( 10 );
        DAC_Reading.DAC_ADC_Initial [ SensorPos ] = SPI_Read ( DAC );
        DAC_Reading.DAC_mV_Initial  [ SensorPos ] = ( uint16_t ) ( DAC_Reading.DAC_ADC_Initial [ SensorPos ] /* * ADC_OFFSET */ );
        uart_write_blocking ( UART_SEN , DAC_Change , 9 );
        while ( ( ( uiRxBufferSlaveGet + 2 ) > uiRxBufferSlavePut ) && uiCommsTimeout ); //  Wait for ACK / NAK
        RX_Status = Check_Response ( );

        if ( RX_ACK == RX_Status )
        {
            uart_write_blocking ( UART_SEN , DAC_Zero , 8 );
            while ( ( ( uiRxBufferSlaveGet + 2 ) > uiRxBufferSlavePut ) && uiCommsTimeout ); //  Wait for ACK / NAK
            RX_Status = Check_Response ( );

            if ( RX_ACK == RX_Status )
            {
                sleep_ms ( 10 );
                DAC_Reading.DAC_ADC_Zero [ SensorPos ] = SPI_Read ( DAC );
                DAC_Reading.DAC_mV_Zero  [ SensorPos ] = ( uint16_t ) ( DAC_Reading.DAC_ADC_Zero [ SensorPos ] * DAC_ZERO_ADC_OFFSET );
                uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                while ( ( ( uiRxBufferSlaveGet + 2 ) > uiRxBufferSlavePut ) && uiCommsTimeout ); // Wait for ACK / NAK
                RX_Status = Check_Response ( );

                if ( RX_ACK == RX_Status )
                {
                    uart_write_blocking ( UART_SEN , DAC_FSD , 8 );
                    while ( ( ( uiRxBufferSlaveGet + 2 ) > uiRxBufferSlavePut ) && uiCommsTimeout ); // Wait for ACK / NAK
                    RX_Status = Check_Response ( );

                    if ( RX_ACK == RX_Status )
                    {
                        sleep_ms ( 10 );
                        DAC_Reading.DAC_ADC_FSD [ SensorPos ] = SPI_Read ( DAC );
                        DAC_Reading.DAC_mV_FSD  [ SensorPos ] = ( uint16_t ) ( DAC_Reading.DAC_ADC_FSD [ SensorPos ] * DAC_FSD_ADC_OFFSET );
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
                // Nothing to do
            }
        }
        else
        {
            // Nothing to do
        }

        sleep_ms ( 10 );
    }
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
uint16_t SPI_Read ( uint8_t channel )
{
    uint8_t  Byte_HI = 0;
    uint8_t  Byte_LO = 0;
    uint16_t Value_ADC = 0;

    if ( TEMPERATURE == channel )
    {
        SPI_TxBuffer [ 0 ] = ADC_TEMP_CHANNEL;
        SPI_CS_LOW;
        spi_write_read_blocking ( SPI_ID , SPI_TxBuffer , SPI_RxBuffer , 4 );
        SPI_CS_HIGH;

        Byte_LO   = ( uint8_t  ) ( ( SPI_RxBuffer [ 2 ] >> 1 ) + ( ( SPI_RxBuffer [ 1 ] ) << 7 ) );
        Byte_HI   = ( uint8_t  ) ( ( SPI_RxBuffer [ 1 ] & 0b00011110 ) >> 1 );
        Value_ADC = ( uint16_t ) ( ( Byte_HI << 8 ) + Byte_LO );
    }
    else if ( DAC == channel )
    {
        sleep_ms ( 10 );
        SPI_TxBuffer [ 0 ] = ADC_DAC_CHANNEL;
        SPI_CS_LOW;
        spi_write_read_blocking ( SPI_ID , SPI_TxBuffer , SPI_RxBuffer , 4 );
        SPI_CS_HIGH;

        Byte_LO   = ( uint8_t  ) ( ( SPI_RxBuffer [ 2 ] >> 1 ) + ( ( SPI_RxBuffer [ 1 ] ) << 7 ) );
        Byte_HI   = ( uint8_t  ) ( ( SPI_RxBuffer [ 1 ] & 0b00011110 ) >> 1 );
        Value_ADC = ( uint16_t ) ( ( Byte_HI << 8 ) + Byte_LO );
    }
    else
    {
        // Nothing to do
    }

    return Value_ADC;
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

/*** end of file ***/
