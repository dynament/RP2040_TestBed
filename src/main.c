/*
*******************************************************************************
 *  Author:             Craig Hemingway                                       *
 *  Company:            Dynament Ltd.                                         *
 *                      Status Scientific Controls Ltd.                       *
 *  Project :           24-Way Premier IR Sensor Jig                          *
 *  Filename:           main.c                                                *
 *  Date:               30/11/2012                                            *
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
#include <stdio.h>
#include <string.h>
#include <hardware/i2c.h>
#include <hardware/spi.h>
#include <hardware/watchdog.h>
#include <pico/binary_info.h>

struct repeating_timer timer_10ms;
struct repeating_timer timer_getTemp;
struct repeating_timer timer_heartbeat;

// DAC
const    uint8_t  DAC_BUSY           = 0x01;
const    uint8_t  DAC_INITIAL        = 0x02;
const    uint8_t  DAC_CHECK_IN_SPEC  = 0x03;
const    uint8_t  DAC_CALIBRATE      = 0x04;
volatile uint8_t  SensorPass [ 3 ]   = { 0 , 0 , 0 };
uint32_t SensorState = 0;

// I2C
// const    uint8_t  I2C_ADDRESS_MASTER = 0x25;
// const    uint8_t  I2C_ADDRESS_SLAVE  = 0x26;
const    uint16_t I2C_TX_PERIOD      = 1000; // ms

// SPI
const uint8_t SPI_NUM_RX_BYTES = 2;
const uint8_t SYNC_BYTE        = 0x55;
uint8_t SPI_RxBuffer [ SPI_BUFFER_LENGTH ];
uint8_t SPI_TxBuffer [ SPI_BUFFER_LENGTH ];
volatile uint8_t SPI_RxWrite = 0;

// UART
uint8_t  aucRxBufferMaster [ P2P_BUFFER_MASTER_SIZE ]   __attribute__( ( aligned ( 16 ) ) );
uint8_t  aucRxBufferSlave  [ P2P_BUFFER_SLAVE_SIZE  ]   __attribute__( ( aligned ( 16 ) ) );
volatile uint16_t uiRxBufferMasterGet = 0;
volatile uint16_t uiRxBufferMasterPut = 0;
volatile uint16_t uiRxBufferSlaveGet  = 0;
volatile uint16_t uiRxBufferSlavePut  = 0;

// DAC check
const uint8_t ADC_MAX_RETRY           = 5;
const uint8_t DAC_CHECK_IDLE          = 0b00000000;
const uint8_t DAC_CHECK_INITIAL       = 0b00000001;
const uint8_t DAC_CHECK_ZERO          = 0b00000010;
const uint8_t DAC_CHECK_FSD           = 0b00000100;
const uint8_t DAC_CHECK_SPARE         = 0b00001000;
const uint8_t DAC_CHECK_IS_READY      = 0x10;
const uint8_t DAC_CHECK_RUNNING       = 0x0B;
const uint8_t DAC_CHECK_NOT_RUNNING   = 0x0F;
uint8_t DAC_Check_Option = DAC_CHECK_IDLE;

struct DAC_Reading_t
{
    uint16_t DAC_ADC_Initial [ 24 ];
    uint16_t DAC_ADC_Zero    [ 24 ];
    uint16_t DAC_mV_Zero     [ 24 ];
    uint16_t DAC_ADC_FSD     [ 24 ];
    uint16_t DAC_mV_FSD      [ 24 ];
};

struct DAC_Reading_t DAC_Reading;

const uint8_t ACK_String [ ] = { 0x10 , 0x16 };
const uint8_t DAC_Change [ ] = { 0x10 , 0x15 , 0xE5 , 0xA2 , 0x0C , 0x10 , 0x1F , 0x01 , 0xE7 };
const uint8_t DAC_FSD    [ ] = { 0x10 , 0x1A , 0x01 , 0x02 , 0x10 , 0x1F , 0x00 , 0x5C };
const uint8_t DAC_Zero   [ ] = { 0x10 , 0x1A , 0x01 , 0x01 , 0x10 , 0x1F , 0x00 , 0x5B };
const uint8_t NAK_String [ ] = { 0x10 , 0x19 };
const uint8_t RX_ACK   = 1;
const uint8_t RX_ERROR = 9;
const uint8_t RX_NAK   = 2;

bool     b_Flag_GetTemperature    = false;
bool     b_ucHighTemperatureFlag  = false;
bool     b_ucSerialNumberRequest  = false;
uint8_t  countPC                  = 0;
uint8_t  countSensor              = 0;
uint8_t  RX_Status                = 0;
uint8_t  SensorPos                = 0;
uint16_t uiCommsStatus            = 0;
uint16_t uiTemperatureCutoffPoint = 0;
uint16_t uiTemperatureResetPoint  = 0;

volatile bool     b_DAC_Ready = true;
volatile uint8_t  SPI_RxData  = 0;
volatile uint16_t ADC_Timeout = 0;

volatile bool     b_ucFlagJigCommand  = false;
volatile bool     b_ucPassThroughMode = false;
volatile bool     b_ucPcCommsFlag     = false;
volatile bool     b_ucSensorCommsFlag = false;
volatile uint16_t uiCommsTimeout      = 0;
volatile uint16_t uiCommsMode         = COMMS_WAIT;

uint8_t  Check_Response       ( void );
uint16_t ADC_Read             ( uint8_t channel );
void     DAC_Check            ( void );
void     getTemperatureStatus ( void );

// Timer interrupts
bool temperature_5000ms_callback ( struct repeating_timer *t )
{
    b_Flag_GetTemperature = true;
}

bool timer_10ms_callback ( struct repeating_timer *t )
{
    if ( true == b_ucHighTemperatureFlag )
    {
        LED_YELLOW_ON;  // PC Comms led on, high temperature
        LED_RED_ON;     // Sensor comms led on, high temperature
    }
    else
    {
        if ( 1 == b_ucPcCommsFlag )
        {
            LED_YELLOW_ON;  // PC comms data led on
        }
        else
        {
            // Nothing to do
        }
    
        if ( 1 == b_ucSensorCommsFlag )
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
            b_ucPcCommsFlag = false;
            countPC         = 0;
        }
        else
        {
            // Nothing to do
        }
    
        countSensor++;
        if ( countSensor > 19 )
        {
            LED_RED_OFF;    // Sensor comms data led off
            b_ucSensorCommsFlag = false;
            countSensor         = 0;
        }
        else
        {
            // Nothing to do
        }
    }

    if ( 0 < uiCommsTimeout )
    {
        uiCommsTimeout--;
    }
    else
    {
        // Nothing to do
    }

    if ( 0 < ADC_Timeout )
    {
        ADC_Timeout--;
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

// SPI Slave ( RX ) interrupt handler
void on_spi_slave_rx ( )
{
    // Read RX byte
    SPI_RxBuffer [ SPI_RxWrite ] = spi_get_hw ( SPI_SLAVE ) -> dr;

    // Write TX byte
    if ( SYNC_BYTE == SPI_RxBuffer [ SPI_RxWrite ] )
    {
        spi_get_hw ( SPI_SLAVE ) -> dr = SPI_RxBuffer [ SPI_RxWrite ];
    }
    else if ( ( SYNC_BYTE == SPI_RxBuffer [ SPI_RxWrite - 1 ] ) && ( 10 > SPI_RxBuffer [ SPI_RxWrite ] ) )
    {
        DAC_Check_Option = SPI_RxBuffer [ SPI_RxWrite ];    // Button press
        memset ( SPI_RxBuffer , 0 , sizeof ( SPI_RxBuffer ) );
        b_DAC_Ready = false;
        SPI_RxWrite = 0;
    }
    else if ( ( SYNC_BYTE == SPI_RxBuffer [ SPI_RxWrite - 1 ] ) && ( DAC_CHECK_IS_READY == SPI_RxBuffer [ SPI_RxWrite ] ) ) // UI polling test bed
    {
        spi_get_hw ( SPI_SLAVE ) -> dr = SPI_RxBuffer [ SPI_RxWrite ];
    }
    else if ( ( SYNC_BYTE == SPI_RxBuffer [ SPI_RxWrite - 2 ] ) && ( DAC_CHECK_IS_READY == SPI_RxBuffer [ SPI_RxWrite - 1 ] ) ) // UI polling test bed
    {
        if ( b_DAC_Ready )
        {
            spi_get_hw ( SPI_SLAVE ) -> dr = DAC_CHECK_NOT_RUNNING;
        }
        else
        {
            spi_get_hw ( SPI_SLAVE ) -> dr = DAC_CHECK_RUNNING;
        }
    }
    else if ( ( SYNC_BYTE == SPI_RxBuffer [ SPI_RxWrite - 3 ] ) && ( DAC_CHECK_IS_READY == SPI_RxBuffer [ SPI_RxWrite - 2 ] ) )
    {
        spi_get_hw ( SPI_SLAVE ) -> dr = SensorPass [ 0 ];  // Send sensor data ( 24 to 17 )
    }
    else if ( ( SYNC_BYTE == SPI_RxBuffer [ SPI_RxWrite - 4 ] ) && ( DAC_CHECK_IS_READY == SPI_RxBuffer [ SPI_RxWrite - 3 ] ) )
    {
        spi_get_hw ( SPI_SLAVE ) -> dr = SensorPass [ 1 ];  // Send senor data ( 16 to 9 )
    }
    else if ( ( SYNC_BYTE == SPI_RxBuffer [ SPI_RxWrite - 5 ] ) && ( DAC_CHECK_IS_READY == SPI_RxBuffer [ SPI_RxWrite - 4 ] ) )
    {
        spi_get_hw ( SPI_SLAVE ) -> dr = SensorPass [ 2 ];  // Send sensor data ( 8 to 1 )
        memset ( SPI_RxBuffer , 0 , sizeof ( SPI_RxBuffer ) );
        SPI_RxWrite = 0;
    }

    SPI_RxWrite++;

    if ( SPI_RxWrite == SPI_BUFFER_LENGTH )
    {
        memset ( SPI_RxBuffer , 0 , sizeof ( SPI_RxBuffer ) );
        SPI_RxWrite = 0;
    }
}

// UART_RX interrupt handler
void on_uart_rx ( )
{
    if ( uart_is_readable ( UART_PC ) )
    {
        b_ucPcCommsFlag = true;
        aucRxBufferMaster [ uiRxBufferMasterPut ] = uart_getc ( UART_PC );  // Store character in buffer
        uiRxBufferMasterPut++;

        if ( P2P_BUFFER_MASTER_SIZE == uiRxBufferMasterPut )
        {
            uiRxBufferMasterPut = 0; // At end wrap to beginning
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
        b_ucSensorCommsFlag = true;
        aucRxBufferSlave [ uiRxBufferSlavePut++ ] = uart_getc ( UART_SEN ); // Store character in buffer

        if ( P2P_BUFFER_SLAVE_SIZE == uiRxBufferSlavePut )
        {
            uiRxBufferSlavePut = 0; // At end wrap to beginning
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
    uint8_t SPI_RxRead = 0;

    // Useful information for picotool
    bi_decl ( bi_program_description ( "RP2040 Premier" ) );

    // Initialise standard stdio types
    stdio_init_all ( );

    // Set up watchdog
    watchdog_enable ( WATCHDOG_MILLISECONDS , 1 );

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
    add_repeating_timer_ms ( 5000 , temperature_5000ms_callback    , NULL , &timer_getTemp   );
    add_repeating_timer_ms (   10 , timer_10ms_callback            , NULL , &timer_10ms      );
    add_repeating_timer_ms (  500 , timer_heartbeat_500ms_callback , NULL , &timer_heartbeat );

    // Set up I2C Master ( ADC )
    i2c_init           ( I2C_ADC , I2C_BAUD_RATE * 1000 );
    i2c_set_slave_mode ( I2C_ADC , false , 0 );
    gpio_set_function  ( I2C_ADC_SCL_PIN , GPIO_FUNC_I2C );
    gpio_set_function  ( I2C_ADC_SDA_PIN , GPIO_FUNC_I2C );
    gpio_pull_up       ( I2C_ADC_SCL_PIN );
    gpio_pull_up       ( I2C_ADC_SDA_PIN );

    // Set up SPI Slave ( UI )
    spi_init          ( SPI_SLAVE    , SPI_BAUD_RATE * 1000 );
    spi_set_slave     ( SPI_SLAVE    , true );
    gpio_set_function ( SPI_CS_PIN   , GPIO_FUNC_SPI );
    gpio_set_function ( SPI_MISO_PIN , GPIO_FUNC_SPI );
    gpio_set_function ( SPI_MOSI_PIN , GPIO_FUNC_SPI );
    gpio_set_function ( SPI_SCK_PIN  , GPIO_FUNC_SPI );
    gpio_set_pulls    ( SPI_MISO_PIN , false , true  );
    // Set up SPI_RX interrupt
    spi_get_hw ( SPI_SLAVE ) -> imsc = SPI_SSPIMSC_RXIM_BITS;    // Enable auto RX
    irq_set_exclusive_handler ( SPI0_IRQ , on_spi_slave_rx );
    irq_set_enabled           ( SPI0_IRQ , true            );

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

    b_ucHighTemperatureFlag = false;
    b_ucPassThroughMode     = false;
    b_ucPcCommsFlag         = false;
    b_ucSensorCommsFlag     = false;
    b_ucSerialNumberRequest = false;

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

    // Infinite loop
    for ( ; ; )
    {
        watchdog ( );

        if ( 0 != DAC_Check_Option )
        {
            DAC_Check ( );
        }

        if ( true == b_Flag_GetTemperature )
        {
            getTemperatureStatus ( );
            b_Flag_GetTemperature = false;
        }
        else
        {
            // Nothing to do
        }

        if ( false == b_ucHighTemperatureFlag )
        {
            RELAY_ON;   // Turn on power to sensors

            b_ucFlagJigCommand = false;
            uiCommsStatus    = p2pPollMaster ( );
            
            if ( uiCommsStatus )
            {
                if ( COMMS_WRITE == uiCommsMode )
                {
                    if ( true == b_ucPassThroughMode )
                    {
                        b_ucPassThroughMode = false;
                        uiCommsMode         = COMMS_WAIT;
                        uiCommsStatus       = p2pPollSlaveWritePassThrough ( );
                    }
                    else if ( true == b_ucSerialNumberRequest )
                    {
                        b_ucSerialNumberRequest = false;
                        uiCommsMode             = COMMS_WAIT;
                    }
                    else
                    {
                        uiCommsStatus = p2pPollSlaveWrite ( );
                    }
                }
                else
                {
                    if ( true == b_ucSerialNumberRequest )
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

    uiTemperatureCurrent = ADC_Read ( TEMPERATURE );

    if ( uiTemperaturePrevious < uiTemperatureCurrent )
    {
        uiDiff = uiTemperatureCurrent - uiTemperaturePrevious;
    }
    else
    {
        uiDiff = uiTemperaturePrevious - uiTemperatureCurrent;
    }

    if ( 5000 > uiDiff )
    {
        if ( uiTemperatureCutoffPoint > uiTemperatureCurrent )
        {
            b_ucHighTemperatureFlag = true;   // Temperature above the cutoff point
        }
        else
        {
            if ( true == b_ucHighTemperatureFlag )
            {
                // Compensate for hysteresis
                if ( uiTemperatureResetPoint < uiTemperatureCurrent )
                {
                    // Temperature below the reset point
                    b_ucHighTemperatureFlag = false;
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

    for ( uiRxBufferSlaveGet = 0 ; uiRxBufferSlaveGet < uiRxBufferSlavePut ; uiRxBufferSlaveGet++ )
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
    uint8_t Retry = 0;

    uiRxBufferSlaveGet = 0;
    uiRxBufferSlavePut = 0;

    if ( DAC_CHECK_INITIAL == DAC_Check_Option )   // Initial Voltage present test
    {
        memset ( DAC_Reading.DAC_ADC_Initial , 0 , sizeof ( DAC_Reading.DAC_ADC_Initial) );
        SensorState = 0;

        for ( SensorPos = 0 ; SensorPos < 24 ; SensorPos++ )
        {
            watchdog ( );
            Set_MUX ( SensorPos );
            sleep_ms ( 10 );
            Retry = 0;

            do
            {
                DAC_Reading.DAC_ADC_Initial [ SensorPos ] = ADC_Read ( DAC );
                if ( 0x8000 <= DAC_Reading.DAC_ADC_Initial [ SensorPos ] )
                {
                    // Negative value ( sensor faulty or position not populated )
                    DAC_Reading.DAC_ADC_Initial [ SensorPos ] = 0;
                }
                sleep_ms ( 100 );
                Retry++;
            } while ( ( 0 == DAC_Reading.DAC_ADC_Initial [ SensorPos ] ) && ( ADC_MAX_RETRY > Retry ) );

            if ( 100 < DAC_Reading.DAC_ADC_Initial [ SensorPos ] )
            {
                SensorState |= 1 << SensorPos;
            }
            else
            {
                SensorState &= ~( 1 << SensorPos );
            }

            SensorPass [ 0 ] = ( uint8_t ) ( SensorState >> 16 );
            SensorPass [ 1 ] = ( uint8_t ) ( SensorState >>  8 );
            SensorPass [ 2 ] = ( uint8_t ) ( SensorState       );
        }
    }

    else if ( DAC_CHECK_ZERO == DAC_Check_Option )
    {

    }
    
    else if ( DAC_CHECK_FSD == DAC_Check_Option )
    {

    }

    else    // Invalid option
    {
        // Nothing to do
    }

    DAC_Check_Option = 0;   // Reset button press data

    b_DAC_Ready = true;
/*
    for ( SensorPos = 0 ; SensorPos < 24 ; SensorPos++ )
    {
        watchdog ( );
        uiCommsTimeout = 500;
        RX_Status = 0;
        Set_MUX ( SensorPos );
        sleep_ms ( 10 );
        uart_write_blocking ( UART_SEN , DAC_Change , 9 );
        while ( ( ( uiRxBufferSlaveGet + 2 ) > uiRxBufferSlavePut ) && uiCommsTimeout ); //  Wait for ACK / NAK
        RX_Status = Check_Response ( );

        if ( ( RX_ACK == RX_Status ) && uiCommsTimeout )
        {
            uart_write_blocking ( UART_SEN , DAC_Zero , 8 );
            while ( ( ( uiRxBufferSlaveGet + 2 ) > uiRxBufferSlavePut ) && uiCommsTimeout ); //  Wait for ACK / NAK
            RX_Status = Check_Response ( );

            if ( ( RX_ACK == RX_Status ) && uiCommsTimeout )
            {
                sleep_ms ( 100 );
                DAC_Reading.DAC_ADC_Zero [ SensorPos ] = ADC_Read ( DAC );
                sleep_ms ( 100 );
                DAC_Reading.DAC_ADC_Zero [ SensorPos + 24 ] = ADC_Read ( DAC );
                sleep_ms ( 100 );
                DAC_Reading.DAC_ADC_Zero [ SensorPos + 48 ] = ADC_Read ( DAC );
                DAC_Reading.DAC_ADC_Zero [ SensorPos ] = ( uint16_t ) ( ( DAC_Reading.DAC_ADC_Zero [ SensorPos ] + DAC_Reading.DAC_ADC_Zero [ SensorPos + 24 ] + DAC_Reading.DAC_ADC_Zero [ SensorPos + 48 ] ) /3 );
                DAC_Reading.DAC_mV_Zero  [ SensorPos ] = ( uint16_t ) ( DAC_Reading.DAC_ADC_Zero [ SensorPos ] * DAC_ZERO_ADC_OFFSET_0V400 );
                uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                while ( ( ( uiRxBufferSlaveGet + 2 ) > uiRxBufferSlavePut ) && uiCommsTimeout ); // Wait for ACK / NAK
                RX_Status = Check_Response ( );

                if ( ( RX_ACK == RX_Status ) && uiCommsTimeout )
                {
                    uart_write_blocking ( UART_SEN , DAC_FSD , 8 );
                    while ( ( ( uiRxBufferSlaveGet + 2 ) > uiRxBufferSlavePut ) && uiCommsTimeout ); // Wait for ACK / NAK
                    RX_Status = Check_Response ( );

                    if ( ( RX_ACK == RX_Status ) && uiCommsTimeout )
                    {
                        sleep_ms ( 100 );
                        DAC_Reading.DAC_ADC_FSD [ SensorPos ] = ADC_Read ( DAC );
                        sleep_ms ( 100 );
                        DAC_Reading.DAC_ADC_FSD [ SensorPos + 24 ] = ADC_Read ( DAC );
                        sleep_ms ( 100 );
                        DAC_Reading.DAC_ADC_FSD [ SensorPos + 48 ] = ADC_Read ( DAC );
                        DAC_Reading.DAC_ADC_FSD [ SensorPos ] = ( uint16_t ) ( ( DAC_Reading.DAC_ADC_FSD [ SensorPos ] + DAC_Reading.DAC_ADC_FSD [ SensorPos + 24 ] + DAC_Reading.DAC_ADC_FSD [ SensorPos + 48 ] ) / 3 );
                        DAC_Reading.DAC_mV_FSD  [ SensorPos ] = ( uint16_t ) ( DAC_Reading.DAC_ADC_FSD [ SensorPos ] * DAC_FSD_ADC_OFFSET_2V000 );
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
    sleep_ms ( 10 );
    */
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

uint16_t ADC_Read ( uint8_t channel )
{
    uint8_t  ADC_Buffer [ 10 ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };
    uint8_t  Byte_HI   = 0;
    uint8_t  Byte_LO   = 0;
    uint16_t Value_ADC = 0;

    if ( TEMPERATURE == channel )
    {
        ADC_Buffer [ 0 ] = ADC_TEMP_CHANNEL;
        i2c_write_blocking ( I2C_ADC , 0b01101000 , ADC_Buffer , 1 , 0 );
        sleep_us ( 100 );

        ADC_Timeout = 100;

        do
        {
            i2c_read_blocking  ( I2C_ADC , 0b01101000 , ADC_Buffer , 3 , 0 );
            sleep_us ( 10 );
        } while ( ADC_Timeout && ( ADC_Buffer [ 2 ] & ( 1 << 7 ) ) );

        Value_ADC = ( uint16_t ) ( ( ADC_Buffer [ 0 ] << 8 ) + ADC_Buffer [ 1 ] );
    }
    else if ( DAC == channel )
    {
        ADC_Buffer [ 0 ] = ADC_DAC_CHANNEL;
        i2c_write_blocking ( I2C_ADC , 0b01101000 , ADC_Buffer , 1 , 0 );
        sleep_us ( 100 );

        ADC_Timeout = 100;

        do
        {
            i2c_read_blocking  ( I2C_ADC , 0b01101000 , ADC_Buffer , 3 , 0 );
            sleep_us ( 10 );
        } while ( ADC_Timeout && ( ADC_Buffer [ 2 ] & ( 1 << 7 ) ) );

        Value_ADC = ( uint16_t ) ( ( ADC_Buffer [ 0 ] << 8 ) + ADC_Buffer [ 1 ] );
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
