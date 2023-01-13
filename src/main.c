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

struct repeating_timer timer_counter;
struct repeating_timer timer_getTemp;
struct repeating_timer timer_heartbeat;

// DAC
const    uint8_t  TOLERANCE          = 10;  // mV
const    uint8_t  DAC_BUSY           = 0x01;
const    uint8_t  DAC_INITIAL        = 0x02;
const    uint8_t  DAC_CHECK_IN_SPEC  = 0x03;
const    uint8_t  DAC_CALIBRATE      = 0x04;
uint8_t  SensorPass [ 4 ] = { 0 , 0 , 0 , 0 };

// I2C
const uint16_t I2C_TX_PERIOD = 1000;    // ms

// SPI
const uint8_t SPI_NUM_RX_BYTES = 2;
const uint8_t SYNC_BYTE        = 0x55;
uint8_t SPI_RxBuffer [ SPI_BUFFER_LENGTH ];
uint8_t SPI_TxBuffer [ SPI_BUFFER_LENGTH ];

// UART
uint8_t  g_aucRxBufferMaster [ P2P_BUFFER_MASTER_SIZE ]   __attribute__( ( aligned ( 16 ) ) );
uint8_t  g_aucRxBufferSlave  [ P2P_BUFFER_SLAVE_SIZE  ]   __attribute__( ( aligned ( 16 ) ) );
volatile uint16_t g_uiRxBufferMasterGet = 0;
volatile uint16_t g_uiRxBufferMasterPut = 0;
volatile uint16_t g_uiRxBufferSlaveGet  = 0;
volatile uint16_t g_uiRxBufferSlavePut  = 0;

// DAC check
const uint8_t ADC_MAX_RETRY                = 5;
const uint8_t DAC_CHECK_IDLE               = 0b00000000;
const uint8_t DAC_CHECK_INITIAL            = 0b00000001;
const uint8_t DAC_CHECK_CALIBRATION        = 0b00000010;
const uint8_t DAC_CHECK_ADJUST_CALIBRATION = 0b00000100;
const uint8_t DAC_CHECK_SPARE              = 0b00001000;
const uint8_t DAC_CHECK_IS_READY           = 0x10;
const uint8_t DAC_CHECK_RUNNING            = 0x0B;
const uint8_t DAC_CHECK_NOT_RUNNING        = 0x0F;
const uint8_t DAC_ERROR_RANGE_MV           = 10;
uint8_t g_DAC_Check_Option = DAC_CHECK_IDLE;

union DAC_Settings
{
    float    VarFloat;
    uint32_t VarInt;
};

struct DAC_Reading_t
{
    uint16_t DAC_ADC_Initial [ 24 ];
    uint16_t DAC_ADC_Zero    [ 24 ];
    uint16_t DAC_mV_Zero     [ 24 ];
    uint16_t DAC_ADC_FSD     [ 24 ];
    uint16_t DAC_mV_FSD      [ 24 ];
    uint16_t DAC_ADC_5000    [ 24 ];
    uint16_t DAC_mV_5000     [ 24 ];
    uint16_t DAC_ADC_60000   [ 24 ];
    uint16_t DAC_mV_60000    [ 24 ];
};

struct DAC_Reading_t DAC_Reading;

const uint8_t ACK_String              [ ] = { 0x10 , 0x16 };
const uint8_t DAC_5000                [ ] = { 0x10 , 0x1A , 0x01 , 0x03 , 0x10 , 0x1F , 0x00 , 0x5D };
const uint8_t DAC_60000               [ ] = { 0x10 , 0x1A , 0x01 , 0x04 , 0x10 , 0x1F , 0x00 , 0x5E };
const uint8_t DAC_Change              [ ] = { 0x10 , 0x15 , 0xE5 , 0xA2 , 0x0C , 0x10 , 0x1F , 0x01 , 0xE7 };
const uint8_t DAC_ChangeSettings      [ ] = { 0x10 , 0x15 , 0xE5 , 0xA2 , 0x2E , 0x10 , 0x1F , 0x02 , 0x09 };
const uint8_t DAC_FSD                 [ ] = { 0x10 , 0x1A , 0x01 , 0x02 , 0x10 , 0x1F , 0x00 , 0x5C };
const uint8_t DAC_ReadConfig          [ ] = { 0x10 , 0x13 , 0x00 , 0x10 , 0x1F , 0x00 , 0x52 };
const uint8_t DAC_ReadExtendedConfig  [ ] = { 0x10 , 0x13 , 0x2E , 0x10 , 0x1F , 0x00 , 0x80 };
const uint8_t DAC_VMON_4V             [ ] = { 0x10 , 0x1A , 0x01 , 0x06 , 0x10 , 0x1F , 0x00 , 0x60 };
const uint8_t DAC_Zero                [ ] = { 0x10 , 0x1A , 0x01 , 0x01 , 0x10 , 0x1F , 0x00 , 0x5B };
const uint8_t NAK_String              [ ] = { 0x10 , 0x19 };
const uint8_t RX_ACK   = 1;
const uint8_t RX_ERROR = 9;
const uint8_t RX_NAK   = 2;

bool     g_b_Flag_GetTemperature    = false;
bool     g_b_ucHighTemperatureFlag  = false;
uint16_t g_uiTemperatureCutoffPoint = 0;
uint16_t g_uiTemperatureResetPoint  = 0;

volatile bool     b_DAC_Ready = true;
volatile uint8_t  SPI_RxData  = 0;
volatile uint16_t ADC_Timeout = 0;

volatile bool     g_b_ucFlagJigCommand  = false;
volatile bool     g_b_ucPassThroughMode = false;
volatile bool     g_b_ucPcCommsFlag     = false;
volatile bool     g_b_ucSensorCommsFlag = false;
volatile uint16_t g_uiCommsTimeout      = 0;
volatile uint16_t g_uiCommsMode         = COMMS_WAIT;

uint8_t  Check_Response       ( void );
uint16_t ADC_Read             ( uint8_t channel );
void     DAC_Check            ( void );
void     getTemperatureStatus ( void );

// Timer interrupts
bool get_temperature_callback ( struct repeating_timer *t )
{
    g_b_Flag_GetTemperature = true;
}

bool timer_counter_callback ( struct repeating_timer *t )
{
    static uint8_t countPC     = 0;
    static uint8_t countSensor = 0;

    if ( true == g_b_ucHighTemperatureFlag )
    {
        LED_YELLOW_ON;  // PC Comms led on, high temperature
        LED_RED_ON;     // Sensor comms led on, high temperature
    }
    else
    {
        if ( 1 == g_b_ucPcCommsFlag )
        {
            LED_YELLOW_ON;  // PC comms data led on
        }
        else
        {
            // Nothing to do
        }
    
        if ( 1 == g_b_ucSensorCommsFlag )
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
            g_b_ucPcCommsFlag = false;
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
            g_b_ucSensorCommsFlag = false;
            countSensor         = 0;
        }
        else
        {
            // Nothing to do
        }
    }

    if ( 0 < g_uiCommsTimeout )
    {
        g_uiCommsTimeout--;
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

bool timer_heartbeat_callback ( struct repeating_timer *t )
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
    static uint8_t SPI_RxWrite = 0;

    // Read RX byte
    SPI_RxBuffer [ SPI_RxWrite ] = spi_get_hw ( SPI_SLAVE ) -> dr;

    // Write TX byte
    if ( SYNC_BYTE == SPI_RxBuffer [ SPI_RxWrite ] )
    {
        spi_get_hw ( SPI_SLAVE ) -> dr = SPI_RxBuffer [ SPI_RxWrite ];
    }
    else if ( ( SYNC_BYTE == SPI_RxBuffer [ SPI_RxWrite - 1 ] ) && ( 10 > SPI_RxBuffer [ SPI_RxWrite ] ) )
    {
        g_DAC_Check_Option = SPI_RxBuffer [ SPI_RxWrite ];    // Button press
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
        // memset ( SPI_RxBuffer , 0 , sizeof ( SPI_RxBuffer ) );
        // SPI_RxWrite = 0;
    }
    else if ( ( SYNC_BYTE == SPI_RxBuffer [ SPI_RxWrite - 6 ] ) && ( DAC_CHECK_IS_READY == SPI_RxBuffer [ SPI_RxWrite - 5 ] ) )
    {
        spi_get_hw ( SPI_SLAVE ) -> dr = SensorPass [ 3 ];  // Send position of sensor under test
        memset ( SPI_RxBuffer , 0 , sizeof ( SPI_RxBuffer ) );
        SPI_RxWrite = 0;
    }
    else
    {
        // Nothing to do
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
        g_b_ucPcCommsFlag = true;
        g_aucRxBufferMaster [ g_uiRxBufferMasterPut ] = uart_getc ( UART_PC );  // Store character in buffer
        g_uiRxBufferMasterPut++;

        if ( P2P_BUFFER_MASTER_SIZE == g_uiRxBufferMasterPut )
        {
            g_uiRxBufferMasterPut = 0; // At end wrap to beginning
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
        g_b_ucSensorCommsFlag = true;
        g_aucRxBufferSlave [ g_uiRxBufferSlavePut++ ] = uart_getc ( UART_SEN ); // Store character in buffer

        if ( P2P_BUFFER_SLAVE_SIZE == g_uiRxBufferSlavePut )
        {
            g_uiRxBufferSlavePut = 0; // At end wrap to beginning
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
    bool     b_ucSerialNumberRequest = false;
    uint8_t  SPI_RxRead              = 0;
    uint16_t uiCommsStatus           = 0;

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
    add_repeating_timer_ms ( 5000 , get_temperature_callback , NULL , &timer_getTemp   );
    add_repeating_timer_ms (   10 , timer_counter_callback   , NULL , &timer_counter   );
    add_repeating_timer_ms (  500 , timer_heartbeat_callback , NULL , &timer_heartbeat );

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

    g_b_ucHighTemperatureFlag = false;
    g_b_ucPassThroughMode     = false;
    g_b_ucPcCommsFlag         = false;
    g_b_ucSensorCommsFlag     = false;
    b_ucSerialNumberRequest = false;

    // Turn on power to sensors
    RELAY_ON;

    g_uiCommsTimeout = 500;   // 5 seconds

    while ( g_uiCommsTimeout );

    // Clear comms buffer on spurious characters
    g_uiRxBufferMasterGet = 0;
    g_uiRxBufferMasterPut = 0;
    g_uiRxBufferSlaveGet  = 0;
    g_uiRxBufferSlavePut  = 0;

    // Get temperature limits
    g_uiTemperatureCutoffPoint = ( uint16_t ) DEG_90;
    g_uiTemperatureResetPoint  = ( uint16_t ) DEG_82;

    // Infinite loop
    for ( ; ; )
    {
        watchdog ( );

        if ( 0 != g_DAC_Check_Option )
        {
            DAC_Check ( );
        }

        if ( true == g_b_Flag_GetTemperature )
        {
            getTemperatureStatus ( );
            g_b_Flag_GetTemperature = false;
        }
        else
        {
            // Nothing to do
        }

        if ( false == g_b_ucHighTemperatureFlag )
        {
            RELAY_ON;   // Turn on power to sensors

            g_b_ucFlagJigCommand = false;
            uiCommsStatus    = p2pPollMaster ( );
            
            if ( uiCommsStatus )
            {
                if ( COMMS_WRITE == g_uiCommsMode )
                {
                    if ( true == g_b_ucPassThroughMode )
                    {
                        g_b_ucPassThroughMode = false;
                        g_uiCommsMode         = COMMS_WAIT;
                        uiCommsStatus       = p2pPollSlaveWritePassThrough ( );
                    }
                    else if ( true == b_ucSerialNumberRequest )
                    {
                        b_ucSerialNumberRequest = false;
                        g_uiCommsMode             = COMMS_WAIT;
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
        if ( g_uiTemperatureCutoffPoint > uiTemperatureCurrent )
        {
            g_b_ucHighTemperatureFlag = true;   // Temperature above the cutoff point
        }
        else
        {
            if ( true == g_b_ucHighTemperatureFlag )
            {
                // Compensate for hysteresis
                if ( g_uiTemperatureResetPoint < uiTemperatureCurrent )
                {
                    // Temperature below the reset point
                    g_b_ucHighTemperatureFlag = false;
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

    for ( g_uiRxBufferSlaveGet = 0 ; g_uiRxBufferSlaveGet < g_uiRxBufferSlavePut ; g_uiRxBufferSlaveGet++ )
    {
        if ( 0x10 == ( g_aucRxBufferSlave [ g_uiRxBufferSlaveGet ] ) )
        {
            break;
        }
        else
        {
            // Keep looking
        }
    }

    if ( 0 == ( memcmp ( g_aucRxBufferSlave + g_uiRxBufferSlaveGet , ACK_String , 2 ) ) )
    {
        g_uiRxBufferSlaveGet = g_uiRxBufferSlavePut;
        result = RX_ACK;
    }
    else if ( 0 == ( memcmp ( g_aucRxBufferSlave + g_uiRxBufferSlaveGet , NAK_String , 2 ) ) )
    {
        g_uiRxBufferSlaveGet = g_uiRxBufferSlavePut;
        result = RX_NAK;
    }
    else
    {
        result = RX_ERROR;   // Unexpected or no response
    }

    uiRxBufferSlavePrevious = g_uiRxBufferSlavePut;
    return result;
}

void DAC_Check ( void )
{
    union DAC_Settings DAC_5000_NewSetting;
    union DAC_Settings DAC_60000_NewSetting;
    // union DAC_Settings DAC_5000_Setting;
    // union DAC_Settings DAC_60000_Setting;
    // union DAC_Settings DAC_FSD_Setting;
    // union DAC_Settings DAC_Zero_Setting;
    // float    DAC_5000_NewSetting_Float    = 0;
    // float    DAC_60000_NewSetting_Float   = 0;
    float    DAC_5000_Setting_Float       = 0;
    float    DAC_60000_Setting_Float      = 0;
    float    DAC_FSD_Setting_Float        = 0;
    float    DAC_Zero_Setting_Float       = 0;
    uint8_t  DAC_5000_Setting_Array  [ 4 ] = { 0 , 0 , 0 , 0 };
    uint8_t  DAC_60000_Setting_Array [ 4 ] = { 0 , 0 , 0 , 0 };
    uint8_t  DAC_FSD_Setting_Array   [ 4 ] = { 0 , 0 , 0 , 0 };
    uint8_t  DAC_Zero_Setting_Array  [ 4 ] = { 0 , 0 , 0 , 0 };
    uint8_t  SensorString_RX [ P2P_BUFFER_SLAVE_SIZE ];
    uint8_t  SensorString_TX [ P2P_BUFFER_SLAVE_SIZE ];
    uint8_t  Offset               = 0;
    uint8_t  Retry                = 0;
    uint8_t  RX_Status            = 0;
    uint8_t  Sample               = 0;
    uint8_t  SensorPos            = 0;
    uint16_t ADC_Result [ 10 ]    = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };
    uint16_t Checksum             = 0;
    uint16_t DAC_5000_Setting_mV  = 0;
    uint16_t DAC_60000_Setting_mV = 0;
    uint16_t DAC_FSD_Setting_mV   = 0;
    uint16_t DAC_Zero_Setting_mV  = 0;
    uint16_t Length               = 0;
    uint16_t Sensor_RxRead        = 0;
    uint16_t Sensor_RxWrite       = 0;
    uint32_t SensorState          = 0;

    if ( DAC_CHECK_INITIAL == g_DAC_Check_Option )   // Initial Voltage present test
    {
        b_DAC_Ready = false;
        memset ( DAC_Reading.DAC_ADC_Initial , 0 , sizeof ( DAC_Reading.DAC_ADC_Initial ) );
        memset ( SensorPass , 0 , sizeof ( SensorPass ) );

        for ( SensorPos = 0 ; SensorPos < 24 ; SensorPos++ )
        {
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

            if ( ADC_MAX_RETRY == Retry )
            {
                // Service watchdog only on sensor fail
                // Takes too long if too many sensors have failed or not been fitted
                watchdog ( );
            }
            else
            {
                // Nothing to do
            }

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
            SensorPass [ 3 ] = SensorPos;
        }
    }

    else if ( DAC_CHECK_CALIBRATION == g_DAC_Check_Option )
    {
        b_DAC_Ready = false;
        memset ( DAC_Reading.DAC_ADC_FSD  , 0 , sizeof ( DAC_Reading.DAC_ADC_FSD  ) );
        memset ( DAC_Reading.DAC_ADC_Zero , 0 , sizeof ( DAC_Reading.DAC_ADC_Zero ) );
        memset ( DAC_Reading.DAC_mV_FSD   , 0 , sizeof ( DAC_Reading.DAC_mV_FSD   ) );
        memset ( DAC_Reading.DAC_mV_Zero  , 0 , sizeof ( DAC_Reading.DAC_mV_Zero  ) );
        memset ( DAC_FSD_Setting_Array    , 0 , sizeof ( DAC_FSD_Setting_Array    ) );
        memset ( DAC_Zero_Setting_Array   , 0 , sizeof ( DAC_Zero_Setting_Array   ) );
        memset ( SensorPass      , 0 , sizeof ( SensorPass      ) );
        memset ( SensorString_RX , 0 , sizeof ( SensorString_RX ) );
        SensorState = 0;

        for ( SensorPos = 0 ; SensorPos < 24 ; SensorPos++ )
        {
            DAC_FSD_Setting_Float  = 0;
            DAC_Zero_Setting_Float = 0;
            g_uiRxBufferSlaveGet   = 0;
            g_uiRxBufferSlavePut   = 0;
            Length                 = 0;
            Offset                 = 0;
            RX_Status              = 0;
            Sample                 = 0;
            Sensor_RxRead          = 0;
            Sensor_RxWrite         = 0;

            // Service watchdog because reading back from each sensor takes too long
            watchdog ( );

            Set_MUX ( SensorPos );
            sleep_ms ( 10 );

            // Get DAC_ZERO settings from sensor
            uart_write_blocking ( UART_SEN , DAC_ReadConfig , 7 );
            g_uiCommsTimeout = 100;
            while ( g_uiCommsTimeout );   // Wait for long response string
            Sensor_RxWrite = g_uiRxBufferSlavePut;

            for ( Sensor_RxRead = 0 ; Sensor_RxRead < Sensor_RxWrite ; Sensor_RxRead++ )
            {
                if ( ( DLE == g_aucRxBufferSlave [ Sensor_RxRead ] ) && ( DLE == g_aucRxBufferSlave [ Sensor_RxRead + 1 ] ))
                {
                    // Byte stuffing detected
                    SensorString_RX [ Sensor_RxRead ] = g_aucRxBufferSlave [ Sensor_RxRead + Offset ];
                    Offset++;
                    Sensor_RxRead++;
                }
                else
                {
                    SensorString_RX [ Sensor_RxRead ] = g_aucRxBufferSlave [ Sensor_RxRead + Offset ];
                }
            }

            Length = Sensor_RxRead - Offset - 1;

            for ( Sensor_RxWrite = 0 ; Sensor_RxWrite < 4 ; Sensor_RxWrite++ )
            {
                DAC_Zero_Setting_Array [ Sensor_RxWrite ] = SensorString_RX [ Sensor_RxWrite + 167 ];
                DAC_FSD_Setting_Array  [ Sensor_RxWrite ] = SensorString_RX [ Sensor_RxWrite + 171 ];
            }

            // DAC_Zero_Setting_float = *( float * ) &DAC_Zero_Setting_Array;
            // DAC_FSD_Setting_float  = *( float * ) &DAC_FSD_Setting_Array;
            // DAC_Zero_Setting_mV    = ( uint16_t ) ( DAC_Zero_Setting_float * 1000 );
            // DAC_FSD_Setting_mV     = ( uint16_t ) ( DAC_FSD_Setting_float  * 1000 );

            DAC_Zero_Setting_Float = *( float * ) &DAC_Zero_Setting_Array; 
            DAC_FSD_Setting_Float  = *( float * ) &DAC_FSD_Setting_Array; 
            DAC_Zero_Setting_mV    = ( uint16_t ) ( DAC_Zero_Setting_Float * 1000 );
            DAC_FSD_Setting_mV     = ( uint16_t ) ( DAC_FSD_Setting_Float  * 1000 );

            // Set DAC_Zero
            g_uiCommsTimeout     = 100;
            g_uiRxBufferSlaveGet = 0;
            g_uiRxBufferSlavePut = 0;
            uart_write_blocking ( UART_SEN , DAC_Change , 9 );
            while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
            RX_Status = Check_Response ( );

            if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
            {
                g_uiRxBufferSlaveGet = 0;
                g_uiRxBufferSlavePut = 0;
                uart_write_blocking ( UART_SEN , DAC_Zero , 8 );
                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                RX_Status = Check_Response ( );

                if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                {
                    Sample = 0;
                    memset ( ADC_Result , 0 , sizeof ( ADC_Result ) );

                    do
                    {
                        ADC_Result [ Sample ] = ADC_Read ( DAC );
                        if ( 0x8000 <= ADC_Result [ Sample ] )
                        {
                            // Negative value ( sensor faulty or position not populated )
                            ADC_Result [ Sample ] = 0;
                        }
                        sleep_ms ( 10 );
                        Sample++;
                    } while ( Sample < 3 );
                }
                DAC_Reading.DAC_ADC_Zero [ SensorPos ] = ( uint16_t ) ( ( ADC_Result [ 0 ] + ADC_Result [ 1 ] + ADC_Result [ 2 ] ) / 3 );
            }

            // Set DAC_FSD
            g_uiRxBufferSlaveGet = 0;
            g_uiRxBufferSlavePut = 0;
            uart_write_blocking ( UART_SEN , DAC_Change , 9 );
            while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
            RX_Status = Check_Response ( );

            if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
            {
                g_uiRxBufferSlaveGet = 0;
                g_uiRxBufferSlavePut = 0;
                uart_write_blocking ( UART_SEN , DAC_FSD , 8 );
                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                RX_Status = Check_Response ( );

                if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                {
                    Sample = 0;
                    memset ( ADC_Result , 0 , sizeof ( ADC_Result ) );

                    do
                    {
                        ADC_Result [ Sample ] = ADC_Read ( DAC );
                        if ( 0x8000 <= ADC_Result [ Sample ] )
                        {
                            // Negative value ( sensor faulty or position not populated )
                            ADC_Result [ Sample ] = 0;
                        }
                        sleep_ms ( 10 );
                        Sample++;
                    } while ( Sample < 3 );
                }
                DAC_Reading.DAC_ADC_FSD [ SensorPos ] = ( uint16_t ) ( ( ADC_Result [ 0 ] + ADC_Result [ 1 ] + ADC_Result [ 2 ] ) / 3 );
            }

            DAC_Reading.DAC_mV_Zero [ SensorPos ] = ( uint16_t ) ( DAC_Reading.DAC_ADC_Zero [ SensorPos ] / DAC_ZERO_ADC_OFFSET_0V400 );
            DAC_Reading.DAC_mV_FSD  [ SensorPos ] = ( uint16_t ) ( DAC_Reading.DAC_ADC_FSD  [ SensorPos ] / DAC_FSD_ADC_OFFSET_2V000  );

            if ( ( ( DAC_Zero_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_Zero [ SensorPos ] ) || ( ( DAC_Zero_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_Zero [ SensorPos ] ) || ( ( DAC_FSD_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_FSD [ SensorPos ] ) || ( ( DAC_FSD_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_FSD [ SensorPos ] ) ) 
            {
                SensorState &= ~( 1 << SensorPos );
            }
            else
            {
                SensorState |= 1 << SensorPos;
            }

            SensorPass [ 0 ] = ( uint8_t ) ( SensorState >> 16 );
            SensorPass [ 1 ] = ( uint8_t ) ( SensorState >>  8 );
            SensorPass [ 2 ] = ( uint8_t ) ( SensorState       );
            SensorPass [ 3 ] = SensorPos;
        }
    }
    
    else if ( DAC_CHECK_ADJUST_CALIBRATION == g_DAC_Check_Option )
    {
        b_DAC_Ready = false;
        memset ( DAC_Reading.DAC_ADC_5000  , 0 , sizeof ( DAC_Reading.DAC_ADC_5000  ) );
        memset ( DAC_Reading.DAC_ADC_60000 , 0 , sizeof ( DAC_Reading.DAC_ADC_60000 ) );
        memset ( DAC_Reading.DAC_ADC_FSD   , 0 , sizeof ( DAC_Reading.DAC_ADC_FSD   ) );
        memset ( DAC_Reading.DAC_ADC_Zero  , 0 , sizeof ( DAC_Reading.DAC_ADC_Zero  ) );
        memset ( DAC_Reading.DAC_mV_5000   , 0 , sizeof ( DAC_Reading.DAC_mV_5000   ) );
        memset ( DAC_Reading.DAC_mV_60000  , 0 , sizeof ( DAC_Reading.DAC_mV_60000  ) );
        memset ( DAC_Reading.DAC_mV_FSD    , 0 , sizeof ( DAC_Reading.DAC_mV_FSD    ) );
        memset ( DAC_Reading.DAC_mV_Zero   , 0 , sizeof ( DAC_Reading.DAC_mV_Zero   ) );
        memset ( DAC_FSD_Setting_Array     , 0 , sizeof ( DAC_FSD_Setting_Array     ) );
        memset ( DAC_Zero_Setting_Array    , 0 , sizeof ( DAC_Zero_Setting_Array    ) );
        memset ( SensorPass      , 0 , sizeof ( SensorPass      ) );
        memset ( SensorString_RX , 0 , sizeof ( SensorString_RX ) );
        memset ( SensorString_TX , 0 , sizeof ( SensorString_TX ) );
        SensorState = 0;

        for ( SensorPos = 0 ; SensorPos < 24 ; SensorPos++ )
        {
            Retry = 0;

            do
            {
                Checksum                   = 0;
                DAC_5000_NewSetting.VarFloat  = 0;
                DAC_60000_NewSetting.VarFloat = 0;
                DAC_5000_Setting_Float     = 0;
                DAC_60000_Setting_Float    = 0;
                DAC_FSD_Setting_Float      = 0;
                DAC_Zero_Setting_Float     = 0;
                g_uiRxBufferSlaveGet       = 0;
                g_uiRxBufferSlavePut       = 0;
                Length                     = 0;
                Offset                     = 0;
                RX_Status                  = 0;
                Sample                     = 0;
                Sensor_RxRead              = 0;
                Sensor_RxWrite             = 0;

                // Service watchdog because reading back from each sensor takes too long
                watchdog ( );

                Set_MUX ( SensorPos );
                sleep_ms ( 10 );

                // Get DAC_ZERO & DAC_FSD settings from sensor
                uart_write_blocking ( UART_SEN , DAC_ReadConfig , 7 );
                g_uiCommsTimeout = 100;
                while ( g_uiCommsTimeout );   // Wait for long response string
                Sensor_RxWrite = g_uiRxBufferSlavePut;

                for ( Sensor_RxRead = 0 ; Sensor_RxRead < Sensor_RxWrite ; Sensor_RxRead++ )
                {
                    if ( ( DLE == g_aucRxBufferSlave [ Sensor_RxRead ] ) && ( DLE == g_aucRxBufferSlave [ Sensor_RxRead + 1 ] ))
                    {
                        // Byte stuffing detected
                        SensorString_RX [ Sensor_RxRead ] = g_aucRxBufferSlave [ Sensor_RxRead + Offset ];
                        Offset++;
                        Sensor_RxRead++;
                    }
                    else
                    {
                        SensorString_RX [ Sensor_RxRead ] = g_aucRxBufferSlave [ Sensor_RxRead + Offset ];
                    }
                }

                Length = Sensor_RxRead - Offset - 1;

                for ( Sensor_RxWrite = 0 ; Sensor_RxWrite < 4 ; Sensor_RxWrite++ )
                {
                    DAC_Zero_Setting_Array [ Sensor_RxWrite ] = SensorString_RX [ Sensor_RxWrite + 167 ];
                    DAC_FSD_Setting_Array  [ Sensor_RxWrite ] = SensorString_RX [ Sensor_RxWrite + 171 ];
                }

                DAC_Zero_Setting_Float = *( float * ) &DAC_Zero_Setting_Array;
                DAC_FSD_Setting_Float  = *( float * ) &DAC_FSD_Setting_Array;
                DAC_Zero_Setting_mV    = ( uint16_t ) ( DAC_Zero_Setting_Float * 1000 );
                DAC_FSD_Setting_mV     = ( uint16_t ) ( DAC_FSD_Setting_Float  * 1000 );

                // Set DAC_Zero
                g_uiCommsTimeout     = 100;
                g_uiRxBufferSlaveGet = 0;
                g_uiRxBufferSlavePut = 0;
                uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); // Wait for ACK / NAK
                RX_Status = Check_Response ( );

                if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                {
                    g_uiRxBufferSlaveGet = 0;
                    g_uiRxBufferSlavePut = 0;
                    uart_write_blocking ( UART_SEN , DAC_Zero , 8 );
                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                    RX_Status = Check_Response ( );

                    if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                    {
                        Sample = 0;
                        memset ( ADC_Result , 0 , sizeof ( ADC_Result ) );

                        do
                        {
                            ADC_Result [ Sample ] = ADC_Read ( DAC );
                            if ( 0x8000 <= ADC_Result [ Sample ] )
                            {
                                // Negative value ( sensor faulty or position not populated )
                                ADC_Result [ Sample ] = 0;
                            }
                            sleep_ms ( 10 );
                            Sample++;
                        } while ( Sample < 3 );
                    }
                    DAC_Reading.DAC_ADC_Zero [ SensorPos ] = ( uint16_t ) ( ( ADC_Result [ 0 ] + ADC_Result [ 1 ] + ADC_Result [ 2 ] ) / 3 );
                }

                // Set DAC_FSD
                g_uiRxBufferSlaveGet = 0;
                g_uiRxBufferSlavePut = 0;
                uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                RX_Status = Check_Response ( );

                if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                {
                    g_uiRxBufferSlaveGet = 0;
                    g_uiRxBufferSlavePut = 0;
                    uart_write_blocking ( UART_SEN , DAC_FSD , 8 );
                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                    RX_Status = Check_Response ( );

                    if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                    {
                        Sample = 0;
                        memset ( ADC_Result , 0 , sizeof ( ADC_Result ) );

                        do
                        {
                            ADC_Result [ Sample ] = ADC_Read ( DAC );
                            if ( 0x8000 <= ADC_Result [ Sample ] )
                            {
                                // Negative value ( sensor faulty or position not populated )
                                ADC_Result [ Sample ] = 0;
                            }
                            sleep_ms ( 10 );
                            Sample++;
                        } while ( Sample < 3 );
                    }
                    DAC_Reading.DAC_ADC_FSD [ SensorPos ] = ( uint16_t ) ( ( ADC_Result [ 0 ] + ADC_Result [ 1 ] + ADC_Result [ 2 ] ) / 3 );
                }

                DAC_Reading.DAC_mV_Zero [ SensorPos ] = ( uint16_t ) ( DAC_Reading.DAC_ADC_Zero [ SensorPos ] / DAC_ZERO_ADC_OFFSET_0V400 );
                DAC_Reading.DAC_mV_FSD  [ SensorPos ] = ( uint16_t ) ( DAC_Reading.DAC_ADC_FSD  [ SensorPos ] / DAC_FSD_ADC_OFFSET_2V000  );

                if ( ( ( DAC_Zero_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_Zero [ SensorPos ] ) || ( ( DAC_Zero_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_Zero [ SensorPos ] ) || ( ( DAC_FSD_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_FSD [ SensorPos ] ) || ( ( DAC_FSD_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_FSD [ SensorPos ] ) ) 
                {
                    // Out of spec - Make changes and try again
                    memset ( SensorString_RX , 0 , sizeof ( SensorString_RX ) );
                    memset ( SensorString_TX , 0 , sizeof ( SensorString_TX ) );
                    g_uiRxBufferSlaveGet = 0;
                    g_uiRxBufferSlavePut = 0;
                    Length               = 0;
                    Offset               = 0;
                    RX_Status            = 0;
                    Sample               = 0;
                    Sensor_RxRead        = 0;
                    Sensor_RxWrite       = 0;

                    // Get DAC_5000 & DAC_60000 settings from sensor
                    uart_write_blocking ( UART_SEN , DAC_ReadExtendedConfig , 7 );
                    g_uiCommsTimeout = 100;
                    while ( g_uiCommsTimeout );   // Wait for long response string
                    Sensor_RxWrite = g_uiRxBufferSlavePut;

                    for ( Sensor_RxRead = 0 ; Sensor_RxRead < Sensor_RxWrite ; Sensor_RxRead++ )
                    {
                        if ( ( DLE == g_aucRxBufferSlave [ Sensor_RxRead ] ) && ( DLE == g_aucRxBufferSlave [ Sensor_RxRead + 1 ] ))
                        {
                            // Byte stuffing detected
                            SensorString_RX [ Sensor_RxRead ] = g_aucRxBufferSlave [ Sensor_RxRead + Offset ];
                            Offset++;
                            Sensor_RxRead++;
                        }
                        else
                        {
                            SensorString_RX [ Sensor_RxRead ] = g_aucRxBufferSlave [ Sensor_RxRead + Offset ];
                        }
                    }

                    Length = Sensor_RxRead - Offset - 1;

                    for ( Sensor_RxWrite = 0 ; Sensor_RxWrite < 4 ; Sensor_RxWrite++ )
                    {
                        DAC_5000_Setting_Array  [ Sensor_RxWrite ] = SensorString_RX [ Sensor_RxWrite + 3 ];
                        DAC_60000_Setting_Array [ Sensor_RxWrite ] = SensorString_RX [ Sensor_RxWrite + 7 ];
                    }

                    DAC_5000_Setting_Float  = *( float * ) &DAC_5000_Setting_Array;
                    DAC_60000_Setting_Float = *( float * ) &DAC_60000_Setting_Array;
                    DAC_5000_Setting_mV     = ( uint16_t ) ( DAC_5000_Setting_Float  * 1000 );
                    DAC_60000_Setting_mV    = ( uint16_t ) ( DAC_60000_Setting_Float * 1000 );

                    // Set DAC 5000
                    g_uiCommsTimeout     = 100;
                    g_uiRxBufferSlaveGet = 0;
                    g_uiRxBufferSlavePut = 0;
                    uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                    RX_Status = Check_Response ( );

                    if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                    {
                        g_uiRxBufferSlaveGet = 0;
                        g_uiRxBufferSlavePut = 0;
                        uart_write_blocking ( UART_SEN , DAC_5000 , 8 );
                        while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                        RX_Status = Check_Response ( );

                        if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                        {
                            Sample = 0;
                            memset ( ADC_Result , 0 , sizeof ( ADC_Result ) );

                            do
                            {
                                ADC_Result [ Sample ] = ADC_Read ( DAC );
                                if ( 0x8000 <= ADC_Result [ Sample ] )
                                {
                                    // Negative value ( sensor faulty or position not populated )
                                    ADC_Result [ Sample ] = 0;
                                }
                                sleep_ms ( 10 );
                                Sample++;
                            } while ( Sample < 3 );
                        }
                        DAC_Reading.DAC_ADC_5000 [ SensorPos ] = ( uint16_t ) ( ( ADC_Result [ 0 ] + ADC_Result [ 1 ] + ADC_Result [ 2 ] ) / 3 );
                    }

                    // Set DAC 60000
                    g_uiCommsTimeout     = 100;
                    g_uiRxBufferSlaveGet = 0;
                    g_uiRxBufferSlavePut = 0;
                    uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                    RX_Status = Check_Response ( );

                    if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                    {
                        g_uiRxBufferSlaveGet = 0;
                        g_uiRxBufferSlavePut = 0;
                        uart_write_blocking ( UART_SEN , DAC_60000 , 8 );
                        while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                        RX_Status = Check_Response ( );

                        if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                        {
                            Sample = 0;
                            memset ( ADC_Result , 0 , sizeof ( ADC_Result ) );

                            do
                            {
                                ADC_Result [ Sample ] = ADC_Read ( DAC );
                                if ( 0x8000 <= ADC_Result [ Sample ] )
                                {
                                    // Negative value ( sensor faulty or position not populated )
                                    ADC_Result [ Sample ] = 0;
                                }
                                sleep_ms ( 10 );
                                Sample++;
                            } while ( Sample < 3 );
                        }
                        DAC_Reading.DAC_ADC_60000 [ SensorPos ] = ( uint16_t ) ( ( ADC_Result [ 0 ] + ADC_Result [ 1 ] + ADC_Result [ 2 ] ) / 3 );
                    }

                    DAC_Reading.DAC_mV_5000  [ SensorPos ] = ( uint16_t ) ( DAC_Reading.DAC_ADC_5000  [ SensorPos ] / DAC_5000_ADC_OFFSET);// DAC_ZERO_ADC_OFFSET_0V400 );
                    DAC_Reading.DAC_mV_60000 [ SensorPos ] = ( uint16_t ) ( DAC_Reading.DAC_ADC_60000 [ SensorPos ] / DAC_60000_ADC_OFFSET);// DAC_FSD_ADC_OFFSET_2V000  );

                    // Update DAC 5000 & DAC 60000 values with values read by ADC ( modify string )
                    // Update original string with new DAC 5000 & DAC 60000 settings
                    DAC_5000_NewSetting.VarFloat  = ( float ) ( DAC_Reading.DAC_mV_5000  [ SensorPos ] ) / 1000;
                    DAC_60000_NewSetting.VarFloat = ( float ) ( DAC_Reading.DAC_mV_60000 [ SensorPos ] ) / 1000;
                    SensorString_RX[10]=(uint8_t)(DAC_60000_NewSetting.VarInt>>24);
                    SensorString_RX[9]=(uint8_t)(DAC_60000_NewSetting.VarInt>>16);
                    SensorString_RX[8]=(uint8_t)(DAC_60000_NewSetting.VarInt>>8);
                    SensorString_RX[7]=(uint8_t)(DAC_60000_NewSetting.VarInt);
                    SensorString_RX[6]=(uint8_t)(DAC_5000_NewSetting.VarInt>>24);
                    SensorString_RX[5]=(uint8_t)(DAC_5000_NewSetting.VarInt>>16);
                    SensorString_RX[4]=(uint8_t)(DAC_5000_NewSetting.VarInt>>8);
                    SensorString_RX[3]=(uint8_t)(DAC_5000_NewSetting.VarInt);

                    // Byte stuffing
                    Length = SensorString_RX[2];
                    Offset=0;
                    Sensor_RxRead=0;
                    memcpy(SensorString_TX,SensorString_RX,3);
                    for(Sensor_RxWrite=0;Sensor_RxWrite<Length;Sensor_RxWrite++)
                    {
                        if ( 0x10==SensorString_RX[Sensor_RxWrite+Offset+3])//Byte stuffing required
                        {
                            SensorString_TX[Sensor_RxWrite+Offset+3]=0x10;
                            Offset++;
                            SensorString_TX[Sensor_RxWrite+Offset+3]=0x10;
                        }
                        else
                        {
                            SensorString_TX[Sensor_RxWrite+Offset+3]=SensorString_RX[Sensor_RxWrite+3];
                        }
                    }
                    SensorString_TX[3+Length+Offset]=0x10;
                    SensorString_TX[3+Length+Offset+1]=0x1F;

                    // Calculate checksum & add to string
                    Length=3+Length+Offset+2;//strlen(SensorString_TX);
                    for(Sensor_RxWrite=0;Sensor_RxWrite<Length;Sensor_RxWrite++)
                    {
                        Checksum+=SensorString_TX[Sensor_RxWrite];
                    }
                    SensorString_TX[Length]=(uint8_t)(Checksum>>8);
                    SensorString_TX[Length+1]=(uint8_t)(Checksum);

                    // Update sensor
                    g_uiCommsTimeout     = 100;
                    g_uiRxBufferSlaveGet = 0;
                    g_uiRxBufferSlavePut = 0;
                    uart_write_blocking ( UART_SEN , DAC_ChangeSettings , 9 );
                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                    RX_Status = Check_Response ( );

                    if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                    {
                        g_uiRxBufferSlaveGet = 0;
                        g_uiRxBufferSlavePut = 0;
                        uart_write_blocking ( UART_SEN , SensorString_TX , Length + 2 );
                        while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                        RX_Status = Check_Response ( );

                        if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                        {
                            // Sample = 0;
                            // memset ( ADC_Result , 0 , sizeof ( ADC_Result ) );

                            // do
                            // {
                                // ADC_Result [ Sample ] = ADC_Read ( DAC );
                                // if ( 0x8000 <= ADC_Result [ Sample ] )
                                // {
                                    // Negative value ( sensor faulty or position not populated )
                                    // ADC_Result [ Sample ] = 0;
                                // }
                                // sleep_ms ( 10 );
                                // Sample++;
                            // } while ( Sample < 3 );
                            // Set DAC VMON
                            g_uiCommsTimeout     = 100;
                            g_uiRxBufferSlaveGet = 0;
                            g_uiRxBufferSlavePut = 0;
                            uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                            while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                            RX_Status = Check_Response ( );

                            // Set DAC VMON
                            if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                            {
                                g_uiRxBufferSlaveGet = 0;
                                g_uiRxBufferSlavePut = 0;
                                uart_write_blocking ( UART_SEN , DAC_VMON_4V , 8 );
                                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                                RX_Status = Check_Response ( );

                                if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                                {
                                    // Sample = 0;
                                    // memset ( ADC_Result , 0 , sizeof ( ADC_Result ) );

                                    // do
                                    // {
                                        // ADC_Result [ Sample ] = ADC_Read ( DAC );
                                        // if ( 0x8000 <= ADC_Result [ Sample ] )
                                        // {
                                            // Negative value ( sensor faulty or position not populated )
                                            // ADC_Result [ Sample ] = 0;
                                        // }
                                        // sleep_ms ( 10 );
                                        // Sample++;
                                    // } while ( Sample < 3 );
                                    // Set DAC 5000
                                    g_uiCommsTimeout     = 100;
                                    g_uiRxBufferSlaveGet = 0;
                                    g_uiRxBufferSlavePut = 0;
                                    uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                                    RX_Status = Check_Response ( );

                                    if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                                    {
                                        g_uiRxBufferSlaveGet = 0;
                                        g_uiRxBufferSlavePut = 0;
                                        uart_write_blocking ( UART_SEN , DAC_5000 , 8 );
                                        while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                                        RX_Status = Check_Response ( );

                                        if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                                        {
                                        //    Sample = 0;
                                            // memset ( ADC_Result , 0 , sizeof ( ADC_Result ) );

                                            // do
                                            // {
                                                // ADC_Result [ Sample ] = ADC_Read ( DAC );
                                                // if ( 0x8000 <= ADC_Result [ Sample ] )
                                                // {
                                                    // Negative value ( sensor faulty or position not populated )
                                                    // ADC_Result [ Sample ] = 0;
                                                // }
                                                // sleep_ms ( 10 );
                                                // Sample++;
                                            // } while ( Sample < 3 );
                                            // Set DAC 60000
                                            g_uiCommsTimeout     = 100;
                                            g_uiRxBufferSlaveGet = 0;
                                            g_uiRxBufferSlavePut = 0;
                                            uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                                            while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                                            RX_Status = Check_Response ( );

                                            if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                                            {
                                                g_uiRxBufferSlaveGet = 0;
                                                g_uiRxBufferSlavePut = 0;
                                                uart_write_blocking ( UART_SEN , DAC_60000 , 8 );
                                                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout ); //  Wait for ACK / NAK
                                                RX_Status = Check_Response ( );

                                                if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                                                {
                                                    // Sample = 0;
                                                    // memset ( ADC_Result , 0 , sizeof ( ADC_Result ) );

                                                    // do
                                                    // {
                                                        // ADC_Result [ Sample ] = ADC_Read ( DAC );
                                                        // if ( 0x8000 <= ADC_Result [ Sample ] )
                                                       // {
                                                            // Negative value ( sensor faulty or position not populated )
                                                            // ADC_Result [ Sample ] = 0;
                                                        // }
                                                        // sleep_ms ( 10 );
                                                        // Sample++;
                                                    // } while ( Sample < 3 );
                                                    }
                                                // DAC_Reading.DAC_ADC_60000 [ SensorPos ] = ( uint16_t ) ( ( ADC_Result [ 0 ] + ADC_Result [ 1 ] + ADC_Result [ 2 ] ) / 3 );
                                            }
                                        }
                                        // DAC_Reading.DAC_ADC_60000 [ SensorPos ] = ( uint16_t ) ( ( ADC_Result [ 0 ] + ADC_Result [ 1 ] + ADC_Result [ 2 ] ) / 3 );
                                    }

                                }
                                // DAC_Reading.DAC_ADC_60000 [ SensorPos ] = ( uint16_t ) ( ( ADC_Result [ 0 ] + ADC_Result [ 1 ] + ADC_Result [ 2 ] ) / 3 );
                            }
                        }
                        // DAC_Reading.DAC_ADC_60000 [ SensorPos ] = ( uint16_t ) ( ( ADC_Result [ 0 ] + ADC_Result [ 1 ] + ADC_Result [ 2 ] ) / 3 );


                    }










                    Retry++;
                }
                else
                {
                    // Force exit from do / while
                    Retry = ADC_MAX_RETRY;
                }

            } while ( Retry < ADC_MAX_RETRY );

            if ( ( ( DAC_Zero_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_Zero [ SensorPos ] ) || ( ( DAC_Zero_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_Zero [ SensorPos ] ) || ( ( DAC_FSD_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_FSD [ SensorPos ] ) || ( ( DAC_FSD_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_FSD [ SensorPos ] ) ) 
            {
                SensorState &= ~( 1 << SensorPos );
            }
            else
            {
                SensorState |= 1 << SensorPos;
            }

            SensorPass [ 0 ] = ( uint8_t ) ( SensorState >> 16 );
            SensorPass [ 1 ] = ( uint8_t ) ( SensorState >>  8 );
            SensorPass [ 2 ] = ( uint8_t ) ( SensorState       );
            SensorPass [ 3 ] = SensorPos;
        }
    }

    else    // Invalid option
    {
        // Nothing to do
    }

    b_DAC_Ready        = true;
    g_DAC_Check_Option = DAC_CHECK_IDLE;  // Reset button press data
    
    // Service watchdog before re-entering main program
    watchdog ( );
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
