/******************************************************************************
 * Project:         24-way Premier IR sensor jig                              *
 * Filename:        main.c                                                    *
 * Author:          Craig Hemingway                                           *
 * Company:         Dynament Ltd.                                             *
 *                  Status Scientific Controls Ltd.                           *
 * Date:            12/06/2024                                                *
 * File Version:   	4.0.1                                                     *
 *  Version history:                                                          *
 *                 4.0.1 (12/06/2024) - watchdog calls placed in teh read     *
 *                         byte routines due to the comms routine "hanging"   *
 *                         for many seconds waiting for incomlete messages    *
 *                 4.0.0 - 14/03/2023 - Craig Hemingway                       *
 *                     PIC code ported over to RP2040                         *
 *                 3.0.0 - 27/01/2014 - Frank Kups                            *
 *                     Latest program for sensor jig Version 4                *
 *                          Latest program for sensor jig Version 4           *
 * Hardware:        RP2040                                                    *
 * Tools Used:      Visual Studio Code -> 1.73.1                              *
 *                  Compiler           -> GCC 11.3.1 arm-none-eabi            *
 *                                                                            *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <main.h>

#include <comms.h>
#include <dac.h>
#include <stdio.h>
#include <string.h>
#include <hardware/i2c.h>
#include <hardware/spi.h>
#include <hardware/watchdog.h>
#include <pico/binary_info.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
struct repeating_timer timer_counter;
struct repeating_timer timer_getTemp;
struct repeating_timer timer_heartbeat;

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
extern const uint8_t DAC_CHECK_IS_READY;
extern const uint8_t DAC_CHECK_RUNNING;
extern const uint8_t DAC_CHECK_NOT_RUNNING;
extern uint8_t g_DAC_Check_Option;
extern uint8_t SensorPass [ 4 ];

bool     g_b_Flag_GetTemperature    = false;
bool     g_b_ucHighTemperatureFlag  = false;
uint16_t g_uiTemperatureCutoffPoint = 0;
uint16_t g_uiTemperatureResetPoint  = 0;

extern volatile bool     b_DAC_Ready;
volatile uint16_t g_ADC_Timeout = 0;

volatile bool     g_b_ucFlagJigCommand  = false;
volatile bool     g_b_ucPassThroughMode = false;
volatile bool     g_b_ucPcCommsFlag     = false;
volatile bool     g_b_ucSensorCommsFlag = false;
volatile uint16_t g_uiCommsTimeout      = 0;
volatile uint16_t g_uiCommsMode         = COMMS_WAIT;

/* Private function prototypes -----------------------------------------------*/
static void getTemperatureStatus ( void );

/* User code -----------------------------------------------------------------*/
// Timer interrupts
static bool get_temperature_isr ( struct repeating_timer *t )
{
    g_b_Flag_GetTemperature = true;
}

bool timer_counter_isr ( struct repeating_timer *t )
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

    if ( 0 < g_ADC_Timeout )
    {
        g_ADC_Timeout--;
    }
    else
    {
        // Nothing to do
    }
}

static bool timer_heartbeat_isr ( struct repeating_timer *t )
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
static void spi_slave_rx_isr ( )
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
static void uart_rx_isr ( )
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
    gpio_init    ( A0_PIN          );
    gpio_init    ( A1_PIN          );
    gpio_init    ( A2_PIN          );
    gpio_init    ( EN1_PIN         );
    gpio_init    ( EN2_PIN         );
    gpio_init    ( EN3_PIN         );
    gpio_init    ( LED_PICO_PIN    );
    gpio_init    ( LED_RED_PIN     );
    gpio_init    ( LED_YELLOW_PIN  );
    gpio_init    ( RELAY_PIN       );
    gpio_init    ( SENSOR_PLATINUM );
    gpio_set_dir ( A0_PIN          , GPIO_OUT );
    gpio_set_dir ( A1_PIN          , GPIO_OUT );
    gpio_set_dir ( A2_PIN          , GPIO_OUT );
    gpio_set_dir ( EN1_PIN         , GPIO_OUT );
    gpio_set_dir ( EN2_PIN         , GPIO_OUT );
    gpio_set_dir ( EN3_PIN         , GPIO_OUT );
    gpio_set_dir ( LED_PICO_PIN    , GPIO_OUT );
    gpio_set_dir ( LED_RED_PIN     , GPIO_OUT );
    gpio_set_dir ( LED_YELLOW_PIN  , GPIO_OUT );
    gpio_set_dir ( RELAY_PIN       , GPIO_OUT );
    gpio_set_dir ( SENSOR_PLATINUM , GPIO_IN  );

    gpio_set_pulls ( SENSOR_PLATINUM , true , false ); // Set internal pullup

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
    add_repeating_timer_ms ( 5000 , get_temperature_isr , NULL , &timer_getTemp   );
    add_repeating_timer_ms (   10 , timer_counter_isr   , NULL , &timer_counter   );
    add_repeating_timer_ms (  500 , timer_heartbeat_isr , NULL , &timer_heartbeat );

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
    irq_set_exclusive_handler ( SPI0_IRQ , spi_slave_rx_isr );
    irq_set_enabled           ( SPI0_IRQ , true             );

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
    irq_set_exclusive_handler ( UART1_IRQ , uart_rx_isr  );
    irq_set_enabled           ( UART1_IRQ , true         );
    uart_set_irq_enables      ( UART_PC   , true , false );  // Enable UART interrupt ( RX only )
    irq_set_exclusive_handler ( UART0_IRQ , uart_rx_isr  );
    irq_set_enabled           ( UART0_IRQ , true         );
    uart_set_irq_enables      ( UART_SEN  , true , false );  // Enable UART interrupt ( RX only )

    g_b_ucHighTemperatureFlag = false;
    g_b_ucPassThroughMode     = false;
    g_b_ucPcCommsFlag         = false;
    g_b_ucSensorCommsFlag     = false;
    b_ucSerialNumberRequest   = false;

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
        Watchdog ( );

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
static void getTemperatureStatus ( void )
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

        g_ADC_Timeout = 100;

        do
        {
            i2c_read_blocking ( I2C_ADC , 0b01101000 , ADC_Buffer , 3 , 0 );
            sleep_us ( 10 );
        } while ( g_ADC_Timeout && ( ADC_Buffer [ 2 ] & ( 1 << 7 ) ) );

        Value_ADC = ( uint16_t ) ( ( ADC_Buffer [ 0 ] << 8 ) + ADC_Buffer [ 1 ] );
    }
    else if ( DAC == channel )
    {
        ADC_Buffer [ 0 ] = ADC_DAC_CHANNEL;
        i2c_write_blocking ( I2C_ADC , 0b01101000 , ADC_Buffer , 1 , 0 );
        sleep_us ( 100 );

        g_ADC_Timeout = 100;

        do
        {
            i2c_read_blocking ( I2C_ADC , 0b01101000 , ADC_Buffer , 3 , 0 );
            sleep_us ( 10 );
        } while ( g_ADC_Timeout && ( ADC_Buffer [ 2 ] & ( 1 << 7 ) ) );

        Value_ADC = ( uint16_t ) ( ( ADC_Buffer [ 0 ] << 8 ) + ADC_Buffer [ 1 ] );
    }
    else
    {
        // Nothing to do
    }

    return Value_ADC;
}

void BaudRate_Update ( uint16_t baudrate )
{
    uart_init ( UART_PC  , baudrate );
    uart_init ( UART_SEN , baudrate );
}

void Watchdog ( void )
{
    watchdog_update ( );
}

/* End of file ---------------------------------------------------------------*/
