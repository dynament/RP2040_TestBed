/******************************************************************************
 * Project:         24-way Premier IR sensor jig                              *
 * Filename:        dac.c                                                     *
 * Author:          Craig Hemingway                                           *
 * Company:         Dynament Ltd.                                             *
 *                  Status Scientific Controls Ltd.                           *
 * Date:            14/03/2023                                                *
 * File Version:   	1.0.0                                                     *
 * Version history: 1.0.0 - 14/03/2023 - Craig Hemingway                      *
 *                      Initial release                                       *
 * Hardware:        RP2040                                                    *
 * Tools Used:      Visual Studio Code -> 1.73.1                              *
 *                  Compiler           -> GCC 11.3.1 arm-none-eabi            *
 *                                                                            *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <main.h>

#include <comms.h>
#include <dac.h>
#include <string.h>

// DAC
const    uint8_t  TOLERANCE          = 10;  // mV
const    uint8_t  DAC_BUSY           = 0x01;
const    uint8_t  DAC_INITIAL        = 0x02;
const    uint8_t  DAC_CHECK_IN_SPEC  = 0x03;
const    uint8_t  DAC_CALIBRATE      = 0x04;
uint8_t  SensorPass [ 4 ] = { 0 , 0 , 0 , 0 };

// UART
extern uint8_t  g_aucRxBufferSlave [ P2P_BUFFER_SLAVE_SIZE ];
extern volatile uint16_t g_uiRxBufferSlaveGet;
extern volatile uint16_t g_uiRxBufferSlavePut;

// DAC check
const uint8_t ADC_MAX_RETRY                = 5;
const uint8_t DAC_CHECK_IDLE               = 0b00000000;
const uint8_t DAC_CHECK_INITIAL            = 0b00000001;
const uint8_t DAC_CHECK_CALIBRATION        = 0b00000010;
const uint8_t DAC_CHECK_ADJUST_CALIBRATION = 0b00000100;
const uint8_t DAC_CHECK_RESET              = 0b00001000;
const uint8_t DAC_CHECK_IS_READY           = 0x10;
const uint8_t DAC_CHECK_RUNNING            = 0x0B;
const uint8_t DAC_CHECK_NOT_RUNNING        = 0x0F;
const uint8_t DAC_ERROR_RANGE_MV           = 10;
uint8_t g_DAC_Check_Option = DAC_CHECK_IDLE;

union DAC_Settings_t
{
    float    VarFloat;
    uint32_t VarInt;
};

struct DAC_Reading_t
{
    uint16_t DAC_ADC_Initial;
    uint16_t DAC_ADC_Zero;
    uint16_t DAC_mV_Zero;
    uint16_t DAC_ADC_FSD;
    uint16_t DAC_mV_FSD;
    uint16_t DAC_ADC_5000;
    uint16_t DAC_mV_5000;
    uint16_t DAC_ADC_60000;
    uint16_t DAC_mV_60000;
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

volatile bool b_DAC_Ready = true;

extern volatile uint16_t g_uiCommsTimeout;

/* Private function prototypes -----------------------------------------------*/

/* User code -----------------------------------------------------------------*/
uint8_t UART_CheckResponse ( void )
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
    union DAC_Settings_t DAC_5000_NewSetting;
    union DAC_Settings_t DAC_60000_NewSetting;
    float    DAC_5000_Setting_Float        = 0;
    float    DAC_60000_Setting_Float       = 0;
    float    DAC_FSD_Setting_Float         = 0;
    float    DAC_Zero_Setting_Float        = 0;
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
        DAC_Reading.DAC_ADC_Initial = 0;
        memset ( SensorPass , 0 , sizeof ( SensorPass ) );

        // Reset sensors ( warmup )
        RELAY_OFF;
        sleep_ms ( 100 );
        RELAY_ON;
        sleep_ms ( 200 );

        for ( SensorPos = 0 ; SensorPos < 24 ; SensorPos++ )
        {
            MUX_Set ( SensorPos );
            sleep_ms ( 10 );
            DAC_Reading.DAC_ADC_Initial = 0;
            Retry = 0;

            do
            {
                DAC_Reading.DAC_ADC_Initial = ADC_Read ( DAC );
                if ( 0x8000 <= DAC_Reading.DAC_ADC_Initial  )
                {
                    // Negative value ( sensor faulty or position not populated )
                    DAC_Reading.DAC_ADC_Initial = 0;
                }
                sleep_ms ( 100 );
                Retry++;
            } while ( ( 0 == DAC_Reading.DAC_ADC_Initial ) && ( ADC_MAX_RETRY > Retry ) );

            if ( ADC_MAX_RETRY == Retry )
            {
                // Service watchdog only on sensor fail
                // Takes too long if too many sensors have failed or not been fitted
                Watchdog ( );
            }
            else
            {
                // Nothing to do
            }

            if ( 100 < DAC_Reading.DAC_ADC_Initial )
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
            SensorPass [ 3 ] = ( SensorPos + 1 );
        }
    }

    else if ( DAC_CHECK_CALIBRATION == g_DAC_Check_Option )
    {
        b_DAC_Ready = false;
        memset ( DAC_FSD_Setting_Array  , 0 , sizeof ( DAC_FSD_Setting_Array  ) );
        memset ( DAC_Zero_Setting_Array , 0 , sizeof ( DAC_Zero_Setting_Array ) );
        memset ( SensorPass             , 0 , sizeof ( SensorPass             ) );
        memset ( SensorString_RX        , 0 , sizeof ( SensorString_RX        ) );
        DAC_Reading.DAC_ADC_FSD  = 0;
        DAC_Reading.DAC_ADC_Zero = 0;
        DAC_Reading.DAC_mV_FSD   = 0;
        DAC_Reading.DAC_mV_Zero  = 0;
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
            Watchdog ( );

            MUX_Set ( SensorPos );
            sleep_ms ( 10 );

            // Get DAC_ZERO settings from sensor
            uart_write_blocking ( UART_SEN , DAC_ReadConfig , 7 );
            g_uiCommsTimeout = 100;
            while ( g_uiCommsTimeout ); // Wait for long response string
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
            RX_Status = UART_CheckResponse ( );

            if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
            {
                g_uiRxBufferSlaveGet = 0;
                g_uiRxBufferSlavePut = 0;
                uart_write_blocking ( UART_SEN , DAC_Zero , 8 );
                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                RX_Status = UART_CheckResponse ( );

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
                    } while ( Sample < 5 );
                }
                DAC_Reading.DAC_ADC_Zero = ( uint16_t ) ( ( ADC_Result [ 2 ] + ADC_Result [ 3 ] + ADC_Result [ 4 ] ) / 3 );
            }

            // Set DAC_FSD
            g_uiRxBufferSlaveGet = 0;
            g_uiRxBufferSlavePut = 0;
            uart_write_blocking ( UART_SEN , DAC_Change , 9 );
            while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
            RX_Status = UART_CheckResponse ( );

            if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
            {
                g_uiRxBufferSlaveGet = 0;
                g_uiRxBufferSlavePut = 0;
                uart_write_blocking ( UART_SEN , DAC_FSD , 8 );
                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                RX_Status = UART_CheckResponse ( );

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
                    } while ( Sample < 5 );
                }
                DAC_Reading.DAC_ADC_FSD = ( uint16_t ) ( ( ADC_Result [ 2 ] + ADC_Result [ 3 ] + ADC_Result [ 4 ] ) / 3 );
            }

            DAC_Reading.DAC_mV_Zero = ( uint16_t ) ( DAC_Reading.DAC_ADC_Zero / DAC_ZERO_ADC_OFFSET_0V400 );
            DAC_Reading.DAC_mV_FSD  = ( uint16_t ) ( DAC_Reading.DAC_ADC_FSD  / DAC_FSD_ADC_OFFSET_2V000  );

            if ( ( ( DAC_Zero_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_Zero ) || ( ( DAC_Zero_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_Zero ) || ( ( DAC_FSD_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_FSD ) || ( ( DAC_FSD_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_FSD ) ) 
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
            SensorPass [ 3 ] = ( SensorPos + 1 );
        }
    }
    
    else if ( DAC_CHECK_ADJUST_CALIBRATION == g_DAC_Check_Option )
    {
        b_DAC_Ready = false;
        memset ( DAC_FSD_Setting_Array  , 0 , sizeof ( DAC_FSD_Setting_Array  ) );
        memset ( DAC_Zero_Setting_Array , 0 , sizeof ( DAC_Zero_Setting_Array ) );
        memset ( SensorPass             , 0 , sizeof ( SensorPass             ) );
        memset ( SensorString_RX        , 0 , sizeof ( SensorString_RX        ) );
        memset ( SensorString_TX        , 0 , sizeof ( SensorString_TX        ) );
        SensorState               = 0;

        for ( SensorPos = 0 ; SensorPos < 24 ; SensorPos++ )
        {
            Retry = 0;

            do
            {
                Checksum                      = 0;
                DAC_5000_NewSetting.VarFloat  = 0;
                DAC_60000_NewSetting.VarFloat = 0;
                DAC_5000_Setting_Float        = 0;
                DAC_60000_Setting_Float       = 0;
                DAC_FSD_Setting_Float         = 0;
                DAC_Zero_Setting_Float        = 0;
                g_uiRxBufferSlaveGet          = 0;
                g_uiRxBufferSlavePut          = 0;
                Length                        = 0;
                Offset                        = 0;
                RX_Status                     = 0;
                Sample                        = 0;
                Sensor_RxRead                 = 0;
                Sensor_RxWrite                = 0;

                DAC_Reading.DAC_ADC_5000  = 0;
                DAC_Reading.DAC_ADC_60000 = 0;
                DAC_Reading.DAC_ADC_FSD   = 0;
                DAC_Reading.DAC_ADC_Zero  = 0;
                DAC_Reading.DAC_mV_5000   = 0;
                DAC_Reading.DAC_mV_60000  = 0;
                DAC_Reading.DAC_mV_FSD    = 0;
                DAC_Reading.DAC_mV_Zero   = 0;

                // Service watchdog because reading back from each sensor takes too long
                Watchdog ( );

                MUX_Set ( SensorPos );
                sleep_ms ( 10 );

                // Get DAC_ZERO & DAC_FSD settings from sensor
                uart_write_blocking ( UART_SEN , DAC_ReadConfig , 7 );
                g_uiCommsTimeout = 100;
                while ( g_uiCommsTimeout ); // Wait for long response string
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
                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                RX_Status = UART_CheckResponse ( );

                if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                {
                    g_uiRxBufferSlaveGet = 0;
                    g_uiRxBufferSlavePut = 0;
                    uart_write_blocking ( UART_SEN , DAC_Zero , 8 );
                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                    RX_Status = UART_CheckResponse ( );

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
                        } while ( Sample < 5 );
                    }
                    DAC_Reading.DAC_ADC_Zero = ( uint16_t ) ( ( ADC_Result [ 2 ] + ADC_Result [ 3 ] + ADC_Result [ 4 ] ) / 3 );
                }

                // Set DAC_FSD
                g_uiRxBufferSlaveGet = 0;
                g_uiRxBufferSlavePut = 0;
                uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                RX_Status = UART_CheckResponse ( );

                if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                {
                    g_uiRxBufferSlaveGet = 0;
                    g_uiRxBufferSlavePut = 0;
                    uart_write_blocking ( UART_SEN , DAC_FSD , 8 );
                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                    RX_Status = UART_CheckResponse ( );

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
                        } while ( Sample < 5 );
                    }
                    DAC_Reading.DAC_ADC_FSD = ( uint16_t ) ( ( ADC_Result [ 2 ] + ADC_Result [ 3 ] + ADC_Result [ 4 ] ) / 3 );
                }

                DAC_Reading.DAC_mV_Zero = ( uint16_t ) ( DAC_Reading.DAC_ADC_Zero / DAC_ZERO_ADC_OFFSET_0V400 );
                DAC_Reading.DAC_mV_FSD  = ( uint16_t ) ( DAC_Reading.DAC_ADC_FSD  / DAC_FSD_ADC_OFFSET_2V000  );

                if ( ( ( DAC_Zero_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_Zero ) || ( ( DAC_Zero_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_Zero ) || ( ( DAC_FSD_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_FSD ) || ( ( DAC_FSD_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_FSD ) ) 
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
                    while ( g_uiCommsTimeout ); // Wait for long response string
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
                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                    RX_Status = UART_CheckResponse ( );

                    if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                    {
                        g_uiRxBufferSlaveGet = 0;
                        g_uiRxBufferSlavePut = 0;
                        uart_write_blocking ( UART_SEN , DAC_5000 , 8 );
                        while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                        RX_Status = UART_CheckResponse ( );

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
                            } while ( Sample < 5 );
                        }
                        DAC_Reading.DAC_ADC_5000 = ( uint16_t ) ( ( ADC_Result [ 2 ] + ADC_Result [ 3 ] + ADC_Result [ 4 ] ) / 3 );
                    }

                    // Set DAC 60000
                    g_uiCommsTimeout     = 100;
                    g_uiRxBufferSlaveGet = 0;
                    g_uiRxBufferSlavePut = 0;
                    uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                    RX_Status = UART_CheckResponse ( );

                    if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                    {
                        g_uiRxBufferSlaveGet = 0;
                        g_uiRxBufferSlavePut = 0;
                        uart_write_blocking ( UART_SEN , DAC_60000 , 8 );
                        while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                        RX_Status = UART_CheckResponse ( );

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
                            } while ( Sample < 5 );
                        }
                        DAC_Reading.DAC_ADC_60000 = ( uint16_t ) ( ( ADC_Result [ 2 ] + ADC_Result [ 3 ] + ADC_Result [ 4 ] ) / 3 );
                    }

                    DAC_Reading.DAC_mV_5000  = ( uint16_t ) ( DAC_Reading.DAC_ADC_5000  / DAC_5000_ADC_OFFSET  );
                    DAC_Reading.DAC_mV_60000 = ( uint16_t ) ( DAC_Reading.DAC_ADC_60000 / DAC_60000_ADC_OFFSET );

                    // Update DAC 5000 & DAC 60000 values with values read by ADC ( modify string )
                    // Update original string with new DAC 5000 & DAC 60000 settings
                    DAC_5000_NewSetting.VarFloat  = ( float ) ( DAC_Reading.DAC_mV_5000  ) / 1000;
                    DAC_60000_NewSetting.VarFloat = ( float ) ( DAC_Reading.DAC_mV_60000 ) / 1000;
                    SensorString_RX [ 10 ] = ( uint8_t ) ( DAC_60000_NewSetting.VarInt >> 24 );
                    SensorString_RX [  9 ] = ( uint8_t ) ( DAC_60000_NewSetting.VarInt >> 16 );
                    SensorString_RX [  8 ] = ( uint8_t ) ( DAC_60000_NewSetting.VarInt >>  8 );
                    SensorString_RX [  7 ] = ( uint8_t ) ( DAC_60000_NewSetting.VarInt       );
                    SensorString_RX [  6 ] = ( uint8_t ) ( DAC_5000_NewSetting.VarInt  >> 24 );
                    SensorString_RX [  5 ] = ( uint8_t ) ( DAC_5000_NewSetting.VarInt  >> 16 );
                    SensorString_RX [  4 ] = ( uint8_t ) ( DAC_5000_NewSetting.VarInt  >>  8 );
                    SensorString_RX [  3 ] = ( uint8_t ) ( DAC_5000_NewSetting.VarInt        );

                    // Byte stuffing
                    Length        = SensorString_RX [ 2 ];
                    Offset        = 0;
                    Sensor_RxRead = 0;
                    memcpy ( SensorString_TX , SensorString_RX , 3 );
                    for ( Sensor_RxWrite = 0 ; Sensor_RxWrite < Length ; Sensor_RxWrite++ )
                    {
                        if ( 0x10 == SensorString_RX [ Sensor_RxWrite + Offset + 3 ] )  // Byte stuffing required
                        {
                            SensorString_TX [ Sensor_RxWrite + Offset + 3 ] = 0x10;
                            Offset++;
                            SensorString_TX [ Sensor_RxWrite + Offset + 3 ] = 0x10;
                        }
                        else
                        {
                            SensorString_TX [ Sensor_RxWrite + Offset + 3 ] = SensorString_RX [ Sensor_RxWrite + 3 ];
                        }
                    }
                    SensorString_TX [ 3 + Length + Offset     ] = 0x10;
                    SensorString_TX [ 3 + Length + Offset + 1 ] = 0x1F;

                    // Calculate checksum & add to string
                    Length = 3 + Length + Offset + 2;
                    for ( Sensor_RxWrite = 0 ; Sensor_RxWrite < Length ; Sensor_RxWrite++ )
                    {
                        Checksum += SensorString_TX [ Sensor_RxWrite ];
                    }
                    SensorString_TX [ Length     ] = ( uint8_t ) ( Checksum >> 8 );
                    SensorString_TX [ Length + 1 ] = ( uint8_t ) ( Checksum      );

                    // Update sensor
                    g_uiCommsTimeout     = 100;
                    g_uiRxBufferSlaveGet = 0;
                    g_uiRxBufferSlavePut = 0;
                    uart_write_blocking ( UART_SEN , DAC_ChangeSettings , 9 );
                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                    RX_Status = UART_CheckResponse ( );

                    if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                    {
                        g_uiRxBufferSlaveGet = 0;
                        g_uiRxBufferSlavePut = 0;
                        uart_write_blocking ( UART_SEN , SensorString_TX , Length + 2 );
                        while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                        RX_Status = UART_CheckResponse ( );

                        if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                        {
                            // Set DAC VMON
                            g_uiCommsTimeout     = 100;
                            g_uiRxBufferSlaveGet = 0;
                            g_uiRxBufferSlavePut = 0;
                            uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                            while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                            RX_Status = UART_CheckResponse ( );

                            // Set DAC VMON
                            if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                            {
                                g_uiRxBufferSlaveGet = 0;
                                g_uiRxBufferSlavePut = 0;
                                uart_write_blocking ( UART_SEN , DAC_VMON_4V , 8 );
                                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                                RX_Status = UART_CheckResponse ( );

                                if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                                {
                                    // Set DAC 5000
                                    g_uiCommsTimeout     = 100;
                                    g_uiRxBufferSlaveGet = 0;
                                    g_uiRxBufferSlavePut = 0;
                                    uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                                    while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                                    RX_Status = UART_CheckResponse ( );

                                    if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                                    {
                                        g_uiRxBufferSlaveGet = 0;
                                        g_uiRxBufferSlavePut = 0;
                                        uart_write_blocking ( UART_SEN , DAC_5000 , 8 );
                                        while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                                        RX_Status = UART_CheckResponse ( );

                                        if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                                        {
                                            // Set DAC 60000
                                            g_uiCommsTimeout     = 100;
                                            g_uiRxBufferSlaveGet = 0;
                                            g_uiRxBufferSlavePut = 0;
                                            uart_write_blocking ( UART_SEN , DAC_Change , 9 );
                                            while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                                            RX_Status = UART_CheckResponse ( );

                                            if ( ( RX_ACK == RX_Status ) && g_uiCommsTimeout )
                                            {
                                                g_uiRxBufferSlaveGet = 0;
                                                g_uiRxBufferSlavePut = 0;
                                                uart_write_blocking ( UART_SEN , DAC_60000 , 8 );
                                                while ( ( ( g_uiRxBufferSlaveGet + 2 ) > g_uiRxBufferSlavePut ) && g_uiCommsTimeout );  // Wait for ACK / NAK
                                                RX_Status = UART_CheckResponse ( );
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
                    Retry++;
                }
                else
                {
                    // Force exit from do / while
                    Retry = ADC_MAX_RETRY;
                }

            } while ( Retry < ADC_MAX_RETRY );

            if ( ( ( DAC_Zero_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_Zero ) || ( ( DAC_Zero_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_Zero ) || ( ( DAC_FSD_Setting_mV - DAC_ERROR_RANGE_MV ) > DAC_Reading.DAC_mV_FSD ) || ( ( DAC_FSD_Setting_mV + DAC_ERROR_RANGE_MV ) < DAC_Reading.DAC_mV_FSD ) ) 
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
            SensorPass [ 3 ] = ( SensorPos + 1 );
        }
    }
    else if ( DAC_CHECK_RESET == g_DAC_Check_Option )
    {
        memset ( SensorPass , 0 , sizeof ( SensorPass ) );
    }
    else    // Invalid option
    {
        // Nothing to do
    }

    b_DAC_Ready        = true;
    g_DAC_Check_Option = DAC_CHECK_IDLE;  // Reset button press data
    
    // Service watchdog before re-entering main program
    Watchdog ( );
}

void MUX_Set ( uint8_t sensor )
{
    if ( gpio_get ( SENSOR_PLATINUM ) )
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

    else    // Axiom
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
                EN1_HIGH;
                EN2_LOW;
                EN3_LOW;
            break;

            case 5:
                A2_HIGH;
                A1_LOW;
                A0_HIGH;
                EN1_HIGH;
                EN2_LOW;
                EN3_LOW;
            break;

            case 6:
                A2_HIGH;
                A1_HIGH;
                A0_LOW;
                EN1_HIGH;
                EN2_LOW;
                EN3_LOW;
            break;

            case 7:
                A2_HIGH;
                A1_HIGH;
                A0_HIGH;
                EN1_HIGH;
                EN2_LOW;
                EN3_LOW;
            break;

            case 8:
                A2_LOW;
                A1_LOW;
                A0_LOW;
                EN1_LOW;
                EN2_HIGH;
                EN3_LOW;
            break;

            case 9:
                A2_LOW;
                A1_LOW;
                A0_HIGH;
                EN1_LOW;
                EN2_HIGH;
                EN3_LOW;
            break;

            case 10:
                A2_LOW;
                A1_HIGH;
                A0_LOW;
                EN1_LOW;
                EN2_HIGH;
                EN3_LOW;
            break;

            case 11:
                A2_LOW;
                A1_HIGH;
                A0_HIGH;
                EN1_LOW;
                EN2_HIGH;
                EN3_LOW;
            break;

            case 12:
                A2_HIGH;
                A1_LOW;
                A0_LOW;
                EN1_LOW;
                EN2_HIGH;
                EN3_LOW;
            break;

            case 13:
                A2_HIGH;
                A1_LOW;
                A0_HIGH;
                EN1_LOW;
                EN2_HIGH;
                EN3_LOW;
            break;

            case 14:
                A2_HIGH;
                A1_HIGH;
                A0_LOW;
                EN1_LOW;
                EN2_HIGH;
                EN3_LOW;
            break;

            case 15:
                A2_HIGH;
                A1_HIGH;
                A0_HIGH;
                EN1_LOW;
                EN2_HIGH;
                EN3_LOW;
            break;

            case 16:
                A2_LOW;
                A1_LOW;
                A0_LOW;
                EN1_LOW;
                EN2_LOW;
                EN3_HIGH;
            break;

            case 17:
                A2_LOW;
                A1_LOW;
                A0_HIGH;
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
                A2_LOW;
                A1_HIGH;
                A0_HIGH;
                EN1_LOW;
                EN2_LOW;
                EN3_HIGH;
            break;

            case 20:
                A2_HIGH;
                A1_LOW;
                A0_LOW;
                EN1_LOW;
                EN2_LOW;
                EN3_HIGH;
            break;

            case 21:
                A2_HIGH;
                A1_LOW;
                A0_HIGH;
                EN1_LOW;
                EN2_LOW;
                EN3_HIGH;
            break;

            case 22:
                A2_HIGH;
                A1_HIGH;
                A0_LOW;
                EN1_LOW;
                EN2_LOW;
                EN3_HIGH;
            break;

            case 23:
                A2_HIGH;
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
}

/*** end of file ***/
