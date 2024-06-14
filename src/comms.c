/******************************************************************************
 * Project:         24-way Premier IR sensor jig                              *
 * Filename:        comms.c                                                   *
 * Author:          Craig Hemingway                                           *
 * Company:         Dynament Ltd.                                             *
 *                  Status Scientific Controls Ltd.                           *
 * Date:            12/06/2024                                                *
 * File Version:   	4.1.2                                                     *
 * Version history: See separate file Release Notes.txt                       *
 * Hardware:        RP2040                                                    *
 * Tools Used:      Visual Studio Code -> 1.73.1                              *
 *                  Compiler           -> GCC 11.3.1 arm-none-eabi            *
 *                                                                            *
 ******************************************************************************/

#include <comms.h>

#include <main.h>
#include <dac.h>
#include <p2p.h>
#include <stdbool.h>
#include <string.h>

extern uint8_t g_aucRxBufferMaster [ ];
extern uint8_t g_aucRxBufferSlave  [ ];

extern volatile bool     g_b_ucFlagJigCommand;
extern volatile bool     g_b_ucPassThroughMode;
extern volatile bool     g_b_ucPcCommsFlag;
extern volatile bool     g_b_ucSensorCommsFlag;
extern volatile uint16_t g_uiCommsMode;
extern volatile uint16_t g_uiCommsTimeout;
extern volatile uint16_t g_uiRxBufferMasterGet;
extern volatile uint16_t g_uiRxBufferMasterPut;
extern volatile uint16_t g_uiRxBufferSlaveGet;
extern volatile uint16_t g_uiRxBufferSlavePut;

bool     g_b_ucJig         = false;
uint8_t  g_ucJigCommand    = 0;
uint8_t  g_ucSeq           = 0;
uint16_t g_uiChecksumSlave = 0;
uint16_t g_uiSerialNo      = 0;

/* Private function prototypes -----------------------------------------------*/
static void p2pTxByteMaster ( uint8_t ucData );
static void p2pTxByteSlave  ( uint8_t ucData );

/* User code -----------------------------------------------------------------*/
uint8_t p2pRxByteMaster ( uint8_t* pucData )
{
    uint8_t ucStatus = 0;

    if ( g_uiRxBufferMasterGet != g_uiRxBufferMasterPut )
    {
        *pucData = g_aucRxBufferMaster [ g_uiRxBufferMasterGet++ ];

        if ( P2P_BUFFER_MASTER_SIZE == g_uiRxBufferMasterGet )
        {
            g_uiRxBufferMasterGet = 0;
        }
        else
        {
            // Nothing to do
        }

        ucStatus = p2pRxOk;
    }
    else
    {
        ucStatus = p2pRxNone;
    }
    Watchdog();  // added as the 5 second wait period for broken messages is triggering a watchdog reset
    return ucStatus;
}

uint8_t p2pRxByteSlave ( uint8_t* pucData )
{
    uint8_t ucStatus = p2pRxNone;

    if ( g_uiRxBufferSlaveGet != g_uiRxBufferSlavePut )
    {
        // Get byte out of buffer
        *pucData = g_aucRxBufferSlave [ g_uiRxBufferSlaveGet++ ];

        if ( P2P_BUFFER_SLAVE_SIZE == g_uiRxBufferSlaveGet )
        {
            g_uiRxBufferSlaveGet = 0;
        }
        else
        {
            // Nothing to do
        }

        ucStatus = p2pRxOk;
    }
    else
    {
        ucStatus = p2pRxNone;
    }
    Watchdog();  // added as the 5 second wait period for broken messages is triggering a watchdog reset

    return ucStatus;
}

uint16_t p2pPollMaster ( void )
{
    bool     b_ucWaitMaster      = true;
    uint8_t  ucChecksumType      = CSUM_ERROR;
    uint8_t  ucPosn              = 0;
    uint8_t  ucRxByte            = 0;
    uint8_t  ucStatus            = 0;
    uint16_t uiBufferStartPosn   = 0;
    uint16_t uiChecksumMaster    = 0;
    uint16_t uiChecksumMasterCRC = 0;
    uint16_t uiLoop              = 0;
    uint16_t uiMuxPosn           = 0;
    uint16_t uiRxChecksumMaster  = 0;

    g_b_ucJig        = false;
    g_uiCommsMode    = COMMS_WAIT;
    g_uiCommsTimeout = COMMS_TIMEOUT;

    
    while ( ( true == b_ucWaitMaster ) && g_uiCommsTimeout )  // Wait until command received
    {
        ucStatus = p2pRxByteMaster ( &ucRxByte );

        if ( p2pRxOk == ucStatus )
        {
            g_uiCommsTimeout = COMMS_TIMEOUT;   // Maximum 1 second
            if ( DLE == ucRxByte )
            {
                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                // Search for DLE RD sequence
                if ( ( RD == ucRxByte ) && ( g_uiCommsTimeout ) )
                {
                    g_uiCommsMode = COMMS_READ;
                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                    if ( ( 0xFF == ucRxByte ) && ( g_uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                        if ( g_uiCommsTimeout )
                        {
                            addToChecksum    ( uiChecksumMaster    , ucRxByte );
                            addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                            // This byte is the mux position, 1 to 24
                            // A value outside this indicates that the command is for the Jig and not the sensor
                            uiMuxPosn = ( uint16_t ) ucRxByte;

                            if ( 0 < uiMuxPosn )
                            {
                                g_b_ucFlagJigCommand = false;
                                uiMuxPosn--;    // Position 1 = '0'

                               MUX_Set ( uiMuxPosn );
                            }
                            else
                            {
                                g_b_ucFlagJigCommand = true;
                            }

                            if ( DLE == ucRxByte )  // Position byte 0x10
                            {
                                // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                            }
                            else
                            {
                                // Nothing to do
                            }

                            // Wait for remaining data
                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                            if ( g_uiCommsTimeout )
                            {
                                g_ucJigCommand = ucRxByte;
                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                if ( DLE == ucRxByte )  // Command byte 0x10
                                {
                                    // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                if ( g_uiCommsTimeout )
                                {
                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                    if ( g_uiCommsTimeout )
                                    {
                                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                        // Next 2 bytes are the checksum
                                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                        if ( g_uiCommsTimeout )
                                        {
                                            uiRxChecksumMaster = ucRxByte;
                                            uiRxChecksumMaster <<= 8;

                                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                            if ( g_uiCommsTimeout )
                                            {
                                                uiRxChecksumMaster += ucRxByte;

                                                if ( uiChecksumMaster == uiRxChecksumMaster )
                                                {
                                                    ucChecksumType = CSUM_SIMPLE;
                                                }
                                                else if ( uiChecksumMasterCRC == uiRxChecksumMaster )
                                                {
                                                    ucChecksumType = CSUM_CRC;
                                                }
                                                else
                                                {
                                                    // Checksum error
                                                    ucChecksumType = CSUM_ERROR;
                                                    // Reason
                                                    p2pTxByteMaster ( DLE );
                                                    p2pTxByteMaster ( NAK );
                                                    p2pTxByteMaster ( eP2pNAKchecksumFailed );
                                                }

                                                // Wait minimum 10 milliseconds for multiplexers to stabalise
                                                sleep_ms ( 10 );

                                                if ( true == g_b_ucJig )
                                                {
                                                    // Remove sequencing bytes
                                                    memmove ( ( g_aucRxBufferMaster ) , ( g_aucRxBufferMaster + 3 ) , P2P_BUFFER_MASTER_SIZE - 3 );
                                                    g_uiRxBufferMasterGet -= 3;
                                                }
                                                else
                                                {
                                                    // Nothing to do
                                                }

                                                g_uiChecksumSlave = 0;

                                                if ( false == g_b_ucFlagJigCommand )
                                                {
                                                    // Forward message on to sensor
                                                    // Strip out 0xFF, position bytes preforming byte stuffing
                                                    for ( uiLoop = uiBufferStartPosn ; uiLoop < ( g_uiRxBufferMasterGet - 2 ) ; uiLoop++ )
                                                    {
                                                        if ( ( uiBufferStartPosn + 2 ) == uiLoop )
                                                        {
                                                            // Do not send 0xFF
                                                        }
                                                        else if ( ( uiBufferStartPosn + 3 ) == uiLoop )
                                                        {
                                                            // Do not send position
                                                        }
                                                        else if ( ( ( uiBufferStartPosn + 4 ) == uiLoop ) && ( DLE == g_aucRxBufferMaster [ uiBufferStartPosn + 3 ] ) )
                                                        {
                                                            // Do not send DLE when position == 0x10
                                                        }
                                                        else
                                                        {
                                                            p2pTxByteSlave ( g_aucRxBufferMaster [ uiLoop ] );

                                                            if ( CSUM_SIMPLE == ucChecksumType )
                                                            {
                                                                addToChecksum ( g_uiChecksumSlave , g_aucRxBufferMaster [ uiLoop ] );
                                                            }
                                                            else
                                                            {
                                                                addToChecksumCRC ( g_uiChecksumSlave , g_aucRxBufferMaster [ uiLoop ] );
                                                            }
                                                        }
                                                    }
                                                    // Send the re-calculated checksum
                                                    p2pTxByteSlave ( ( g_uiChecksumSlave >> 8   ) );
                                                    p2pTxByteSlave ( ( g_uiChecksumSlave & 0xFF ) );
                                                }
                                                else
                                                {
                                                    // Jig read command
                                                    if ( 0xFD == g_ucJigCommand )
                                                    {
                                                        g_uiChecksumSlave = 0;
                                                        // Send the Jig serial number to the calling program
                                                        // uiSerialNo
                                                        p2pTxByteMaster ( DLE );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( g_uiChecksumSlave , DLE );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( g_uiChecksumSlave , DLE );
                                                        }

                                                        p2pTxByteMaster ( DAT );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( g_uiChecksumSlave , DAT );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( g_uiChecksumSlave , DAT );
                                                        }

                                                        p2pTxByteMaster ( 0x02 );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( g_uiChecksumSlave , 0x02 );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( g_uiChecksumSlave , 0x02 );
                                                        }

                                                        p2pTxByteMaster ( g_uiSerialNo & 0xff );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( g_uiChecksumSlave , ( g_uiSerialNo >> 8 ) );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( g_uiChecksumSlave , ( g_uiSerialNo >> 8 ));
                                                        }

                                                        p2pTxByteMaster ( g_uiSerialNo >> 8 );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( g_uiChecksumSlave , ( g_uiSerialNo & 0xFF ) );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( g_uiChecksumSlave , ( g_uiSerialNo & 0xFF ) );
                                                        }

                                                        p2pTxByteMaster ( DLE );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( g_uiChecksumSlave , DLE );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( g_uiChecksumSlave , DLE );
                                                        }

                                                        p2pTxByteMaster ( EoF );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( g_uiChecksumSlave , EoF );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( g_uiChecksumSlave , EoF );
                                                        }

                                                        p2pTxByteMaster ( g_uiChecksumSlave >> 8   );
                                                        p2pTxByteMaster ( g_uiChecksumSlave & 0xFF );

                                                        g_b_ucFlagJigCommand = false;   // Force exit
                                                        g_ucJigCommand     = 0;
                                                    }
                                                    else
                                                    {
                                                        // Nothing to do
                                                    }
                                                    b_ucWaitMaster = false;   // Force ecit
                                                }
                                                b_ucWaitMaster = false;   // Force exit
                                            }
                                            else
                                            {
                                                b_ucWaitMaster = false;   // Force exit
                                            }
                                        }
                                        else
                                        {
                                            b_ucWaitMaster = false;   // Force exit
                                        }
                                    }
                                    else
                                    {
                                        b_ucWaitMaster = false;   // Force exit
                                    }
                                }
                                else
                                {
                                    b_ucWaitMaster = false;   // Force exit
                                }
                            }
                            else
                            {
                                b_ucWaitMaster = false;   // Force exit
                            }
                        }
                        else
                        {
                            b_ucWaitMaster = false;   // Force exit
                        }
                    }
                    else
                    {
                        // Test for 24-way jig sequence request
                        if ( ( 0x00 == ucRxByte ) && ( g_uiCommsTimeout ) )
                        {
//                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );
                        }
                        else
                        {
                            // Nothing to do
                        }

//                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );
//                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );
//                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );
//                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );
//                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                        // Clear buffer
                        memset ( g_aucRxBufferMaster , 0 , P2P_BUFFER_SLAVE_SIZE );
                        g_b_ucJig             = true;
                        g_uiRxBufferMasterGet = 0;
                        g_uiRxBufferMasterPut = 0;

                        // Send out standard response
                        p2pTxByteMaster ( DLE   );
                        p2pTxByteMaster ( SEQ   );
                        p2pTxByteMaster ( g_ucSeq );
                        p2pTxByteMaster ( DLE   );
                        p2pTxByteMaster ( DAT   );
                        p2pTxByteMaster ( 0x02  );
                        p2pTxByteMaster ( 0x01  );
                        p2pTxByteMaster ( 0x18  );
                        p2pTxByteMaster ( DLE   );
                        p2pTxByteMaster ( EoF   );
                        p2pTxByteMaster ( 0x00  );
                        p2pTxByteMaster ( 0x74  );
                    }
                }

                // Search for DLE WR sequence
                else if ( ( WR == ucRxByte ) && ( g_uiCommsTimeout ) )
                {
                    g_uiCommsTimeout = 100;   // Maximum wait time of 1 second for write operation

                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                    if ( ( WP1 == ucRxByte ) && ( g_uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                    }
                    else
                    {
                        // Nothing to do
                    }

                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                    if ( ( WP2 == ucRxByte ) && ( g_uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                    }
                    else
                    {
                        // Nothing to do
                    }

                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                    if ( ( ( 0xFF == ucRxByte ) || ( 0xFE == ucRxByte ) ) && ( g_uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                        if ( 0xFE == ucRxByte )
                        {
                            g_b_ucPassThroughMode = true;
                            NOP;
                        }
                        else
                        {
                            g_b_ucPassThroughMode = false;
                        }

                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                        if ( g_uiCommsTimeout )
                        {
                            addToChecksum    ( uiChecksumMaster, ucRxByte     );
                            addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                            ucPosn    = ucRxByte;
                            uiMuxPosn = ( uint16_t ) ucPosn;

                            if ( 0 < uiMuxPosn )
                            {
                                g_b_ucFlagJigCommand = false;
                                uiMuxPosn--;    // Position 1 = '0'

                                // This byte is the mux posn
                                uiMuxPosn = ( uint16_t ) ucRxByte - 1;  // Position 1 = '0'
                                MUX_Set ( uiMuxPosn );
                            }
                            else
                            {
                                g_b_ucFlagJigCommand = true;
                            }

                            if ( DLE == ucRxByte )  // Position byte 0x10
                            {
                                // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                            }
                            else
                            {
                                // Nothing to do
                            }

                            if ( false == g_b_ucPassThroughMode )
                            {
                                // Wait for remaining data
                                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                g_ucJigCommand = ucRxByte;
                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                            }
                            else
                            {
                                // No byte after 0xFE passthrough byte
                                NOP;
                            }

                            if ( g_uiCommsTimeout )
                            {
                                if ( DLE == ucRxByte )  // Command byte 0x10
                                {
                                    // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                if ( g_uiCommsTimeout )
                                {
                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                    if ( true == g_b_ucPassThroughMode )
                                    {
                                        // Nothing to do
                                    }
                                    else
                                    {
                                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                    }

                                    if ( g_uiCommsTimeout )
                                    {
                                        if ( ( true == g_b_ucPassThroughMode ) && ( DLE != ucPosn ) )
                                        {
                                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                            addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                            addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                        }
                                        else
                                        {
                                            NOP;    // Trap for programming position 16 ( =DLE )
                                        }

                                        // Next 2 bytes are the checksum
                                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                        if ( g_uiCommsTimeout )
                                        {
                                            uiRxChecksumMaster = ucRxByte;
                                            uiRxChecksumMaster <<= 8;

                                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                            if ( true == g_b_ucPassThroughMode )
                                            {
                                                NOP;
                                            }
                                            else
                                            {
                                                // Nothing to do
                                            }

                                            if ( g_uiCommsTimeout )
                                            {
                                                uiRxChecksumMaster += ucRxByte;

                                                if ( uiChecksumMaster == uiRxChecksumMaster )
                                                {
                                                    ucChecksumType = CSUM_SIMPLE;
                                                }
                                                else if ( uiChecksumMasterCRC == uiRxChecksumMaster )
                                                {
                                                    ucChecksumType = CSUM_CRC;
                                                }
                                                else
                                                {
                                                    // Checksum error
                                                    ucChecksumType = CSUM_ERROR;
                                                    // Reason
                                                    p2pTxByteMaster ( DLE );
                                                    p2pTxByteMaster ( NAK );
                                                    p2pTxByteMaster ( eP2pNAKchecksumFailed );

                                                }

                                                b_ucWaitMaster = false;

                                                // Wait minimum 10 milliseconds for multiplexers to stabalise
                                                sleep_ms ( 10 );

                                                // Get remaining parts of write command
                                                if ( true == g_b_ucJig )
                                                {
                                                    p2pTxByteMaster ( DLE   );
                                                    p2pTxByteMaster ( SEQ   );
                                                    p2pTxByteMaster ( g_ucSeq );
                                                }
                                                else
                                                {
                                                    // Nothing to do
                                                }

                                                p2pTxByteMaster ( DLE );
                                                p2pTxByteMaster ( ACK );
                                                g_uiCommsMode = COMMS_WRITE;

                                                // Wait for data
                                                b_ucWaitMaster = false;   // Force exit
                                            }
                                            else
                                            {
                                                b_ucWaitMaster = false;   // Force exit
                                            }
                                        }
                                        else
                                        {
                                            b_ucWaitMaster = false;   // Force exit
                                        }
                                    }
                                    else
                                    {
                                        b_ucWaitMaster = false;   // Force exit
                                    }
                                }
                                else
                                {
                                    b_ucWaitMaster = false;   // Force exit
                                }
                            }
                            else
                            {
                                b_ucWaitMaster = false;   // Force exit
                            }
                        }
                        else
                        {
                            b_ucWaitMaster = false;   // Force exit
                        }
                    }
                    else
                    {
                        b_ucWaitMaster = false;   // Force exit
                    }
                }
                else if ( ( SEQ == ucRxByte ) && ( g_uiCommsTimeout ) )
                {
                    // The mentorPC library sends out 0x10, 0x21, 0x?? ( sequence byte )
                    // When the sequence byte == 0x10, this upsets the normal DLE message
                    // Ignore this byte at all times as this message is not implemented

                    // Get sequence byte
                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                    g_b_ucJig             = true;
                    g_ucSeq               = ucRxByte;
                    uiChecksumMaster    = 0;
                    uiChecksumMasterCRC = 0;
                }
                else
                {
                    // Nothing to do
                }
                NOP;
            }
            else
            {
                // Search for DLE WR sequence
                uiChecksumMaster    = 0;
                uiChecksumMasterCRC = 0;
            }
        }
        else
        {
            // Nothing to do
        }
    }

    if ( 0 == g_uiCommsTimeout )
    {
        if ( true == g_b_ucJig )
        {
            p2pTxByteMaster ( DLE );
            p2pTxByteMaster ( NAK );
            p2pTxByteMaster (   1 );
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

    return g_uiCommsTimeout;
}

// Sensor response to READ request
uint16_t p2pPollSlaveRead ( void )
{
    static uint8_t ucError = 0;

    bool     b_ucWaitSlave      = true;
    uint8_t  ucProgressCount    = 0;
    uint8_t  ucRxByte           = 0;
    uint8_t  ucStatus           = 0;
    uint16_t uiDataLength       = 0;
    uint16_t uiDataLengthToSend = 0;
    uint16_t uiLoop             = 0;

    // Wait for sensor response
    while ( ( true == b_ucWaitSlave ) && ( g_uiCommsTimeout ) )
    {
        ucStatus = p2pRxByteSlave ( &ucRxByte );

        if ( p2pRxOk == ucStatus )
        {
            // Search for DLE DAT sequence
            if ( DLE == ucRxByte )
            {
                while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                if ( DAT == ucRxByte )
                {
                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                    uiDataLength       = ucRxByte;
                    uiDataLengthToSend = uiDataLength + 7;  // Includes start & end of frame bytes

                    // Wait for remaining bytes
                    while ( uiDataLength && g_uiCommsTimeout )
                    {
                        while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                        if ( DLE == ucRxByte )
                        {
                            // Wait for next DLE - Byte stuffing extra byte not included in data length
                            while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                            uiDataLength--;
                            uiDataLengthToSend++;   // Include byte stuffing bytes
                        }
                        else
                        {
                            uiDataLength--;
                        }
                    }

                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );  // DLE
                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );  // EoF
                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );  // Checksum high byte
                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );  // Checksum low byte

                    // Send sequence if originally requested
                    if ( true == g_b_ucJig )
                    {
                        p2pTxByteMaster ( DLE   );
                        p2pTxByteMaster ( SEQ   );
                        p2pTxByteMaster ( g_ucSeq );
                    }
                    else
                    {
                        // Nothing to do
                    }

                    // Send buffer back to PC
                    for ( uiLoop = ( g_uiRxBufferSlavePut - uiDataLengthToSend ) ; uiLoop < uiDataLengthToSend ; uiLoop++ )
                    {
                        p2pTxByteMaster ( g_aucRxBufferSlave [ uiLoop ] );
                    }

                    ucProgressCount++;

                    b_ucWaitSlave = false;
                }
                else if ( NAK == ucRxByte )
                {
                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );  // Wait for reason
                    
                    // Send to PC
                    p2pTxByteMaster ( DLE );
                    p2pTxByteMaster ( NAK );
                    p2pTxByteMaster ( ucRxByte );   // Send reason
                    b_ucWaitSlave = false;            // Exit
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

    if ( 0 == g_uiCommsTimeout )
    {
        ucError++;

        p2pTxByteMaster ( DLE   );
        p2pTxByteMaster ( SEQ   );
        p2pTxByteMaster ( g_ucSeq );

        p2pTxByteMaster ( DLE  );
        p2pTxByteMaster ( NAK  );
        p2pTxByteMaster ( 0x82 ); // Send reason
    }
    else
    {
        // Nothing to do
    }

    // Clear buffers
    memset ( g_aucRxBufferMaster , 0 , P2P_BUFFER_MASTER_SIZE );
    g_uiRxBufferMasterGet = 0;
    g_uiRxBufferMasterPut = 0;

    memset ( g_aucRxBufferSlave , 0 , P2P_BUFFER_SLAVE_SIZE );
    g_uiRxBufferSlavePut = 0;
    g_uiRxBufferSlavePut = 0;

    return g_uiCommsTimeout;
}

// Sensor WRITE request
uint16_t p2pPollSlaveWrite ( void )
{
    bool     b_ucWaitSlave       = true;
    uint8_t  ucChecksumType      = CSUM_ERROR;
    uint8_t  ucDataLength        = 0;
    uint8_t  ucDataLengthToSend  = 0;
    uint8_t  ucRxByte            = 0;
    uint8_t  ucStatus            = 0;
    uint8_t  g_ucSequenceOffset  = 0;
    uint16_t uiBaudRate          = 38400;
    uint16_t uiBufferEndPosn     = 0;
    uint16_t uiBufferStartPosn   = 0;
    uint16_t uiChecksumMaster    = 0;
    uint16_t uiChecksumMasterCRC = 0;
    uint16_t uiLoop              = 0;
    uint16_t uiRxChecksumMaster  = 0;

    // Wait for sensor response
    while ( ( true == b_ucWaitSlave ) && ( g_uiCommsTimeout ) )
    {

        ucStatus = p2pRxByteMaster ( &ucRxByte );

        if ( p2pRxOk == ucStatus )
        {
            g_uiCommsTimeout = 500; // 5 seconds - Temperature compensation command write takes a long time

            if ( DLE == ucRxByte )
            {
                uiBufferStartPosn = g_uiRxBufferMasterPut - 1;

                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                // Search for DLE DAT sequence)
                if ( ( DAT == ucRxByte ) && ( g_uiCommsTimeout ) )
                {
                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                    
                    // Wait for length
                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );
                    
                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    ucDataLength       = ucRxByte;
                    ucDataLengthToSend = ucDataLength + 7;  // Include start & end of frame bytes

                    // Wait for remaining bytes
                    while ( ucDataLength && g_uiCommsTimeout )
                    {
                        
                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );
                        
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                        
                        if ( DLE == ucRxByte )
                        {
                            // Wait for next DLE - Byte stuffing extra byte not included in data length
                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );
                            
                            addToChecksum    ( uiChecksumMaster    , ucRxByte );
                            addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                            ucDataLength--;
                            ucDataLengthToSend++;   // Include byte stuffing bytes
                        }
                        else
                        {
                            ucDataLength--;
                        }
                    }

                    // wait for DLE
                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    // Wait for EOF
                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    // Next 2 bytes are the checksum
                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );
                    
                    if ( g_uiCommsTimeout )
                    {
                        uiRxChecksumMaster = ucRxByte;
                        uiRxChecksumMaster <<= 8;

                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                        if  ( g_uiCommsTimeout )
                        {
                            uiRxChecksumMaster += ucRxByte;

                            if ( uiChecksumMaster == uiRxChecksumMaster )
                            {
                                ucChecksumType = CSUM_SIMPLE;
                            }
                            else if ( uiChecksumMasterCRC == uiRxChecksumMaster )
                            {
                                ucChecksumType = CSUM_CRC;
                            }
                            else
                            {
                                // Checksum error
                                ucChecksumType = CSUM_ERROR;
                                // Reason
                                if ( true == g_b_ucJig )
                                {
                                    p2pTxByteMaster ( DLE   );
                                    p2pTxByteMaster ( SEQ   );
                                    p2pTxByteMaster ( g_ucSeq );
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                p2pTxByteMaster ( DLE );
                                p2pTxByteMaster ( NAK );
                                p2pTxByteMaster ( eP2pNAKchecksumFailed );
                            }
                        }
                        else
                        {
                            // Nothing to do
                        }

                        if ( CSUM_ERROR != ucChecksumType )
                        {
                            if ( true == g_b_ucJig )
                            {
                                // Ignore sequence bytes
                                g_ucSequenceOffset  = 3;
                                uiBufferEndPosn   = 12;
                                uiBufferStartPosn = 3;
                            }
                            else
                            {
                                g_ucSequenceOffset  = 0;
                                uiBufferEndPosn   = 9;
                                uiBufferStartPosn = 0;
                            }

                            if ( 0xFF == g_aucRxBufferMaster [ 0 ] )
                            {
                                // Ignore programming byte from failed position
                                g_ucSequenceOffset++;
                                uiBufferEndPosn++;
                                uiBufferStartPosn++;
                            }
                            else
                            {
                                // Nothing to do
                            }

                            g_uiChecksumSlave = 0;

                            if ( false == g_b_ucFlagJigCommand )
                            {
                                // Forward write command on to sensor
                                // Strip out 0xFF, position bytes performing byte stuffing
                                for ( uiLoop = uiBufferStartPosn ; uiLoop < uiBufferEndPosn ; uiLoop++ )
                                {
                                    if ( ( uiBufferStartPosn + 4 ) == uiLoop )
                                    {
                                        // Do not send 0xff
                                    }
                                    else if ( ( uiBufferStartPosn + 5 ) == uiLoop )
                                    {
                                        // Do not send position
                                    }
                                    else if ( ( uiBufferStartPosn + 6 ) == uiLoop )
                                    {
                                        // Sensor position 0x10 and Variable ID 0x10
                                        if ( ( DLE == g_aucRxBufferMaster [ uiBufferStartPosn + 5 ] ) && ( DLE == g_aucRxBufferMaster [ uiBufferStartPosn + 7 ] ))
                                        {
                                            // Do not send DLE when position = 0x10
                                            // Take into account extra stuffed byte when Variable ID = 0x10
                                            uiBufferEndPosn++;
                                            uiBufferEndPosn++;
                                        }
                                        // Sensor position 0x10 and any Variable ID other than 0x10
                                        else if ( ( DLE == g_aucRxBufferMaster [ uiBufferStartPosn + 5 ] ) && ( DLE != g_aucRxBufferMaster [ uiBufferStartPosn + 7 ] ))
                                        {
                                            // Do not send DLE when position = 0x10
                                            uiBufferEndPosn++;
                                        }
                                        // // Any sensor other than position 0x10 and Variable ID 0x10
                                        // else if ( ( DLE != g_aucRxBufferMaster [ uiBufferStartPosn + 5 ] ) && ( DLE == g_aucRxBufferMaster [ uiBufferStartPosn + 7 ] ))
                                        // {
                                        //     // Take into account extra stuffed byte when Variable ID = 0x10
                                        //     uiBufferEndPosn++;
                                        // }
                                        else
                                        {
                                            // Any sensor other than position 0x10 and Variable ID 0x10
                                            if ( ( DLE != g_aucRxBufferMaster [ uiBufferStartPosn + 5 ] ) && ( DLE == g_aucRxBufferMaster [ uiBufferStartPosn + 6 ] ))
                                            {
                                                // Take into account extra stuffed byte when Variable ID = 0x10
                                                uiBufferEndPosn++;
                                            }
                                            p2pTxByteSlave ( g_aucRxBufferMaster [ uiLoop ] );

                                            if ( CSUM_SIMPLE == ucChecksumType )
                                            {
                                                addToChecksum ( g_uiChecksumSlave , g_aucRxBufferMaster [ uiLoop ] );
                                            }
                                            else
                                            {
                                                addToChecksumCRC ( g_uiChecksumSlave , g_aucRxBufferMaster [ uiLoop ] );
                                            }
                                        }
                                    }
                                    else
                                    {
                                        p2pTxByteSlave ( g_aucRxBufferMaster [ uiLoop ] );
                                        if ( CSUM_SIMPLE == ucChecksumType )
                                        {
                                            addToChecksum ( g_uiChecksumSlave , g_aucRxBufferMaster [ uiLoop ] );
                                        }
                                        else
                                        {
                                            addToChecksumCRC ( g_uiChecksumSlave , g_aucRxBufferMaster [ uiLoop ] );
                                        }
                                    }
                                }

                                uiBufferStartPosn += uiLoop;
                                uiBufferStartPosn -= 1;

                                // Reset sensor response pointers
                                g_uiRxBufferSlaveGet = 0;
                                g_uiRxBufferSlavePut = 0;

                                // Send re-calculated checksum
                                p2pTxByteSlave ( ( g_uiChecksumSlave >> 8   ) );
                                p2pTxByteSlave ( ( g_uiChecksumSlave & 0xff ) );

                                // Wait for acknowledge from sensor
                                // Wait for sensor response
                                while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                if ( 0 == g_uiCommsTimeout )
                                {
                                    if ( true == g_b_ucJig )
                                    {
                                        p2pTxByteMaster ( DLE   );
                                        p2pTxByteMaster ( SEQ   );
                                        p2pTxByteMaster ( g_ucSeq );
                                    }
                                    else
                                    {
                                        // Nothing to do
                                    }

                                    // Timeout message
                                    p2pTxByteMaster ( DLE );
                                    p2pTxByteMaster ( NAK );
                                    p2pTxByteMaster ( eP2pNAKdeviceFault );
                                }
                                else
                                {
                                    if ( DLE == ucRxByte )
                                    {
                                        // Wait for sensor response
                                        while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                        if ( 0 == g_uiCommsTimeout )
                                        {
                                            if ( true == g_b_ucJig )
                                            {
                                                p2pTxByteMaster ( DLE );
                                                p2pTxByteMaster ( SEQ   );
                                                p2pTxByteMaster ( g_ucSeq );
                                            }
                                            else
                                            {
                                                // Nothing to do
                                            }

                                            // Timeout message
                                            p2pTxByteMaster ( DLE );
                                            p2pTxByteMaster ( NAK );
                                            p2pTxByteMaster ( eP2pNAKdeviceFault );
                                        }
                                        else if ( ACK == ucRxByte )
                                        {
                                            // Send data
                                            uiBufferStartPosn = uiLoop + 2; // Skip checksum bytes

                                            // Forward data to sensor as-is
                                            for ( uiLoop = uiBufferStartPosn ; uiLoop < ( uiBufferStartPosn + ucDataLengthToSend ) ; uiLoop++ )
                                            {
                                                p2pTxByteSlave ( g_aucRxBufferMaster [ uiLoop ] );
                                            }

                                            // Sensor takes about 7.5 seconds to carry out command
                                            g_uiCommsTimeout = 1000;  // Max timeout of 10 seconds

                                            // Wait for sensor response
                                            while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                            if ( 0 == g_uiCommsTimeout )
                                            {
                                                if ( true == g_b_ucJig )
                                                {
                                                    p2pTxByteMaster ( DLE   );
                                                    p2pTxByteMaster ( SEQ   );
                                                    p2pTxByteMaster ( g_ucSeq );
                                                }
                                                else
                                                {
                                                    // Nothing to do
                                                }

                                                // Timeout message
                                                p2pTxByteMaster ( DLE );
                                                p2pTxByteMaster ( NAK );
                                                p2pTxByteMaster ( eP2pNAKdeviceFault );
                                            }
                                            else
                                            {
                                                if ( DLE == ucRxByte )
                                                {
                                                    // Wait for sensor response
                                                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                                    if ( 0 == g_uiCommsTimeout )
                                                    {
                                                        if ( true == g_b_ucJig )
                                                        {
                                                            p2pTxByteMaster ( DLE   );
                                                            p2pTxByteMaster ( SEQ   );
                                                            p2pTxByteMaster ( g_ucSeq );
                                                        }
                                                        else
                                                        {
                                                            // Nothing to do
                                                        }

                                                        // Timeout message
                                                        p2pTxByteMaster ( DLE );
                                                        p2pTxByteMaster ( NAK );
                                                        p2pTxByteMaster ( eP2pNAKdeviceFault );
                                                    }
                                                    else if ( ACK == ucRxByte )
                                                    {
                                                        if ( true == g_b_ucJig )
                                                        {
                                                            p2pTxByteMaster ( DLE   );
                                                            p2pTxByteMaster ( SEQ   );
                                                            p2pTxByteMaster ( g_ucSeq );
                                                        }
                                                        else
                                                        {
                                                            // Nothing to do
                                                        }

                                                        // Correct response
                                                        p2pTxByteMaster ( DLE );
                                                        p2pTxByteMaster ( ACK );
                                                        NOP;
                                                    }
                                                    else if ( NAK == ucRxByte )
                                                    {
                                                        // NAK response
                                                        // Now wait for sensor reason
                                                        while ( ( p2pRxOk != ( p2pRxByteSlave( &ucRxByte ) ) ) && ( g_uiCommsTimeout ) );

                                                        if ( true == g_b_ucJig )
                                                        {
                                                            p2pTxByteMaster ( DLE   );
                                                            p2pTxByteMaster ( SEQ   );
                                                            p2pTxByteMaster ( g_ucSeq );
                                                        }
                                                        else
                                                        {
                                                            // Nothing to do
                                                        }

                                                        p2pTxByteMaster ( DLE );
                                                        p2pTxByteMaster ( NAK );
                                                        p2pTxByteMaster ( ucRxByte );   // Send reason
                                                    }
                                                    else
                                                    {
                                                        if ( true == g_b_ucJig )
                                                        {
                                                            p2pTxByteMaster ( DLE   );
                                                            p2pTxByteMaster ( SEQ   );
                                                            p2pTxByteMaster ( g_ucSeq );
                                                        }
                                                        else
                                                        {
                                                            // Nothing to do
                                                        }

                                                        // Incorrect response
                                                        p2pTxByteMaster ( DLE );
                                                        p2pTxByteMaster ( NAK );
                                                        p2pTxByteMaster ( eP2pNAKinvalidData );
                                                    }
                                                }
                                                else
                                                {
                                                    if ( true == g_b_ucJig )
                                                    {
                                                        p2pTxByteMaster ( DLE   );
                                                        p2pTxByteMaster ( SEQ   );
                                                        p2pTxByteMaster ( g_ucSeq );
                                                    }
                                                    else
                                                    {
                                                        // Nothing to do
                                                    }

                                                    // Incorrect response
                                                    p2pTxByteMaster ( DLE );
                                                    p2pTxByteMaster ( NAK );
                                                    p2pTxByteMaster ( eP2pNAKinvalidData );
                                                }
                                            }
                                        }
                                        else if ( NAK == ucRxByte )
                                        {
                                            // NAK response
                                            // Now wait for sensor reason
                                            while ( ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk ) && ( g_uiCommsTimeout ) );

                                            if ( true == g_b_ucJig )
                                            {
                                                p2pTxByteMaster ( DLE   );
                                                p2pTxByteMaster ( SEQ   );
                                                p2pTxByteMaster ( g_ucSeq );
                                            }
                                            else
                                            {
                                                // Nothing to do
                                            }

                                            p2pTxByteMaster ( DLE );
                                            p2pTxByteMaster ( NAK );
                                            p2pTxByteMaster ( ucRxByte );   // Send reason
                                        }
                                        else
                                        {
                                            if ( true == g_b_ucJig )
                                            {
                                                p2pTxByteMaster ( DLE   );
                                                p2pTxByteMaster ( SEQ   );
                                                p2pTxByteMaster ( g_ucSeq );
                                            }
                                            else
                                            {
                                                // Nothing to do
                                            }

                                            // Incorrect response
                                            p2pTxByteMaster ( DLE );
                                            p2pTxByteMaster ( NAK );
                                            p2pTxByteMaster ( eP2pNAKinvalidData );
                                        }
                                    }
                                    else
                                    {
                                        if ( true == g_b_ucJig )
                                        {
                                            p2pTxByteMaster ( DLE   );
                                            p2pTxByteMaster ( SEQ   );
                                            p2pTxByteMaster ( g_ucSeq );
                                        }
                                        else
                                        {
                                            // Nothing to do
                                        }

                                        // Incorrect response
                                        p2pTxByteMaster ( DLE  );
                                        p2pTxByteMaster ( NAK  );
                                        p2pTxByteMaster ( 0x82 );   // Old style Jig expects 0x82 on failed position
                                    }
                                    b_ucWaitSlave = false;
                                }
                            }
                            else
                            {
                                // Jig write command
                                // Incorrect response
                                p2pTxByteMaster ( DLE );
                                p2pTxByteMaster ( ACK );

                                if ( 0xFB == g_ucJigCommand )
                                {
                                    // Power command
                                    switch ( g_aucRxBufferMaster [ 14 ] )
                                    {
                                        case 0:
                                            uiBaudRate = 4800;
                                        break;

                                        case 1:
                                            uiBaudRate = 9600;
                                        break;
                                        
                                        case 2:
                                            uiBaudRate = 19200;
                                        break;

                                        default:
                                            uiBaudRate = 38400;
                                        break;
                                    }
                                    BaudRate_Update ( uiBaudRate );
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                if ( 0xFC == g_ucJigCommand )
                                {
                                    // Power command
                                    if ( 0x50 == g_aucRxBufferMaster [ 14 ] )
                                    {
                                        if ( gpio_get ( SENSOR_PLATINUM ) )
                                        {
                                            RELAY_OFF;  // Turn off power to sensors
                                        }
                                        else
                                        {
                                            // Nothing to do
                                        }
                                    }
                                    else
                                    {
                                        RELAY_ON;   // Turn on power to sensors
                                    }

                                    // Wait for 2 seconds before the main loop re-enables the Power to the sensors
                                    // Wait 15 secs in total for sensors to startup
                                    g_uiCommsTimeout = 200;   // 2 seconds

                                    while ( g_uiCommsTimeout );
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                if ( 0xFD == g_ucJigCommand )
                                {
                                    // Write Jig Serial No.
                                    g_uiSerialNo = g_aucRxBufferMaster [ 15 ];
                                    g_uiSerialNo <<= 8;
                                    g_uiSerialNo |= g_aucRxBufferMaster [ 14 ];
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                b_ucWaitSlave = false;
                            }
                        }
                        else
                        {
                            b_ucWaitSlave = false;
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

    memset ( g_aucRxBufferMaster , 0 , P2P_BUFFER_MASTER_SIZE );

    g_uiRxBufferMasterGet = 0;
    g_uiRxBufferMasterPut = 0;

    memset ( g_aucRxBufferSlave , 0 , P2P_BUFFER_SLAVE_SIZE );

    g_uiRxBufferSlaveGet = 0;
    g_uiRxBufferSlavePut = 0;

    if ( !g_uiCommsTimeout )
    {
        return g_uiCommsTimeout;
    }

    return g_uiCommsTimeout;
}

uint16_t p2pPollSlaveWritePassThrough ( void )
{
    bool     b_ucProgramFlag  = false;
    uint8_t  ucPosition       = 0;
    uint8_t  ucRxByteMaster   = 0;
    uint16_t uiBytesToReceive = 0;
    uint16_t uiLines          = 0;

    ucPosition = g_aucRxBufferMaster [ 8 ];

    if ( 16 == ucPosition )
    {
        uiBytesToReceive = 0x15;
    }
    else
    {
        uiBytesToReceive = 0x14;
    }

    if ( g_uiCommsTimeout )
    {
        p2pTxByteMaster ( DLE   );
        p2pTxByteMaster ( SEQ   );
        p2pTxByteMaster ( g_ucSeq );
        p2pTxByteMaster ( DLE   );
        p2pTxByteMaster ( ACK   );

        while ( ( uiBytesToReceive != g_uiRxBufferMasterPut ) && g_uiCommsTimeout );    // Wait until command received

        g_uiRxBufferMasterGet = 0;
        g_uiRxBufferMasterPut = 0;

        g_uiCommsTimeout = 500;   // 5 seconds;

        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByteMaster ) ) ) && ( g_uiCommsTimeout ) );

        if ( 'P' == ucRxByteMaster )
        {
            b_ucProgramFlag = true;

            memset ( g_aucRxBufferMaster , 0 , P2P_BUFFER_MASTER_SIZE );

            g_uiRxBufferMasterGet = 0;
            g_uiRxBufferMasterPut = 0;
            g_uiRxBufferSlaveGet  = 0;
            g_uiRxBufferSlavePut  = 0;

            p2pTxByteSlave ( 'P' );

            while ( ( 1 > g_uiRxBufferSlavePut ) && ( g_uiCommsTimeout ) );

            if ( g_uiCommsTimeout )
            {
                p2pTxByteMaster ( g_aucRxBufferSlave [ 0 ] );
            }
            else
            {
                NOP;
            }

            if ( 0x42 == g_aucRxBufferSlave [ 0 ] )
            {
                // Sensor resonse with 'B'
                while ( ( true == b_ucProgramFlag ) && ( g_uiCommsTimeout ) )
                {
                    g_uiRxBufferSlavePut = 0;

                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByteMaster ) ) ) && ( g_uiCommsTimeout ) );

                    if ( g_uiCommsTimeout )
                    {
                        if ( 0xFF == ucRxByteMaster )
                        {
                            // End programming
                            b_ucProgramFlag = false;
                        }
                        else
                        {
                            p2pTxByteSlave ( ucRxByteMaster );

                            if ( 0x0A == ucRxByteMaster )
                            {
                                // Wait for 0x0A character from sensor - line programmed
                                while ( ( 1 > g_uiRxBufferSlavePut ) && ( g_uiCommsTimeout ) );

                                if ( g_uiCommsTimeout )
                                {
                                    // Clear buffer
                                    p2pTxByteMaster ( g_aucRxBufferSlave [ 0 ] );

                                    uiLines++;
                                    g_uiCommsTimeout = 200;   // 2 seconds
                                    Watchdog ( );           // Watchdog must be serviced - Blocking portion of code
                                }
                                else
                                {
                                    // End programming
                                    b_ucProgramFlag = false;
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
                        // Nothing to do
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
            // Nothing to do
        }
    }
    else
    {
        p2pTxByteMaster ( DLE );
        p2pTxByteMaster ( SEQ );
        p2pTxByteMaster ( g_ucSeq );
        p2pTxByteMaster ( DLE   );
        p2pTxByteMaster ( NAK   );
        p2pTxByteMaster ( 0x82  );
    }

    memset ( g_aucRxBufferMaster , 0 , P2P_BUFFER_MASTER_SIZE );

    g_uiRxBufferMasterGet = 0;
    g_uiRxBufferMasterPut = 0;
    g_uiRxBufferSlaveGet  = 0;
    g_uiRxBufferSlavePut  = 0;
    
    return g_uiCommsTimeout;
}

static void p2pTxByteMaster ( uint8_t ucData )
{
    uart_tx_wait_blocking ( UART_PC );
    uart_putc ( UART_PC , ucData );
}

static void p2pTxByteSlave ( uint8_t ucData )
{
    uart_tx_wait_blocking ( UART_SEN );
    uart_putc ( UART_SEN , ucData );
}

void reportDeviceFault ( void )
{
    // No response, send data to PC
    p2pTxByteMaster ( DLE );
    p2pTxByteMaster ( NAK );
    p2pTxByteMaster ( eP2pNAKdeviceFault ); // Send reason
}

/*** end of file ***/
