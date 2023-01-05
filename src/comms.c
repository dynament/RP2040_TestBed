/*
*******************************************************************************
 *  Author:             Frank Kups                                            *
 *  Company:            Dynament Ltd.                                         *
 *                      Status Scientific Controls Ltd.                       *
 *  Project :           24-Way Premier IR Sensor Jig                          *
 *  Filename:   		comms.c                                               *
 *  Date:		        30/11/2022                                            *
 *  File Version:   	4.0.0                                                 *
 *  Version history:    4.0.0 - 30/11/2022 - Craig Hemingway                  *
 *                          PIC code ported over to RP2040                    *
 *                      3.0.0 - 27/01/2014 - Frank Kups                       *
 *                          Latest program for sensor jig Version 4           *
 *  Tools Used: Visual Studio Code -> 1.73.1                                  *
 *              Compiler           -> GCC 11.3.1 arm-none-eabi                *
 *                                                                            *
 ******************************************************************************
*/

#include <comms.h>

#include <main.h>
#include <p2p.h>
#include <stdbool.h>
#include <string.h>

extern uint8_t aucRxBufferMaster [ ];
extern uint8_t aucRxBufferSlave  [ ];

extern volatile bool     b_ucFlagJigCommand;
extern volatile bool     b_ucPassThroughMode;
extern volatile bool     b_ucPcCommsFlag;
extern volatile bool     b_ucSensorCommsFlag;
extern volatile uint16_t uiCommsMode;
extern volatile uint16_t uiCommsTimeout;
extern volatile uint16_t uiRxBufferMasterGet;
extern volatile uint16_t uiRxBufferMasterPut;
extern volatile uint16_t uiRxBufferSlaveGet;
extern volatile uint16_t uiRxBufferSlavePut;

bool     b_ucJig         = false;
uint8_t  ucJigCommand    = 0;
uint8_t  ucPosn          = 0;
uint8_t  ucProgressCount = 0;
uint8_t  ucSeq           = 0;
uint16_t uiChecksumSlave = 0;
uint16_t uiSerialNo      = 0;

void p2pTxByteMaster ( uint8_t ucData );
void p2pTxByteSlave  ( uint8_t ucData );

uint8_t p2pRxByteMaster ( uint8_t* pucData )
{
    uint8_t ucStatus = 0;

    if ( uiRxBufferMasterGet != uiRxBufferMasterPut )
    {
        *pucData = aucRxBufferMaster [ uiRxBufferMasterGet++ ];

        if ( P2P_BUFFER_MASTER_SIZE == uiRxBufferMasterGet )
        {
            uiRxBufferMasterGet = 0;
        }
        else
        {
            // Nothing to do
        }

        ucStatus      = p2pRxOk;
        // ucPcCommsFlag = 1;
    }
    else
    {
        ucStatus = p2pRxNone;
    }

    return ucStatus;
}

uint8_t p2pRxByteSlave ( uint8_t* pucData )
{
    uint8_t ucStatus = p2pRxNone;

    if ( uiRxBufferSlaveGet != uiRxBufferSlavePut )
    {
        // Get byte out of buffer
        *pucData = aucRxBufferSlave [ uiRxBufferSlaveGet++ ];

        if ( P2P_BUFFER_SLAVE_SIZE == uiRxBufferSlaveGet )
        {
            uiRxBufferSlaveGet = 0;
        }
        else
        {
            // Nothing to do
        }

        ucStatus          = p2pRxOk;
        // ucSensorCommsFlag = 1;
    }
    else
    {
        ucStatus = p2pRxNone;
    }

    return ucStatus;
}

uint16_t p2pPollMaster ( void )
{
    bool     b_ucWaitMaster      = true;
    uint8_t  ucChecksumType      = CSUM_ERROR;
    uint8_t  ucRxByte            = 0;
    uint8_t  ucStatus            = 0;
    uint16_t uiBufferStartPosn   = 0;
    uint16_t uiChecksumMaster    = 0;
    uint16_t uiChecksumMasterCRC = 0;
    uint16_t uiLoop              = 0;
    uint16_t uiMuxPosn           = 0;
    uint16_t uiRxChecksumMaster  = 0;

    b_ucJig        = false;
    uiCommsMode    = COMMS_WAIT;
    uiCommsTimeout = 500;

    
    while ( ( true == b_ucWaitMaster ) && uiCommsTimeout )  // Wait until command received
    {
        ucStatus = p2pRxByteMaster ( &ucRxByte );

        if ( p2pRxOk == ucStatus )
        {
            uiCommsTimeout = 100;   // Maximum 1 second
            if ( DLE == ucRxByte )
            {
                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                // Search for DLE RD sequence
                if ( ( RD == ucRxByte ) && ( uiCommsTimeout ) )
                {
                    uiCommsMode = COMMS_READ;
                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                    if ( ( 0xFF == ucRxByte ) && ( uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                        if ( uiCommsTimeout )
                        {
                            addToChecksum    ( uiChecksumMaster    , ucRxByte );
                            addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                            // This byte is the mux position, 1 to 24
                            // A value outside this indicates that the command is for the Jig and not the sensor
                            uiMuxPosn = ( uint16_t ) ucRxByte;

                            if ( 0 < uiMuxPosn )
                            {
                                b_ucFlagJigCommand = false;
                                uiMuxPosn--;    // Position 1 = '0'

                               Set_MUX ( uiMuxPosn );
                            }
                            else
                            {
                                b_ucFlagJigCommand = true;
                            }

                            if ( DLE == ucRxByte )  // Position byte 0x10
                            {
                                // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                            }
                            else
                            {
                                // Nothing to do
                            }

                            // Wait for remaining data
                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                            if ( uiCommsTimeout )
                            {
                                ucJigCommand = ucRxByte;
                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                if ( DLE == ucRxByte )  // Command byte 0x10
                                {
                                    // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                if ( uiCommsTimeout )
                                {
                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                    if ( uiCommsTimeout )
                                    {
                                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                        // Next 2 bytes are the checksum
                                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                        if ( uiCommsTimeout )
                                        {
                                            uiRxChecksumMaster = ucRxByte;
                                            uiRxChecksumMaster <<= 8;

                                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                            if ( uiCommsTimeout )
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

                                                if ( true == b_ucJig )
                                                {
                                                    // Remove sequencing bytes
                                                    memmove ( ( aucRxBufferMaster ) , ( aucRxBufferMaster + 3 ) , P2P_BUFFER_MASTER_SIZE - 3 );
                                                    uiRxBufferMasterGet -= 3;
                                                }
                                                else
                                                {
                                                    // Nothing to do
                                                }

                                                uiChecksumSlave = 0;

                                                if ( false == b_ucFlagJigCommand )
                                                {
                                                    // Forward message on to sensor
                                                    // Strip out 0xFF, position bytes preforming byte stuffing
                                                    for ( uiLoop = uiBufferStartPosn ; uiLoop < ( uiRxBufferMasterGet - 2 ) ; uiLoop++ )
                                                    {
                                                        if ( ( uiBufferStartPosn + 2 ) == uiLoop )
                                                        {
                                                            // Do not send 0xFF
                                                        }
                                                        else if ( ( uiBufferStartPosn + 3 ) == uiLoop )
                                                        {
                                                            // Do not send position
                                                        }
                                                        else if ( ( ( uiBufferStartPosn + 4 ) == uiLoop ) && ( DLE == aucRxBufferMaster [ uiBufferStartPosn + 3 ] ) )
                                                        {
                                                            // Do not send DLE when position == 0x10
                                                        }
                                                        else
                                                        {
                                                            p2pTxByteSlave ( aucRxBufferMaster [ uiLoop ] );

                                                            if ( CSUM_SIMPLE == ucChecksumType )
                                                            {
                                                                addToChecksum ( uiChecksumSlave , aucRxBufferMaster [ uiLoop ] );
                                                            }
                                                            else
                                                            {
                                                                addToChecksumCRC ( uiChecksumSlave , aucRxBufferMaster [ uiLoop ] );
                                                            }
                                                        }
                                                    }
                                                    // Send the re-calculated checksum
                                                    p2pTxByteSlave ( ( uiChecksumSlave >> 8   ) );
                                                    p2pTxByteSlave ( ( uiChecksumSlave & 0xFF ) );
                                                }
                                                else
                                                {
                                                    // Jig read command
                                                    if ( 0xFD == ucJigCommand )
                                                    {
                                                        uiChecksumSlave = 0;
                                                        // Send the Jig serial number to the calling program
                                                        // uiSerialNo
                                                        p2pTxByteMaster ( DLE );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , DLE );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , DLE );
                                                        }

                                                        p2pTxByteMaster ( DAT );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , DAT );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , DAT );
                                                        }

                                                        p2pTxByteMaster ( 0x02 );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , 0x02 );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , 0x02 );
                                                        }

                                                        p2pTxByteMaster ( uiSerialNo & 0xff );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , ( uiSerialNo >> 8 ) );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , ( uiSerialNo >> 8 ));
                                                        }

                                                        p2pTxByteMaster ( uiSerialNo >> 8 );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , ( uiSerialNo & 0xFF ) );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , ( uiSerialNo & 0xFF ) );
                                                        }

                                                        p2pTxByteMaster ( DLE );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , DLE );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , DLE );
                                                        }

                                                        p2pTxByteMaster ( EoF );

                                                        if ( CSUM_SIMPLE == ucChecksumType )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , EoF );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , EoF );
                                                        }

                                                        p2pTxByteMaster ( uiChecksumSlave >> 8   );
                                                        p2pTxByteMaster ( uiChecksumSlave & 0xFF );

                                                        b_ucFlagJigCommand = false;   // Force exit
                                                        ucJigCommand     = 0;
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
                        if ( ( 0x00 == ucRxByte ) && ( uiCommsTimeout ) )
                        {
                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );
                        }
                        else
                        {
                            // Nothing to do
                        }

                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );
                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );
                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );
                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );
                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                        // Clear buffer
                        memset ( aucRxBufferMaster , 0 , P2P_BUFFER_SLAVE_SIZE );
                        b_ucJig             = true;
                        uiRxBufferMasterGet = 0;
                        uiRxBufferMasterPut = 0;

                        // Send out standard response
                        p2pTxByteMaster ( DLE   );
                        p2pTxByteMaster ( SEQ   );
                        p2pTxByteMaster ( ucSeq );
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
                else if ( ( WR == ucRxByte ) && ( uiCommsTimeout ) )
                {
                    uiCommsTimeout = 100;   // Maximum wait time of 1 second for write operation

                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                    if ( ( WP1 == ucRxByte ) && ( uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                    }
                    else
                    {
                        // Nothing to do
                    }

                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                    if ( ( WP2 == ucRxByte ) && ( uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                    }
                    else
                    {
                        // Nothing to do
                    }

                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                    if ( ( ( 0xFF == ucRxByte ) || ( 0xFE == ucRxByte ) ) && ( uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                        if ( 0xFE == ucRxByte )
                        {
                            b_ucPassThroughMode = true;
                            NOP;
                        }
                        else
                        {
                            b_ucPassThroughMode = false;
                        }

                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                        if ( uiCommsTimeout )
                        {
                            addToChecksum    ( uiChecksumMaster, ucRxByte     );
                            addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                            ucPosn    = ucRxByte;
                            uiMuxPosn = ( uint16_t ) ucPosn;

                            if ( 0 < uiMuxPosn )
                            {
                                b_ucFlagJigCommand = false;
                                uiMuxPosn--;    // Position 1 = '0'

                                // This byte is the mux posn
                                uiMuxPosn = ( uint16_t ) ucRxByte - 1;  // Position 1 = '0'
                                Set_MUX ( uiMuxPosn );
                            }
                            else
                            {
                                b_ucFlagJigCommand = true;
                            }

                            if ( DLE == ucRxByte )  // Position byte 0x10
                            {
                                // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                            }
                            else
                            {
                                // Nothing to do
                            }

                            if ( false == b_ucPassThroughMode )
                            {
                                // Wait for remaining data
                                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                ucJigCommand = ucRxByte;
                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                            }
                            else
                            {
                                // No byte after 0xFE passthrough byte
                                NOP;
                            }

                            if ( uiCommsTimeout )
                            {
                                if ( DLE == ucRxByte )  // Command byte 0x10
                                {
                                    // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                if ( uiCommsTimeout )
                                {
                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                    if ( true == b_ucPassThroughMode )
                                    {
                                        // Nothing to do
                                    }
                                    else
                                    {
                                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                    }

                                    if ( uiCommsTimeout )
                                    {
                                        if ( ( true == b_ucPassThroughMode ) && ( DLE != ucPosn ) )
                                        {
                                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                            addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                            addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                        }
                                        else
                                        {
                                            NOP;    // Trap for programming position 16 ( =DLE )
                                        }

                                        // Next 2 bytes are the checksum
                                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                        if ( uiCommsTimeout )
                                        {
                                            uiRxChecksumMaster = ucRxByte;
                                            uiRxChecksumMaster <<= 8;

                                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                            if ( true == b_ucPassThroughMode )
                                            {
                                                NOP;
                                            }
                                            else
                                            {
                                                // Nothing to do
                                            }

                                            if ( uiCommsTimeout )
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
                                                if ( true == b_ucJig )
                                                {
                                                    p2pTxByteMaster ( DLE   );
                                                    p2pTxByteMaster ( SEQ   );
                                                    p2pTxByteMaster ( ucSeq );
                                                }
                                                else
                                                {
                                                    // Nothing to do
                                                }

                                                p2pTxByteMaster ( DLE );
                                                p2pTxByteMaster ( ACK );
                                                uiCommsMode = COMMS_WRITE;

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
                else if ( ( SEQ == ucRxByte ) && ( uiCommsTimeout ) )
                {
                    // The mentorPC library sends out 0x10, 0x21, 0x?? ( sequence byte )
                    // When the sequence byte == 0x10, this upsets the normal DLE message
                    // Ignore this byte at all times as this message is not implemented

                    // Get sequence byte
                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                    b_ucJig             = true;
                    ucSeq               = ucRxByte;
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

    if ( 0 == uiCommsTimeout )
    {
        if ( true == b_ucJig )
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

    return uiCommsTimeout;
}

// Sensor response to READ request
uint16_t p2pPollSlaveRead ( void )
{
    static uint8_t ucError = 0;

    bool     b_ucWaitSlave      = true;
    uint8_t  ucRxByte           = 0;
    uint8_t  ucStatus           = 0;
    uint16_t uiDataLength       = 0;
    uint16_t uiDataLengthToSend = 0;
    uint16_t uiLoop             = 0;

    // Wait for sensor response
    while ( ( true == b_ucWaitSlave ) && ( uiCommsTimeout ) )
    {
        ucStatus = p2pRxByteSlave ( &ucRxByte );

        if ( p2pRxOk == ucStatus )
        {
            // Search for DLE DAT sequence
            if ( DLE == ucRxByte )
            {
                while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                if ( DAT == ucRxByte )
                {
                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                    uiDataLength       = ucRxByte;
                    uiDataLengthToSend = uiDataLength + 7;  // Includes start & end of frame bytes

                    // Wait for remaining bytes
                    while ( uiDataLength && uiCommsTimeout )
                    {
                        while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                        if ( DLE == ucRxByte )
                        {
                            // Wait for next DLE - Byte stuffing extra byte not included in data length
                            while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                            uiDataLength--;
                            uiDataLengthToSend++;   // Include byte stuffing bytes
                        }
                        else
                        {
                            uiDataLength--;
                        }
                    }

                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );  // DLE
                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );  // EoF
                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );  // Checksum high byte
                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );  // Checksum low byte

                    // Send sequence if originally requested
                    if ( true == b_ucJig )
                    {
                        p2pTxByteMaster ( DLE   );
                        p2pTxByteMaster ( SEQ   );
                        p2pTxByteMaster ( ucSeq );
                    }
                    else
                    {
                        // Nothing to do
                    }

                    // Send buffer back to PC
                    for ( uiLoop = ( uiRxBufferSlavePut - uiDataLengthToSend ) ; uiLoop < uiDataLengthToSend ; uiLoop++ )
                    {
                        p2pTxByteMaster ( aucRxBufferSlave [ uiLoop ] );
                    }

                    ucProgressCount++;

                    b_ucWaitSlave = false;
                }
                else if ( NAK == ucRxByte )
                {
                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );  // Wait for reason
                    
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

    if ( 0 == uiCommsTimeout )
    {
        ucError++;

        p2pTxByteMaster ( DLE   );
        p2pTxByteMaster ( SEQ   );
        p2pTxByteMaster ( ucSeq );

        p2pTxByteMaster ( DLE  );
        p2pTxByteMaster ( NAK  );
        p2pTxByteMaster ( 0x82 ); // Send reason
    }
    else
    {
        // Nothing to do
    }

    // Clear buffers
    memset ( aucRxBufferMaster , 0 , P2P_BUFFER_MASTER_SIZE );
    uiRxBufferMasterGet = 0;
    uiRxBufferMasterPut = 0;

    memset ( aucRxBufferSlave , 0 , P2P_BUFFER_SLAVE_SIZE );
    uiRxBufferSlavePut = 0;
    uiRxBufferSlavePut = 0;

    return uiCommsTimeout;
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
    uint8_t  ucSequenceOffset    = 0;
    uint16_t uiBaudRate          = 38400;
    uint16_t uiBufferEndPosn     = 0;
    uint16_t uiBufferStartPosn   = 0;
    uint16_t uiChecksumMaster    = 0;
    uint16_t uiChecksumMasterCRC = 0;
    uint16_t uiLoop              = 0;
    uint16_t uiRxChecksumMaster  = 0;

    // Wait for sensor response
    while ( ( true == b_ucWaitSlave ) && ( uiCommsTimeout ) )
    {
        ucStatus = p2pRxByteMaster ( &ucRxByte );

        if ( p2pRxOk == ucStatus )
        {

            uiCommsTimeout = 500; // 5 seconds - Temperature compensation command write takes a long time

            if ( DLE == ucRxByte )
            {
                uiBufferStartPosn = uiRxBufferMasterPut - 1;

                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                // Search for DLE DAT sequence)
                if ( ( DAT == ucRxByte ) && ( uiCommsTimeout ) )
                {
                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                    
                    // Wait for length
                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );
                    
                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    ucDataLength       = ucRxByte;
                    ucDataLengthToSend = ucDataLength + 7;  // Include start & end of frame bytes

                    // Wait for remaining bytes
                    while ( ucDataLength && uiCommsTimeout )
                    {
                        
                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );
                        
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                        
                        if ( DLE == ucRxByte )
                        {
                            // Wait for next DLE - Byte stuffing extra byte not included in data length
                            while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );
                            
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
                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    // Wait for EOF
                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    // Next 2 bytes are the checksum
                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );
                    
                    if ( uiCommsTimeout )
                    {
                        uiRxChecksumMaster = ucRxByte;
                        uiRxChecksumMaster <<= 8;

                        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                        if  ( uiCommsTimeout )
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
                                if ( true == b_ucJig )
                                {
                                    p2pTxByteMaster ( DLE   );
                                    p2pTxByteMaster ( SEQ   );
                                    p2pTxByteMaster ( ucSeq );
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
                            if ( true == b_ucJig )
                            {
                                // Ignore sequence bytes
                                ucSequenceOffset  = 3;
                                uiBufferEndPosn   = 12;
                                uiBufferStartPosn = 3;
                            }
                            else
                            {
                                ucSequenceOffset  = 0;
                                uiBufferEndPosn   = 9;
                                uiBufferStartPosn = 0;
                            }

                            if ( 0xFF == aucRxBufferMaster [ 0 ] )
                            {
                                // Ignore programming byte from failed position
                                ucSequenceOffset++;
                                uiBufferEndPosn++;
                                uiBufferStartPosn++;
                            }
                            else
                            {
                                // Nothing to do
                            }

                            uiChecksumSlave = 0;

                            if ( false == b_ucFlagJigCommand )
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
                                        if ( ( DLE == aucRxBufferMaster [ uiBufferStartPosn + 5 ] ) )
                                        {
                                            // Do not send DLE when position = 0x10
                                            uiBufferEndPosn++;
                                        }
                                        else
                                        {
                                            p2pTxByteSlave ( aucRxBufferMaster [ uiLoop ] );

                                            if ( CSUM_SIMPLE == ucChecksumType )
                                            {
                                                addToChecksum ( uiChecksumSlave , aucRxBufferMaster [ uiLoop ] );
                                            }
                                            else
                                            {
                                                addToChecksumCRC ( uiChecksumSlave , aucRxBufferMaster [ uiLoop ] );
                                            }
                                        }
                                    }
                                    else
                                    {
                                        p2pTxByteSlave ( aucRxBufferMaster [ uiLoop ] );
                                        if ( CSUM_SIMPLE == ucChecksumType )
                                        {
                                            addToChecksum ( uiChecksumSlave , aucRxBufferMaster [ uiLoop ] );
                                        }
                                        else
                                        {
                                            addToChecksumCRC ( uiChecksumSlave , aucRxBufferMaster [ uiLoop ] );
                                        }
                                    }
                                }

                                uiBufferStartPosn += uiLoop;
                                uiBufferStartPosn -= 1;

                                // Send re-calculated checksum
                                p2pTxByteSlave ( ( uiChecksumSlave >> 8   ) );
                                p2pTxByteSlave ( ( uiChecksumSlave & 0xff ) );

                                // Wait for acknowledge from sensor
                                // Wait for sensor response
                                while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                if ( 0 == uiCommsTimeout )
                                {
                                    if ( true == b_ucJig )
                                    {
                                        p2pTxByteMaster ( DLE   );
                                        p2pTxByteMaster ( SEQ   );
                                        p2pTxByteMaster ( ucSeq );
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
                                        while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                        if ( 0 == uiCommsTimeout )
                                        {
                                            if ( true == b_ucJig )
                                            {
                                                p2pTxByteMaster ( DLE );
                                                p2pTxByteMaster ( SEQ   );
                                                p2pTxByteMaster ( ucSeq );
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
                                                p2pTxByteSlave ( aucRxBufferMaster [ uiLoop ] );
                                            }

                                            // Sensor takes about 7.5 seconds to carry out command
                                            uiCommsTimeout = 1000;  // Max timeout of 10 seconds

                                            // Wait for sensor response
                                            while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                            if ( 0 == uiCommsTimeout )
                                            {
                                                if ( true == b_ucJig )
                                                {
                                                    p2pTxByteMaster ( DLE   );
                                                    p2pTxByteMaster ( SEQ   );
                                                    p2pTxByteMaster ( ucSeq );
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
                                                    while ( ( p2pRxOk != ( p2pRxByteSlave ( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                                    if ( 0 == uiCommsTimeout )
                                                    {
                                                        if ( true == b_ucJig )
                                                        {
                                                            p2pTxByteMaster ( DLE   );
                                                            p2pTxByteMaster ( SEQ   );
                                                            p2pTxByteMaster ( ucSeq );
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
                                                        if ( true == b_ucJig )
                                                        {
                                                            p2pTxByteMaster ( DLE   );
                                                            p2pTxByteMaster ( SEQ   );
                                                            p2pTxByteMaster ( ucSeq );
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
                                                        while ( ( p2pRxOk != ( p2pRxByteSlave( &ucRxByte ) ) ) && ( uiCommsTimeout ) );

                                                        if ( true == b_ucJig )
                                                        {
                                                            p2pTxByteMaster ( DLE   );
                                                            p2pTxByteMaster ( SEQ   );
                                                            p2pTxByteMaster ( ucSeq );
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
                                                        if ( true == b_ucJig )
                                                        {
                                                            p2pTxByteMaster ( DLE   );
                                                            p2pTxByteMaster ( SEQ   );
                                                            p2pTxByteMaster ( ucSeq );
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
                                                    if ( true == b_ucJig )
                                                    {
                                                        p2pTxByteMaster ( DLE   );
                                                        p2pTxByteMaster ( SEQ   );
                                                        p2pTxByteMaster ( ucSeq );
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
                                            while ( ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                            if ( true == b_ucJig )
                                            {
                                                p2pTxByteMaster ( DLE   );
                                                p2pTxByteMaster ( SEQ   );
                                                p2pTxByteMaster ( ucSeq );
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
                                            if ( true == b_ucJig )
                                            {
                                                p2pTxByteMaster ( DLE   );
                                                p2pTxByteMaster ( SEQ   );
                                                p2pTxByteMaster ( ucSeq );
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
                                        if ( true == b_ucJig )
                                        {
                                            p2pTxByteMaster ( DLE   );
                                            p2pTxByteMaster ( SEQ   );
                                            p2pTxByteMaster ( ucSeq );
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

                                if ( 0xFB == ucJigCommand )
                                {
                                    // Power command
                                    switch ( aucRxBufferMaster [ 14 ] )
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
                                    UpdateBaudRate ( uiBaudRate );
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                if ( 0xFC == ucJigCommand )
                                {
                                    // Power command
                                    if ( 0x50 == aucRxBufferMaster [ 14 ] )
                                    {
                                        RELAY_OFF;  // Turn off power to sensors
                                    }
                                    else
                                    {
                                        RELAY_ON;   // Turn on power to sensors
                                    }

                                    // Wait for 2 seconds before the main loop re-enables the Power to the sensors
                                    // Wait 15 secs in total for sensors to startup
                                    uiCommsTimeout = 200;   // 2 seconds

                                    while ( uiCommsTimeout );
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                if ( 0xFD == ucJigCommand )
                                {
                                    // Write Jig Serial No.
                                    uiSerialNo = aucRxBufferMaster [ 15 ];
                                    uiSerialNo <<= 8;
                                    uiSerialNo |= aucRxBufferMaster [ 14 ];
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

    memset ( aucRxBufferMaster , 0 , P2P_BUFFER_MASTER_SIZE );

    uiRxBufferMasterGet = 0;
    uiRxBufferMasterPut = 0;

    memset ( aucRxBufferSlave , 0 , P2P_BUFFER_SLAVE_SIZE );

    uiRxBufferSlaveGet = 0;
    uiRxBufferSlavePut = 0;

    return uiCommsTimeout;
}

uint16_t p2pPollSlaveWritePassThrough ( void )
{
    bool     b_ucProgramFlag  = false;
    uint8_t  ucPosition       = 0;
    uint8_t  ucRxByteMaster   = 0;
    uint16_t uiBytesToReceive = 0;
    uint16_t uiLines          = 0;

    ucPosition = aucRxBufferMaster [ 8 ];

    if ( 16 == ucPosition )
    {
        uiBytesToReceive = 0x15;
    }
    else
    {
        uiBytesToReceive = 0x14;
    }

    if ( uiCommsTimeout )
    {
        p2pTxByteMaster ( DLE   );
        p2pTxByteMaster ( SEQ   );
        p2pTxByteMaster ( ucSeq );
        p2pTxByteMaster ( DLE   );
        p2pTxByteMaster ( ACK   );

        while ( ( uiBytesToReceive != uiRxBufferMasterPut ) && uiCommsTimeout );    // Wait until command received

        uiRxBufferMasterGet = 0;
        uiRxBufferMasterPut = 0;

        uiCommsTimeout = 500;   // 5 seconds;

        while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByteMaster ) ) ) && ( uiCommsTimeout ) );

        if ( 'P' == ucRxByteMaster )
        {
            b_ucProgramFlag = true;

            memset ( aucRxBufferMaster , 0 , P2P_BUFFER_MASTER_SIZE );

            uiRxBufferMasterGet = 0;
            uiRxBufferMasterPut = 0;
            uiRxBufferSlaveGet  = 0;
            uiRxBufferSlavePut  = 0;

            p2pTxByteSlave ( 'P' );

            while ( ( 1 > uiRxBufferSlavePut ) && ( uiCommsTimeout ) );

            if ( uiCommsTimeout )
            {
                p2pTxByteMaster ( aucRxBufferSlave [ 0 ] );
            }
            else
            {
                NOP;
            }

            if ( 0x42 == aucRxBufferSlave [ 0 ] )
            {
                // Sensor resonse with 'B'
                while ( ( true == b_ucProgramFlag ) && ( uiCommsTimeout ) )
                {
                    uiRxBufferSlavePut = 0;

                    while ( ( p2pRxOk != ( p2pRxByteMaster ( &ucRxByteMaster ) ) ) && ( uiCommsTimeout ) );

                    if ( uiCommsTimeout )
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
                                while ( ( 1 > uiRxBufferSlavePut ) && ( uiCommsTimeout ) );

                                if ( uiCommsTimeout )
                                {
                                    // Clear buffer
                                    p2pTxByteMaster ( aucRxBufferSlave [ 0 ] );

                                    uiLines++;
                                    uiCommsTimeout = 200;   // 2 seconds
                                    watchdog ( );           // Watchdog must be serviced - Blocking portion of code
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
        p2pTxByteMaster ( ucSeq );
        p2pTxByteMaster ( DLE   );
        p2pTxByteMaster ( NAK   );
        p2pTxByteMaster ( 0x82  );
    }

    memset ( aucRxBufferMaster , 0 , P2P_BUFFER_MASTER_SIZE );

    uiRxBufferMasterGet = 0;
    uiRxBufferMasterPut = 0;
    uiRxBufferSlaveGet  = 0;
    uiRxBufferSlavePut  = 0;
    
    return uiCommsTimeout;
}

void p2pTxByteMaster ( uint8_t ucData )
{
    uart_tx_wait_blocking ( UART_PC );
    uart_putc ( UART_PC , ucData );
}

void p2pTxByteSlave ( uint8_t ucData )
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
