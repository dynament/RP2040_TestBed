/*******************************************************************
 *  Author:		Frank Kups                                         *
 *  Company:            Status Scientific Controls Ltd             *
 *  Project :           24-Way Premier IR Sensor Jig               *
 *  Filename:		Comms.c                                        *
 *  Date:		10/07/2012                                         *
 *  File Version:	1.0.0                                          *
 *  Other Files Required: p24F32KA01.gld, libpPIC24Fxxx-coff.a     *
 *  Tools Used: MPLAB GL  -> 8.84                                  *
 *              Compiler  -> 3.31                                  *
 *                                                                 *
 *******************************************************************
 */

#include "main.h"

uint8_t  ucFlagJigCommand  = false;
uint8_t  ucJig             = false;
uint8_t  ucJigCommand      = false;
uint8_t  ucPassThroughMode = false;
uint8_t  ucPosn            = 0;
uint8_t  ucProgressCount   = 0;
uint8_t  ucSeq             = 0;
uint16_t uiChecksumSlave   = 0;
uint16_t uiCommsMode       = COMMS_WAIT;
uint16_t uiSerialNo        = 0;

void p2pTxByteMaster ( uint8_t ucData );
void p2pTxByteSlave  ( uint8_t ucData );

uint8_t p2pRxByteMaster ( uint8_t* pucData )
{
    uint8_t ucStatus = 0;

    if ( uiRxBufferMasterGet != uiRxBufferMasterPut )
    {
        *pucData = aucRxBufferMaster [ uiRxBufferMasterGet++ ];

        if ( uiRxBufferMasterGet == P2P_BUFFER_MASTER_SIZE )
        {
            uiRxBufferMasterGet = 0;
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

    return ucStatus;
}

uint8_t p2pRxByteSlave ( uint8_t* pucData )
{
    uint8_t ucStatus = p2pRxNone;

    if ( uiRxBufferSlaveGet != uiRxBufferSlavePut )
    {
        // Get byte out of buffer
        *pucData = aucRxBufferSlave [ uiRxBufferSlaveGet++ ];

        if ( uiRxBufferSlaveGet == P2P_BUFFER_SLAVE_SIZE )
        {
            uiRxBufferSlaveGet = 0;
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

    return ucStatus;
}

uint16_t p2pPollMaster ( void )
{
    uint8_t  ucChecksumType      = CSUM_ERROR;
    uint8_t  ucRxByte            = 0;
    uint8_t  ucStatus            = 0;
    uint8_t  ucWaitMaster        = true;
    uint16_t uiBufferStartPosn   = 0;
    uint16_t uiChecksumMaster    = 0;
    uint16_t uiChecksumMasterCRC = 0;
    uint16_t uiLoop              = 0;
    uint16_t uiMuxPosn           = 0;
    uint16_t uiRxChecksumMaster  = 0;

    ucJig       = false;
    uiCommsMode = COMMS_WAIT;

    while ( ucWaitMaster == true )  // Wait until command received
    {
        watchdog ( );
        ucStatus = p2pRxByteMaster ( &ucRxByte );

        if ( ucStatus == p2pRxOk )
        {
            uiCommsTimeout = 100;   // Maximum 1 second
            if ( ucRxByte == DLE )
            {
                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                // Search for DLE RD sequence
                if ( ( ucRxByte == RD ) && ( uiCommsTimeout ) )
                {
                    uiCommsMode = COMMS_READ;
                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                    watchdog ( );

                    if ( ( ucRxByte == 0xFF ) && ( uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                        while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                        if ( uiCommsTimeout )
                        {
                            addToChecksum    ( uiChecksumMaster    , ucRxByte );
                            addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                            // This byte is the mux position, 1 to 24
                            // A value outside this indicates that the command is for the Jig and not the sensor
                            uiMuxPosn = ( uint16_t ) ucRxByte;

                            if ( uiMuxPosn > 0 )
                            {
                                ucFlagJigCommand = false;
                                uiMuxPosn--;    // Position 1 = '0'

                               Set_MUX ( uiMuxPosn );
                            }
                            else
                            {
                                ucFlagJigCommand = true;
                            }

                            if ( ucRxByte == DLE )  // Position byte 0x10
                            {
                                // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                            }
                            else
                            {
                                // Nothing to do
                            }

                            // Wait for remaining data
                            while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                            watchdog ( );

                            if ( uiCommsTimeout )
                            {
                                ucJigCommand = ucRxByte;
                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                if ( ucRxByte == DLE )  // Command byte 0x10
                                {
                                    // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                                watchdog ( );

                                if ( uiCommsTimeout )
                                {
                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                    if ( uiCommsTimeout )
                                    {
                                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                        // Next 2 bytes are the checksum
                                        while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                        if ( uiCommsTimeout )
                                        {
                                            uiRxChecksumMaster = ucRxByte;
                                            uiRxChecksumMaster <<= 8;

                                            while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                            if ( uiCommsTimeout )
                                            {
                                                uiRxChecksumMaster += ucRxByte;

                                                if ( uiRxChecksumMaster == uiChecksumMaster )
                                                {
                                                    ucChecksumType = CSUM_SIMPLE;
                                                }
                                                else if ( uiRxChecksumMaster == uiChecksumMasterCRC )
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

                                                if ( ucJig == true )
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

                                                if ( ucFlagJigCommand == false )
                                                {
                                                    // Forward message on to sensor
                                                    // Strip out 0xFF, position bytes preforming byte stuffing
                                                    for ( uiLoop = uiBufferStartPosn ; uiLoop < ( uiRxBufferMasterGet - 2 ) ; uiLoop++ )
                                                    {
                                                        if ( uiLoop == ( uiBufferStartPosn + 2 ) )
                                                        {
                                                            // Do not send 0xFF
                                                        }
                                                        else if ( uiLoop == ( uiBufferStartPosn + 3 ) )
                                                        {
                                                            // Do not send position
                                                        }
                                                        else if ( ( uiLoop == ( uiBufferStartPosn + 4 ) ) && ( aucRxBufferMaster [ uiBufferStartPosn + 3 ] == DLE ) )
                                                        {
                                                            // Do not send DLE when position == 0x10
                                                        }
                                                        else
                                                        {
                                                            p2pTxByteSlave ( aucRxBufferMaster [ uiLoop ] );

                                                            if ( ucChecksumType == CSUM_SIMPLE )
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
                                                    if ( ucJigCommand == 0xFD )
                                                    {
                                                        uiChecksumSlave = 0;
                                                        // Send the Jig serial number to the calling program
                                                        // uiSerialNo
                                                        p2pTxByteMaster ( DLE );

                                                        if ( ucChecksumType == CSUM_SIMPLE )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , DLE );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , DLE );
                                                        }

                                                        p2pTxByteMaster ( DAT );

                                                        if ( ucChecksumType == CSUM_SIMPLE )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , DAT );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , DAT );
                                                        }

                                                        p2pTxByteMaster ( 0x02 );

                                                        if ( ucChecksumType == CSUM_SIMPLE )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , 0x02 );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , 0x02 );
                                                        }

                                                        p2pTxByteMaster ( uiSerialNo & 0xff );

                                                        if ( ucChecksumType == CSUM_SIMPLE )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , ( uiSerialNo >> 8 ) );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , ( uiSerialNo >> 8 ));
                                                        }

                                                        p2pTxByteMaster ( uiSerialNo >> 8 );

                                                        if ( ucChecksumType == CSUM_SIMPLE )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , ( uiSerialNo & 0xFF ) );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , ( uiSerialNo & 0xFF ) );
                                                        }

                                                        p2pTxByteMaster ( DLE );

                                                        if ( ucChecksumType == CSUM_SIMPLE )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , DLE );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , DLE );
                                                        }

                                                        p2pTxByteMaster ( EoF );

                                                        if ( ucChecksumType == CSUM_SIMPLE )
                                                        {
                                                            addToChecksum ( uiChecksumSlave , EoF );
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC ( uiChecksumSlave , EoF );
                                                        }

                                                        p2pTxByteMaster ( uiChecksumSlave >> 8   );
                                                        p2pTxByteMaster ( uiChecksumSlave & 0xFF );

                                                        ucFlagJigCommand = false;   // Force exit
                                                        ucJigCommand     = 0;
                                                    }
                                                    else
                                                    {
                                                        // Nothing to do
                                                    }
                                                    ucWaitMaster = false;   // Force ecit
                                                }
                                                ucWaitMaster = false;   // Force exit
                                            }
                                            else
                                            {
                                                ucWaitMaster = false;   // Force exit
                                            }
                                        }
                                        else
                                        {
                                            ucWaitMaster = false;   // Force exit
                                        }
                                    }
                                    else
                                    {
                                        ucWaitMaster = false;   // Force exit
                                    }
                                }
                                else
                                {
                                    ucWaitMaster = false;   // Force exit
                                }
                            }
                            else
                            {
                                ucWaitMaster = false;   // Force exit
                            }
                        }
                        else
                        {
                            ucWaitMaster = false;   // Force exit
                        }
                    }
                    else
                    {
                        // Test for 24-way jig sequence request
                        if ( ( ucRxByte == 0x00 ) && ( uiCommsTimeout ) )
                        {
                            while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                            watchdog ( );
                        }
                        else
                        {
                            // Nothing to do
                        }

                        while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                        watchdog ( );
                        while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                        watchdog ( );
                        while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                        watchdog ( );
                        while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                        watchdog ( );
                        while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                        watchdog ( );

                        // Clear buffer
                        memset ( aucRxBufferMaster , 0 , P2P_BUFFER_SLAVE_SIZE );
                        ucJig               = true;
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

                // Search for DLE WR sequence)
                else if ( ( ucRxByte == WR ) && ( uiCommsTimeout ) )
                {
                    uiCommsTimeout = 100;   // Maximum wait time of 1 second for write operation

                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                    watchdog ( );

                    if ( ( ucRxByte == WP1 ) && ( uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                    }
                    else
                    {
                        // Nothing to do
                    }

                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                    watchdog ( );

                    if ( ( ucRxByte == WP2 ) && ( uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                    }
                    else
                    {
                        // Nothing to do
                    }

                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                    watchdog ( );

                    if ( ( ( ucRxByte == 0xFF ) || ( ucRxByte == 0xFE ) ) && ( uiCommsTimeout ) )
                    {
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                        if ( ucRxByte == 0xFE )
                        {
                            ucPassThroughMode = true;
                            NOP;
                        }
                        else
                        {
                            ucPassThroughMode = false;
                        }

                        watchdog ( );
                        while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                        if ( uiCommsTimeout )
                        {
                            addToChecksum    ( uiChecksumMaster, ucRxByte     );
                            addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                            ucPosn    = ucRxByte;
                            uiMuxPosn = ( uint16_t ) ucPosn;

                            if ( uiMuxPosn > 0 )
                            {
                                ucFlagJigCommand = false;
                                uiMuxPosn--;    // Position 1 = '0'

                                // This byte is the mux posn
                                uiMuxPosn = ( uint16_t ) ucRxByte - 1;  // Position 1 = '0'
                                Set_MUX ( uiMuxPosn );
                            }
                            else
                            {
                                ucFlagJigCommand = true;
                            }

                            if ( ucRxByte == DLE )  // Position byte 0x10
                            {
                                // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                                watchdog ( );

                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                            }
                            else
                            {
                                // Nothing to do
                            }

                            if ( ucPassThroughMode == false )
                            {
                                // Wait for remaining data
                                while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                ucJigCommand = ucRxByte;
                                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                            }
                            else
                            {
                                // No byte after 0xFE passthrough byte
                                NOP;
                            }
                            watchdog ( );

                            if ( uiCommsTimeout )
                            {
                                if ( ucRxByte == DLE )  // Command byte 0x10
                                {
                                    // Take into account extra DLE - Sensor position 16 adds an extra DLE
                                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                                    watchdog ( );

                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                                watchdog ( );

                                if ( uiCommsTimeout )
                                {
                                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                                    if ( ucPassThroughMode == true )
                                    {
                                        watchdog ( );
                                    }
                                    else
                                    {
                                        while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                    }

                                    watchdog ( );

                                    if ( uiCommsTimeout )
                                    {
                                        if ( ( ucPassThroughMode == true ) && ( ucPosn != DLE ) )
                                        {
                                            while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                            addToChecksum    ( uiChecksumMaster    , ucRxByte );
                                            addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                                        }
                                        else
                                        {
                                            NOP;    // Trap for programming position 16 ( =DLE )
                                        }

                                        // Next 2 bytes are the checksum
                                        while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                                        watchdog ( );

                                        if ( uiCommsTimeout )
                                        {
                                            uiRxChecksumMaster = ucRxByte;
                                            uiRxChecksumMaster <<= 8;

                                            while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                                            watchdog ( );

                                            if ( ucPassThroughMode == true )
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

                                                if ( uiRxChecksumMaster == uiChecksumMaster )
                                                {
                                                    ucChecksumType = CSUM_SIMPLE;
                                                }
                                                else if ( uiRxChecksumMaster == uiChecksumMasterCRC )
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

                                                ucWaitMaster = false;

                                                // Wait minimum 10 milliseconds for multiplexers to stabalise
                                                sleep_ms ( 10 );

                                                // Get remaining parts of write command
                                                if ( ucJig == true )
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

                                                if ( ucPassThroughMode == true )
                                                {
                                                    watchdog ( );
                                                }
                                                else
                                                {
                                                    // Nothing to do
                                                }

                                                // Wait for data
                                                ucWaitMaster = false;   // Force exit
                                            }
                                            else
                                            {
                                                ucWaitMaster = false;   // Force exit
                                            }
                                        }
                                        else
                                        {
                                            ucWaitMaster = false;   // Force exit
                                        }
                                    }
                                    else
                                    {
                                        ucWaitMaster = false;   // Force exit
                                    }
                                }
                                else
                                {
                                    ucWaitMaster = false;   // Force exit
                                }
                            }
                            else
                            {
                                ucWaitMaster = false;   // Force exit
                            }
                        }
                        else
                        {
                            ucWaitMaster = false;   // Force exit
                        }
                    }
                    else
                    {
                        ucWaitMaster = false;   // Force exit
                    }
                }
                else if ( ( ucRxByte == SEQ ) && ( uiCommsTimeout ) )
                {
                    // The mentorPC library sends out 0x10, 0x21, 0x?? ( sequence byte )
                    // When the sequence byte == 0x10, this upsets the normal DLE message
                    // Ignore this byte at all times as this message is not implemented

                    // Get sequence byte
                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                    watchdog ( );

                    ucJig               = true;
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

    if ( uiCommsTimeout == 0 )
    {
        if ( ucJig == true )
        {
            p2pTxByteMaster ( DLE );
            p2pTxByteMaster ( NAK );
            p2pTxByteMaster ( 1   );
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

    uint8_t  ucRxByte           = 0;
    uint8_t  ucStatus           = 0;
    uint8_t  ucWaitSlave        = true;
    uint16_t uiDataLength       = 0;
    uint16_t uiDataLengthToSend = 0;
    uint16_t uiLoop             = 0;

    // Wait for sensor response
    while ( ( uiCommsTimeout ) && ( ucWaitSlave == true ) )
    {
        watchdog ( );
        ucStatus = p2pRxByteSlave ( &ucRxByte );

        if ( ucStatus == p2pRxOk )
        {
            // Search for DLE DAT sequence
            if ( ucRxByte == DLE )
            {
                while ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk );
                watchdog ( );

                if ( ucRxByte == DAT )
                {
                    while ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk );
                    watchdog ( );

                    uiDataLength       = ucRxByte;
                    uiDataLengthToSend = uiDataLength + 7;  // Includes start & end of frame bytes

                    // Wait for remaining bytes
                    while ( uiDataLength )
                    {
                        while ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk );
                        watchdog ( );

                        if ( ucRxByte == DLE )
                        {
                            // Wait for next DLE - Byte stuffing extra byte not included in data length
                            while ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk );

                            uiDataLength--;
                            uiDataLengthToSend++;   // Include byte stuffing bytes
                        }
                        else
                        {
                            uiDataLength--;
                        }
                    }

                    while ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk );  // DLE

                    watchdog ( );
                    while ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk );  // EoF

                    watchdog ( );
                    while ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk );  // Checksum high byte

                    watchdog ( );
                    while ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk );  // Checksum low byte

                    // Send sequence if originally requested
                    if ( ucJig == true )
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

                    ucWaitSlave = false;
                }
                else if ( ucRxByte == NAK )
                {
                    while ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk );  // Wait for reason
                    
                    watchdog ( );

                    // Send to PC
                    p2pTxByteMaster ( DLE );
                    p2pTxByteMaster ( NAK );
                    p2pTxByteMaster ( ucRxByte );   // Send reason
                    ucWaitSlave = false;            // Exit
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

    if ( uiCommsTimeout == 0 )
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
    uint8_t  ucChecksumType      = CSUM_ERROR;
    uint8_t  ucDataLength        = 0;
    uint8_t  ucDataLengthToSend  = 0;
    uint8_t  ucRxByte            = 0;
    uint8_t  ucStatus            = 0;
    uint8_t  ucSequenceOffset    = 0;
    uint8_t  ucWaitSlave         = true;
    uint16_t uiBaudRate          = 38400;
    uint16_t uiBufferEndPosn     = 0;
    uint16_t uiBufferStartPosn   = 0;
    uint16_t uiChecksumMaster    = 0;
    uint16_t uiChecksumMasterCRC = 0;
    uint16_t uiLoop              = 0;
    uint16_t uiRxChecksumMaster  = 0;

    // Wait for sensor response
    while ( ( uiCommsTimeout ) && ( ucWaitSlave == true ) )
    {
        watchdog ( );
        ucStatus = p2pRxByteMaster ( &ucRxByte );

        if ( ucStatus == p2pRxOk )
        {

            uiCommsTimeout = 500; // 5 seconds - Temperature compensation command write takes a long time

            if ( ucRxByte == DLE )
            {
                uiBufferStartPosn = uiRxBufferMasterPut - 1;

                addToChecksum    ( uiChecksumMaster    , ucRxByte );
                addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                watchdog ( );

                // Search for DLE DAT sequence)
                if ( ( ucRxByte == DAT ) && ( uiCommsTimeout ) )
                {
                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                    
                    // Wait for length
                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                    
                    watchdog ( );
                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    ucDataLength       = ucRxByte;
                    ucDataLengthToSend = ucDataLength + 7;  // Include start & end of frame bytes

                    // Wait for remaining bytes
                    while ( ucDataLength )
                    {
                        watchdog ( );
                        
                        while ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk );
                        
                        watchdog ( );
                        addToChecksum    ( uiChecksumMaster    , ucRxByte );
                        addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );
                        
                        if ( ucRxByte == DLE )
                        {
                            // Wait for next DLE - Byte stuffing extra byte not included in data length
                            while ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk );
                            
                            watchdog ( );
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
                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                    watchdog ( );
                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    // Wait for EOF
                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) )!= p2pRxOk ) && ( uiCommsTimeout ) );

                    watchdog ( );
                    addToChecksum    ( uiChecksumMaster    , ucRxByte );
                    addToChecksumCRC ( uiChecksumMasterCRC , ucRxByte );

                    // Next 2 bytes are the checksum
                    while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );
                    
                    watchdog ( );

                    if ( uiCommsTimeout )
                    {
                        uiRxChecksumMaster = ucRxByte;
                        uiRxChecksumMaster <<= 8;

                        while ( ( ( p2pRxByteMaster ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                        watchdog ( );

                        if  ( uiCommsTimeout )
                        {
                            uiRxChecksumMaster += ucRxByte;

                            if ( uiRxChecksumMaster == uiChecksumMaster )
                            {
                                ucChecksumType = CSUM_SIMPLE;
                            }
                            else if ( uiRxChecksumMaster == uiChecksumMasterCRC )
                            {
                                ucChecksumType = CSUM_CRC;
                            }
                            else
                            {
                                // Checksum error
                                ucChecksumType = CSUM_ERROR;
                                // Reason
                                if ( ucJig == true )
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

                        if ( ucChecksumType != CSUM_ERROR )
                        {
                            if ( ucJig == true )
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

                            if ( aucRxBufferMaster [ 0 ] == 0xFF )
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

                            if ( ucFlagJigCommand == false )
                            {
                                // Forward write command on to sensor
                                // Strip out 0xFF, position bytes performing byte stuffing
                                for ( uiLoop = uiBufferStartPosn ; uiLoop < uiBufferEndPosn ; uiLoop++ )
                                {
                                    if ( uiLoop == ( uiBufferStartPosn + 4 ) )
                                    {
                                        // Do not send 0xff
                                    }
                                    else if ( uiLoop == ( uiBufferStartPosn + 5 ) )
                                    {
                                        // Do not send position
                                    }
                                    else if ( uiLoop == ( uiBufferStartPosn + 6 ) )
                                    {
                                        if ( ( aucRxBufferMaster [ uiBufferStartPosn + 5 ] ) == DLE )
                                        {
                                            // Do not send DLE when position = 0x10
                                            uiBufferEndPosn++;
                                        }
                                        else
                                        {
                                            p2pTxByteSlave ( aucRxBufferMaster [ uiLoop ] );

                                            if ( ucChecksumType == CSUM_SIMPLE )
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
                                        if ( ucChecksumType == CSUM_SIMPLE )
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
                                while ( ( ( p2pRxByteSlave ( &ucRxByte ) )!= p2pRxOk ) && ( uiCommsTimeout ) );

                                watchdog ( );

                                if ( uiCommsTimeout == 0 )
                                {
                                    if ( ucJig == true )
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
                                    if ( ucRxByte == DLE )
                                    {
                                        // Wait for sensor response
                                        while ( ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                        watchdog ( );

                                        if ( uiCommsTimeout == 0 )
                                        {
                                            if ( ucJig == true )
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
                                        else if ( ucRxByte == ACK )
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
                                            while ( ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                            watchdog ( );

                                            if ( uiCommsTimeout == 0 )
                                            {
                                                if ( ucJig == true )
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
                                                if ( ucRxByte == DLE )
                                                {
                                                    // Wait for sensor response
                                                    while ( ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                                    if ( uiCommsTimeout == 0 )
                                                    {
                                                        if ( ucJig == true )
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
                                                    else if ( ucRxByte == ACK )
                                                    {
                                                        if ( ucJig == true )
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
                                                    else if ( ucRxByte == NAK )
                                                    {
                                                        // NAK response
                                                        // Now wait for sensor reason
                                                        while ( ( ( p2pRxByteSlave( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                                        if ( ucJig == true )
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
                                                        if (ucJig == true)
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
                                                    if ( ucJig == true )
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
                                        else if ( ucRxByte == NAK )
                                        {
                                            // NAK response
                                            // Now wait for sensor reason
                                            while ( ( ( p2pRxByteSlave ( &ucRxByte ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                                            watchdog ( );

                                            if ( ucJig == true )
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
                                            if ( ucJig == true )
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
                                        if ( ucJig == true )
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
                                    ucWaitSlave = false;
                                }
                            }
                            else
                            {
                                // Jig write command
                                // Incorrect response
                                p2pTxByteMaster ( DLE );
                                p2pTxByteMaster ( ACK );

                                if ( ucJigCommand == 0xFB )
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

                                if ( ucJigCommand == 0xFC )
                                {
                                    // Power command
                                    if ( aucRxBufferMaster [ 14 ] == 0x50 )
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

                                    while ( uiCommsTimeout )
                                    {
                                        watchdog ( );
                                    }
                                }
                                else
                                {
                                    // Nothing to do
                                }

                                if ( ucJigCommand == 0xFD )
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

                                ucWaitSlave = false;
                            }
                        }
                        else
                        {
                            ucWaitSlave = false;
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
    uint8_t  ucPosition       = 0;
    uint8_t  ucProgramFlag    = false;
    uint8_t  ucRxByteMaster   = 0;
    uint16_t uiBytesToReceive = 0;
    uint16_t uiLines          = 0;

    ucPosition = aucRxBufferMaster [ 8 ];

    if ( ucPosition == 16 )
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

        while ( uiRxBufferMasterPut != uiBytesToReceive )   // Wait until command received
        {
            watchdog ( );
        }

        uiRxBufferMasterGet = 0;
        uiRxBufferMasterPut = 0;

        uiCommsTimeout = 500;   // 5 seconds;

        while ( ( ( p2pRxByteMaster ( &ucRxByteMaster ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

        if ( ucRxByteMaster == 'P' )
        {
            ucProgramFlag = true;

            memset ( aucRxBufferMaster , 0 , P2P_BUFFER_MASTER_SIZE );

            uiRxBufferMasterGet = 0;
            uiRxBufferMasterPut = 0;
            uiRxBufferSlaveGet  = 0;
            uiRxBufferSlavePut  = 0;

            p2pTxByteSlave ( 'P' );

            while ( ( uiRxBufferSlavePut < 1 ) && ( uiCommsTimeout ) )
            {
                watchdog ( );
            }

            if ( uiCommsTimeout )
            {
                p2pTxByteMaster ( aucRxBufferSlave [ 0 ] );
            }
            else
            {
                NOP;
            }

            if ( aucRxBufferSlave [ 0 ] == 0x42 )
            {
                // Sensor resonse with 'B'
                while ( ( ucProgramFlag == true ) && ( uiCommsTimeout ) )
                {
                    uiRxBufferSlavePut = 0;

                    while ( ( ( p2pRxByteMaster ( &ucRxByteMaster ) ) != p2pRxOk ) && ( uiCommsTimeout ) );

                    if ( uiCommsTimeout )
                    {
                        if ( ucRxByteMaster == 0xFF )
                        {
                            // End programming
                            ucProgramFlag = false;
                        }
                        else
                        {
                            p2pTxByteSlave ( ucRxByteMaster );

                            if ( ucRxByteMaster == 0x0A )
                            {
                                // Wait for 0x0A character from sensor - line programmed
                                while ( ( uiRxBufferSlavePut < 1 ) && ( uiCommsTimeout ) );

                                if ( uiCommsTimeout )
                                {
                                    uiLines++;
                                    // Clear buffer
                                    p2pTxByteMaster ( aucRxBufferSlave [ 0 ] );
                                    memset ( aucRxBufferMaster , 0 , P2P_BUFFER_MASTER_SIZE );

                                    uiCommsTimeout      = 200;  // 2 seconds
                                    uiRxBufferMasterPut = 0;
                                    uiRxBufferMasterPut = 0;
                                    uiRxBufferSlaveGet  = 0;
                                    uiRxBufferSlavePut  = 0;
                                }
                                else
                                {
                                    // End programming
                                    ucProgramFlag = false;
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
