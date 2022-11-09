/*******************************************************************
 *  Author:		Frank Kups                                 *
 *  Company:            Status Scientific Controls Ltd             *
 *  Project :           24-Way Premier IR Sensor Jig               *
 *  Filename:		Comms.c                                    *
 *  Date:		10/07/2012                                 *
 *  File Version:	1.0.0                                      *
 *  Other Files Required: p24F32KA01.gld, libpPIC24Fxxx-coff.a     *
 *  Tools Used: MPLAB GL  -> 8.84                                  *
 *              Compiler  -> 3.31                                  *
 *                                                                 *
 *******************************************************************
 */

// #include <p24fxxxx.h>
// #include <libpic30.h>	// must be after defines
// #include <string.h>
// #include "MisraTypes.h"
// #include "p2p.h"
// #include "def.h"
#include "main.h"

uint16_t uiChecksumSlave;
uint16_t uiCommsMode;

/// Control codes (slip resistant with Hamming distance of 2)

enum
{
    DLE = 0x10, // Data Link Escape
    RD = 0x13, // ReaD
    WR = 0x15, // WRite
    ACK = 0x16, // ACKnowledge
    NAK = 0x19, // Negative AcKnowledge
    DAT = 0x1a, // DATa (single frame)
    SEG = 0x1c, // data SEGment frame
    EoF = 0x1f, // End Of Frame
    WP1 = 0xe5, // Write Password byte 1
    WP2 = 0xa2 // Write Password byte 2
};

enum
{
    CSUM_ERROR = 0x00, // Checsum error
    CSUM_SIMPLE = 0x01, // Simple checksum
    CSUM_CRC = 0x02, // CRC checsum
};

extern uint8_t aucRxBufferMaster[];
extern uint16_t uiRxBufferMasterPut;
extern uint8_t aucRxBufferSlave[];
extern uint16_t uiRxBufferSlavePut;
extern uint16_t uiRxBufferMasterGet;
extern uint16_t uiRxBufferSlaveGet;
extern volatile uint16_t uiCommsTimeout;

extern void watchdog(void);
extern void p2pSendNAK(NAKReason reason);

void p2pTxByteSlave(uint8_t ucData);
void p2pTxByteMaster(uint8_t ucData);


void testSensorComms(void)
{
    uint16_t uiChecksumMasterCRC = 0;
    
    addToChecksumCRC(uiChecksumMasterCRC, 0x10);
    addToChecksumCRC(uiChecksumMasterCRC, 0x15);
    addToChecksumCRC(uiChecksumMasterCRC, 0xe5);
    addToChecksumCRC(uiChecksumMasterCRC, 0xa2);
    addToChecksumCRC(uiChecksumMasterCRC, 0x16);
    addToChecksumCRC(uiChecksumMasterCRC, 0x10);
    addToChecksumCRC(uiChecksumMasterCRC, 0x1f);

}

    
void reportDeviceFault(void)
{
    // no response, send data to PC
    p2pTxByteMaster(DLE);
    p2pTxByteMaster(NAK);
    p2pTxByteMaster(eP2pNAKdeviceFault); // send reason
}

uint8_t p2pRxByteMaster(uint8_t* pucData)
{
    uint8_t ucStatus;

    // asm ("disi #1000"); //		DISABLE_INTERRUPTS
    if (uiRxBufferMasterGet != uiRxBufferMasterPut)
    {
        *pucData = aucRxBufferMaster[uiRxBufferMasterGet++];
        if (uiRxBufferMasterGet == P2P_BUFFER_MASTER_SIZE)
        {
            uiRxBufferMasterGet = 0;
        }
        ucStatus = p2pRxOk;
    }
    else
    {
        ucStatus = p2pRxNone;
    }
    // asm ("disi #0"); //		ENABLE_INTERRUPTS

    return ucStatus;
}

uint8_t p2pRxByteSlave(uint8_t* pucData)
{
    uint8_t ucStatus;

    // asm ("disi #1000"); //		DISABLE_INTERRUPTS

    if (uiRxBufferSlaveGet != uiRxBufferSlavePut)
    {
        // get byte out of buffer
        *pucData = aucRxBufferSlave[uiRxBufferSlaveGet++];
        if (uiRxBufferSlaveGet == P2P_BUFFER_SLAVE_SIZE)
        {
            uiRxBufferSlaveGet = 0;
        }
        ucStatus = p2pRxOk;
    }
    else
    {
        ucStatus = p2pRxNone;
    }
    // asm ("disi #0"); //		ENABLE_INTERRUPTS

    return ucStatus;
}


// get command from master data buffer

uint16_t p2pPollMaster(void)
{
    uint8_t ucChecksumType = CSUM_ERROR;
    uint8_t ucWaitMaster = true;
    uint16_t uiBufferStartPosn = 0;
    uint16_t uiMuxPosn = 0;
    uint8_t ucRxByte;
    uint8_t ucStatus;
    uint16_t uiChecksumMaster = 0;
    uint16_t uiChecksumMasterCRC = 0;
    uint16_t uiRxChecksumMaster = 0;
    uint16_t uiLoop;
    uiCommsMode = COMMS_NONE;

    while (ucWaitMaster == true) // wait until command received
    {
        watchdog_update();
        ucStatus = p2pRxByteMaster(&ucRxByte);

        if (ucStatus == p2pRxOk)
        {
            uiCommsTimeout = 200; // 2 seconds
            if (ucRxByte == DLE)
            {
                addToChecksum(uiChecksumMaster, ucRxByte);
                addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);

                while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));

                // search for DLE RD sequence)
                if ((ucRxByte == RD) && (uiCommsTimeout))
                {
                    uiCommsMode = COMMS_READ;
                    addToChecksum(uiChecksumMaster, ucRxByte);
                    addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                    if ((ucRxByte == 0xFF) && (uiCommsTimeout))
                    {
                        uiBufferStartPosn = uiRxBufferMasterPut - 3;
                        addToChecksum(uiChecksumMaster, ucRxByte);
                        addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                        while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));

                        if (uiCommsTimeout)
                        {
                            addToChecksum(uiChecksumMaster, ucRxByte);
                            addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);

                            // this byte is the mux posn
                            uiMuxPosn = (uint16_t) ucRxByte - 1; // posn 1 = '0'
/*
                            // disable multiplexers before enabling the selected mux to
                            // avoid possible connection of two sensors
                            LATA &= ~7; // use LAT because of RMW operation

                            if (uiMuxPosn < 24)
                            {
                                // convert mux number to a value between 0 and 7
                                if (uiMuxPosn < 8)
                                {
                                    uiMuxPosn <<= 13;   // move into upper 3 bits (15,14,13)
                                    LATB &= ~0xE000;    // clear previous setting
                                    LATB |= uiMuxPosn;  // update multiplexer
                                    //EN1_8 = 1;
                                    LATA |= 1;
                                }
                                else if (uiMuxPosn < 16)
                                {
                                    uiMuxPosn -= 8;
                                    uiMuxPosn <<= 13;   // move into upper 3 bits (15,14,13)
                                    LATB &= ~0xE000;    // clear previous setting
                                    LATB |= uiMuxPosn;  // update multiplexer
                                    //EN9_16 = 1;
                                    LATA |= 2;
                                }
                                else
                                {
                                    uiMuxPosn -= 16;
                                    uiMuxPosn <<= 13;   // move into upper 3 bits (15,14,13)
                                    LATB &= ~0xE000;    // clear previous setting
                                    LATB |= uiMuxPosn;  // update multiplexer
                                    //EN17_24 = 1;
                                    LATA |= 4;
                                }
                            }
                            else
                            {
                                // incorrect mux assignment, all muxes disabled, no response from sensor
                                LATA = 0;
                            }
*/
                            Set_MUX ( uiMuxPosn );

                            if (ucRxByte == DLE) // position byte 0x10
                            {
                                // take into account extra DLE - sensor position 16 adds an extra DLE
                                while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                addToChecksum(uiChecksumMaster, ucRxByte);
                                addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                            }

                            // now wait for remaining data
                            while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                            if (uiCommsTimeout)
                            {
                                addToChecksum(uiChecksumMaster, ucRxByte);
                                addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);

                                if (ucRxByte == DLE) // command byte 0x10
                                {
                                    // take into account extra DLE - sensor position 16 adds an extra DLE
                                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                    addToChecksum(uiChecksumMaster, ucRxByte);
                                    addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                                }

                                while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                if (uiCommsTimeout)
                                {
                                    addToChecksum(uiChecksumMaster, ucRxByte);
                                    addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                    if (uiCommsTimeout)
                                    {
                                        addToChecksum(uiChecksumMaster, ucRxByte);
                                        addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                                        // next 2 bytes are the checksum
                                        while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                        if (uiCommsTimeout)
                                        {
                                            uiRxChecksumMaster = ucRxByte;
                                            uiRxChecksumMaster <<= 8;

                                            while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                            if (uiCommsTimeout)
                                            {
                                                uiRxChecksumMaster += ucRxByte;

                                                if (uiRxChecksumMaster == uiChecksumMaster)
                                                {
                                                    ucChecksumType = CSUM_SIMPLE;
                                                }
                                                else if (uiRxChecksumMaster == uiChecksumMasterCRC)
                                                {
                                                    ucChecksumType = CSUM_CRC;
                                                }
                                                else
                                                {
                                                    // checksum error
                                                    ucChecksumType = CSUM_ERROR;
                                                    //reason
                                                    p2pTxByteMaster(DLE);
                                                    p2pTxByteMaster(NAK);
                                                    p2pTxByteMaster(eP2pNAKchecksumFailed);

                                                }

                                                uiChecksumSlave = 0;
                                                // forward message on to sensor
                                                // strip out 0xff, position bytes preforming byte stuffing
                                                for (uiLoop = uiBufferStartPosn; uiLoop < (uiRxBufferMasterGet - 2); uiLoop++)
                                                {
                                                    if (uiLoop == (uiBufferStartPosn + 2))
                                                    {
                                                        //do not send 0xff
                                                    }
                                                    else if (uiLoop == (uiBufferStartPosn + 3))
                                                    {
                                                        //do not send position

                                                    }
                                                    else if ((uiLoop == (uiBufferStartPosn + 4)) && (aucRxBufferMaster[uiBufferStartPosn + 3] == DLE))
                                                    {
                                                        //do not send DLE when position = 0x10

                                                    }
                                                    else
                                                    {
                                                        p2pTxByteSlave(aucRxBufferMaster[uiLoop]);
                                                        if (ucChecksumType == CSUM_SIMPLE)
                                                        {
                                                            addToChecksum(uiChecksumSlave, aucRxBufferMaster[uiLoop]);
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC(uiChecksumSlave, aucRxBufferMaster[uiLoop]);
                                                        }
                                                    }
                                                }
                                                // now send the re-calculated checksum
                                                p2pTxByteSlave((uiChecksumSlave >> 8));
                                                p2pTxByteSlave((uiChecksumSlave & 0xff));

                                                ucWaitMaster = false;
                                            }
                                            else
                                            {
                                                ucWaitMaster = false;
                                            }
                                        }
                                        else
                                        {
                                            ucWaitMaster = false; // force exit
                                        }
                                    }
                                    else
                                    {
                                        ucWaitMaster = false; // force exit
                                    }
                                }
                                else
                                {
                                    ucWaitMaster = false; // force exit
                                }
                            }
                            else
                            {
                                ucWaitMaster = false; // force exit
                            }
                        }
                        else
                        {
                            ucWaitMaster = false; // force exit
                        }
                    }
                    else
                    {
                        ucWaitMaster = false; // force exit
                    }

                }
                // search for DLE WR sequence)
                else if ((ucRxByte == WR) && (uiCommsTimeout))
                {
                    uiCommsTimeout = 200; // maximum wait time of 2 seconds for write operation

                    addToChecksum(uiChecksumMaster, ucRxByte);
                    addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);

                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                    if ((ucRxByte == WP1) && (uiCommsTimeout))
                    {
                        addToChecksum(uiChecksumMaster, ucRxByte);
                        addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                    }

                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                    if ((ucRxByte == WP2) && (uiCommsTimeout))
                    {
                        addToChecksum(uiChecksumMaster, ucRxByte);
                        addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                    }

                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                    if ((ucRxByte == 0xFF) && (uiCommsTimeout))
                    {
                        uiBufferStartPosn = uiRxBufferMasterPut - 5;
                        addToChecksum(uiChecksumMaster, ucRxByte);
                        addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                        while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));

                        if (uiCommsTimeout)
                        {
                            addToChecksum(uiChecksumMaster, ucRxByte);
                            addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);

                            // this byte is the mux posn
                            uiMuxPosn = (uint16_t) ucRxByte - 1; // posn 1 = '0'
/*
                            // disable multiplexers before enabling the selected mux to
                            // avoid possible connection of two sensors
                            LATA &= ~7; // use LAT because of RMW operation

                            if (uiMuxPosn < 24)
                            {
                                // convert mux number to a value between 0 and 7
                                if (uiMuxPosn < 8)
                                {
                                    uiMuxPosn <<= 13; // move into upper 3 bits (15,14,13)
                                    LATB &= ~0xE000; // clear previous setting
                                    LATB |= uiMuxPosn; // update multiplexer
                                    //EN1_8 = 1;
                                    LATA |= 1;
                                }
                                else if (uiMuxPosn < 16)
                                {
                                    uiMuxPosn -= 8;
                                    uiMuxPosn <<= 13; // move into upper 3 bits (15,14,13)
                                    LATB &= ~0xE000; // clear previous setting
                                    LATB |= uiMuxPosn; // update multiplexer
                                    //EN9_16 = 1;
                                    LATA |= 2;
                                }
                                else
                                {
                                    uiMuxPosn -= 16;
                                    uiMuxPosn <<= 13; // move into upper 3 bits (15,14,13)
                                    LATB &= ~0xE000; // clear previous setting
                                    LATB |= uiMuxPosn; // update multiplexer
                                    //EN17_24 = 1;
                                    LATA |= 4;
                                }
                            }
                            else
                            {
                                // incorrect mux assignment, all muxes disabled, no response from sensor
                            }
*/
                            Set_MUX ( uiMuxPosn );

                            if (ucRxByte == DLE) // position byte 0x10
                            {
                                // take into account extra DLE - sensor position 16 adds an extra DLE
                                while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                addToChecksum(uiChecksumMaster, ucRxByte);
                                addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                            }

                            // now wait for remaining data
                            while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                            if (uiCommsTimeout)
                            {
                                addToChecksum(uiChecksumMaster, ucRxByte);
                                addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);

                                if (ucRxByte == DLE) // command byte 0x10
                                {
                                    // take into account extra DLE - sensor position 16 adds an extra DLE
                                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                    addToChecksum(uiChecksumMaster, ucRxByte);
                                    addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                                }

                                while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                if (uiCommsTimeout)
                                {
                                    addToChecksum(uiChecksumMaster, ucRxByte);
                                    addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                    if (uiCommsTimeout)
                                    {
                                        addToChecksum(uiChecksumMaster, ucRxByte);
                                        addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);

                                        // next 2 bytes are the checksum
                                        while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                        if (uiCommsTimeout)
                                        {
                                            uiRxChecksumMaster = ucRxByte;
                                            uiRxChecksumMaster <<= 8;

                                            while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                            if (uiCommsTimeout)
                                            {
                                                uiRxChecksumMaster += ucRxByte;

                                                if (uiRxChecksumMaster == uiChecksumMaster)
                                                {
                                                    ucChecksumType = CSUM_SIMPLE;
                                                }
                                                else if (uiRxChecksumMaster == uiChecksumMasterCRC)
                                                {
                                                    ucChecksumType = CSUM_CRC;
                                                }
                                                else
                                                {
                                                    // checksum error
                                                    ucChecksumType = CSUM_ERROR;
                                                    //reason
                                                    p2pTxByteMaster(DLE);
                                                    p2pTxByteMaster(NAK);
                                                    p2pTxByteMaster(eP2pNAKchecksumFailed);

                                                }

                                                ucWaitMaster = false;

                                                uiChecksumSlave = 0;
                                                // forward message on to sensor
                                                // strip out 0xff, position bytes preforming byte stuffing
                                                for (uiLoop = uiBufferStartPosn; uiLoop < (uiRxBufferMasterGet - 2); uiLoop++)
                                                {
                                                    if (uiLoop == (uiBufferStartPosn + 4))
                                                    {
                                                        //do not send 0xff
                                                    }
                                                    else if (uiLoop == (uiBufferStartPosn + 5))
                                                    {
                                                        //do not send position

                                                    }
                                                    else if ((uiLoop == (uiBufferStartPosn + 5)) && (aucRxBufferMaster[uiBufferStartPosn + 6] == DLE))
                                                    {
                                                        //do not send DLE when position = 0x10

                                                    }
                                                    else
                                                    {
                                                        p2pTxByteSlave(aucRxBufferMaster[uiLoop]);
                                                        if (ucChecksumType == CSUM_SIMPLE)
                                                        {
                                                            addToChecksum(uiChecksumSlave, aucRxBufferMaster[uiLoop]);
                                                        }
                                                        else
                                                        {
                                                            addToChecksumCRC(uiChecksumSlave, aucRxBufferMaster[uiLoop]);
                                                        }
                                                    }
                                                }
                                                // now send the re-calculated checksum
                                                p2pTxByteSlave((uiChecksumSlave >> 8));
                                                p2pTxByteSlave((uiChecksumSlave & 0xff));

                                                ucWaitMaster = false;

                                                // wait for sensor response
                                                while ((p2pRxByteSlave(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                                if (uiCommsTimeout)
                                                {
                                                    if (ucRxByte == DLE)
                                                    {
                                                        while ((p2pRxByteSlave(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                                        if (ucRxByte == ACK)
                                                        {
                                                            // valid response
                                                            p2pTxByteMaster(DLE);
                                                            p2pTxByteMaster(ACK);

                                                            uiCommsMode = COMMS_WRITE;

                                                            // now the wait for data
                                                            ucWaitMaster = false; // force exit

                                                        }
                                                        else
                                                        {
                                                            // timeout message
                                                            p2pTxByteMaster(DLE);
                                                            p2pTxByteMaster(NAK);
                                                            p2pTxByteMaster(eP2pNAKinvalidData);
                                                        }
                                                    }
                                                    else
                                                    {
                                                        // timeout message
                                                        p2pTxByteMaster(DLE);
                                                        p2pTxByteMaster(NAK);
                                                        p2pTxByteMaster(eP2pNAKinvalidData);
                                                    }
                                                }
                                                else
                                                {
                                                    // timeout message
                                                    p2pTxByteMaster(DLE);
                                                    p2pTxByteMaster(NAK);
                                                    p2pTxByteMaster(eP2pNAKdeviceFault);
                                                }

                                            }
                                            else
                                            {
                                                ucWaitMaster = false; // force exit
                                            }
                                        }
                                        else
                                        {
                                            ucWaitMaster = false; // force exit
                                        }
                                    }
                                    else
                                    {
                                        ucWaitMaster = false; // force exit
                                    }
                                }
                                else
                                {
                                    ucWaitMaster = false; // force exit
                                }
                            }
                            else
                            {
                                ucWaitMaster = false; // force exit
                            }
                        }
                        else
                        {
                            ucWaitMaster = false; // force exit
                        }
                    }
                    else
                    {
                        ucWaitMaster = false; // force exit
                    }
                }
                else if ((ucRxByte == 0x21) && (uiCommsTimeout))
                {

                    // the mentorPC library sends out 0x10, 0x21, counts byte
                    // when the counts byte = 0x10 this upsets the normal DLE message
                    // ignore this byte at all times as this message is not implemented

                    // ignore next counts byte
                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                    // ignore the next DLE byte
                    //while((p2pRxByteSlave(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));

                    uiChecksumMaster = 0;
                    uiChecksumMasterCRC = 0;

                    //while(ucWaitMaster == true);

                }

            }
            else
            {
                // search for DLE WR sequence)
                uiChecksumMaster = 0;
                uiChecksumMasterCRC = 0;

            }
        }
    }

    if (uiCommsTimeout == 0)
    {
        uiCommsTimeout = 0;
    }

    return uiCommsTimeout;
}


// fixed baud rate of 38400

void commsInit(void)
{
    // Slave - sensor
    // U2BRG = (8000000L / (4L * 38400)) - 1;
    // U2MODE = 0x8008; // Enable uart, 	BRGH = 1;
    // U2STA = 0x0510;
    // IPC7bits.U2RXIP = 5; // level 5 priority
    // IFS1bits.U2RXIF = 0; // clear interrupt
    // IEC1bits.U2RXIE = 1; // Enable UART2 interrupts

    // Master - PC
    // U1BRG = (8000000L / (4L * 38400)) - 1;
    // U1MODE = 0x8008; // Enable uart, 	BRGH = 1;
    // U1STA = 0x0510;
    // IPC2bits.U1RXIP = 7; // level 7 priority
    // IFS0bits.U1RXIF = 0; // clear interrupt
    // IEC0bits.U1RXIE = 1; // Enable UART2 interrupts

}

void p2pTxByteSlave(uint8_t ucData) // Write to sensors
{
    uart_tx_wait_blocking(UART_SEN);
    uart_putc(UART_SEN, ucData);
    // uart_is_writable(UART_SEN);
    // while (U2STAbits.TRMT == 0);    // Wait until shift register is empty ( '0' = byte being processed )
    // while (U2STAbits.UTXBF);        // Wait until TX buffer is empty ( '1' = byte being processed )
    // U2TXREG = ucData;               // Write byte to UART peripheral
}

void p2pTxByteMaster(uint8_t ucData)    // Write to PC
{
    uart_tx_wait_blocking(UART_PC);
    uart_putc(UART_PC, ucData);
    // uart_is_writable(UART_PC);
    // while (U1STAbits.TRMT == 0);
    // while (U1STAbits.UTXBF);
    // U1TXREG = ucData;
}




// sensor response to READ request

uint16_t p2pPollSlaveRead(void)
{
    static uint8_t ucError = 0;
    uint16_t uiLoop;
    uint8_t ucWaitSlave = true;
    uint8_t ucRxByte;
    uint8_t ucStatus;
    uint8_t ucDataLength;
    uint8_t ucDataLengthToSend;

    // wait for sensor response
    while ((uiCommsTimeout) && (ucWaitSlave == true))
    {
        watchdog_update();
        ucStatus = p2pRxByteSlave(&ucRxByte);
        if (ucStatus == p2pRxOk)
        {
            // search for DLE DAT sequence)
            if (ucRxByte == DLE)
            {
                while (p2pRxByteSlave(&ucRxByte) != p2pRxOk);
                if (ucRxByte == DAT)
                {
                    while (p2pRxByteSlave(&ucRxByte) != p2pRxOk);
                    ucDataLength = ucRxByte;
                    ucDataLengthToSend = ucDataLength + 7; // include start / end of frame bytes
                    // now wait for remaining bytes
                    while (ucDataLength)
                    {
                        while (p2pRxByteSlave(&ucRxByte) != p2pRxOk);
                        if (ucRxByte == DLE)
                        {
                            // wait for next DLE - byte stuffing extra byte not included in data length
                            while (p2pRxByteSlave(&ucRxByte) != p2pRxOk);
                            ucDataLength--;
                            ucDataLengthToSend++; // include byte stuffing bytes
                        }
                        else
                        {
                            ucDataLength--;
                        }
                    }
                    while (p2pRxByteSlave(&ucRxByte) != p2pRxOk); // DLE
                    while (p2pRxByteSlave(&ucRxByte) != p2pRxOk); // EoF
                    while (p2pRxByteSlave(&ucRxByte) != p2pRxOk); // checksum high byte
                    while (p2pRxByteSlave(&ucRxByte) != p2pRxOk); // checksum low byte

                    // send buffer back to PC
                    for (uiLoop = (uiRxBufferSlavePut - ucDataLengthToSend); uiLoop < ucDataLengthToSend; uiLoop++)
                    {
                        p2pTxByteMaster(aucRxBufferSlave[uiLoop]);
                    }

                    ucWaitSlave = false;
                }
                else if (ucRxByte == NAK)
                {
                    while (p2pRxByteSlave(&ucRxByte) != p2pRxOk); // wait for reason
                    // send to PC
                    p2pTxByteMaster(DLE);
                    p2pTxByteMaster(NAK);
                    p2pTxByteMaster(ucRxByte); // send reason
                    ucWaitSlave = false; // exit

                }
                else
                {
                }
            }
        }

    }

    if (uiCommsTimeout == 0)
    {
        ucError++;
    }

    // clear buffer
    // asm ("disi #1000"); //		DISABLE_INTERRUPTS
    memset(aucRxBufferMaster, 0, P2P_BUFFER_MASTER_SIZE);
    uiRxBufferMasterPut = 0;
    uiRxBufferMasterGet = 0;

    memset(aucRxBufferSlave, 0, P2P_BUFFER_SLAVE_SIZE);
    uiRxBufferSlavePut = 0;
    uiRxBufferSlaveGet = 0;

    // asm ("disi #0"); //		ENABLE_INTERRUPTS

    return uiCommsTimeout;
}


// sensor response to WRITE request

uint16_t p2pPollSlaveWrite(void)
{
    uint8_t ucChecksumType = CSUM_ERROR;
    uint8_t ucWaitSlave = true;
    uint8_t ucDataLength;
    uint8_t ucDataLengthToSend;
    uint16_t uiBufferStartPosn = 0;
    uint8_t ucRxByte;
    uint8_t ucStatus;
    uint16_t uiRxChecksumMaster = 0;
    uint16_t uiChecksumMaster = 0;
    uint16_t uiChecksumMasterCRC = 0;
    uint16_t uiLoop;

    // wait for sensor response
    while ((uiCommsTimeout) && (ucWaitSlave == true))
    {

        watchdog_update();
        ucStatus = p2pRxByteMaster(&ucRxByte);

        if (ucStatus == p2pRxOk)
        {
            uiCommsTimeout = 500; // 5 seconds - temp comp command write take a long time
            if (ucRxByte == DLE)
            {
                uiBufferStartPosn = uiRxBufferMasterPut - 1;

                addToChecksum(uiChecksumMaster, ucRxByte);
                addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);

                while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));

                // search for DLE DAT sequence)
                if ((ucRxByte == DAT) && (uiCommsTimeout))
                {
                    addToChecksum(uiChecksumMaster, ucRxByte);
                    addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                    // wait for length
                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                    addToChecksum(uiChecksumMaster, ucRxByte);
                    addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);

                    ucDataLength = ucRxByte;
                    ucDataLengthToSend = ucDataLength + 7; // include start / end of frame bytes

                    // now wait for remaining bytes
                    while (ucDataLength)
                    {
                        while (p2pRxByteMaster(&ucRxByte) != p2pRxOk);
                        addToChecksum(uiChecksumMaster, ucRxByte);
                        addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                        if (ucRxByte == DLE)
                        {
                            // wait for next DLE - byte stuffing extra byte not included in data length
                            while (p2pRxByteMaster(&ucRxByte) != p2pRxOk);
                            addToChecksum(uiChecksumMaster, ucRxByte);
                            addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                            ucDataLength--;
                            ucDataLengthToSend++; // include byte stuffing bytes
                        }
                        else
                        {
                            ucDataLength--;
                        }
                    }

                    // wait for DLE
                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                    addToChecksum(uiChecksumMaster, ucRxByte);
                    addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);
                    // wait for EOF
                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                    addToChecksum(uiChecksumMaster, ucRxByte);
                    addToChecksumCRC(uiChecksumMasterCRC, ucRxByte);

                    // next 2 bytes are the checksum
                    while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                    if (uiCommsTimeout)
                    {
                        uiRxChecksumMaster = ucRxByte;
                        uiRxChecksumMaster <<= 8;

                        while ((p2pRxByteMaster(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                        if (uiCommsTimeout)
                        {
                            uiRxChecksumMaster += ucRxByte;

                            if (uiRxChecksumMaster == uiChecksumMaster)
                            {
                                ucChecksumType = CSUM_SIMPLE;
                            }
                            else if (uiRxChecksumMaster == uiChecksumMasterCRC)
                            {
                                ucChecksumType = CSUM_CRC;
                            }
                            else
                            {
                                // checksum error
                                ucChecksumType = CSUM_ERROR;
                                //reason
                                p2pTxByteMaster(DLE);
                                p2pTxByteMaster(NAK);
                                p2pTxByteMaster(eP2pNAKchecksumFailed);

                            }
                        }

                        if (ucChecksumType != CSUM_ERROR)
                        {
                            // forward data to sensor as is
                            for (uiLoop = uiBufferStartPosn; uiLoop < (uiBufferStartPosn + ucDataLengthToSend); uiLoop++)
                            {
                                p2pTxByteSlave(aucRxBufferMaster[uiLoop]);
                            }

                            // now wait for sensor response
                            while ((p2pRxByteSlave(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));

                            if (uiCommsTimeout == 0)
                            {
                                // timeout message
                                p2pTxByteMaster(DLE);
                                p2pTxByteMaster(NAK);
                                p2pTxByteMaster(eP2pNAKdeviceFault);
                            }
                            else
                            {
                                if (ucRxByte == DLE)
                                {
                                    // now wait for sensor response
                                    while ((p2pRxByteSlave(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));
                                    if (uiCommsTimeout == 0)
                                    {
                                        // timeout message
                                        p2pTxByteMaster(DLE);
                                        p2pTxByteMaster(NAK);
                                        p2pTxByteMaster(eP2pNAKdeviceFault);
                                    }
                                    else if (ucRxByte == ACK)
                                    {
                                        // correct response
                                        p2pTxByteMaster(DLE);
                                        p2pTxByteMaster(ACK);
                                    }
                                    else if (ucRxByte == NAK)
                                    {
                                        // NAK response
                                        // now wait for sensor reason
                                        while ((p2pRxByteSlave(&ucRxByte) != p2pRxOk) && (uiCommsTimeout));

                                        p2pTxByteMaster(DLE);
                                        p2pTxByteMaster(NAK);
                                        p2pTxByteMaster(ucRxByte); // send reason
                                    }
                                    else
                                    {
                                        // incorrect response
                                        p2pTxByteMaster(DLE);
                                        p2pTxByteMaster(NAK);
                                        p2pTxByteMaster(eP2pNAKinvalidData);
                                    }

                                }
                                else
                                {
                                    // incorrect response
                                    p2pTxByteMaster(DLE);
                                    p2pTxByteMaster(NAK);
                                    p2pTxByteMaster(eP2pNAKinvalidData);
                                }
                            }

                            ucWaitSlave = false;
                        }
                        else
                        {
                            ucWaitSlave = false;
                        }
                    }
                }
            }
        }
    }

    // clear buffer
    // asm ("disi #1000"); //		DISABLE_INTERRUPTS
    memset(aucRxBufferMaster, 0, P2P_BUFFER_MASTER_SIZE);
    uiRxBufferMasterPut = 0;
    uiRxBufferMasterGet = 0;

    memset(aucRxBufferSlave, 0, P2P_BUFFER_SLAVE_SIZE);
    uiRxBufferSlavePut = 0;
    uiRxBufferSlaveGet = 0;

    // asm ("disi #0"); //		ENABLE_INTERRUPTS

    return uiCommsTimeout;
}
