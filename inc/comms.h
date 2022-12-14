/*
*******************************************************************************
 *  Author:             Frank Kups                                            *
 *  Company:            Dynament Ltd.                                         *
 *                      Status Scientific Controls Ltd.                       *
 *  Project :           24-Way Premier IR Sensor Jig                          *
 *  Filename:   		comms.h                                               *
 *  Date:		        30/11/2022                                            *
 *  File Version:   	5.0.0                                                 *
 *  Version history:    5.0.0 - 30/11/2022 - Craig Hemingway                  *
 *                          PIC code ported over to RP2040                    *
 *                      4.0.3 - 22/08/2011 - Frank Kups                       *
 *                          Latest program for sensor jig Version 4           *
 *  Tools Used: Visual Studio Code -> 1.73.1                                  *
 *              Compiler           -> GCC 11.3.1 arm-none-eabi                *
 *                                                                            *
 ******************************************************************************
*/

#ifndef _COMMS
#define _COMMS

#include <stdint.h>

#define COMMS_WAIT      0
#define COMMS_READ      1
#define COMMS_WRITE     2

#define P2P_BUFFER_MASTER_SIZE  280
#define P2P_BUFFER_SLAVE_SIZE   280

// Variable ID's
enum
{
	idConfig = 0,
	idLiveData,						//  1
	idZeroSensor1,					//  2
	idSpanSensor1,					//  3
	idFirmwareVersion,				//  4
	idFirmwareUpdate,				//  5
	idLiveDataSimple,				//  6
	idPrivateData,					//  7
	idZeroProfile1,					//  8
	idSpanProfile1,					//  9
	idTemperatureProfile,			// 10
	idUserData,						// 11
	idSetAnalogueOutput,			// 12
	idSetTempComp,					// 13
	idReadTempComp,					// 14
	idDetAnalysis,					// 15
	idMinMaxTemperature,			// 16
	idCommsOverride,				// 17
	idTemperatureOffset,			// 18
	idSpanSensor2,					// 19
	idZeroProfile2,					// 20
	idSpanProfile2,					// 21
	idZeroSensor2,					// 22
	idSpanSensor3,					// 23
	idSpanProfile3,					// 24
	idTempCompDataCH4Z,				// 25
	idTempCompDataCO2Z,				// 26
	idTempCompDataCH4LS,			// 27
	idTempCompDataCH4HS,			// 28
	idTempCompDataHCS,				// 29
	idTempCompDataCO2S,				// 30
	idTempCompTemperatureCH4Z,		// 31
	idTempCompTemperatureCO2Z,		// 32
	idTempCompTemperatureCH4LS,		// 33
	idTempCompTemperatureCH4HS,		// 34
	idTempCompTemperatureHCS,		// 35
	idTempCompTemperatureCO2S,		// 36
	idZeroTempComp,					// 37
	idSpanTempComp,					// 38
	idTempCompEndFill,				// 39
	idTempCompAveragingOn,			// 40
	idTempCompAveragingOff,			// 41
	idFactoryRestore,				// 42
	idSetRange,						// 43
	idLiveData2,					// 44
	idLiveDataASCII,				// 45
};

// Control codes ( slip resistant with Hamming distance of 2 )
enum
{
    DLE = 0x10, // Data Link Escape
    RD  = 0x13, // ReaD
    WR  = 0x15, // WRite
    ACK = 0x16, // ACKnowledge
    NAK = 0x19, // Negative AcKnowledge
    DAT = 0x1A, // DATa ( single frame )
    EoF = 0x1F, // End Of Frame
    SEQ = 0x21, // frame SEQuence
    WP1 = 0xE5, // Write Password byte 1
    WP2 = 0xA2, // Write Password byte 2
};

enum
{
    CSUM_ERROR  = 0x00, // Checksum error
    CSUM_SIMPLE = 0x01, // Simple checksum
    CSUM_CRC    = 0x02, // CRC checsum
};

uint16_t p2pPollMaster                ( void );
uint16_t p2pPollSlaveRead             ( void );
uint16_t p2pPollSlaveWrite            ( void );
uint16_t p2pPollSlaveWritePassThrough ( void );

#endif

/*** end of file ***/
