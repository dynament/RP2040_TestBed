/*******************************************************************
 *	Author:			Frank Kups                                     *
 *	Company:		Status Scientific Controls Ltd             	   *
 *	Project :		Premier IR Sensor							   *
 *  Filename:		Comms.h                                        *
 *  Date:			22/08/2011                                     *
 *  File Version:	4.03                                           *
 *  Other Files Required: p30f3013boot.gld, libp30F3013A.a         *
 *  Tools Used: MPLAB GL  -> 8.73                                  *
 *              Compiler  -> 3.30c                                 *
 *                                                                 *
 *******************************************************************
*/

#ifndef _COMMS
#define _COMMS

#include "MisraTypes.h"

#define BAUD_RATE 38400L


void commsInit(void);

// Variable ID's
enum {
	idConfig = 0,
	idLiveData,						//1
	idZeroSensor1,					//2
	idSpanSensor1,					//3
	idFirmwareVersion,				//4
	idFirmwareUpdate,				//5
	idLiveDataSimple,				//6
	idPrivateData,					//7
	idZeroProfile1,					//8
	idSpanProfile1,					//9
	idTemperatureProfile,			//10
	idUserData,						//11
	idSetAnalogueOutput,			//12
	idSetTempComp,					//13
	idReadTempComp,					//14
	idDetAnalysis,					//15
	idMinMaxTemperature,			//16
	idCommsOverride,				//17
	idTemperatureOffset,			//18
	idSpanSensor2,					//19
	idZeroProfile2,					//20
	idSpanProfile2,					//21
	idZeroSensor2,					//22
	idSpanSensor3,					//23
	idSpanProfile3,					//24
	idTempCompDataCH4Z,				//25
	idTempCompDataCO2Z,				//26
	idTempCompDataCH4LS,			//27
	idTempCompDataCH4HS,			//28
	idTempCompDataHCS,				//29
	idTempCompDataCO2S,				//30
	idTempCompTemperatureCH4Z,		//31
	idTempCompTemperatureCO2Z,		//32
	idTempCompTemperatureCH4LS,		//33
	idTempCompTemperatureCH4HS,		//34
	idTempCompTemperatureHCS,		//35
	idTempCompTemperatureCO2S,		//36
	idZeroTempComp,					//37
	idSpanTempComp,					//38
	idTempCompEndFill,				//39
	idTempCompAveragingOn,			//40
	idTempCompAveragingOff,			//41
	idFactoryRestore,				//42
	idSetRange,						//43
	idLiveData2,					//44
	idLiveDataASCII,				//45
};

#endif

float32_t calculateSpanFactor(unsigned char range);
void taskComms(unsigned char wait);

