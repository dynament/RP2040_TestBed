/*
*******************************************************************************
 *  Author:             Craig Hemingway                                       *
 *  Company:            Dynament Ltd.                                         *
 *                      Status Scientific Controls Ltd.                       *
 *  Project :           24-Way Premier IR Sensor Jig                          *
 *  Filename:           dac.h                                                *
 *  Date:               04/01/2023                                            *
 *  File Version:   	4.0.0                                                 *
 *  Version history:    4.0.0 - 04/01/2023 - Craig Hemingway                  *
 *                          PIC code ported over to RP2040                    *
 *                          Additional DAC check & adjust features            *
 *                      3.0.0 - 27/01/2014 - Frank Kups                       *
 *                          Latest program for sensor jig Version 4           *
 *  Tools Used: Visual Studio Code -> 1.73.1                                  *
 *              Compiler           -> GCC 11.3.1 arm-none-eabi                *
 *                                                                            *
 ******************************************************************************
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DAC_H
#define __DAC_H

#include <pico/stdlib.h>

uint8_t Check_Response ( void );
void    DAC_Check      ( void );
void    Set_MUX        ( uint8_t sensor );

#endif /* __DAC_H */

/*** end of file ***/
