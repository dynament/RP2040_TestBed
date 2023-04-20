/*
*******************************************************************************
 *  Author:             Craig Hemingway                                       *
 *  Company:            Dynament Ltd.                                         *
 *                      Status Scientific Controls Ltd.                       *
 *  Project :           24-Way Premier IR Sensor Jig                          *
 *  Filename:           dac.h                                                 *
 *  Date:               28/03/2023                                            *
 *  File Version:   	1.0.0                                                 *
 *  Version history:    1.0.0 - 28/03/2023 - Craig Hemingway                  *
 *                          Initial release                                   *
 *  Tools Used: Visual Studio Code -> 1.73.1                                  *
 *              Compiler           -> GCC 11.3.1 arm-none-eabi                *
 *                                                                            *
 ******************************************************************************
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DAC_H
#define __DAC_H

/* Includes ------------------------------------------------------------------*/
#include <pico/stdlib.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/
uint8_t UART_CheckResponse ( void );
void    DAC_Check          ( void );
void    MUX_Set            ( uint8_t sensor );

/* Exported defines ----------------------------------------------------------*/

#endif /* __DAC_H */

/*** end of file ***/
