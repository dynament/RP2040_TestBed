/*******************************************************************
 *  Author:	   Frank Kups                                      *
 *  Company:       Status Scientific Controls Ltd             	   *
 *  Project :	   24-way Jig for IR Sensor    			   *
 *  Filename:	   Def.h                                           *
 *  Date:	   18/07/2012                                      *
 *  File Version:  1.00                                            *
 *  Other Files Required: p24F32KA01.gld, libpPIC24Fxxx-coff.a     *
 *  Tools Used:    MPLAB GL  -> 8.84                               *
 *                 Compiler  -> 3.31                               *
 *                                                                 *
 *******************************************************************
*/

#ifndef _DEF
#define _DEF

// #include <p24fxxxx.h>

#define P2P_BUFFER_MASTER_SIZE 240
#define P2P_BUFFER_SLAVE_SIZE 240

#define EN1_8           PORTAbits.RA0
#define EN9_16          PORTAbits.RA1
#define EN17_24         PORTAbits.RA2
#define EN1_8_TRIS      TRISAbits.TRISA0
#define EN9_16_TRIS     TRISAbits.TRISA1
#define EN17_24_TRIS    TRISAbits.TRISA2

#define CLOCK_OUT       LATAbits.LATA3
#define CLOCK_OUT_TRIS  TRISAbits.TRISA3

#define MUX_A0          PORTBbits.RB13
#define MUX_A1          PORTBbits.RB14
#define MUX_A2          PORTBbits.RB15
#define MUX_A0_TRIS     TRISBbits.TRISB13
#define MUX_A1_TRIS     TRISBbits.TRISB14
#define MUX_A2_TRIS     TRISBbits.TRISB15

#define COMMS_NONE 0
#define COMMS_READ 1
#define COMMS_WRITE 2

#endif


