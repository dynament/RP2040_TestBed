/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 Dynament Infrared Gas Sensors Ltd.
  * Copyright (c) 2022 Status Scientific Controls Ltd.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "hardware/pio_instructions.h"
#include "hardware/spi.h"
#include "hardware/watchdog.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "comms.h"
#include "p2p.h"

#define NOP     pio_encode_nop ( )
#define DEG82   ( uint16_t ) ( ( 82.0 * ( -11.77 ) ) + 1863.9 )
#define DEG90   ( uint16_t ) ( ( 90.0 * ( -11.77 ) ) + 1863.9 )

#define ADC_DAC_CHANNEL         /*0b00010000*/0x18
#define ADC_TEMP_CHANNEL        /*0b00010011*/0x1A
#define DAC                     1
#define TEMPERATURE             0
#define WATCHDOG_MILLISECONDS   8000    // Maximum 8 300 ms

// GPIO
#define A0_PIN          18
#define A1_PIN          19
#define A2_PIN          20
#define EN1_PIN         21
#define EN2_PIN         26
#define EN3_PIN         27
#define LED_PICO_PIN    25
#define LED_RED_PIN      7
#define LED_YELLOW_PIN   6
#define RELAY_PIN       22
#define SPI_CS          13
#define SPI_MISO        12
#define SPI_MOSI        11
#define SPI_SCK         10
#define UART_SEN_RX_PIN 16  // UART0_TX
#define UART_SEN_TX_PIN 17  // UART0_RX
#define UART_PC_RX_PIN   9  // UART1_RX
#define UART_PC_TX_PIN   8  // UART1_TX

#define A0_HIGH         gpio_put ( A0_PIN         , 1 )
#define A0_LOW          gpio_put ( A0_PIN         , 0 )
#define A1_HIGH         gpio_put ( A1_PIN         , 1 )
#define A1_LOW          gpio_put ( A1_PIN         , 0 )
#define A2_HIGH         gpio_put ( A2_PIN         , 1 )
#define A2_LOW          gpio_put ( A2_PIN         , 0 )
#define EN1_HIGH        gpio_put ( EN1_PIN        , 1 )
#define EN1_LOW         gpio_put ( EN1_PIN        , 0 )
#define EN2_HIGH        gpio_put ( EN2_PIN        , 1 )
#define EN2_LOW         gpio_put ( EN2_PIN        , 0 )
#define EN3_HIGH        gpio_put ( EN3_PIN        , 1 )
#define EN3_LOW         gpio_put ( EN3_PIN        , 0 )
#define LED_PICO_OFF    gpio_put ( LED_PICO_PIN   , 0 )
#define LED_PICO_ON     gpio_put ( LED_PICO_PIN   , 1 )
#define LED_RED_OFF     gpio_put ( LED_RED_PIN    , 0 )
#define LED_RED_ON      gpio_put ( LED_RED_PIN    , 1 )
#define LED_YELLOW_OFF  gpio_put ( LED_YELLOW_PIN , 0 )
#define LED_YELLOW_ON   gpio_put ( LED_YELLOW_PIN , 1 )
#define RELAY_OFF       gpio_put ( RELAY_PIN      , 0 )
#define RELAY_ON        gpio_put ( RELAY_PIN      , 1 )

// SPI
#define SPI_BAUD_RATE       1000000
#define SPI_BUFFER_LENGTH   10
#define SPI_CS_HIGH         gpio_put ( SPI_CS , 1 );
#define SPI_CS_LOW          gpio_put ( SPI_CS , 0 );
#define SPI_ID              spi1

// UART
#define DATA_BITS           8
#define PARITY              UART_PARITY_NONE
#define STOP_BITS           1
#define UART_SEN            uart0
#define UART_PC             uart1
#define UART_BAUD_RATE      38400
#define UART_BUFFER_LENGTH  500
#define UART_TIMEOUT        1000

void Set_MUX        ( uint8_t sensor );
void UpdateBaudRate ( uint16_t baudrate );
void watchdog       ( void );

#endif /* __MAIN_H */
