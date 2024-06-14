/******************************************************************************
 * Project:         24-way Premier IR sensor jig                              *
 * Filename:        main.y                                                    *
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <hardware/pio_instructions.h>
#include <pico/stdlib.h>

/* Program Version*/
#define MAJOR_VERSION 4
#define MINOR_VERSION 1
#define BUILD_VERSION 2


/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/
uint16_t ADC_Read        ( uint8_t channel );
void     BaudRate_Update ( uint16_t baudrate );
void     Watchdog        ( void );

/* Exported defines ----------------------------------------------------------*/
// System setup
#define NOP                     pio_encode_nop ( )
#define WATCHDOG_MILLISECONDS   8000    // Maximum 8 300 ms

// For FM20 sensor:                           mV/degC , mV @ 0 degC , ADC VRef , Maximum counts of ADC
#define DEG_82  ( uint16_t ) ( ( ( ( 82.0 * ( -11.77 ) ) + 1863.9 ) / 2048.0 ) * 32767.0 )
#define DEG_90  ( uint16_t ) ( ( ( ( 90.0 * ( -11.77 ) ) + 1863.9 ) / 2048.0 ) * 32767.0 )

// ADC
#define ADC_DAC_CHANNEL             0b10001000;  // Start conversion , Channel 1 , One-shot conversion , 16bit , G1
#define ADC_TEMP_CHANNEL            0b10101000;  // Start conversion , Channel 2 , One-shot conversion , 16bit , G1
#define DAC                         1
#define TEMPERATURE                 0
// ADC calculation offsets
#define DAC_ZERO_ADC_OFFSET_0V400    9.738
#define DAC_FSD_ADC_OFFSET_2V000     9.324
#define DAC_5000_ADC_OFFSET         10.084
#define DAC_60000_ADC_OFFSET         9.277

// GPIO
#define A0_PIN              18
#define A1_PIN              19
#define A2_PIN              20
#define EN1_PIN             21
#define EN2_PIN             26
#define EN3_PIN             27
#define I2C_ADC_SCL_PIN     13
#define I2C_ADC_SDA_PIN     12
#define LED_PICO_PIN        25
#define LED_RED_PIN          7
#define LED_YELLOW_PIN       6
#define RELAY_PIN           22
#define SENSOR_PLATINUM     10
#define SPI_CS_PIN           5
#define SPI_MISO_PIN         3
#define SPI_MOSI_PIN         4
#define SPI_SCK_PIN          2
#define UART_SEN_RX_PIN     16  // UART0_TX
#define UART_SEN_TX_PIN     17  // UART0_RX
#define UART_PC_RX_PIN       9  // UART1_RX
#define UART_PC_TX_PIN       8  // UART1_TX

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

// I2C ( Master )
#define I2C_ADC             i2c0
#define I2C_BAUD_RATE       100     // kHz

// SPI ( Slave )
#define SPI_BAUD_RATE       100     // kHz
#define SPI_BUFFER_LENGTH   100
#define SPI_CS              gpio_get ( SPI_CS_PIN   )
#define SPI_MOSI            gpio_get ( SPI_MOSI_PIN )
#define SPI_SCK             gpio_get ( SPI_SCK_PIN  )
#define SPI_SLAVE           spi0

// UART
#define DATA_BITS           8
#define PARITY              UART_PARITY_NONE
#define STOP_BITS           1
#define UART_SEN            uart0
#define UART_PC             uart1
#define UART_BAUD_RATE      38400
#define UART_BUFFER_LENGTH  500
#define UART_TIMEOUT        1000

#endif /* __MAIN_H */

/* End of file ---------------------------------------------------------------*/
