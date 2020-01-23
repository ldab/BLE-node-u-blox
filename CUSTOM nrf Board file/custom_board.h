#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// In this file PIN25 is used as button SWITCH_1, if the GREEN led
// should be used it is possible to defined that one instead.
#define LEDS_NUMBER    3
//#define LEDS_NUMBER    3

#define LED_R          NRF_GPIO_PIN_MAP(0,13)   // RED -> ARDUINO_4_PIN
#define LED_B          NRF_GPIO_PIN_MAP(1,00)   // BLUE
#define LED_G          NRF_GPIO_PIN_MAP(0,25)   // GREEN
//#define LED_G          NRF_GPIO_PIN_MAP(0,25)   // GREEN

#define LEDS_ACTIVE_STATE 0

#define LEDS_LIST { LED_R, LED_B, LED_G }
//#define LEDS_LIST { LED_R, LED_B }

#define LEDS_INV_MASK  LEDS_MASK

#define BSP_LED_0      LED_R
#define BSP_LED_1      LED_B
#define BSP_LED_2      LED_G
//#define BSP_LED_2      LED_G

#define BUTTONS_NUMBER 2

#define BUTTON_1       25  // SWITCH_1
#define BUTTON_2       2   // SWITCH_2
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1, BUTTON_2 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2

#define RX_PIN_NUMBER  NRF_GPIO_PIN_MAP(0,29)
#define TX_PIN_NUMBER  NRF_GPIO_PIN_MAP(1,13)
#define CTS_PIN_NUMBER NRF_GPIO_PIN_MAP(1,12)
#define RTS_PIN_NUMBER NRF_GPIO_PIN_MAP(0,31)
#define HWFC           true

#define BSP_QSPI_SCK_PIN   19
#define BSP_QSPI_CSN_PIN   17
#define BSP_QSPI_IO0_PIN   20
#define BSP_QSPI_IO1_PIN   21
#define BSP_QSPI_IO2_PIN   22
#define BSP_QSPI_IO3_PIN   23

// Arduino board mappings
#define ARDUINO_SCL_PIN             24  // SCL signal pin
#define ARDUINO_SDA_PIN             16  // SDA signal pin

#define ARDUINO_13_PIN              NRF_GPIO_PIN_MAP(0, 7)  //SCK
#define ARDUINO_12_PIN              NRF_GPIO_PIN_MAP(0, 2)  //MISO
#define ARDUINO_11_PIN              NRF_GPIO_PIN_MAP(0, 15) //MOSI
#define ARDUINO_10_PIN              NRF_GPIO_PIN_MAP(0, 14) //CS
//#define ARDUINO_9_PIN               NRF_GPIO_PIN_MAP(0, 12) // HARD ACCESS PIN
//#define ARDUINO_8_PIN               NRF_GPIO_PIN_MAP(1, 9) // HARD ACCESS PIN

#define ARDUINO_7_PIN               NRF_GPIO_PIN_MAP(0, 10)
#define ARDUINO_6_PIN               NRF_GPIO_PIN_MAP(0, 9)
//#define ARDUINO_5_PIN               NRF_GPIO_PIN_MAP(0, 11) // HARD ACCESS PIN
#define ARDUINO_4_PIN               NRF_GPIO_PIN_MAP(0, 13) // RED LED
#define ARDUINO_3_PIN               RTS_PIN_NUMBER
#define ARDUINO_2_PIN               CTS_PIN_NUMBER
#define ARDUINO_1_PIN               TX_PIN_NUMBER
#define ARDUINO_0_PIN               RX_PIN_NUMBER // Digital pin 0

#define ARDUINO_A0_PIN              NRF_GPIO_PIN_MAP(0, 4)
#define ARDUINO_A1_PIN              NRF_GPIO_PIN_MAP(0, 30)
#define ARDUINO_A2_PIN              NRF_GPIO_PIN_MAP(0, 5)
#define ARDUINO_A3_PIN              NRF_GPIO_PIN_MAP(0, 2)
#define ARDUINO_A4_PIN              ARDUINO_SDA_PIN//NRF_GPIO_PIN_MAP(0, 28)
#define ARDUINO_A5_PIN              ARDUINO_SCL_PIN//NRF_GPIO_PIN_MAP(0, 3)

#ifdef __cplusplus
}
#endif

#endif // NINA_B3_H