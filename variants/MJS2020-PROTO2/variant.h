/*
 * Copyright (c) 2017-2018 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#pragma once

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define STM32L0_CONFIG_LSECLK             32768
#define STM32L0_CONFIG_HSECLK             0
#define STM32L0_CONFIG_SYSOPT             0

#define STM32L0_CONFIG_PIN_VBUS           STM32L0_GPIO_PIN_NONE

#define STM32L0_CONFIG_PIN_VBAT           STM32L0_GPIO_PIN_PC4
#define STM32L0_CONFIG_CHANNEL_VBAT       ADC_CHANNEL_14
#define STM32L0_CONFIG_VBAT_PERIOD        40
#define STM32L0_CONFIG_VBAT_SCALE         ((float)1.27)



#define USBCON

/** Master clock frequency */
#define VARIANT_MCK                       F_CPU

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
#include "USBAPI.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

enum {
    PB6 = 0,
    PB7 = 1,
    PB13 = 2,
    PB12 = 3,
    PB10 = 4,
    PB11 = 5,
    PB15 = 6,
    PB14 = 7,
    PA0 = 8,
    PB8 = 9,
    PA15 = 10,
    PB5 = 11,
    PB4 = 12,
    PB3 = 13,
    // These are mapped twice
    PC4 = 16,
    PC5 = 17,
    /* 18 - 21 are unused? */
    PA11 = 22,
    PA12 = 23,
    PA7 = 24,
    PB1 = 25,
    PB2 = 26,
    PA8 = 27,
    PC10 = 28,
    PC11 = 29,
    PA2 = 30,
    PA3 = 31,
    PD2 = 32,
    PC7 = 33,
    PC8 = 34,
    PC6 = 35,
    PA9 = 36,
    PA10 = 37,
    PC9 = 38,
};

// Number of pins defined in PinDescription array
#define PINS_COUNT           (39u)
#define NUM_DIGITAL_PINS     (31u)
#define NUM_ANALOG_INPUTS    (6u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs
#define PIN_LED_RED          (PC7)
#define PIN_LED_GREEN        (PC8)
#define PIN_LED_BLUE         (PC6)
#define LED_BUILTIN          PIN_LED_RED

// Regulator enable pins
#define PIN_ENABLE_5V        (PC10)
#define PIN_ENABLE_3V_SENS   (PC11)
#define PIN_ENABLE_3V_GPS    (PA8)

// LoRa Radio pins
#define PIN_LORA_SS          (PA15)
#define PIN_LORA_RST         (PA7)
#define PIN_LORA_DIO1        (PB1)
#define PIN_LORA_BUSY        (PB2)

#define PIN_BATTERY          (PC4)
#define PIN_SOLAR            (PC5)
#define PIN_BOARD_ID         (PB0)

/*
 * Analog pins
 */
#define PIN_A0               (16ul)
#define PIN_A1               (17ul)
#define PIN_A2               (18ul)
#define PIN_A3               (19ul)
#define PIN_A4               (20ul)
#define PIN_A5               (21ul)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;

#define ADC_RESOLUTION          12

/*
 * Other pins
 */
#define PIN_BUTTON           (PD2)
static const uint8_t BUTTON = PIN_BUTTON;

/*
 * Serial interfaces
 */

#define SERIAL_INTERFACES_COUNT 3

#define PIN_SERIAL1_RX        (PA10)
#define PIN_SERIAL1_TX        (PA9)

#define PIN_SERIAL2_RX        (PA3)
#define PIN_SERIAL2_TX        (PA2)


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (PB4)
#define PIN_SPI_MOSI         (PB5)
#define PIN_SPI_SCK          (PB3)

static const uint8_t SS   = 10;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         (PB7)
#define PIN_WIRE_SCL         (PB6)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA        (PB11)
#define PIN_WIRE1_SCL        (PB10)

#define PIN_USB_DM           (PA11)
#define PIN_USB_DP           (PA12)


#define PWM_INSTANCE_COUNT    0

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern CDC  SerialUSB;
extern Uart Serial1;
extern Uart Serial2;
#if defined(SERIAL_IS_CONFIGURABLE)
    // This allows reconfiguring Serial at runtime, to allow e.g.
    // switching between SerialUSB and Serial1 based on whether USB is
    // connected or not. To ensure that sketches and libraries do not
    // need to know the difference, this defines a pointer under a
    // different name, which is then dereferenced by the Serial macro
    // below (so normal member access with . still works).
    //
    // It would seem nice to make Serial reconfigurable by default, but
    // since the common superclass for SerialUSB and Serial1 is Stream,
    // which does not have a .begin() method, this would break sketches
    // that call Serial.begin(). So instead, reconfigurable serial is a
    // compile-time option, and sketches that need it must call
    // Serial1.begin() and/or SerialUSB.begin() as appropriate, and can
    // then assign ReconfigurableSerial as needed.
    // TODO: ConfigurableSerial? Other name?
    extern Stream* ConfigurableSerial;
#endif
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2
#define SERIAL_PORT_HARDWARE_OPEN2  Serial1

// Alias Serial to SerialUSB by default, but allow reconfiguring it
#if defined(SERIAL_IS_CONFIGURABLE)
    #define Serial                      (*ConfigurableSerial)
#elif defined(SERIAL_IS_SERIAL1)
    #define Serial                      Serial1
#else
    #define Serial                      SerialUSB
#endif
