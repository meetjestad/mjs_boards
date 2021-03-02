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

#define STM32L0_CONFIG_PIN_VBUS           STM32L0_GPIO_PIN_PA7
#define STM32L0_CONFIG_PIN_VBUS_HAS_DIVIDER

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

// Map chip pin names to the corresponding Arduino logical pin number
enum {
    PA15 = 0,
    PB8  = 1,
    PB9  = 2,
    PB6  = 3,
    PB7  = 4,
    PB10 = 5,
    PB11 = 6,
    PB15 = 7,
    PB14 = 8,
    PB13 = 9,
    PB12 = 10,
    PC10 = 11,
    PA4  = 12,
    PA5  = 13,
    PB0  = 14,
    PA1  = 15,
    PC3  = 16,
    PC0  = 17,
    PC1  = 18,
    PC4  = 19,
    PC5  = 20,
    PA7  = 21,
    PC6  = 22,
    PC7  = 23,
    PC8  = 24,
    PC11 = 25,
    PA8  = 26,
    PC12 = 27,
    PC9  = 28,
    PA6  = 29,
    PD2  = 30,
    PC2  = 31,
    PC13 = 32,
    PB1  = 33,
    PB2  = 34,
    PB5  = 35,
    PB4  = 36,
    PB3  = 37,
    PA9  = 38,
    PA10 = 39,
    PA2  = 40,
    PA3  = 41,
    PA0  = 42,
    PH0  = 43,
    PA13 = 44,
    PA14 = 45,
    PH1  = 46,
    PINS_COUNT,
};

// Define friendly names for all pins
enum {
    // Digital pins
    PIN_D0                = PA15,
    PIN_D1                = PB8,
    PIN_D2                = PB9,

    // Analog pins
    // There are more analog inputs, these are just the ones where
    // analog input actually makes sense.
    PIN_A0                = PA4,
    PIN_DAC0              = PA4,
    PIN_A1                = PA5,
    PIN_DAC1              = PA5,
    PIN_A2                = PB0,
    PIN_A3                = PA1,
    PIN_A4                = PC3,
    PIN_A5                = PC0,
    PIN_A6                = PC1,
    PIN_A7                = PC4,
    PIN_A8                = PC5,
    PIN_A9                = PA0,

    // LEDs
    PIN_LED_RED           = PC8,
    PIN_LED_GREEN         = PC7,
    PIN_LED_BLUE          = PC6,

    // Regulator enable pins
    PIN_ENABLE_3V_SENS    = PC11,
    PIN_ENABLE_3V_GPS     = PA8,
    PIN_ENABLE_5V         = PC12,

    // Voltage & charging pins
    PIN_BATTERY           = PC4,
    PIN_SOLAR             = PC5,
    PIN_VBUS              = PA0,
    PIN_CHARGE_STATUS     = PC9,

    // Lux measurement
    PIN_LUX_RANGE         = PA6,
    PIN_LUX_ADC           = PC3,

    // Button
    PIN_BUTTON            = PD2,

    // LoRa Radio pins
    PIN_LORA_SS           = PC2,
    PIN_LORA_RST          = PC13,
    PIN_LORA_DIO1         = PB1,
    PIN_LORA_BUSY         = PB2,

    // Other
    PIN_GPS_TIMEPULSE     = PA0,

    // SPI, I²C and serial
    PIN_MISO              = PB4,
    PIN_MOSI              = PB5,
    PIN_SCK               = PB3,

    PIN_MISO1             = PB14,
    PIN_MOSI1             = PB15,
    PIN_SCK1              = PB13,
    PIN_NSS1              = PB12,

    PIN_SCL               = PB6,
    PIN_SDA               = PB7,

    PIN_SCL1              = PB10,
    PIN_SDA1              = PB11,

    PIN_SCL2              = PC0,
    PIN_SDA2              = PC1,

    PIN_RX1               = PA10,
    PIN_TX1               = PA9,

    PIN_RX2               = PA3,
    PIN_TX2               = PA2,

    PIN_RX3               = PA1,
    PIN_TX3               = PC10,

    // These are mislabeled on the silkscreen because they are USART4,
    // but Serial3
    PIN_RX4               = PIN_RX3,
    PIN_TX4               = PIN_TX3,

    // SWD
    PIN_SWDIO             = PA13,
    PIN_SWCLK             = PA14,
};

// Define even shorter names for some pins (these are names commonly
// defined by other Arduino cores as well).
enum {
    A0  = PIN_A0,
    A1  = PIN_A1,
    A2  = PIN_A2,
    A3  = PIN_A3,
    A4  = PIN_A4,
    A5  = PIN_A5,
    A6  = PIN_A6,

    DAC0 = PIN_DAC0,
    DAC1 = PIN_DAC1,

    MOSI = PIN_MOSI,
    MISO = PIN_MISO,
    SCK  = PIN_SCK,

    SDA = PIN_SDA,
    SCL = PIN_SCL,
};

// These are macros, since portable sketches tend to #ifdef for these
#define LED_BUILTIN   PIN_LED_RED
#define LED_RED       PIN_LED_RED
#define LED_BLUE      PIN_LED_BLUE
#define LED_GREEN     PIN_LED_GREEN

#define NUM_DIGITAL_PINS     (PINS_COUNT)
#define NUM_ANALOG_INPUTS    (9u)
#define NUM_ANALOG_OUTPUTS   (2u)

#define ADC_RESOLUTION          12
#define DAC_RESOLUTION          12
// This includes the virtual USB Serial
#define SERIAL_INTERFACES_COUNT 4
#define SPI_INTERFACES_COUNT    2
#define WIRE_INTERFACES_COUNT   3
#define PWM_INSTANCE_COUNT      3

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
extern Uart Serial3;
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
    // then assign ConfigurableSerial as needed.
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
