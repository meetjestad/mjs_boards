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

#include "Arduino.h"
#include "wiring_private.h"
#include "stm32l0xx.h"

enum {
    PWM_INSTANCE_TIM2,
    PWM_INSTANCE_TIM3,
    PWM_INSTANCE_TIM21,
};

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[PINS_COUNT] =
{
    // The ordering of these is not as clean as you might want. The Dx
    // pins need to be a the start (so digitalRead(0) gets you D0), and
    // all Ax pins must be consecutive and not among the first
    // NUM_ANALOG_INPUTS (so analogRead(0) gets you A0).
    //
    // So the basic idea is to start with all pins on the prototype
    // header, in a bit of a mangled order because of the Dx pin
    // placement and to make sure that the analog pins have high enough
    // pin numbers. Then, the rest of the analog pins (in Ax order),
    // followed by all other pins in no particular order.

    // 0..2 - Dx pins
    [PA15] = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA15), STM32L0_GPIO_PIN_PA15_TIM2_CH1,  (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM2,  PWM_CHANNEL_1,    ADC_CHANNEL_NONE },
    [PB8]  = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB8),  STM32L0_GPIO_PIN_PB8,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    [PB9]  = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB9),  STM32L0_GPIO_PIN_PB9,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 3..4 SCL/SDA
    [PB6]  = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB6),  STM32L0_GPIO_PIN_PB6,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    [PB7]  = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB7),  STM32L0_GPIO_PIN_PB7,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 5..6 SCL1/SDA1
    [PB10] = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB10), STM32L0_GPIO_PIN_PB10_TIM2_CH3,  (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM2,  PWM_CHANNEL_3,    ADC_CHANNEL_NONE },
    [PB11] = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB11), STM32L0_GPIO_PIN_PB11_TIM2_CH4,  (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM2,  PWM_CHANNEL_4,    ADC_CHANNEL_NONE },

    // 7..10 SPI1
    [PB15] = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB15), STM32L0_GPIO_PIN_PB15,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    [PB14] = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB14), STM32L0_GPIO_PIN_PB14_TIM21_CH2, (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM21, PWM_CHANNEL_2,    ADC_CHANNEL_NONE },
    [PB13] = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB13), STM32L0_GPIO_PIN_PB13_TIM21_CH1, (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM21, PWM_CHANNEL_1,    ADC_CHANNEL_NONE },
    [PB12] = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB12), STM32L0_GPIO_PIN_PB12,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 11 - TX4
    [PC10] = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC10), STM32L0_GPIO_PIN_PC10,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 12..14 - A0..A2 (A0 must be > 9 to support up to A9)
    [PA4]  = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA4),  STM32L0_GPIO_PIN_PA4,            (PIN_ATTR_EXTI|PIN_ATTR_DAC1),                 PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_4,   },
    [PA5]  = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA5),  STM32L0_GPIO_PIN_PA5,            (PIN_ATTR_EXTI|PIN_ATTR_DAC2),                 PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_5    },
    [PB0]  = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB0),  STM32L0_GPIO_PIN_PB0,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_8    },
    // 15 - A3 / RX4
    [PA1]  = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA1),  STM32L0_GPIO_PIN_PA1_TIM2_CH2,   (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM2,  PWM_CHANNEL_2,    ADC_CHANNEL_1    },
    // 16 - A4 / Lux analog
    [PC3]  = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC3),  STM32L0_GPIO_PIN_PC3,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_13   },
    // 17..18 - A5..A6 / Grove Multi
    [PC0]  = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC0),  STM32L0_GPIO_PIN_PC0,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_10   },
    [PC1]  = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC1),  STM32L0_GPIO_PIN_PC1,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_11   },

    // 19 - A7 / Battery divider
    [PC4]  = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC4),  STM32L0_GPIO_PIN_PC4,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_14   },

    // 20 - A8 / Solar divider
    [PC5]  = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC5),  STM32L0_GPIO_PIN_PC5,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_15   },

    // 21 - A9 / USB VBUS divider
    [PA7]  = { GPIOA,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA7), STM32L0_GPIO_PIN_PA7,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_7    },

    // 22..24 - RGB LED
    [PC6]  = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC6),  STM32L0_GPIO_PIN_PC6_TIM3_CH1,   (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM3,  PWM_CHANNEL_1, ADC_CHANNEL_NONE    },
    [PC7]  = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC7),  STM32L0_GPIO_PIN_PC7_TIM3_CH2,   (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM3,  PWM_CHANNEL_2, ADC_CHANNEL_NONE    },
    [PC8]  = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC8),  STM32L0_GPIO_PIN_PC8_TIM3_CH3,   (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM3,  PWM_CHANNEL_3, ADC_CHANNEL_NONE    },

    // 25..27 PSU Enable (3V_SENS, 3V_GPS, 5V)
    [PC11] = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC11), STM32L0_GPIO_PIN_PC11,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    [PA8]  = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA8),  STM32L0_GPIO_PIN_PA8,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    [PC12] = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC12), STM32L0_GPIO_PIN_PC12,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 28 - Charger status
    [PC9]  = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC9),  STM32L0_GPIO_PIN_PC9,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 29 - Lux range selection
    [PA6]  = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA6),  STM32L0_GPIO_PIN_PA6,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_6    },

    // 30 - User Button
    [PD2]  = { GPIOD, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PD2),  STM32L0_GPIO_PIN_PD2,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 31..34 LoRA I/O (NSS, RST, DIO1, BUSY)
    [PC2]  = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC2),  STM32L0_GPIO_PIN_PC2,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_12   },
    [PC13] = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC13), STM32L0_GPIO_PIN_PC13,           (PIN_ATTR_EXTI|PIN_ATTR_WKUP2),                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    [PB1]  = { GPIOB,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB1), STM32L0_GPIO_PIN_PB1,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_9    },
    [PB2]  = { GPIOB,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB2), STM32L0_GPIO_PIN_PB2,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 35..37 LoRa SPI (MOSI, MISO, SCK)
    [PB5]  = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB5),  STM32L0_GPIO_PIN_PB5,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    [PB4]  = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB4),  STM32L0_GPIO_PIN_PB4,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    [PB3]  = { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB3),  STM32L0_GPIO_PIN_PB3,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 38..39 FTDI (UART TX, UART RX)
    [PA9]  = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA9),  STM32L0_GPIO_PIN_PA9,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_9    },
    [PA10] = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA10), STM32L0_GPIO_PIN_PA10,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 40..43 - GPS (UART2 TX, UART2 RX, 2xTIMEPULSE)
    [PA2]  = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA2),  STM32L0_GPIO_PIN_PA2,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_2    },
    [PA3]  = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA3),  STM32L0_GPIO_PIN_PA3,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_3    },
    [PA0]  = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA0),  STM32L0_GPIO_PIN_PA0,            (PIN_ATTR_EXTI|PIN_ATTR_WKUP1),                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_0    },
    [PH0]  = { GPIOH, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PH0),  STM32L0_GPIO_PIN_PH0,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 44..45 - SWD (SWDIO, SWDCLK)
    [PA13] = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA13), STM32L0_GPIO_PIN_PA13,           (PIN_ATTR_EXTI|PIN_ATTR_SWD),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    [PA14] = { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA14), STM32L0_GPIO_PIN_PA14,           (PIN_ATTR_EXTI|PIN_ATTR_SWD),                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 46 - SPI Flash CS (optional chip)
    [PH1]  = { GPIOH, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PH1),  STM32L0_GPIO_PIN_PH1,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // USB (DM, DP)
    // [PA11] = { GPIOA,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA11), STM32L0_GPIO_PIN_PA11,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    // [PA12] = { GPIOA,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA12), STM32L0_GPIO_PIN_PA12,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // RTC Crystal
    // [PC14] = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC14), STM32L0_GPIO_PIN_PC14,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    // [PC15] = { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC15), STM32L0_GPIO_PIN_PC15,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
};

extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT] = {
    [PWM_INSTANCE_TIM2] = STM32L0_TIMER_INSTANCE_TIM2,
    [PWM_INSTANCE_TIM3] = STM32L0_TIMER_INSTANCE_TIM3,
    [PWM_INSTANCE_TIM21] = STM32L0_TIMER_INSTANCE_TIM21,
};

static uint8_t stm32l0_usart1_rx_fifo[32];

extern const stm32l0_uart_params_t g_Serial1Params = {
    STM32L0_UART_INSTANCE_USART1,
    STM32L0_UART_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_DMA1_CH3_USART1_RX,
    STM32L0_DMA_CHANNEL_DMA1_CH2_USART1_TX,
    &stm32l0_usart1_rx_fifo[0],
    sizeof(stm32l0_usart1_rx_fifo),
    {
        STM32L0_GPIO_PIN_PA10_USART1_RX,
        STM32L0_GPIO_PIN_PA9_USART1_TX,
        STM32L0_GPIO_PIN_NONE,
        STM32L0_GPIO_PIN_NONE,
    },
};


static uint8_t stm32l0_usart2_rx_fifo[32];

extern const stm32l0_uart_params_t g_Serial2Params = {
    STM32L0_UART_INSTANCE_USART2,
    STM32L0_UART_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_DMA1_CH6_USART2_RX,
    STM32L0_DMA_CHANNEL_DMA1_CH4_USART2_TX,
    &stm32l0_usart2_rx_fifo[0],
    sizeof(stm32l0_usart2_rx_fifo),
    {
        STM32L0_GPIO_PIN_PA3_USART2_RX,
        STM32L0_GPIO_PIN_PA2_USART2_TX,
        STM32L0_GPIO_PIN_NONE,
        STM32L0_GPIO_PIN_NONE,
    },
};


static uint8_t stm32l0_usart4_rx_fifo[32];

extern const stm32l0_uart_params_t g_Serial3Params = {
    STM32L0_UART_INSTANCE_USART4,
    STM32L0_UART_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_NONE,
    STM32L0_DMA_CHANNEL_NONE,
    &stm32l0_usart4_rx_fifo[0],
    sizeof(stm32l0_usart4_rx_fifo),
    {
        STM32L0_GPIO_PIN_PA1_USART4_RX,
        STM32L0_GPIO_PIN_PC10_USART4_TX,
        STM32L0_GPIO_PIN_NONE,
        STM32L0_GPIO_PIN_NONE,
    },
};


extern const stm32l0_spi_params_t g_SPI1Params = {
    STM32L0_SPI_INSTANCE_SPI2,
    STM32L0_SPI_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_NONE,
    STM32L0_DMA_CHANNEL_NONE,
    {
        STM32L0_GPIO_PIN_PB15_SPI2_MOSI,
        STM32L0_GPIO_PIN_PB14_SPI2_MISO,
        STM32L0_GPIO_PIN_PB13_SPI2_SCK,
        STM32L0_GPIO_PIN_NONE,
    },
};


extern const stm32l0_spi_params_t g_SPIParams = {
    STM32L0_SPI_INSTANCE_SPI1,
    STM32L0_SPI_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_NONE,
    STM32L0_DMA_CHANNEL_NONE,
    {
        STM32L0_GPIO_PIN_PB5_SPI1_MOSI,
        STM32L0_GPIO_PIN_PB4_SPI1_MISO,
        STM32L0_GPIO_PIN_PB3_SPI1_SCK,
        STM32L0_GPIO_PIN_NONE,
    },
};


extern const stm32l0_i2c_params_t g_WireParams = {
    STM32L0_I2C_INSTANCE_I2C1,
    STM32L0_I2C_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_DMA1_CH7_I2C1_RX,
    STM32L0_DMA_CHANNEL_NONE,
    {
        STM32L0_GPIO_PIN_PB6_I2C1_SCL,
        STM32L0_GPIO_PIN_PB7_I2C1_SDA,
    },
};


extern const stm32l0_i2c_params_t g_Wire1Params = {
    STM32L0_I2C_INSTANCE_I2C2,
    STM32L0_I2C_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_DMA1_CH5_I2C2_RX,
    STM32L0_DMA_CHANNEL_NONE,
    {
        STM32L0_GPIO_PIN_PB10_I2C2_SCL,
        STM32L0_GPIO_PIN_PB11_I2C2_SDA,
    },
};

extern const stm32l0_i2c_params_t g_Wire2Params = {
    STM32L0_I2C_INSTANCE_I2C3,
    STM32L0_I2C_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_NONE,
    STM32L0_DMA_CHANNEL_NONE,
    {
        STM32L0_GPIO_PIN_PC0_I2C3_SCL,
        STM32L0_GPIO_PIN_PC1_I2C3_SDA,
    },
};

#if defined(SERIAL_IS_CONFIGURABLE)
// When the sketch does not set this, let Serial point to USB
Stream* ConfigurableSerial = &SerialUSB;
#endif

void initVariant()
{
    // The board uses the ABS06-127-32.768KHZ-7 crystal. According to ST
    // AN2867 ("Oscillator design guide for STM8AF/AL/S, STM32 MCUs and
    // MPUs"), this crystal has a gmcrit around 0.7477 μA/V. This means
    // configuring medium low drive strength, which works for crystals
    // up to 0.75 μA/V gmcrit.
    RCC->CSR |= (RCC_CSR_LSEDRV_0 << RCC_CSR_LSEDRV_Pos); // DRV_0 == 0x1 == MEDIUM LOW

    //CMWX1ZZABZ_Initialize(STM32L0_GPIO_PIN_PH1, STM32L0_GPIO_PIN_NONE);
}
