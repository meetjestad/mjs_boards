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

#if !defined(_STM32L0_GPIO_H)
#define _STM32L0_GPIO_H

#include "armv6m.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define GPIO_MODE_MASK                  0x0003
#define GPIO_MODE_SHIFT                 0
#define GPIO_MODE_INPUT                 0x0000
#define GPIO_MODE_OUTPUT                0x0001
#define GPIO_MODE_ALTERNATE             0x0002
#define GPIO_MODE_ANALOG                0x0003
#define GPIO_OTYPE_MASK                 0x0004
#define GPIO_OTYPE_SHIFT                2
#define GPIO_OTYPE_PUSHPULL             0x0000
#define GPIO_OTYPE_OPENDRAIN            0x0004
#define GPIO_OSPEED_MASK                0x0018
#define GPIO_OSPEED_SHIFT               3
#define GPIO_OSPEED_LOW                 0x0000
#define GPIO_OSPEED_MEDIUM              0x0008
#define GPIO_OSPEED_HIGH                0x0010
#define GPIO_OSPEED_VERY_HIGH           0x0018
#define GPIO_PUPD_MASK                  0x0060
#define GPIO_PUPD_SHIFT                 5
#define GPIO_PUPD_NONE                  0x0000
#define GPIO_PUPD_PULLUP                0x0020
#define GPIO_PUPD_PULLDOWN              0x0040
#define GPIO_PARK_SHIFT                 7
#define GPIO_PARK_MASK                  0x0180
#define GPIO_PARK_NONE                  0x0000
#define GPIO_PARK_PULLUP                0x0080
#define GPIO_PARK_PULLDOWN              0x0100
#define GPIO_PARK_HIZ                   0x0180

#if defined(STM32L082xx)

#define GPIO_PIN_PA0                    0x00
#define GPIO_PIN_PA1                    0x01
#define GPIO_PIN_PA2                    0x02
#define GPIO_PIN_PA3                    0x03
#define GPIO_PIN_PA4                    0x04
#define GPIO_PIN_PA5                    0x05
#define GPIO_PIN_PA6                    0x06
#define GPIO_PIN_PA7                    0x07
#define GPIO_PIN_PA8                    0x08
#define GPIO_PIN_PA9                    0x09
#define GPIO_PIN_PA10                   0x0a
#define GPIO_PIN_PA11                   0x0b
#define GPIO_PIN_PA12                   0x0c
#define GPIO_PIN_PA13                   0x0d
#define GPIO_PIN_PA14                   0x0e
#define GPIO_PIN_PA15                   0x0f
#define GPIO_PIN_PB0                    0x10
#define GPIO_PIN_PB1                    0x11
#define GPIO_PIN_PB2                    0x12
#define GPIO_PIN_PB3                    0x13
#define GPIO_PIN_PB4                    0x14
#define GPIO_PIN_PB5                    0x15
#define GPIO_PIN_PB6                    0x16
#define GPIO_PIN_PB7                    0x17
#define GPIO_PIN_PB8                    0x18
#define GPIO_PIN_PB9                    0x19
#define GPIO_PIN_PB10                   0x1a
#define GPIO_PIN_PB11                   0x1b
#define GPIO_PIN_PB12                   0x1c
#define GPIO_PIN_PB13                   0x1d
#define GPIO_PIN_PB14                   0x1e
#define GPIO_PIN_PB15                   0x1f
#define GPIO_PIN_PC0                    0x20
#define GPIO_PIN_PC1                    0x21
#define GPIO_PIN_PC2                    0x22
#define GPIO_PIN_PC13                   0x2d
#define GPIO_PIN_PC14                   0x2e
#define GPIO_PIN_PC15                   0x2f
#define GPIO_PIN_PH0                    0x70
#define GPIO_PIN_PH1                    0x71
#define GPIO_PIN_NONE                   0xff

#endif

#define GPIO_PIN_INDEX_MASK             0x000f
#define GPIO_PIN_INDEX_SHIFT            0
#define GPIO_PIN_GROUP_MASK             0x00f0
#define GPIO_PIN_GROUP_SHIFT            4
#define GPIO_PIN_AFSEL_MASK             0x0700
#define GPIO_PIN_AFSEL_SHIFT            8

#define GPIO_PIN_IO_MASK                0x00ff
#define GPIO_PIN_IO_SHIFT               0

#if defined(STM32L082xx)

#define GPIO_PIN_PA8_MCO                0x0008
#define GPIO_PIN_PA9_MCO                0x0009
#define GPIO_PIN_PA13_SWDIO             0x000d
#define GPIO_PIN_PA14_SWCLK             0x000e
#define GPIO_PIN_PB13_MCO               0x021d
#define GPIO_PIN_PB14_RTC_OUT           0x021e
#define GPIO_PIN_PB15_RTC_REFIN         0x021f

#define GPIO_PIN_PA8_USART1_CK          0x0408
#define GPIO_PIN_PA9_USART1_TX          0x0409
#define GPIO_PIN_PA10_USART1_RX         0x040a
#define GPIO_PIN_PA11_USART1_CTS        0x040b
#define GPIO_PIN_PA12_USART1_RTS_DE     0x040c
#define GPIO_PIN_PB3_USART1_RTS_DE      0x0513
#define GPIO_PIN_PB4_USART1_CTS         0x0514
#define GPIO_PIN_PB5_USART1_CK          0x0515
#define GPIO_PIN_PB6_USART1_TX          0x0016
#define GPIO_PIN_PB7_USART1_RX          0x0017

#define GPIO_PIN_PA0_USART2_CTS         0x0400
#define GPIO_PIN_PA1_USART2_RTS_DE      0x0401
#define GPIO_PIN_PA2_USART2_TX          0x0402
#define GPIO_PIN_PA3_USART2_RX          0x0403
#define GPIO_PIN_PA4_USART2_CK          0x0404
#define GPIO_PIN_PA14_USART2_TX         0x040e
#define GPIO_PIN_PA15_USART2_RX         0x040f

#define GPIO_PIN_PA0_USART4_TX          0x0600
#define GPIO_PIN_PA1_USART4_RX          0x0601
#define GPIO_PIN_PA15_USART4_RTS_DE     0x060f
#define GPIO_PIN_PB7_USART4_CTS         0x0617

#define GPIO_PIN_PB3_USART5_TX          0x0613
#define GPIO_PIN_PB4_USART5_RX          0x0614
#define GPIO_PIN_PB5_USART5_CK          0x0615
#define GPIO_PIN_PB5_USART5_RTS_DE      0x0615

#define GPIO_PIN_PA2_LPUART1_TX         0x0602
#define GPIO_PIN_PA3_LPUART1_RX         0x0603
#define GPIO_PIN_PA6_LPUART1_CTS        0x0406
#define GPIO_PIN_PA13_LPUART1_RX        0x060d
#define GPIO_PIN_PA14_LPUART1_TX        0x060e
#define GPIO_PIN_PB1_LPUART1_RTS_DE     0x0411
#define GPIO_PIN_PB10_LPUART1_TX        0x041a
#define GPIO_PIN_PB11_LPUART1_RX        0x041b
#define GPIO_PIN_PB12_LPUART1_RTS_DE    0x021c
#define GPIO_PIN_PB13_LPUART1_CTS       0x041d
#define GPIO_PIN_PB14_LPUART1_RTS_DE    0x041e
#define GPIO_PIN_PC0_LPUART1_RX         0x0620
#define GPIO_PIN_PC1_LPUART1_TX         0x0621

#define GPIO_PIN_PA9_I2C1_SCL           0x0609
#define GPIO_PIN_PA10_I2C1_SDA          0x060a
#define GPIO_PIN_PB5_I2C1_SMBA          0x0315
#define GPIO_PIN_PB6_I2C1_SCL           0x0116
#define GPIO_PIN_PB7_I2C1_SDA           0x0117
#define GPIO_PIN_PB8_I2C1_SCL           0x0418
#define GPIO_PIN_PB9_I2C1_SDA           0x0419

#define GPIO_PIN_PB10_I2C2_SCL          0x061a
#define GPIO_PIN_PB11_I2C2_SDA          0x061b
#define GPIO_PIN_PB12_I2C2_SMBA         0x051c
#define GPIO_PIN_PB13_I2C2_SCL          0x051d
#define GPIO_PIN_PB14_I2C2_SDA          0x051e

#define GPIO_PIN_PA8_I2C3_SCL           0x0708
#define GPIO_PIN_PA9_I2C3_SMBA          0x0709
#define GPIO_PIN_PB2_I2C3_SMBA          0x0712
#define GPIO_PIN_PB4_I2C3_SDA           0x0714
#define GPIO_PIN_PC0_I2C3_SCL           0x0720
#define GPIO_PIN_PC1_I2C3_SDA           0x0721

#define GPIO_PIN_PA4_SPI1_NSS           0x0004
#define GPIO_PIN_PA5_SPI1_SCK           0x0005
#define GPIO_PIN_PA6_SPI1_MISO          0x0006
#define GPIO_PIN_PA7_SPI1_MOSI          0x0007
#define GPIO_PIN_PA11_SPI1_MISO         0x000b
#define GPIO_PIN_PA12_SPI1_MOSI         0x000c
#define GPIO_PIN_PA15_SPI1_NSS          0x000f
#define GPIO_PIN_PB3_SPI1_SCK           0x0013
#define GPIO_PIN_PB4_SPI1_MISO          0x0014
#define GPIO_PIN_PB5_SPI1_MOSI          0x0015

#define GPIO_PIN_PB9_SPI2_NSS           0x0519
#define GPIO_PIN_PB10_SPI2_SCK          0x051a
#define GPIO_PIN_PB12_SPI2_NSS          0x001c
#define GPIO_PIN_PB13_SPI2_SCK          0x001d
#define GPIO_PIN_PB14_SPI2_MISO         0x001e
#define GPIO_PIN_PB15_SPI2_MOSI         0x001f
#define GPIO_PIN_PC2_SPI2_MISO          0x0222

#define GPIO_PIN_PB9_I2S2_WS            0x0519
#define GPIO_PIN_PB12_I2S2_WS           0x001c
#define GPIO_PIN_PB13_I2S2_CK           0x001d
#define GPIO_PIN_PB14_I2S2_MCK          0x001e
#define GPIO_PIN_PB15_I2S2_SD           0x001f
#define GPIO_PIN_PC2_I2S2_MCK           0x0222

#define GPIO_PIN_PA8_USB_CRS_STBC       0x0208
#define GPIO_PIN_PA13_USB_OE            0x020d
#define GPIO_PIN_PH0_USB_CRS_SYNC       0x0070

#define GPIO_PIN_PA0_TIM2_CH1           0x0200
#define GPIO_PIN_PA0_TIM2_ETR           0x0500
#define GPIO_PIN_PA1_TIM2_CH2           0x0201
#define GPIO_PIN_PA2_TIM2_CH3           0x0202
#define GPIO_PIN_PA3_TIM2_CH4           0x0203
#define GPIO_PIN_PA5_TIM2_CH1           0x0505
#define GPIO_PIN_PA5_TIM2_ETR           0x0205
#define GPIO_PIN_PA15_TIM2_CH1          0x050f
#define GPIO_PIN_PA15_TIM2_ETR          0x020f
#define GPIO_PIN_PB3_TIM2_CH2           0x0213
#define GPIO_PIN_PB10_TIM2_CH3          0x021a
#define GPIO_PIN_PB11_TIM2_CH4          0x021b

#define GPIO_PIN_PA6_TIM3_CH1           0x0206
#define GPIO_PIN_PA7_TIM3_CH2           0x0207
#define GPIO_PIN_PB0_TIM3_CH3           0x0210
#define GPIO_PIN_PB1_TIM3_CH4           0x0211
#define GPIO_PIN_PB4_TIM3_CH1           0x0214
#define GPIO_PIN_PB5_TIM3_CH2           0x0415

#define GPIO_PIN_PA1_TIM21_ETR          0x0501
#define GPIO_PIN_PA2_TIM21_CH1          0x0002
#define GPIO_PIN_PA3_TIM21_CH2          0x0003
#define GPIO_PIN_PB13_TIM21_CH1         0x061d
#define GPIO_PIN_PB14_TIM21_CH2         0x061e

#define GPIO_PIN_PA4_TIM22_ETR          0x0504
#define GPIO_PIN_PA6_TIM22_CH1          0x0506
#define GPIO_PIN_PA7_TIM22_CH2          0x0507
#define GPIO_PIN_PB4_TIM22_CH1          0x0414
#define GPIO_PIN_PB5_TIM22_CH2          0x0415

#define GPIO_PIN_PB2_LPTIM1_OUT         0x0212
#define GPIO_PIN_PB5_LPTIM1_IN1         0x0215
#define GPIO_PIN_PB6_LPTIM1_ETR         0x0216
#define GPIO_PIN_PB7_LPTIM1_IN2         0x0217
#define GPIO_PIN_PC0_LPTIM1_IN1         0x0020
#define GPIO_PIN_PC1_LPTIM1_OUT         0x0021
#define GPIO_PIN_PC2_LPTIM1_IN2         0x0022

#define GPIO_PIN_PA0_COMP1_OUT          0x0700
#define GPIO_PIN_PA6_COMP1_OUT          0x0706
#define GPIO_PIN_PA11_COMP1_OUT         0x070b

#define GPIO_PIN_PA2_COMP2_OUT          0x0702
#define GPIO_PIN_PA7_COMP2_OUT          0x0707
#define GPIO_PIN_PA12_COMP2_OUT         0x070c

#define GPIO_PIN_PA0_TSC_G1_IO1         0x0300
#define GPIO_PIN_PA1_TSC_G1_IO2         0x0301
#define GPIO_PIN_PA2_TSC_G1_IO3         0x0302
#define GPIO_PIN_PA3_TSC_G1_IO4         0x0303
#define GPIO_PIN_PA4_TSC_G2_IO1         0x0304
#define GPIO_PIN_PA5_TSC_G2_IO2         0x0305
#define GPIO_PIN_PA6_TSC_G2_IO3         0x0306
#define GPIO_PIN_PA7_TSC_G2_IO4         0x0307
#define GPIO_PIN_PA9_TSC_G4_IO1         0x0309
#define GPIO_PIN_PA10_TSC_G4_IO2        0x030a
#define GPIO_PIN_PA11_TSC_G4_IO3        0x030b
#define GPIO_PIN_PA12_TSC_G4_IO4        0x030c
#define GPIO_PIN_PB0_TSC_G3_IO2         0x0310
#define GPIO_PIN_PB1_TSC_G3_IO3         0x0311
#define GPIO_PIN_PB2_TSC_G3_IO4         0x0312
#define GPIO_PIN_PB3_TSC_G5_IO1         0x0313
#define GPIO_PIN_PB4_TSC_G5_IO2         0x0314
#define GPIO_PIN_PB6_TSC_G5_IO3         0x0316
#define GPIO_PIN_PB7_TSC_G5_IO4         0x0317
#define GPIO_PIN_PB8_TSC_SYNC           0x0318
#define GPIO_PIN_PB10_TSC_SYNC          0x031a
#define GPIO_PIN_PB11_TSC_G6_IO1        0x031b
#define GPIO_PIN_PB12_TSC_G6_IO2        0x031c
#define GPIO_PIN_PB13_TSC_G6_IO3        0x031d
#define GPIO_PIN_PB14_TSC_G6_IO4        0x031e

#endif

#if defined(STM32L082xx)

#define GPIO_PORT_COUNT                 4 /* A, B, C, H */
#define GPIO_GROUP_COUNT                8 /* A, B, C, D, E, F, G, H */

#endif

typedef struct _stm32l0_gpio_state_t {
    uint32_t                mode[GPIO_PORT_COUNT];
    uint32_t                pupd[GPIO_PORT_COUNT];
} stm32l0_gpio_state_t;

#define GPIO_PIN_MASK(_pin)             (1ul << ((_pin) & 15))

extern void __stm32l0_gpio_initialize(void);

extern void stm32l0_gpio_swd_enable(void);
extern void stm32l0_gpio_swd_disable(void);
extern void stm32l0_gpio_pin_configure(unsigned int pin, unsigned int mode);
extern void stm32l0_gpio_pin_input(unsigned int pin);
extern void stm32l0_gpio_pin_output(unsigned int pin);
extern void stm32l0_gpio_pin_alternate(unsigned int pin);
extern void stm32l0_gpio_pin_analog(unsigned int pin);
extern void stm32l0_gpio_save(stm32l0_gpio_state_t *state);
extern void stm32l0_gpio_restore(stm32l0_gpio_state_t *state);

extern unsigned int __stm32l0_gpio_pin_read(unsigned int pin);
extern void __stm32l0_gpio_pin_write(unsigned int pin, unsigned int data);

static inline unsigned int stm32l0_gpio_pin_read(unsigned int pin)
{
    if (__builtin_constant_p(pin))
    {
	GPIO_TypeDef *GPIO;
	uint32_t group, index;
	
	group = (pin & GPIO_PIN_GROUP_MASK) >> GPIO_PIN_GROUP_SHIFT;
	index = (pin & GPIO_PIN_INDEX_MASK) >> GPIO_PIN_INDEX_SHIFT;
	
	GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);
	
	return ((GPIO->IDR >> index) & 1);
    }
    else
    {
	return __stm32l0_gpio_pin_read(pin);
    }
}

static inline void stm32l0_gpio_pin_write(unsigned int pin, unsigned int data)
{
    if (__builtin_constant_p(pin))
    {
	GPIO_TypeDef *GPIO;
	uint32_t group, index;
	
	group = (pin & GPIO_PIN_GROUP_MASK) >> GPIO_PIN_GROUP_SHIFT;
	index = (pin & GPIO_PIN_INDEX_MASK) >> GPIO_PIN_INDEX_SHIFT;
	
	GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

	if (data) {
	    GPIO->BSRR = (1ul << index);
	} else {
	    GPIO->BRR = (1ul << index);
	}
    }
    else
    {
	__stm32l0_gpio_pin_write(pin, data);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_GPIO_H */