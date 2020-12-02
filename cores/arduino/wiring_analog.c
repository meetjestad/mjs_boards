/*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
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

#ifdef __cplusplus
extern "C" {
#endif

#if defined(PWM_INSTANCE_COUNT)

extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT];

static stm32l0_timer_t stm32l0_pwm[PWM_INSTANCE_COUNT];

static uint8_t _channels[PWM_INSTANCE_COUNT];

#endif /* PWM_INSTANCE_COUNT */

static int _readResolution = 10;
static int _readPeriod = 2;
static int _writeResolution = 8;
static int _writeFrequency = 0;

void analogReference(eAnalogReference reference)
{
    (void)reference;
}

void analogReadResolution(int resolution)
{
    if ((resolution >= 1) && (resolution <= 12))
    {
        _readResolution = resolution;
    }
}

void analogReadPeriod(int period)
{
    _readPeriod = period;
}

static uint32_t __analogReadRoutine(uint32_t channel, uint32_t period)
{
    uint32_t data;

    stm32l0_adc_enable();

    data = stm32l0_adc_read(channel, period);

    stm32l0_adc_disable();

    return data;
}

uint32_t __analogReadInternal(uint32_t channel, uint32_t period)
{
    EXCn_Type ipsr;

    ipsr = __get_IPSR();

    if (ipsr == ThreadMode_EXCn)
    {
        return (uint32_t)armv6m_svcall_2((uint32_t)&__analogReadRoutine, (uint32_t)channel, (uint32_t)period);
    }
    else
    {
        if ((ipsr == SVCall_EXCn) || (ipsr == PendSV_EXCn))
        {
            return __analogReadRoutine(channel, period);
        }
        else
        {
            return 0;
        }
    }
}

uint32_t analogRead(uint32_t ulPin)
{
    uint32_t input;

    if ( ulPin < A0 )
    {
        ulPin += A0 ;
    }

    if ( (ulPin >= PINS_COUNT) || (g_APinDescription[ulPin].adc_channel == ADC_CHANNEL_NONE) )
    {
        return 0;
    }
  
    __analogWriteDisable(ulPin);

    stm32l0_gpio_pin_configure(g_APinDescription[ulPin].pin, (STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_MODE_ANALOG));
    
    input = __analogReadInternal(g_APinDescription[ulPin].adc_channel, _readPeriod);

    return (input >> (12 - _readResolution));
}

void analogWriteResolution( int resolution )
{
    if ((resolution >= 1) && (resolution <= 12))
    {
        _writeResolution = resolution;
    }
}

void analogWriteFrequency( unsigned long frequency )
{
    uint32_t instance, clock, carrier, modulus, divider;

    _writeFrequency = frequency;

    for (instance = 0; instance < PWM_INSTANCE_COUNT; instance++)
    {
        if (stm32l0_pwm[instance].state == STM32L0_TIMER_STATE_ACTIVE)
        {
            clock = stm32l0_timer_clock(&stm32l0_pwm[instance]);

            modulus = 4095;

            if (_writeFrequency)
            {
                carrier = _writeFrequency * modulus;
            }
            else
            {
                carrier = 2000000;
            }
                
            if (carrier > clock)
            {
                carrier = (clock / modulus) * modulus;
            }
            
            divider = (clock + (carrier - 1)) / carrier;

            stm32l0_timer_stop(&stm32l0_pwm[instance]);
            stm32l0_timer_configure(&stm32l0_pwm[instance], divider -1, 0);
            stm32l0_timer_start(&stm32l0_pwm[instance], modulus -1, false);
        }
    }
}
    
void analogWrite(uint32_t ulPin, uint32_t value)
{
    uint32_t instance, clock, carrier, modulus, divider, output;

    // Handle the case the pin isn't usable as PIO
    if ( (ulPin >= PINS_COUNT) || (g_APinDescription[ulPin].GPIO == NULL) )
    {
        return;
    }

    output = value << (12 - _writeResolution);

    if (output > 4095)
    {
        output = 4095;
    }
                    
#if defined(DAC_RESOLUTION)
    if (g_APinDescription[ulPin].attr & (PIN_ATTR_DAC1 | PIN_ATTR_DAC2))
    {
        stm32l0_gpio_pin_configure(g_APinDescription[ulPin].pin, (STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_MODE_ANALOG));
    
        stm32l0_dac_enable(g_APinDescription[ulPin].attr & (PIN_ATTR_DAC1 | PIN_ATTR_DAC2));

        stm32l0_dac_write((g_APinDescription[ulPin].attr & (PIN_ATTR_DAC1 | PIN_ATTR_DAC2)), output);

        return;
    }
#endif /* DAC_RESOLUTION */

#if defined(PWM_INSTANCE_COUNT)
    if (g_APinDescription[ulPin].pwm_instance != PWM_INSTANCE_NONE)
    {
        instance = g_APinDescription[ulPin].pwm_instance;

        if (_channels[instance] & (1u << g_APinDescription[ulPin].pwm_channel))
        {
            stm32l0_timer_compare(&stm32l0_pwm[instance], g_APinDescription[ulPin].pwm_channel, output);
        }
        else
        {
            _channels[instance] |= (1u << g_APinDescription[ulPin].pwm_channel);

            if (stm32l0_pwm[instance].state == STM32L0_TIMER_STATE_NONE)
            {
                stm32l0_timer_create(&stm32l0_pwm[instance], g_PWMInstances[instance], STM32L0_PWM_IRQ_PRIORITY, 0);
            }
            
            if (stm32l0_pwm[instance].state == STM32L0_TIMER_STATE_INIT)
            {
                clock = stm32l0_timer_clock(&stm32l0_pwm[instance]);

                modulus = 4095;

                if (_writeFrequency)
                {
                    carrier = _writeFrequency * modulus;
                }
                else
                {
                    carrier = 2000000;
                }

                if (carrier > clock)
                {
                    carrier = (clock / modulus) * modulus;
                }

                divider = (clock + (carrier - 1)) / carrier;
                
                stm32l0_timer_enable(&stm32l0_pwm[instance], divider -1, 0, NULL, NULL, 0);
                stm32l0_timer_start(&stm32l0_pwm[instance], modulus -1, false);
            }
            
            stm32l0_gpio_pin_configure(g_APinDescription[ulPin].pin, (STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
            
            stm32l0_timer_channel(&stm32l0_pwm[instance], g_APinDescription[ulPin].pwm_channel, output, STM32L0_TIMER_CONTROL_PWM);
        }

        return;
    }
#endif /* PWM_INSTANCE_COUNT */

    // -- Defaults to digital write
    pinMode(ulPin, OUTPUT) ;
    if (output < 2048)
    {
        digitalWrite(ulPin, LOW);
    }
    else
    {
        digitalWrite(ulPin, HIGH );
    }
}

void __analogWriteDisable(uint32_t ulPin)
{
#if defined(PWM_INSTANCE_COUNT)
    uint32_t instance;
#endif /* PWM_INSTANCE_COUNT */

#if defined(DAC_RESOLUTION)
    if (g_APinDescription[ulPin].attr & (PIN_ATTR_DAC1 | PIN_ATTR_DAC2))
    {
        stm32l0_dac_disable(g_APinDescription[ulPin].attr & (PIN_ATTR_DAC1 | PIN_ATTR_DAC2));

        return;
    }
#endif /* DAC_RESOLUTION */

#if defined(PWM_INSTANCE_COUNT)
    if (g_APinDescription[ulPin].pwm_instance != PWM_INSTANCE_NONE)
    {
        instance = g_APinDescription[ulPin].pwm_instance;

        if (_channels[instance] & (1u << g_APinDescription[ulPin].pwm_channel))
        {
            stm32l0_timer_channel(&stm32l0_pwm[instance], g_APinDescription[ulPin].pwm_channel, 0, STM32L0_TIMER_CONTROL_DISABLE);;
            
            _channels[instance] &= ~(1u << g_APinDescription[ulPin].pwm_channel);
            
            if (!_channels[instance])
            {
                stm32l0_timer_stop(&stm32l0_pwm[instance]);
                stm32l0_timer_disable(&stm32l0_pwm[instance]);
            }
        }
    }
#endif /* PWM_INSTANCE_COUNT */
}

#ifdef __cplusplus
}
#endif

