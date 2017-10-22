/**
 * @file    gpio.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * Copyright (c) 2016-2017 NXP
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "fsl_device_registers.h"
#include "RTL.h"
#include "DAP_config.h"
#include "gpio.h"
#include "target_reset.h"
#include "daplink.h"
#include "hic_init.h"

static void busy_wait(uint32_t cycles)
{
    volatile uint32_t i;
    i = cycles;

    while (i > 0) {
        i--;
    }
}

void gpio_init(void)
{
    hic_init();

    // Enable hardfault on unaligned access for the interface only.
    // If this is done in the bootloader than then it might (will) break
    // older application firmware or firmware from 3rd party vendors.
#if  defined(DAPLINK_IF)
    SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif
    // enable clock to ports
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
    // configure pin as GPIO
    LED_CONNECTED_PORT->PCR[LED_CONNECTED_BIT] = PORT_PCR_MUX(1);
    // led off - enable output
    LED_CONNECTED_GPIO->PDOR = 1UL << LED_CONNECTED_BIT;
    LED_CONNECTED_GPIO->PDDR = 1UL << LED_CONNECTED_BIT;
    // led on
    LED_CONNECTED_GPIO->PCOR = 1UL << LED_CONNECTED_BIT;
    // reset button configured as gpio input
    PIN_nRESET_GPIO->PDDR &= ~PIN_nRESET;
    PIN_nRESET_PORT->PCR[PIN_nRESET_BIT] = PORT_PCR_MUX(1);
    /* Enable LVLRST_EN */
    PIN_nRESET_EN_PORT->PCR[PIN_nRESET_EN_BIT] = PORT_PCR_MUX(1)  |  /* GPIO */
            PORT_PCR_ODE_MASK;  /* Open-drain */
    PIN_nRESET_EN_GPIO->PSOR  = PIN_nRESET_EN;
    PIN_nRESET_EN_GPIO->PDDR |= PIN_nRESET_EN;

    // Enable pulldowns on power monitor control signals to reduce power consumption.
    PIN_CTRL0_PORT->PCR[PIN_CTRL0_BIT] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS(0);
    PIN_CTRL1_PORT->PCR[PIN_CTRL1_BIT] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS(0);
    PIN_CTRL2_PORT->PCR[PIN_CTRL2_BIT] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS(0);
    PIN_CTRL3_PORT->PCR[PIN_CTRL3_BIT] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS(0);

    // Enable pulldown on GPIO0_B to prevent it floating.
    PIN_GPIO0_B_PORT->PCR[PIN_GPIO0_B_BIT] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS(0);

    // Keep powered off in bootloader mode
    // to prevent the target from effecting the state
    // of the reset line / reset button
    if (!daplink_is_bootloader()) {
        // configure pin as GPIO
        PIN_POWER_EN_PORT->PCR[PIN_POWER_EN_BIT] = PORT_PCR_MUX(1);
        // set output to 0
        PIN_POWER_EN_GPIO->PCOR = PIN_POWER_EN;
        // switch gpio to output
        PIN_POWER_EN_GPIO->PDDR |= PIN_POWER_EN;
    }
}

void gpio_set_board_power(bool powerEnabled)
{
    if (!daplink_is_bootloader()) {
        if (powerEnabled) {
            // enable power switch
            PIN_POWER_EN_GPIO->PSOR = PIN_POWER_EN;

            // Let the voltage rails stabilize.  This is especailly important
            // during software resets, since the target's 3.3v rail can take
            // 20-50ms to drain.  During this time the target could be driving
            // the reset pin low, causing the bootloader to think the reset
            // button is pressed.
            // Note: With optimization set to -O2 the value 5115 delays for ~1ms @ 20.9Mhz core
            busy_wait(5115 * 50);
        }
        else {
            // disable power switch
            PIN_POWER_EN_GPIO->PCOR = PIN_POWER_EN;
        }
    }
}

void gpio_set_hid_led(gpio_led_state_t state)
{
    if (state) {
        LED_CONNECTED_GPIO->PCOR = LED_CONNECTED; // LED on
    } else {
        LED_CONNECTED_GPIO->PSOR = LED_CONNECTED; // LED off
    }
}

void gpio_set_cdc_led(gpio_led_state_t state)
{
    gpio_set_hid_led(state);
}

void gpio_set_msc_led(gpio_led_state_t state)
{
    gpio_set_hid_led(state);
}

uint8_t gpio_get_sw_reset(void)
{
    return (PIN_nRESET_GPIO->PDIR & PIN_nRESET) ? 1 : 0;
}

void target_forward_reset(bool assert_reset)
{
    // Do nothing - reset button is already tied to the target
    //              reset pin on hic interface hardware
}
