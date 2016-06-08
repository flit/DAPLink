/**
 * @file    hic_init.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
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

#include "hic_init.h"
#include "fsl_clock.h"
#include "fsl_smc.h"
#include "usb_phy.h"
#include "util.h"

static void busy_wait(uint32_t cycles)
{
    volatile uint32_t i;
    i = cycles;

    while (i > 0) {
        i--;
    }
}

//! - MPU is disabled and gated.
//! - 8kB cache is enabled. SRAM is not cached, so no flushing is required for normal operation.
//! - Switch to high-speed run mode.
//! - Turn on 16MHz crystal oscillator.
//! - Turn on 32kHz IRC.
//! - Enable the 480MHz USB PHY PLL.
//! - Ungate USBPHY and USBHS.
//! - Configure the USB PHY.
//! - Enable the phase fractional divider (PFD) output from 480Mhz the USB PLL to output 120MHz.
//! - Configure system clocks to use the PFD.
void hic_init(void)
{
    // Disable the MPU.
    MPU->CESR = 0;
    SIM->SCGC7 &= ~SIM_SCGC7_MPU_MASK;

    // Invalidate and enable code cache.
    LMEM->PCCCR = LMEM_PCCCR_GO_MASK | LMEM_PCCCR_INVW1_MASK | LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_ENCACHE_MASK;

    // Enable LVW IRQ.
    PMC->LVDSC1 = PMC_LVDSC1_LVDV(1); // high trip point
    PMC->LVDSC2 = PMC_LVDSC2_LVWIE_MASK | PMC_LVDSC2_LVWV(3); // high trip point
    NVIC_EnableIRQ(LVD_LVW_IRQn);

    // Enable external oscillator and 32kHz IRC.
    MCG->C1 |= MCG_C1_IRCLKEN_MASK; // Select 32k IR.
    // Configure OSC for very high freq, low power mode.
    MCG->C2 = (MCG->C2 & ~(MCG_C2_RANGE_MASK | MCG_C2_HGO_MASK)) | MCG_C2_RANGE(2);
    OSC0->CR |= OSC_CR_ERCLKEN_MASK; // Enable OSC.
    MCG->C2 |= MCG_C2_EREFS_MASK; // Select OSC as ext ref.

    // Wait for the oscillator to stabilize.
    while (!(MCG->S & MCG_S_OSCINIT0_MASK))
    {
    }

    // Enable USB clock source and init phy. This turns on the 480MHz PLL.
    CLOCK_EnableUsbhs0Clock(kCLOCK_UsbSrcPll0, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
    USB_EhciPhyInit(0, CPU_XTAL_CLK_HZ);

    // Enable USBPHY PFD output.
    USBPHY->ANACTRL = USBPHY_ANACTRL_PFD_FRAC(26)       // 332.3MHz output
                        | USBPHY_ANACTRL_PFD_CLK_SEL(2) // Div 2 = 166.2MHz
                        | USBPHY_ANACTRL_PFD_CLKGATE(0);

    // Wait for PFD to be stable.
    while (!(USBPHY->ANACTRL & USBPHY_ANACTRL_PFD_STABLE_MASK))
    {
    }

    // Set dividers before switching clocks.
    SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(5)       // System/core  /6 = 27.7MHz
                    | SIM_CLKDIV1_OUTDIV2(5)    // Bus          /6 = 27.7Mhz
                    | SIM_CLKDIV1_OUTDIV3(5)    // FlexBus      /6 = 27.7Mhz
                    | SIM_CLKDIV1_OUTDIV4(5);   // Flash        /6 = 27.7MHz

    // Select USBPHY PFD as PLL source.
    MCG->C11 = MCG_C11_PLLCS_MASK;

    // Switch MCGOUTCLK to PLL source.
    MCG->C6 |= MCG_C6_PLLS_MASK;

    SystemCoreClockUpdate();
}

// This IRQ handler will be invoked if VDD falls below the trip point.
void LVD_LVW_IRQHandler(void)
{
    util_assert(0);
    busy_wait(100000);
}

void hic_enable_fast_clock(void)
{
    // Raise clocks in multiple steps.
    SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3)       // System/core  /4 = 41.5MHz
                    | SIM_CLKDIV1_OUTDIV2(3)    // Bus          /4 = 41.5Mhz
                    | SIM_CLKDIV1_OUTDIV3(7)    // FlexBus      /8 = 20.8Mhz
                    | SIM_CLKDIV1_OUTDIV4(7);   // Flash        /8 = 20.8MHz
    busy_wait(50000);

    SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(2)       // System/core  /3 = 55.4MHz
                    | SIM_CLKDIV1_OUTDIV2(5)    // Bus          /6 = 27.7Mhz
                    | SIM_CLKDIV1_OUTDIV3(5)    // FlexBus      /6 = 27.7Mhz
                    | SIM_CLKDIV1_OUTDIV4(5);   // Flash        /6 = 27.7MHz
    busy_wait(100000);

    SystemCoreClockUpdate();
}
