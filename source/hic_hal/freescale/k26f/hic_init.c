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
//#include "usb_device_config.h"
//#include "fsl_usb.h"
//#include "usb_device.h"
#include "usb_phy.h"

/*! @brief Clock configuration structure. */
typedef struct _clock_config
{
    mcg_config_t mcgConfig;       /*!< MCG configuration.      */
    sim_clock_config_t simConfig; /*!< SIM configuration.      */
    osc_config_t oscConfig;       /*!< OSC configuration.      */
    uint32_t coreClock;           /*!< core clock frequency.   */
} clock_config_t;

/* Configuration for enter RUN mode. Core clock = 120MHz. */
const clock_config_t g_defaultClockConfigRun = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModePEE,             /* Work in PEE mode. */
            .irclkEnableMode = kMCG_IrclkEnable, /* MCGIRCLK enable. */
            .ircs = kMCG_IrcSlow,                /* Select IRC32k. */
            .fcrdiv = 0U,                        /* FCRDIV is 0. */

            .frdiv = 4U,
            .drs = kMCG_DrsLow,         /* Low frequency range */
            .dmx32 = kMCG_Dmx32Default, /* DCO has a default range of 25% */
            .oscsel = kMCG_OscselOsc,   /* Select OSC */

            .pll0Config =
                {
                    .enableMode = 0U, .prdiv = 0x00U, .vdiv = 0x04U,
                },
            .pllcs = kMCG_PllClkSelPll0,
        },
    .simConfig =
        {
            .pllFllSel = 1U, /* PLLFLLSEL select PLL. */
            .pllFllDiv = 0U,
            .pllFllFrac = 0U,
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC. */
            .clkdiv1 = 0x01140000U, /* SIM_CLKDIV1. */
        },
    .oscConfig = {.freq = CPU_XTAL_CLK_HZ,
                  .capLoad = 0,
                  .workMode = kOSC_ModeOscLowPower,
                  .oscerConfig =
                      {
                          .enableMode = kOSC_ErClkEnable,
                          .erclkDiv = 0U,
                      }},
    .coreClock = 120000000U, /* Core clock frequency */
};

/* Configuration for HSRUN mode. Core clock = 180MHz. */
const clock_config_t g_defaultClockConfigHsrun = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModePEE,                   /* Work in PEE mode. */
            .irclkEnableMode = kMCG_IrclkEnableInStop, /* MCGIRCLK enable. */
            .ircs = kMCG_IrcSlow,                      /* Select IRC32k.*/
            .fcrdiv = 0U,                              /* FCRDIV is 0. */

            .frdiv = 4U,
            .drs = kMCG_DrsLow,         /* Low frequency range. */
            .dmx32 = kMCG_Dmx32Default, /* DCO has a default range of 25%. */
            .oscsel = kMCG_OscselOsc,   /* Select OSC. */

            .pll0Config =
                {
                    .enableMode = 0U, .prdiv = 0x00U, .vdiv = 0x0EU,
                },
            .pllcs = kMCG_PllClkSelPll0,
        },
    .simConfig =
        {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL. */
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC. */
            .clkdiv1 = 0x02260000U, /* SIM_CLKDIV1. */
        },
    .oscConfig = {.freq = CPU_XTAL_CLK_HZ,
                  .capLoad = 0,
                  .workMode = kOSC_ModeOscLowPower,
                  .oscerConfig =
                      {
                          .enableMode = kOSC_ErClkEnable,
                          .erclkDiv = 0U,
                      }},
    .coreClock = 180000000U, /* Core clock frequency */
};

void BOARD_BootClockRUN(void)
{
    CLOCK_SetSimSafeDivs();

    CLOCK_InitOsc0(&g_defaultClockConfigRun.oscConfig);
    CLOCK_SetXtal0Freq(CPU_XTAL_CLK_HZ);

    CLOCK_BootToPeeMode(g_defaultClockConfigRun.mcgConfig.oscsel, kMCG_PllClkSelPll0,
                        &g_defaultClockConfigRun.mcgConfig.pll0Config);

    CLOCK_SetInternalRefClkConfig(g_defaultClockConfigRun.mcgConfig.irclkEnableMode,
                                  g_defaultClockConfigRun.mcgConfig.ircs, g_defaultClockConfigRun.mcgConfig.fcrdiv);

    CLOCK_SetSimConfig(&g_defaultClockConfigRun.simConfig);

    SystemCoreClock = g_defaultClockConfigRun.coreClock;
}

void BOARD_BootClockHSRUN(void)
{
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeHsrun(SMC);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateHsrun)
    {
    }

    CLOCK_SetSimSafeDivs();

    CLOCK_InitOsc0(&g_defaultClockConfigHsrun.oscConfig);
    CLOCK_SetXtal0Freq(CPU_XTAL_CLK_HZ);

    CLOCK_BootToPeeMode(g_defaultClockConfigHsrun.mcgConfig.oscsel, kMCG_PllClkSelPll0,
                        &g_defaultClockConfigHsrun.mcgConfig.pll0Config);

    CLOCK_SetInternalRefClkConfig(g_defaultClockConfigHsrun.mcgConfig.irclkEnableMode,
                                  g_defaultClockConfigHsrun.mcgConfig.ircs, g_defaultClockConfigHsrun.mcgConfig.fcrdiv);

    CLOCK_SetSimConfig(&g_defaultClockConfigHsrun.simConfig);

    SystemCoreClock = g_defaultClockConfigHsrun.coreClock;
}

// Enable just those clocks required to turn on the 480MHz PLL so we can connect via USB.
void hic_init(void)
{
    // Disable the MPU.
    MPU->CESR = 0;

    // Invalidate and enable code cache.
    LMEM->PCCCR = LMEM_PCCCR_GO_MASK | LMEM_PCCCR_INVW1_MASK | LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_ENCACHE_MASK;

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
}
