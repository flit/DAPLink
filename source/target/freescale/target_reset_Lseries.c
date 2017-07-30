/**
 * @file    target_reset_Lseries.c
 * @brief   Target reset for the Kinetis L series
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

#include "target_reset.h"
#include "swd_host.h"
#include "debug_cm.h"
#include "info.h"

#define MDM_STATUS  0x01000000
#define MDM_CTRL    0x01000004
#define MDM_IDR     0x010000fc     // read-only identification register
#define MDM_ID      0x001c0020     // L series

#define MDM_STATUS_FLASH_MASS_ERASE_ACKNOWLEDGE (1 << 0)
#define MDM_STATUS_FLASH_READY (1 << 1)
#define MDM_STATUS_SYSTEM_SECURITY (1 << 2)
#define MDM_STATUS_MASS_ERASE_ENABLE (1 << 5)
#define MDM_STATUS_CORE_HALTED (1 << 16)

#define MDM_CTRL_FLASH_MASS_ERASE_IN_PROGRESS (1 << 0)
#define MDM_CTRL_DEBUG_REQUEST (1 << 2)
#define MDM_CTRL_SYSTEM_RESET_REQUEST (1 << 3)
#define MDM_CTRL_CORE_HOLD_REQUEST (1 << 4)

#define TIMEOUT_COUNT (1000000)

void target_before_init_debug(void)
{
    swd_set_target_reset(1);
}

void board_init(void)
{
}

void prerun_target_config(void)
{
}

uint8_t mdm_ap_status_check(uint32_t mask, uint32_t expected)
{
    uint32_t val;

    if (!swd_read_ap(MDM_STATUS, &val)) {
        return 0;
    }

    return (val & mask) == expected;
}

uint8_t mdm_ap_status_wait(uint32_t mask, uint32_t expected)
{
    uint32_t val;
    uint32_t timeoutCounter = 0;
    do {
        if (!swd_read_ap(MDM_STATUS, &val)) {
            return 0;
        }

        if (++timeoutCounter > TIMEOUT_COUNT) {
            return 0;
        }
    } while ((val & mask) != expected);

    return 1;
}

uint8_t target_mass_erase(void)
{
    // Make sure mass erase is enabled.
    if (!mdm_ap_status_check(MDM_STATUS_MASS_ERASE_ENABLE, MDM_STATUS_MASS_ERASE_ENABLE)) {
        return 0;
    }

    // Write the mass-erase enable bit.
    if (!swd_write_ap(MDM_CTRL, MDM_CTRL_FLASH_MASS_ERASE_IN_PROGRESS)) {
        return 0;
    }

    // Verify mass erase has started.
    if (!mdm_ap_status_wait(MDM_STATUS_FLASH_MASS_ERASE_ACKNOWLEDGE, MDM_STATUS_FLASH_MASS_ERASE_ACKNOWLEDGE)) {
        return 0;
    }

    // Wait until mass erase completes.
    uint32_t val;
    uint32_t timeoutCounter = 0;
    do {
        if (!swd_read_ap(MDM_CTRL, &val)) {
            return 0;
        }

        if (++timeoutCounter > TIMEOUT_COUNT) {
            return 0;
        }
    } while ((val & MDM_CTRL_FLASH_MASS_ERASE_IN_PROGRESS) != 0);

    // Confirm the mass erase was successful.
    if (!mdm_ap_status_check(MDM_STATUS_SYSTEM_SECURITY, 0)) {
        return 0;
    }

    return 1;
}

uint8_t target_unlock_sequence(void)
{
    uint32_t val;

    // read the device ID
    if (!swd_read_ap(MDM_IDR, &val)) {
        return 0;
    }

    // verify the result
    if (val != MDM_ID) {
        return 0;
    }

    // Assert reset to ensure we can gain control of the device.
    swd_set_target_reset(1);

    // Wait until flash is ready.
    if (!mdm_ap_status_wait(MDM_STATUS_FLASH_READY, MDM_STATUS_FLASH_READY)) {
        swd_set_target_reset(0);
        return 0;
    }

    // Check if security is enabled.
    if (!swd_read_ap(MDM_STATUS, &val)) {
        swd_set_target_reset(0);
        return 0;
    }

    // Perform mass erase if security is enabled.
    if (val & MDM_STATUS_SYSTEM_SECURITY) {
        if (!target_mass_erase()) {
            swd_set_target_reset(0);
            return 0;
        }
    }

    swd_set_target_reset(0);

    return 1;
}

// Check Flash Configuration Field bytes at address 0x400-0x40f to ensure that flash security
// won't be enabled.
//
// FCF bytes:
// [0x0-0x7]=backdoor key
// [0x8-0xb]=flash protection bytes
// [0xc]=FSEC:
//      [7:6]=KEYEN (2'b10 is backdoor key enabled, all others backdoor key disabled)
//      [5:4]=MEEN (2'b10 mass erase disabled, all other mass erase enabled)
//      [3:2]=FSLACC (2'b00 and 2'b11 factory access enabled, 2'b01 and 2'b10 factory access disabled)
//      [1:0]=SEC (2'b10 flash security disabled, all other flash security enabled)
// [0xd]=FOPT
// [0xe]=EEPROM protection bytes (FlexNVM devices only)
// [0xf]=data flash protection bytes (FlexNVM devices only)
//
// This function checks that:
// - FSEC does not disable mass erase or secure the device.
//
uint8_t security_bits_set(uint32_t addr, uint8_t *data, uint32_t size)
{
    const uint32_t fsec_addr = 0x40C;

    if ((addr <= fsec_addr) && (addr + size) > fsec_addr) {
        uint8_t fsec = data[fsec_addr - addr];

        // make sure we can unsecure the device or dont program at all
        if ((fsec & 0x30) == 0x20) {
            // Dont allow programming mass-erase disabled state
            return 1;
        }

        // Security is OK long as we can mass-erase (comment the following out to enable target security)
        if ((fsec & 0x03) != 0x02) {
            return 1;
        }
    }

    return 0;
}

uint8_t target_set_state(TARGET_RESET_STATE state)
{
    uint32_t val;

    switch (state) {
        case RESET_RUN:
            // Disable debug before resetting target, to make sure the core is not held in debug halt.
            swd_set_target_state_hw(NO_DEBUG);
            return swd_set_target_state_hw(RESET_RUN);

        case RESET_PROGRAM:
            // This will unlock the device if necessary.
            if (!swd_init_debug()) {
                return 0;
            }

            // Assert reset to ensure we can gain control of the device.
            swd_set_target_reset(1);

            // Wait until flash is ready.
            if (!mdm_ap_status_wait(MDM_STATUS_FLASH_READY, MDM_STATUS_FLASH_READY)) {
                swd_set_target_reset(0);
                return 0;
            }

            // Prevent the target from resetting if it has invalid code
            if (!swd_write_ap(MDM_CTRL, MDM_CTRL_DEBUG_REQUEST | MDM_CTRL_CORE_HOLD_REQUEST)) {
                swd_set_target_reset(0);
                return 0;
            }

            // Verify bits are set.
            if (!swd_read_ap(MDM_CTRL, &val)) {
                swd_set_target_reset(0);
                return 0;
            }
            if (val & (MDM_CTRL_DEBUG_REQUEST | MDM_CTRL_CORE_HOLD_REQUEST) != (MDM_CTRL_DEBUG_REQUEST | MDM_CTRL_CORE_HOLD_REQUEST)) {
                swd_set_target_reset(0);
                return 0;
            }

            // Enable debug and halt the core (DHCSR <- 0xA05F0003)
            if (!swd_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT)) {
                swd_set_target_reset(0);
                return 0;
            }

            // Release reset.
            swd_set_target_reset(0);

            // Disable holding the core in reset, leave MDM-AP halt on.
            if (!swd_write_ap(MDM_CTRL, MDM_CTRL_DEBUG_REQUEST)) {
                return 0;
            }

            // Wait until core is halted.
            if (!mdm_ap_status_wait(MDM_STATUS_CORE_HALTED, MDM_STATUS_CORE_HALTED)) {
                swd_set_target_reset(0);
                return 0;
            }

            // Release MDM halt once it has taken effect in the DHCSR
            if (!swd_write_ap(MDM_CTRL, 0)) {
                return 0;
            }

            // Verify core is still halted by reading the DHCSR.
            if (!swd_read_word(DBG_HCSR, &val)) {
                return 0;
            }
            if (!(val & S_HALT)) {
                return 0;
            }

            break;

        default:
            return swd_set_target_state_hw(state);
    }

    return 1;
}
