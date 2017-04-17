/**
 * @file    target_reset_Kseries.c
 * @brief   Target reset for the Kinetis K series
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
#include "info.h"

#define MDM_STATUS  0x01000000
#define MDM_CTRL    0x01000004
#define MDM_IDR     0x010000fc
#define MDM_ID      0x001c0040 // K32 series

void target_before_init_debug(void)
{
    swd_set_target_reset(1);
}

void prerun_target_config(void)
{
    // SIM peripheral   0x40026000
    // address offset   0x    1054
    uint32_t UUID_LOC = 0x40026058; // UIDH, UIDM, UIDL
    uint32_t uuid[4] = {0};
    // get a hold of the target
    target_set_state(RESET_PROGRAM);
    // do mass-erase if necessary
    target_unlock_sequence();
    // get target UUID
    swd_read_memory(UUID_LOC, (uint8_t *)&uuid[1], 12);
    // stringify and store the MAC generated from a UUID
    info_set_uuid_target(uuid);
    // Let the target run
    target_set_state(RESET_RUN);
}

void board_init(void)
{
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

    if (!swd_read_ap(MDM_STATUS, &val)) {
        return 0;
    }

    // flash in secured mode
    if (val & (1 << 2)) {
        // hold the device in reset
        swd_set_target_reset(1);

        // write the mass-erase enable bit
        if (!swd_write_ap(MDM_CTRL, 1)) {
            return 0;
        }

        while (1) {
            // wait until mass erase is started
            if (!swd_read_ap(MDM_STATUS, &val)) {
                return 0;
            }

            if (val & 1) {
                break;
            }
        }

        // mass erase in progress
        while (1) {
            // keep reading until procedure is complete
            if (!swd_read_ap(MDM_CTRL, &val)) {
                return 0;
            }

            if (val == 0) {
                break;
            }
        }
    }

    return 1;
}

// K32 devices do not use the traditional location of the FCF at address 0x400. Instead,
// these fields are stored in IFR. Thus, there is nothing to modify.
uint8_t security_bits_set(uint32_t addr, uint8_t *data, uint32_t size)
{
    return 0;
}

uint8_t target_set_state(TARGET_RESET_STATE state)
{
    return swd_set_target_state_hw(state);
}
