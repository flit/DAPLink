/**
 * DAPLink Interface Firmware
 * Copyright (c) 2020, Arm Limited
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

#ifndef DAPLINK_ADDR_H
#define DAPLINK_ADDR_H

/* Device sizes */

#define DAPLINK_ROM_START               0x10000000 // S alias
#define DAPLINK_ROM_SIZE                0x0009d800 // 640 kB - 17 reserved pages = 630 kB

#define DAPLINK_RAM_START               0x30000000 // S alias
#define DAPLINK_RAM_SIZE                0x00040000 // SRAM 0-3

/* ROM sizes */

#define DAPLINK_ROM_BL_START            0x10000000
#define DAPLINK_ROM_BL_SIZE             0x00010000 // 64 kB bootloader

#define DAPLINK_ROM_CONFIG_ADMIN_START  0x10010000
#define DAPLINK_ROM_CONFIG_ADMIN_SIZE   0x00000000

#define DAPLINK_ROM_IF_START            0x10010000
#define DAPLINK_ROM_IF_SIZE             0x0008c800

#define DAPLINK_ROM_CONFIG_USER_START   0x1009c800
#define DAPLINK_ROM_CONFIG_USER_SIZE    0x00001000

/* RAM sizes */

#define DAPLINK_RAM_APP_START           0x30000000
#define DAPLINK_RAM_APP_SIZE            0x0003FF00

#define DAPLINK_RAM_SHARED_START        0x3003FF00
#define DAPLINK_RAM_SHARED_SIZE         0x00000100

/* Flash Programming Info */

#define DAPLINK_SECTOR_SIZE             0x00000200
#define DAPLINK_MIN_WRITE_SIZE          0x00000200

/* USB RAM */
#define DAPLINK_USB_RAM_START           0x40100000
#define DAPLINK_USB_RAM_SIZE            0x00004000

/* Current build */

#if defined(DAPLINK_BL)

#define DAPLINK_ROM_APP_START            DAPLINK_ROM_BL_START
#define DAPLINK_ROM_APP_SIZE             DAPLINK_ROM_BL_SIZE
#define DAPLINK_ROM_UPDATE_START         DAPLINK_ROM_IF_START
#define DAPLINK_ROM_UPDATE_SIZE          DAPLINK_ROM_IF_SIZE

#elif defined(DAPLINK_IF)

#define DAPLINK_ROM_APP_START            DAPLINK_ROM_IF_START
#define DAPLINK_ROM_APP_SIZE             DAPLINK_ROM_IF_SIZE
#define DAPLINK_ROM_UPDATE_START         DAPLINK_ROM_BL_START
#define DAPLINK_ROM_UPDATE_SIZE          DAPLINK_ROM_BL_SIZE

#else

#error "Build must be either bootloader or interface"

#endif

#endif
