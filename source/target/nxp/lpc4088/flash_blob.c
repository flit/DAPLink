/**
 * @file    flash_blob.c
 * @brief   Flash algorithm for the lpc4088
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

#include "flash_blob.h"


static const uint32_t lpc4088_flash_prog_blob[] = {
    0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,
    0x28100b00, 0x210ebf24, 0x00d0eb01, 0xe92d4770, 0xf8df4df0, 0x46068320, 0x460c44c8, 0x0000f8d8, 
    0x1c402500, 0x0f01f010, 0x0000f8c8, 0x461749c2, 0x2080f44f, 0x63c8bf14, 0x05306388, 0xa2fcf8df, 
    0xf04f0d00, 0x44ca0b00, 0xf8cad111, 0xf44fb010, 0xf8ca5080, 0xe9ca6000, 0xf8ca0b01, 0xf8d8b00c, 
    0x4651000c, 0xf1a16882, 0x47900080, 0x2018b9c0, 0xb008f8ca, 0xb003e9ca, 0xf5b4b1cc, 0xbf8c7f80, 
    0x7b80f44f, 0x197046a3, 0x0b00e9ca, 0x000cf8d8, 0x19794aaa, 0x6843444a, 0x0080f1a2, 0xb1104798, 
    0xe8bd2001, 0x445d8df0, 0x040bebb4, 0x2000d1e5, 0x8df0e8bd, 0x41f0e92d, 0x8278f8df, 0x60e0f642, 
    0x4d9f44c8, 0x0008f8c8, 0xf8052000, 0x20aa0f80, 0x20557328, 0x20017328, 0x0c40f805, 0x21122200, 
    0xf91af000, 0x210d2200, 0xf0004610, 0x2200f915, 0x2001210d, 0xf910f000, 0x21132200, 0xf0002002, 
    0x4c8df90b, 0xf4406820, 0x60205000, 0xf4406a20, 0x62202084, 0xf4406c20, 0x64202000, 0x2780f44f, 
    0x117e63e7, 0x6c6861a6, 0x3080f440, 0x20026468, 0x0134f8c5, 0x21072205, 0xf8eef000, 0x21162205, 
    0xf0002000, 0x2205f8e9, 0x2000210f, 0xf8e4f000, 0x21102205, 0xf0002000, 0x2205f8df, 0x20002111, 
    0xf8daf000, 0x21122205, 0xf0002000, 0x4875f8d5, 0x727af44f, 0x69406800, 0x000cf8c8, 0x1008f8d8, 
    0xf1f2fbb1, 0x2134f8d5, 0xc000f8d0, 0x021ff002, 0xf3f2fbb1, 0x486d496c, 0x2103fba1, 0x22c00889, 
    0x47e04448, 0xbf042800, 0xe8bd61e6, 0x63a781f0, 0x200161e6, 0x81f0e8bd, 0x47702000, 0x41f0e92d, 
    0x20324c63, 0x2700444c, 0x60a5251d, 0x0700e9c4, 0xf1044e60, 0x46200114, 0x696047b0, 0x2034b980, 
    0xe9c460a5, 0x48530700, 0x0114f104, 0x68804448, 0x462060e0, 0x696047b0, 0xbf082800, 0x81f0e8bd, 
    0xe8bd2001, 0xf1b081f0, 0xbf325f20, 0x2f00f5b0, 0x47702000, 0x0b04b570, 0xbf242c10, 0xeb00200e, 
    0x203204d4, 0x4e4b4d4a, 0xf105444d, 0xe9c50114, 0x46280400, 0x47b060ac, 0xb9786968, 0xe9c52034, 
    0x483c0400, 0x444860ac, 0x0114f105, 0x60e86880, 0x47b04628, 0x28006968, 0xbd70bf08, 0xbd702001, 
    0x41f0e92d, 0x46054f33, 0x4614444f, 0x4a326878, 0xf0101c40, 0x60780f01, 0x5000f44f, 0x61d0bf14, 
    0xf1b56190, 0xd3055f20, 0xf1a54622, 0xe8bd5020, 0xe6bc41f0, 0x2f00f5b5, 0x4622d305, 0x2000f5a5, 
    0x41f0e8bd, 0xb975e6b3, 0x0100e9d4, 0xe9d44408, 0x44111202, 0x69214408, 0x69614408, 0x69a14408, 
    0x42404408, 0x0b2861e0, 0xbf242810, 0xeb01210e, 0x213200d0, 0xf8df4e1e, 0x444e807c, 0x1000e9c6, 
    0xf10660b0, 0x46300114, 0x697047c0, 0x2033b988, 0x0500e9c6, 0x7000f44f, 0x4002e9c6, 0x613068b8, 
    0x0114f106, 0x47c04630, 0x28006970, 0xe8bdbf08, 0x200181f0, 0x81f0e8bd, 0x1040eb01, 0xeb01490e, 
    0x68010080, 0x0107f021, 0x68016001, 0x60014311, 0x00004770, 0x00000004, 0x20098000, 0x000000b4, 
    0x400fc000, 0x1fff1ff8, 0xcccccccd, 0x00000034, 0x00000014, 0x1fff1ff1, 0x4002c000, 0x00000000, 
    0x00000001, 0x00000000, 0x00000000, 0x00000000, 
};

static const program_target_t flash = {
    0x200000D5, // Init
    0x200001D9, // UnInit
    0x200001DD, // EraseChip
    0x20000227, // EraseSector
    0x20000281, // ProgramPage

    // BKPT : start of blob + 1
    // RSB  : blob start + header + rw data offset
    // RSP  : stack pointer
    {
        0x20000000 + 0x00000001,
        0x20000000 + 0x00000020 + 0x00000400,
        0x20000800
    },

    0x20000000 + 0x00000A00,   // mem buffer location
    0x20000000,                // location to write prog_blob in target RAM
    sizeof(lpc4088_flash_prog_blob), // prog_blob size
    lpc4088_flash_prog_blob,         // address of prog_blob
    0x00000200                 // ram_to_flash_bytes_to_be_written
};
