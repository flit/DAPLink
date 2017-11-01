/**
 * @file    hic_profile.h
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

#ifndef HIC_PROFILE_H
#define HIC_PROFILE_H

#include "IO_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct profile_channel_params {
    uint8_t channelNumber;
    char idString[9];
    char unitString[9];
    uint8_t dataSize;
    uint8_t dataBits;
    uint8_t rangeCount;
    uint8_t freqCount;
    struct {
        float minVal;
        float maxVal;
    } ranges[4];
    uint32_t freqs[16];
} profile_channel_params_t;

uint32_t hic_profile_get_channels(void);
const profile_channel_params_t * hic_profile_get_channel_info(uint32_t channel);

#ifdef __cplusplus
}
#endif

#endif // HIC_PROFILE_H
