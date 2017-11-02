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

//! @brief Channel descriptor.
typedef struct profile_channel_params {
    uint8_t channelNumber;  //!< Channel number, 0-n.
    char idString[9];       //!< Informative name of this channel.
    char unitString[9];     //!< Units of this channel.
    uint8_t dataSize;       //!< Channel data size in bytes (1, 2, or 4).
    uint8_t dataBits;       //!< Channel data size in bits. Must be less than or equal to dataSize * 8.
    uint8_t rangeCount;     //!< Number of valid entries in the ranges field.
    uint8_t freqCount;      //!< Number of valid entries in the freqs field.
    struct {
        float minVal;
        float maxVal;
    } ranges[4];            //!< Minimum and maximum values for each range.
    uint32_t freqs[16];     //!< Frequencies in Hertz supported by this channel.
} profile_channel_params_t;

//! @brief Profiling control commands.
typedef enum _profile_control_commands {
    kProfileStartChannels = 1,
    kProfileStopChannels = 2,
    kProfileSetFrequency = 3,
} profile_control_command_t;


#ifdef __cplusplus
extern "C" {
#endif

uint32_t hic_profile_get_channels(void);
const profile_channel_params_t * hic_profile_get_channel_info(uint32_t channel);

uint32_t hic_profile_control(profile_control_command_t command, uint32_t value);

uint32_t hic_profile_read_data(uint32_t maxLength, void * data);

#ifdef __cplusplus
}
#endif

#endif // HIC_PROFILE_H
