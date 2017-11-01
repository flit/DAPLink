/*
 * Copyright (c) 2017 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------
 *
 * $Date:        x
 * $Revision:    V1.00
 *
 * Project:      CMSIS-DAP Source
 * Title:        TI.c CMSIS-DAP TI I/O
 *
 *---------------------------------------------------------------------------*/
#if (TRACE_DATA_BLOCK_COUNT != 0)

#include "DAP_config.h"
#include "DAP.h"
#include "hic_profile.h"

static inline uint32_t len_to_flag(uint32_t length)
{
  return ((length == sizeof(uint8_t)) ? 0U
            : ((length == sizeof(uint16_t)) ? 1U
            : ((length == sizeof(uint32_t)) ? 2U
            : 0U)));
}

// Process TI Info command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
uint32_t TI_Info (const uint8_t *request, uint8_t *response) {
  uint32_t result = 0U;
  uint32_t infoId = *request++;
  uint32_t channel = *request++;
  const profile_channel_params_t *channelInfo;

  switch (infoId) {
    case DAP_TI_GET_GENERAL:
      *response++ = hic_profile_get_channels();
      break;
    case DAP_TI_GET_CHANNEL_PARAMS:
      if (channel < hic_profile_get_channels()) {
        channelInfo = hic_profile_get_channel_info(channel);
        response[0] = channel;
        // attributes: [1:0] data size (0=byte, 1=short, 2=word)
        //             [3:2] num ranges (0=1, 1=2, 2=4)
        //             [4] DT_TimeStamp
        response[1] = (len_to_flag(channelInfo->dataSize) << 0U) |
                      (len_to_flag(channelInfo->rangeCount) << 2U) |
                      (1U << 4U);
        // bits of data
        response[2] = channelInfo->dataBits;
        // ID string
        memcpy(&response[3], channelInfo->idString, sizeof(uint64_t));
        response += sizeof(uint64_t);
        // units string
        memcpy(&response[7], channelInfo->unitString, sizeof(uint64_t));
        response += sizeof(uint64_t);
        // compression
        response[11] = 0U;
        // ranges
        for (uint32_t i = 0; i < channelInfo->rangeCount; ++i) {
          memcpy(&response[12 + 8 * i], channelInfo->ranges[i].minVal, sizeof(float));
          response += sizeof(float);
          memcpy(&response[16 + 8 * i], channelInfo->ranges[i].maxVal, sizeof(float));
          response += sizeof(float);
        }
        result = 12 + 8 * channelInfo->rangeCount;
      }
      break;
    case DAP_TI_GET_CHANNEL_FREQS:
      if (channel < hic_profile_get_channels()) {
        channelInfo = hic_profile_get_channel_info(channel);
        response[0] = channel;
        response[1] = channelInfo->freqCount;
        for (uint32_t i = 0; i < channelInfo->freqCount; ++i) {
          memcpy(&response[2 + 4 * i], &channelInfo->freqs[i], sizeof(uint32_t));
        }
      }
      break;
    default:
      result = 0U;
  }

  return ((2U << 16) | result);
}

// Process TI Info command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
uint32_t TI_Value (const uint8_t *request, uint8_t *response) {
  *response = DAP_OK;

  return ((0U << 16) | 1U);
}

// Process TI Info command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
uint32_t TI_Capture (const uint8_t *request, uint8_t *response) {
  *response = DAP_OK;

  return ((0U << 16) | 1U);
}

// Process TI Info command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
uint32_t TI_TransferBlock (const uint8_t *request, uint8_t *response) {
  *response = DAP_OK;

  return ((0U << 16) | 1U);
}

#endif // (TRACE_DATA_BLOCK_COUNT != 0)

