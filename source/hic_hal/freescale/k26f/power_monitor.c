/**
 * @file    power_monitor.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2017 NXP
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

#include "power_monitor.h"
#include "tasks.h"
#include "target_reset.h"
#include "swd_host.h"
#include "debug_cm.h"
#include "DAP_Config.h"
#include "hic_profile.h"
#include "fsl_adc16.h"
#include "fsl_pit.h"
#include "fsl_clock.h"
#include <RTL.h>
#include <string.h>

// Declare ADC channel for Energy Monitor Circuit
#define ENERGY_BENCH_VAL_CHN                    (0x17U)
#define ENERGY_BENCH_CAL_CHN                    (0x16U)
// Declare Low Range Limit value; This value determines when the Range bit is flipped from low to high
#define LOW_RANGE_HIGH_LIMIT                    (3972U)
// Declare High Range Limit value; This value determines when the Range bit is flipped from high to low.
#define HIGH_RANGE_LOW_LIMIT                    (15U)

// #if VALIDATE_EM_CIRCUIT
// // Declare Array depth for ADC arrays
// #define ADC_ARRAY_DEPTH                                                 (100U)
// #else
// Declare Array depth for ADC arrays
#define ADC_ARRAY_DEPTH                                                 (300U)
#define CAL_AVERAGE_WEIGHT                                          (1000U)
// #endif

enum {
    kCurrentChannel = 0,
    kPCChannel = 1,
    kVoltageChannel = 2,
    kChannelCount = 3,
};

enum {
    kAdcMaxValue = 65535,
};

//! @brief Profiling thread event masks.
enum {
    kProfileStartFlag = 1,
};

//! @brief Value of the DWT_PCSR register returned when the CPU is halted.
enum {
    kHaltedPCSR = 0xffffffffu,
};

typedef enum gain_range {
    kLowRange,
    kHighRange,
} gain_range_t;

typedef enum calibration_resistor {
    kNoCalibrationResistor = 0,
    kHighRangeHighCurrent = PIN_CTRL0,
    kHighRangeLowCurrent = PIN_CTRL1,
    kLowRangeHighCurrent = PIN_CTRL2,
    kLowRangeLowCurrent = PIN_CTRL3,
} calibration_resistor_t;

typedef struct profile_channel_entry {
    uint32_t timestamp;
    uint32_t value;
} profile_channel_entry_t;

enum {
    kProfileEntries = TRACE_DATA_BLOCK_SIZE / sizeof(profile_channel_entry_t),
};

typedef struct profile_channel {
    profile_channel_entry_t data[kProfileEntries];
    int32_t head;    //!< Index where next entry will be written.
    int32_t count;   //!< Number of valid entries.
    profile_channel_entry_t lastReturned;
    bool isLastValid;
} profile_channel_t;

static U64 s_profilingThreadStack[PROFILING_TASK_STACK / sizeof(U64)];

typedef struct _profiling_state {
    OS_TID profilingTask;
    uint32_t busClock_MHz;
    gain_range_t currentRange;
    volatile uint32_t recordChannelMask;
    OS_SEM adcSem;
    profile_channel_t channels[kChannelCount];
} profiling_state_t;

static profiling_state_t s_state;

volatile bool g_profilingReadActive = false;
volatile bool g_profilingInterrupted = false;

// uint16_t gAdcResultArray1[ADC_ARRAY_DEPTH] = {0};
// #if !VALIDATE_EM_CIRCUIT
// uint16_t gAdcResultArray2[ADC_ARRAY_DEPTH] = {0};
// #endif

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

void calibration_enable(bool enable);
void calibration_select_resistor(calibration_resistor_t whichResistor);
void range_select(gain_range_t whichRange);

void channel_init(profile_channel_t *channel);
inline uint32_t channel_get_count(profile_channel_t *channel);
void channel_push(profile_channel_t *channel, const profile_channel_entry_t *entry);
bool channel_pop(profile_channel_t *channel, profile_channel_entry_t *entry);
uint32_t channel_read(profile_channel_t *channel, uint32_t maxLength, void * data);

void calibrate(void);

bool profiling_setup_pcsr_read(void);
bool profiling_read_pcsr(uint32_t * val);

__task void profiling_thread(void);

void profiling_init_adc(void);

void profiling_start_timestamp_timer(void);
uint32_t profiling_get_timestamp(void);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void calibration_enable(bool enable)
{
    enable
        ? (PIN_CAL_EN_GPIO->PSOR = PIN_CAL_EN)
        : (PIN_CAL_EN_GPIO->PCOR = PIN_CAL_EN);
}

void calibration_select_resistor(calibration_resistor_t whichResistor)
{
    PIN_CTRL0_GPIO->PCOR = PIN_CTRL0 | PIN_CTRL1 | PIN_CTRL2 | PIN_CTRL3;
    PIN_CTRL0_GPIO->PSOR = (uint32_t)whichResistor;
}

void range_select(gain_range_t whichRange)
{
    (whichRange == kLowRange)
        ? (PIN_LOW_RANGE_EN_GPIO->PSOR = PIN_LOW_RANGE_EN)
        : (PIN_LOW_RANGE_EN_GPIO->PCOR = PIN_LOW_RANGE_EN);
}

void channel_init(profile_channel_t *channel)
{
    channel->head = 0;
    channel->count = 0;
    channel->isLastValid = false;
}

inline uint32_t channel_get_count(profile_channel_t *channel)
{
    return channel->count;
}

void channel_push(profile_channel_t *channel, const profile_channel_entry_t *entry)
{
    if (channel->count < kProfileEntries)
    {
        ++channel->count;
    }
    channel->data[channel->head] = *entry;
    channel->head = (++(channel->head)) % kProfileEntries;
}

bool channel_pop(profile_channel_t *channel, profile_channel_entry_t *entry)
{
    if (channel->count == 0)
    {
        return false;
    }

    // Read oldest entry.
    int32_t readIndex = channel->head - channel->count;
    if (readIndex < 0)
    {
        readIndex = kProfileEntries + readIndex;
    }

    *entry = channel->data[readIndex];

    --channel->count;

    return true;
}

uint32_t channel_read(profile_channel_t *channel, uint32_t maxLength, void * data)
{
    uint8_t * dest = (uint8_t *)data;
    int32_t maxReadCount = maxLength / sizeof(profile_channel_entry_t);
    int32_t readCount = MIN(maxReadCount, channel->count);
    if (readCount == 0)
    {
        return 0;
    }

    // Figure out where we start reading from. Read the oldest data first.
    int32_t readIndex = channel->head - channel->count;
    if (readIndex < 0)
    {
        readIndex = kProfileEntries + readIndex;
    }

    int32_t remainingCount = readCount;
    while (remainingCount > 0)
    {
        int32_t thisReadCount = readCount;
        if (readIndex + thisReadCount >= kProfileEntries)
        {
            thisReadCount = kProfileEntries - readIndex;
        }
        uint32_t readSize = sizeof(profile_channel_entry_t) * thisReadCount;
        memcpy(dest, &channel->data[readIndex], readSize);
        dest += readSize;
        remainingCount -= thisReadCount;
        readIndex = (readIndex + thisReadCount) % kProfileEntries;
    }

    channel->count -= readCount;

    return sizeof(profile_channel_entry_t) * readCount;
}

bool profiling_setup_pcsr_read(void)
{
    swd_init_debug();
    // Turn off address increment.
    if (!swd_write_ap(AP_CSW, CSW_RESERVED | CSW_MSTRDBG | CSW_HPROT | CSW_DBGSTAT | CSW_NADDRINC | CSW_SIZE32))
    {
        swd_init_debug();
        return false;
    }

    uint8_t tmp_in[4];
    uint8_t req;

    // put addr in TAR register
    int2array(tmp_in, DWT_PCSR, 4);
    req = SWD_REG_AP | SWD_REG_W | SWD_REG_ADR(AP_TAR);
    if (swd_transfer_retry(req, (uint32_t *)tmp_in) != 0x01)
    {
//         swd_clear_errors();
        swd_init_debug();
        return false;
    }

    g_profilingInterrupted = false;
    return true;
}

bool profiling_read_pcsr(uint32_t * val)
{
    uint8_t tmp_out[4];
    uint8_t req;
    uint8_t ack;

    req = SWD_REG_AP | SWD_REG_R | (3 << 2);
    if (swd_transfer_retry(req, (uint32_t *)tmp_out) != 0x01)
    {
//         swd_clear_errors();
        swd_init_debug();
        g_profilingReadActive = false;
        return false;
    }

    // dummy read
    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = swd_transfer_retry(req, (uint32_t *)tmp_out);

    *val = (tmp_out[3] << 24) | (tmp_out[2] << 16) | (tmp_out[1] << 8) | tmp_out[0];

    bool ok = (ack == 0x01);
    if (!ok)
    {
        *val = ack;
//         swd_clear_errors();
        swd_init_debug();
    }

    return ok;
}

void profiling_init_adc(void)
{
    adc16_config_t adcConfig;
    ADC16_GetDefaultConfig(&adcConfig);
    adcConfig.enableAsynchronousClock = false;
    adcConfig.resolution = kADC16_Resolution16Bit;
    adcConfig.enableContinuousConversion = true;
    adcConfig.clockSource = kADC16_ClockSourceAlt1;
    adcConfig.clockDivider = kADC16_ClockDivider4;
    adcConfig.enableHighSpeed = true;
    adcConfig.longSampleMode = kADC16_LongSampleDisabled;
    adcConfig.enableLowPower = false;
    ADC16_Init(ADC0, &adcConfig);
//     ADC16_Init(ADC1, &adcConfig);

    ADC16_SetHardwareAverage(ADC0, kADC16_HardwareAverageCount32);
    ADC16_DoAutoCalibration(ADC0);
    ADC16_SetHardwareAverage(ADC0, kADC16_HardwareAverageDisabled);

//     ADC16_SetHardwareAverage(ADC1, kADC16_HardwareAverageCount32);
//     ADC16_DoAutoCalibration(ADC1);
//     ADC16_SetHardwareAverage(ADC1, kADC16_HardwareAverageDisabled);

    NVIC_EnableIRQ(ADC0_IRQn);
}

void calibrate(void)
{
#if 0
    float fCalAverage = 0;
    uint16_t ui16CalAverage = 0;
    uint32_t i = 0;
    uint32_t j = 0;

    while (j < 4)
    {
        CAL_PIN_ENABLE();

        for (j = 0; j < 4; j++)
        {

            // Need to clear the array for beginning of calibration routine.
            for (i = 0; i < ADC_ARRAY_DEPTH; i++)
            {
                gAdcResultArray1[i] = 0;
            }

            // Select CTRL0 pin
            SDA_CTRL0_PIN_DISABLE();
            SDA_CTRL1_PIN_DISABLE();
            SDA_CTRL2_PIN_DISABLE();
            SDA_CTRL3_PIN_DISABLE();

            switch (j) {
                case 0:
                    SDA_CTRL0_PIN_ENABLE();
                    RANGE_PIN_HIGH_RANGE();
                    //PIN_LOW_RANGE_EN_GPIO->PSOR = (1 << PIN_LOW_RANGE_EN_BIT);
                    break;
                case 1:
                    SDA_CTRL1_PIN_ENABLE();
                    RANGE_PIN_HIGH_RANGE();
                    //PIN_LOW_RANGE_EN_GPIO->PSOR = (1 << PIN_LOW_RANGE_EN_BIT);
                    break;
                case 2:
                    SDA_CTRL2_PIN_ENABLE();
                    RANGE_PIN_LOW_RANGE();
                    //PIN_LOW_RANGE_EN_GPIO->PSOR = (1 << PIN_LOW_RANGE_EN_BIT);
                    break;
                case 3:
                    SDA_CTRL3_PIN_ENABLE();
                    RANGE_PIN_LOW_RANGE();
                    //PIN_LOW_RANGE_EN_GPIO->PSOR = (1 << PIN_LOW_RANGE_EN_BIT);
                    break;
                default:
                    SDA_CTRL0_PIN_ENABLE();
                    RANGE_PIN_HIGH_RANGE();
                    //PIN_LOW_RANGE_EN_GPIO->PSOR = (1 << PIN_LOW_RANGE_EN_BIT);
                    break;
            }

            // Let Voltage settle
            os_sem_wait(&s_state.adcSem, 0xFFFF);

            // Need to set emIndex to 0 so that the array is filled properly
            emIndex = 0;
            arrayFull = 0;

            ADC0->SC1[0] = ADC_SC1_ADCH(ENERGY_BENCH_VAL_CHN) | ADC_SC1_AIEN_MASK;

            while (!arrayFull)
            {
                // Wait for ADC semaphore to be sent
                os_sem_wait(&s_state.adcSem, 0xFFFF);
            }
            arrayFull = 0;

            ADC0->SC1[0] = ADC_SC1_ADCH(ENERGY_BENCH_VAL_CHN) | ADC_SC1_AIEN_MASK;

            while (!arrayFull)
            {
                // Wait for ADC semaphore to be sent
                os_sem_wait(&s_state.adcSem, 0xFFFF);
            }
            arrayFull = 0;

            // Process ADC cal values
            //for(i=0; i<CAL_AVERAGE_WEIGHT; i++)
            //{
            //  fCalAverage += gAdcResultArray1[i];
            //}

            //fCalAverage /= (CAL_AVERAGE_WEIGHT * 1.0);

            //ui16CalAverage = (uint16_t)fCalAverage;

            //adc_process_result(ui16CalAverage);

//             for(i=0; i< ADC_ARRAY_DEPTH; i++)
//             {
//                 adc_process_result(gAdcResultArray1[i]);
//                 asciiArray[10] = adcResultAscii[0];
//                 asciiArray[11] = adcResultAscii[1];
//                 asciiArray[12] = adcResultAscii[2];
//                 asciiArray[13] = adcResultAscii[3];
//
// //                 if (usb_state == USB_CONNECTED)
// //                 {
// //                     USBD_CDC_ACM_DataSend(asciiArray, 16);
// //                 }
//
//             }
//
//             for(i=32; i< ADC_ARRAY_DEPTH; i++)
//             {
//                 adc_process_result(gAdcResultArray1[i]);
//                 asciiArray[10] = adcResultAscii[0];
//                 asciiArray[11] = adcResultAscii[1];
//                 asciiArray[12] = adcResultAscii[2];
//                 asciiArray[13] = adcResultAscii[3];
//
// //                 if (usb_state == USB_CONNECTED)
// //                 {
// //                     USBD_CDC_ACM_DataSend(asciiArray, 16);
// //                 }
//             }
        }
    }
#endif
}

void ADC0_IRQHandler(void)
{
    uint32_t result = ADC0->R[0];

// #if 0 //VALIDATE_EM_CIRCUIT
//     if (emIndex >= ADC_ARRAY_DEPTH)
//     {
//         arrayFull = 1;
//         ADC0->SC1[0] = 0;
//     }
//     else
//     {
//         gAdcResultArray1[emIndex] = gAdcResult;
//         emIndex++;
//     }
// #else
//     if(activeArray == 2)
//     {
//         gAdcResultArray2[emIndex] = gAdcResult;
//     }
//     else if(activeArray == 1)
//     {
//         gAdcResultArray1[emIndex] = gAdcResult;
//     }
//     else
//     {
//         activeArray = 1;
//         emIndex = 0;
//         gAdcResultArray1[emIndex] = gAdcResult;
//     }

    if (s_state.currentRange == kHighRange)
    {
        // Check result -- If less than 150 uA, switch to range = 0 (drive pin high
        //   for low range.
        //  If greater than 200 uA, switch to range = 1 (drive pin low for high range)
        //
        //  If I understand the I->V circuit correctly, in high range mode,
        //   150 uA is approximately 15 counts.  In low range mode, 200 uA is
        //   3972 counts (assuming 3.3V reference.  So simply use these as limits
        //   of when to switch.
        if (result < HIGH_RANGE_LOW_LIMIT)
        {
            // Need to drive pin high here to make sure we're in low range
            range_select(kLowRange);
            // todo: changing the range will probably mess up the next sample
        }
    }
    else
    {
        // Check result -- If less than 150 uA, switch to range = 0 (drive pin high
        //   for low range.
        //  If greater than 200 uA, switch to range = 1 (drive pin low for high range)
        //
        //  If I understand the I->V circuit correctly, in high range mode,
        //   150 uA is approximately 15 counts.  In low range mode, 200 uA is
        //   3972 counts (assuming 3.3V reference.  So simply use these as limits
        //   of when to switch.
        if (result > LOW_RANGE_HIGH_LIMIT)
        {
            // Need to drive pin low here to make sure we're in high range
            range_select(kHighRange);
            // todo: changing the range will probably mess up the next sample

            // Ignore this sample.
            return;
        }
    }

    // Save sample.
    profile_channel_entry_t entry;
    entry.timestamp = profiling_get_timestamp();
    entry.value = result;
    channel_push(&s_state.channels[kCurrentChannel], &entry);

//     emIndex++;
//     if (emIndex >= ADC_ARRAY_DEPTH)
//     {
//         arrayFull = 1;
//         // Once emIndex gets to greater than or equal to the Array depth,
//         //  need to switch the active array.
//         if (activeArray == 1)
//             activeArray = 2;
//         else
//             activeArray = 1;
//         // Also need to send the semaphore so the array will be processed.
//         //os_sem_send(&s_state.adcSem);
//         // Reset emIndex so we don't write past the arrays limits.
//         emIndex = 0;
//     }
// #endif
}

__task void profiling_thread(void)
{
//     uint16_t currIndex = 0;
//     uint32_t i = 0;
//     uint32_t j = 0;

    while (true)
    {
        // Wait for start flag
        os_evt_wait_or(kProfileStartFlag, NO_TIMEOUT);

        // Start continuous conversions.
        ADC0->SC1[0] = ADC_SC1_ADCH(ENERGY_BENCH_VAL_CHN) | ADC_SC1_AIEN_MASK;

        while (s_state.recordChannelMask)
        {
        }

        // Stop continuous conversions.
        ADC0->SC1[0] = 0x1f;
    }
}

uint32_t hic_profile_control(profile_control_command_t command, uint32_t value)
{
    switch (command) {
        case kProfileStartChannels:
            channel_init(&s_state.channels[kCurrentChannel]);
            if (s_state.recordChannelMask == 0)
            {
                s_state.recordChannelMask |= value;
                os_evt_set(kProfileStartFlag, s_state.profilingTask);
            }
            else
            {
                s_state.recordChannelMask |= value;
            }
            break;

        case kProfileStopChannels:
            s_state.recordChannelMask &= ~value;
            break;

        case kProfileSetFrequency:
            break;

        default:
            break;
    }

    return 0;
}

uint32_t hic_profile_read_data(uint32_t maxLength, void * data)
{
    return 0;
}

void profiling_start_timestamp_timer(void)
{
    // Setup PIT interrupt.
//     PIT_HAL_ClearIntFlag(PIT_BASE, 0);
//     NVIC_ClearPendingIRQ(PIT0_IRQn);
//     NVIC_EnableIRQ(PIT0_IRQn);
//     PIT_HAL_SetIntCmd(PIT_BASE, 0, true);

    s_state.busClock_MHz = CLOCK_GetBusClkFreq() / 1000000;

    pit_config_t pitConfig;
    PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT, &pitConfig);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, 0xffffffff);
    PIT_StartTimer(PIT, kPIT_Chnl_0);
}

uint32_t profiling_get_timestamp(void)
{
//     uint32_t low = ~PIT_HAL_ReadTimerCount(PIT_BASE, 0);
//     uint64_t ts = (uint64_t(s_timestamp_upper_bits) << 32) | low;
//     ts /= s_microsecondDivider;
//     return uint32_t(ts);
    return (~PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_0)) / s_state.busClock_MHz;
}

void PIT0_IRQHandler(void)
{
//     ++s_timestamp_upper_bits;

//     PIT_HAL_ClearIntFlag(PIT_BASE, 0);
}

void profiling_init(void)
{
    // Switch power monitor pins to outputs, driven low by default.
    PIN_LOW_RANGE_EN_GPIO->PCOR = PIN_LOW_RANGE_EN;
    PIN_LOW_RANGE_EN_GPIO->PDDR |= PIN_LOW_RANGE_EN;
    PIN_CAL_EN_GPIO->PCOR = PIN_CAL_EN;
    PIN_CAL_EN_GPIO->PDDR |= PIN_CAL_EN;
    PIN_G1_GPIO->PCOR = PIN_G1;
    PIN_G1_GPIO->PDDR |= PIN_G1;
    PIN_G2_GPIO->PCOR = PIN_G2;
    PIN_G2_GPIO->PDDR |= PIN_G2;

    range_select(kLowRange);
    s_state.recordChannelMask = false;
    channel_init(&s_state.channels[kCurrentChannel]);
    channel_init(&s_state.channels[kPCChannel]);
    channel_init(&s_state.channels[kVoltageChannel]);

    profiling_start_timestamp_timer();
    profiling_init_adc();

    os_sem_init(s_state.adcSem, 0);
    s_state.profilingTask = os_tsk_create_user(profiling_thread, PROFILING_TASK_PRIORITY, (void *)s_profilingThreadStack, PROFILING_TASK_STACK);
}

//! Definition of profile channels.
const profile_channel_params_t g_profileChannelInfo[] = {
            // Channel 0: current measurement
            {
                0, // channelNumber
                "current", // idString
                "µA", // unitString
                sizeof(uint16_t), // dataSize
                16, // dataBits
                1, // rangeCount
                2, // freqCount
                // ranges
                {
                    { 0.0f, 20000.0f }, // 0-20 mA in µA
                },
                // freqs
                {
                    100000, // 100 kHz
                    200000, // 200 kHz
                }
            },
            // Channel 1: PC sample
            {
                1, // channelNumber
                "PC", // idString
                "", // unitString
                sizeof(uint32_t), // dataSize
                32, // dataBits
                0, // rangeCount
                2, // freqCount
                // ranges
                {
                },
                // freqs
                {
                    100000, // 100 kHz
                    200000, // 200 kHz
                }
            },
            // Channel 2: voltage measurement
            {
                2, // channelNumber
                "voltage", // idString
                "mV", // unitString
                sizeof(uint16_t), // dataSize
                16, // dataBits
                1, // rangeCount
                0, // freqCount
                // ranges
                {
                    { 0.0f, 5000.0f }, // 0-5V in mV
                },
                // freqs
                {
                }
            }
        };

enum {
    kProfileChannelCount = ARRAY_SIZE(g_profileChannelInfo),
};

uint32_t hic_profile_get_channels(void)
{
    return kProfileChannelCount;
}

const profile_channel_params_t * hic_profile_get_channel_info(uint32_t channel)
{
    if (channel >= kProfileChannelCount)
    {
        return NULL;
    }
    return &g_profileChannelInfo[channel];
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
