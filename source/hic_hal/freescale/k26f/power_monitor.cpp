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
#include <RTL.h>
#include <algorithm>
#include <string.h>

// Declare ADC semaphore init count
#define ADC_SEM_INIT_COUNT          0
// Declare ADC channel for Energy Monitor Circuit
#define ENERGY_BENCH_VAL_CHN                    (0x17U)
#define ENERGY_BENCH_CAL_CHN                                        (0x16U)
// Declare High Range Val for Energy Monitor circuit.
#define HIGH_RANGE_VAL                          (1U)
// Declare Low Range Val for Energy Monitor circuit.
#define LOW_RANGE_VAL                           (0U)
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

//! @brief Profiling thread event masks.
enum {
    kProfileStartFlag = 1,
};

//! @brief ADC channels.
enum {
    kLowSideAdcChannel = 4,         //!< Channel 4b
    kHighSideAdcChannel = 15,       //!< Channel 15
    kBandgapAdcChannel = 27         //!< Bandgap channel
};

//! @brief Value of the DWT_PCSR register returned when the CPU is halted.
const uint32_t kHaltedPCSR = 0xffffffffu;


/*
 * @brief Manages a circular buffer of profile data.
 */
class ProfileData
{
public:
    struct Entry
    {
        uint32_t timestamp;
        uint32_t pc;
        uint32_t milliamps;
    };

    ProfileData();

    void clear();
    void append(Entry & entry);
    uint32_t read(uint32_t maxLength, void * data);
    uint32_t get_count() { return m_count; }

protected:
    Entry m_data[kProfileEntries];
    int32_t m_head;    //!< Index where next entry will be written.
    int32_t m_count;   //!< Number of valid entries.
    Entry m_lastReturned;
    bool m_isLastValid;
};


static U64 stk_adc_task[ADC_TASK_STACK / sizeof(U64)];

gpio_led_state_t adc_led_value = GPIO_LED_ON;

uint16_t gAdcResult = 0;
uint16_t rangeBit = 0;
uint16_t emIndex = 0;
uint16_t activeArray = 1;
uint16_t arrayFull = 0;


uint16_t gAdcResultArray1[ADC_ARRAY_DEPTH] = {0};
#if !VALIDATE_EM_CIRCUIT
uint16_t gAdcResultArray2[ADC_ARRAY_DEPTH] = {0};
#endif

static OS_SEM adc_sem;

static uint32_t s_busClock_MHz = 0;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

ProfileData::ProfileData()
:   m_head(0),
    m_count(0),
    m_isLastValid(false)
{
}

void ProfileData::clear()
{
    m_count = 0;
    m_head = 0;
    m_isLastValid = false;
}

void ProfileData::append(Entry & entry)
{
    if (m_count < kProfileEntries)
    {
        ++m_count;
    }
    m_data[m_head] = entry;
    m_head = (++m_head) % kProfileEntries;
}

uint32_t ProfileData::read(uint32_t maxLength, void * data)
{
    uint8_t * dest = (uint8_t *)data;
    int32_t maxReadCount = maxLength / sizeof(Entry);
    int32_t readCount = std::min(maxReadCount, m_count);
    if (readCount == 0)
    {
        return 0;
    }

    // Figure out where we start reading from. Read the oldest data first.
    int32_t readIndex = m_head - m_count;
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
        uint32_t readSize = sizeof(Entry) * thisReadCount;
        memcpy(dest, &m_data[readIndex], readSize);
        dest += readSize;
        remainingCount -= thisReadCount;
        readIndex = (readIndex + thisReadCount) % kProfileEntries;
    }

    m_count -= readCount;

    return sizeof(Entry) * readCount;
}

bool profiling_setup_pcsr_read()
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
    NVIC_EnableIRQ(ADC0_IRQn);

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

    if (adcConfig.resolution == kADC16_Resolution8or9Bit)
    {
        gAdcRes = 256;
    }
    else if (adcConfig.resolution == kADC16_Resolution12or13Bit)
    {
        gAdcRes = 4096;
    }
    else if (adcConfig.resolution == kADC16_Resolution10or11Bit)
    {
        gAdcRes = 1024;
    }
    else
    {
        gAdcRes = 65536;
    }

    ADC16_SetHardwareAverage(ADC0, kADC16_HardwareAverageCount32);
    ADC16_DoAutoCalibration(ADC0);
    ADC16_SetHardwareAverage(ADC0, kADC16_HardwareAverageDisabled);

//     ADC16_SetHardwareAverage(ADC1, kADC16_HardwareAverageCount32);
//     ADC16_DoAutoCalibration(ADC1);
//     ADC16_SetHardwareAverage(ADC1, kADC16_HardwareAverageDisabled);
}

void calibrate(void)
{
#if 0
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
            os_sem_wait(&adc_sem, 0xFFFF);

            // Need to set emIndex to 0 so that the array is filled properly
            emIndex = 0;
            arrayFull = 0;

            ADC0->SC1[0] = ADC_SC1_ADCH(ENERGY_BENCH_VAL_CHN) | ADC_SC1_AIEN_MASK;

            while (!arrayFull)
            {
                // Wait for ADC semaphore to be sent
                os_sem_wait(&adc_sem, 0xFFFF);
            }
            arrayFull = 0;

            ADC0->SC1[0] = ADC_SC1_ADCH(ENERGY_BENCH_VAL_CHN) | ADC_SC1_AIEN_MASK;

            while (!arrayFull)
            {
                // Wait for ADC semaphore to be sent
                os_sem_wait(&adc_sem, 0xFFFF);
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

__task void profiling_thread(void)
{
    float gAdcRes = 0;
    status_t calStatus;
    uint16_t currIndex = 0;
    uint32_t i = 0;
    uint32_t j = 0;
    float fCalAverage = 0;
    uint16_t ui16CalAverage = 0;

    ADC0->SC1[0] = ADC_SC1_ADCH(ENERGY_BENCH_VAL_CHN) | ADC_SC1_AIEN_MASK;

    while (true)
    {
        // Wait for start flag
        os_evt_wait_or(kProfileStartFlag, NO_TIMEOUT);

        // Process DAP Command
        os_sem_wait(&adc_sem, 0xFFFF);

        // Semaphore is posted every time an array is filled.  So we need to process the
        // non-active array.
        if (arrayFull)
        {
            if (activeArray == 1)
            {
                for (currIndex = 0; currIndex < ADC_ARRAY_DEPTH; currIndex++)
                {
                    // Test to make sure ADC_Process_Result is working correctly.
                    //ADC_Process_Result(currIndex);
                    ADC_Process_Result(gAdcResultArray2[currIndex]);

                    if(usb_state == USB_CONNECTED)
                    {
                      USBD_CDC_ACM_DataSend(adcResultAcii, 9);
                    }
                }
              }
              else
              {
                  for (currIndex = 0; currIndex < ADC_ARRAY_DEPTH; currIndex++)
                  {
                        // Test to make sure ADC_Process_Result is working correctly.
                    //ADC_Process_Result(currIndex);
                    ADC_Process_Result(gAdcResultArray1[currIndex]);

                    if (usb_state == USB_CONNECTED)
                    {
                        USBD_CDC_ACM_DataSend(adcResultAcii, 9);
                    }
                }
            }

            arrayFull = 0;
        }
    }
}

void ADC0_IRQHandler(void)
{
    gAdcResult = ADC0->R[0];

#if 0 //VALIDATE_EM_CIRCUIT
    if (emIndex >= ADC_ARRAY_DEPTH)
    {
        arrayFull = 1;
        ADC0->SC1[0] = 0;
    }
    else
    {
        gAdcResultArray1[emIndex] = gAdcResult;
        emIndex++;
    }
#else
    if(activeArray == 2)
    {
        gAdcResultArray2[emIndex] = gAdcResult;
    }
    else if(activeArray == 1)
    {
        gAdcResultArray1[emIndex] = gAdcResult;
    }
    else
    {
        activeArray = 1;
        emIndex = 0;
        gAdcResultArray1[emIndex] = gAdcResult;
    }

    if(rangeBit == HIGH_RANGE_VAL)
    {
        // Check result -- If less than 150 uA, switch to range = 0 (drive pin high
        //   for low range.
        //  If greater than 200 uA, switch to range = 1 (drive pin low for high range)
        //
        //  If I understand the I->V circuit correctly, in high range mode,
        //   150 uA is approximately 15 counts.  In low range mode, 200 uA is
        //   3972 counts (assuming 3.3V reference.  So simply use these as limits
        //   of when to switch.
        if(gAdcResult < HIGH_RANGE_LOW_LIMIT)
        {
            // Need to drive pin high here to make sure we're in low range
            RANGE_PIN_LOW_RANGE();

            // Also store current range bit and switch range
            // Store the range bit.
            rangeBit = LOW_RANGE_VAL;
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
        if (gAdcResult > LOW_RANGE_HIGH_LIMIT)
        {
            // Need to drive pin low here to make sure we're in high range
            RANGE_PIN_HIGH_RANGE();

            // Also store current range bit before switching ranges
            // Store the range bit.
            rangeBit = HIGH_RANGE_VAL;
        }
    }

    emIndex++;
    if (emIndex >= ADC_ARRAY_DEPTH)
    {
        arrayFull = 1;
        // Once emIndex gets to greater than or equal to the Array depth,
        //  need to switch the active array.
        if(activeArray == 1)
            activeArray = 2;
        else
            activeArray = 1;
        // Also need to send the semaphore so the array will be processed.
        //os_sem_send(&adc_sem);
        // Reset emIndex so we don't write past the arrays limits.
        emIndex = 0;
    }
#endif
}

void profiling_start_timestamp_timer(void)
{
    // Setup PIT interrupt.
//     PIT_HAL_ClearIntFlag(PIT_BASE, 0);
//     NVIC_ClearPendingIRQ(PIT0_IRQn);
//     NVIC_EnableIRQ(PIT0_IRQn);
//     PIT_HAL_SetIntCmd(PIT_BASE, 0, true);

    s_busClock_MHz = CLOCK_GetBusClkFreq() / 1000000;

    pit_config_t pitConfig;
    PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT, &pitConfig);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, 0xffffffff);
    PIT_StartTimer(PIT, kPIT_Chnl_0);
}

uint32_t profiling_get_timestamp()
{
//     uint32_t low = ~PIT_HAL_ReadTimerCount(PIT_BASE, 0);
//     uint64_t ts = (uint64_t(s_timestamp_upper_bits) << 32) | low;
//     ts /= s_microsecondDivider;
//     return uint32_t(ts);
    return (~PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_0)) / s_busClock_MHz;
}

extern "C" void PIT0_IRQHandler(void)
{
    ++s_timestamp_upper_bits;

//     PIT_HAL_ClearIntFlag(PIT_BASE, 0);
}

void profiling_init(void)
{
    profiling_start_timestamp_timer();
    profiling_init_adc();
    os_tsk_create_user(adc_process, ADC_TASK_PRIORITY, (void *)stk_adc_task, ADC_TASK_STACK);
}

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
            }
            // Channel 1: PC sample
            {
                0, // channelNumber
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
            }
            // Channel 2: voltage measurement
            {
                0, // channelNumber
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

const uint32_t kProfileChannelCount = ARRAY_SIZE(g_profileChannelInfo);

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
    return g_profileChannelInfo[channel];
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
