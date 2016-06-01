/**
 * @file    usbd_MK20D5.c
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

#include "RTL.h"
#include "rl_usb.h"
#include "cortex_m.h"
#include "util.h"
#include "usb_device_config.h"
#include "fsl_usb.h"
#include "usb_device.h"
#include "usb_device_dci.h"
#include "usb_device_ehci.h"
#include "fsl_clock.h"
#include <string.h>

#define EP_COUNT (8)

typedef struct ep_info {
    OS_SEM readSem;
    uint32_t actualBytesRead;
} ep_info_t;

static usb_device_handle s_handle;
static ep_info_t s_ep[EP_COUNT];
U32 LastError;                          /* Last Error                         */

usb_status_t usb_device_callback(usb_device_handle handle, uint32_t callbackEvent, void *eventParam);
usb_status_t usb_device_endpoint_callback(usb_device_handle handle,
                                           usb_device_endpoint_callback_message_struct_t *message,
                                           void *callbackParam);

usb_status_t usb_device_control_endpoint_callback(usb_device_handle handle,
                                           usb_device_endpoint_callback_message_struct_t *message,
                                           void *callbackParam)
{
    ep_info_t *ep = &s_ep[0]; //(uint32_t)callbackParam & ~USB_ENDPOINT_DIRECTION_MASK];
    ep->actualBytesRead = message->length;
//     os_sem_send(&ep->readSem);
    return kStatus_USB_Success;
}

usb_status_t USB_DeviceControlPipeInit()
{
    usb_device_endpoint_init_struct_t epInitStruct;
    usb_device_endpoint_callback_struct_t endpointCallback;
    usb_status_t error;

    endpointCallback.callbackFn = usb_device_control_endpoint_callback;
    endpointCallback.callbackParam = (void *)0;

    epInitStruct.zlt = 1U;
    epInitStruct.transferType = USB_ENDPOINT_CONTROL;
    epInitStruct.endpointAddress = USB_CONTROL_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
    epInitStruct.maxPacketSize = USB_CONTROL_MAX_PACKET_SIZE;
    /* Initialize the control IN pipe */
    error = USB_DeviceInitEndpoint(s_handle, &epInitStruct, &endpointCallback);

    if (kStatus_USB_Success != error)
    {
        return error;
    }
    epInitStruct.endpointAddress = USB_CONTROL_ENDPOINT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
    /* Initialize the control OUT pipe */
    error = USB_DeviceInitEndpoint(s_handle, &epInitStruct, &endpointCallback);

    if (kStatus_USB_Success != error)
    {
        USB_DeviceDeinitEndpoint(s_handle,
                                 USB_CONTROL_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
        return error;
    }

    return kStatus_USB_Success;
}

usb_status_t usb_device_callback(usb_device_handle handle, uint32_t callbackEvent, void *eventParam)
{
    switch (callbackEvent)
    {
        case kUSB_DeviceEventBusReset:
        {
            uint8_t speed;
            USB_DeviceGetStatus(handle, kUSB_DeviceStatusSpeed, &speed);
            USBD_HighSpeed = (speed == USB_SPEED_HIGH);

            usbd_reset_core();

            // Setup EP0.
            USB_DeviceControlPipeInit();

#ifdef __RTX
            if (USBD_RTX_DevTask) {
                isr_evt_set(USBD_EVT_RESET, USBD_RTX_DevTask);
            }
#else
            if (USBD_P_Reset_Event) {
                USBD_P_Reset_Event();
            }
#endif
            break;
        }

        case kUSB_DeviceEventSuspend:
#ifdef __RTX
            if (USBD_RTX_DevTask) {
                isr_evt_set(USBD_EVT_SUSPEND, USBD_RTX_DevTask);
            }
#else
            if (USBD_P_Suspend_Event) {
                USBD_P_Suspend_Event();
            }
#endif
            break;

        case kUSB_DeviceEventResume:
#ifdef __RTX
            if (USBD_RTX_DevTask) {
                isr_evt_set(USBD_EVT_RESUME, USBD_RTX_DevTask);
            }
#else
            if (USBD_P_Resume_Event) {
                USBD_P_Resume_Event();
            }
#endif
            break;

        case kUSB_DeviceEventError:
            LastError = 1;
#ifdef __RTX
            if (USBD_RTX_DevTask) {
                isr_evt_set(USBD_EVT_ERROR, USBD_RTX_DevTask);
            }
#else
            if (USBD_P_Error_Event) {
                USBD_P_Error_Event(LastError);
            }
#endif
            break;

        case kUSB_DeviceEventDetach:
        case kUSB_DeviceEventAttach:
        case kUSB_DeviceEventSetConfiguration:
        case kUSB_DeviceEventSetInterface:
            break;
    }
    return kStatus_USB_Success;
}

usb_status_t usb_device_endpoint_callback(usb_device_handle handle,
                                           usb_device_endpoint_callback_message_struct_t *message,
                                           void *callbackParam)
{
    ep_info_t *ep = &s_ep[(uint32_t)callbackParam & ~USB_ENDPOINT_DIRECTION_MASK];
    ep->actualBytesRead = message->length;
    os_sem_send(&ep->readSem);
    return kStatus_USB_Success;
}

/*
 *  USB Device Interrupt enable
 *   Called by USBD_Init to enable the USB Interrupt
 *    Return Value:    None
 */

void USBD_IntrEna(void)
{
    NVIC_EnableIRQ(USBHS_IRQn);            /* Enable OTG interrupt */
}


/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB
 *   Return Value:    None
 */

void USBD_Init(void)
{
    memset(s_ep, 0, sizeof(s_ep));
    USB_DeviceInit(kUSB_ControllerEhci0, usb_device_callback, &s_handle);
    NVIC_EnableIRQ(USBHS_IRQn);
}


/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USBD_Connect(uint32_t con)
{
    if (con) {
        USB_DeviceRun(s_handle);
    } else {
        USB_DeviceStop(s_handle);
    }
}


/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */

void USBD_Reset(void)
{
}


/*
 *  USB Device Suspend Function
 *   Called automatically on USB Device Suspend
 *    Return Value:    None
 */

void USBD_Suspend(void)
{
//     USB_DeviceSetStatus(s_handle, kUSB_DeviceStatusBusSuspend, NULL);
}


/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */

void USBD_Resume(void)
{
//     USB_DeviceSetStatus(s_handle, kUSB_DeviceStatusBusResume, NULL);
}


/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */

void USBD_WakeUp(void)
{
//     uint32_t i = 50000;
//
//     if (USBD_DeviceStatus & USB_GETSTATUS_REMOTE_WAKEUP) {
//         USB0->CTL |=  USB_CTL_RESUME_MASK;
//
//         while (i--) {
//             __nop();
//         }
//
//         USB0->CTL &= ~USB_CTL_RESUME_MASK;
//     }
}


/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */

void USBD_WakeUpCfg(uint32_t cfg)
{
    /* Not needed                                                               */
}


/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *    Return Value:    None
 */

void USBD_SetAddress(uint32_t  adr, uint32_t setup)
{
    uint8_t address = adr;
    USB_DeviceSetStatus(s_handle, kUSB_DeviceStatusAddress, &address);
}


/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */

void USBD_Configure(uint32_t cfg)
{
}

/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */
void USBD_ConfigEP(USB_ENDPOINT_DESCRIPTOR *pEPD)
{
    ep_info_t *ep = &s_ep[pEPD->bEndpointAddress & ~USB_ENDPOINT_DIRECTION_MASK];
    os_sem_init(&ep->readSem, 0);
    ep->actualBytesRead = 0;

    usb_device_endpoint_init_struct_t epInit;
    epInit.maxPacketSize = pEPD->wMaxPacketSize;
    epInit.endpointAddress = pEPD->bEndpointAddress;
    epInit.transferType = pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK;
    epInit.zlt = 1;

    usb_device_endpoint_callback_struct_t callback;
    callback.callbackFn = usb_device_endpoint_callback;
    callback.callbackParam = (void *)pEPD->bEndpointAddress;
    USB_DeviceInitEndpoint(s_handle, &epInit, &callback);

//     USBD_ResetEP(num);
}


/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USBD_DirCtrlEP(uint32_t dir)
{
    /* Not needed                                                               */
}


/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP(uint32_t EPNum)
{
}


/*
 *  Disable USB Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP(uint32_t EPNum)
{
}


/*
 *  Reset USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP(uint32_t EPNum)
{
}

/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP(uint32_t EPNum)
{
    USB_DeviceStallEndpoint(s_handle, EPNum);
}


/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP(uint32_t EPNum)
{
    USB_DeviceUnstallEndpoint(s_handle, EPNum);
}


/*
 *  Clear USB Device Endpoint Buffer
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClearEPBuf(uint32_t EPNum)
{
}

/*
 *  Read USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *    Return Value:    Number of bytes read
 */

uint32_t USBD_ReadEP(uint32_t EPNum, uint8_t *pData, uint32_t size)
{
    ep_info_t *ep = &s_ep[EPNum & ~USB_ENDPOINT_DIRECTION_MASK];

    USB_DeviceRecvRequest(s_handle, EPNum, pData, size);

    os_sem_wait(&ep->readSem, 0xffff);

    return ep->actualBytesRead;
}


/*
 *  Write USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */

uint32_t USBD_WriteEP(uint32_t EPNum, uint8_t *pData, uint32_t cnt)
{
    USB_DeviceSendRequest(s_handle, EPNum, pData, cnt);

    return (cnt);
}

/*
 *  Get USB Device Last Frame Number
 *    Parameters:      None
 *    Return Value:    Frame Number
 */

uint32_t USBD_GetFrame(void)
{
//     USB_DeviceGetStatus(s_handle, kUSB_DeviceStatusSynchFrame, void *param)
    return 0;
}

#ifdef __RTX

/*
 *  Get USB Device Last Error Code
 *    Parameters:      None
 *    Return Value:    Error Code
 */

U32 USBD_GetError(void)
{
    return (LastError);
}
#endif


/*
 *  USB Device Interrupt Service Routine
 */
void USBHS_IRQHandler(void)
{
    NVIC_DisableIRQ(USBHS_IRQn);
    USBD_SignalHandler();
}

/*
 *  USB Device Service Routine
 */

void USBD_Handler(void)
{
    USB_DeviceEhciIsrFunction(s_handle);
    NVIC_EnableIRQ(USBHS_IRQn);
}

