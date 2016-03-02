/**
 * @file BLE.c
 * @breaf Cerevo CDP-TZ01B Eddystone beacon.
 * BLE
 *
 * @author Cerevo Inc.
 */

/*
Copyright 2015 Cerevo Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <stdio.h>
#include "TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "GPIO_TZ10xx.h"
#include "RNG_TZ10xx.h"
#include "RTC_TZ10xx.h"
#include "Driver_UART.h"

#include "twic_interface.h"
#include "blelib.h"

#include "TZ01_system.h"
#include "TZ01_console.h"

extern TZ10XX_DRIVER_PMU  Driver_PMU;
extern TZ10XX_DRIVER_RNG  Driver_RNG;
extern TZ10XX_DRIVER_RTC  Driver_RTC;
extern TZ10XX_DRIVER_GPIO Driver_GPIO;
extern ARM_DRIVER_UART Driver_UART1;

static uint8_t msg[80];

static uint64_t hrgn_bdaddr  = 0xc00100000000;   //

static void init_io_state(void);

/*--- GATT profile definition ---*/
uint8_t bnmsg_gap_device_name[] = "CDP-TZ01B";
uint8_t bnmsg_gap_appearance[] = {0x00, 0x00};

/* BLElib unique id. */
enum {
    GATT_UID_GAP_SERVICE = 0,
    GATT_UID_GAP_DEVICE_NAME,
    GATT_UID_GAP_APPEARANCE,
};

/* GAP */
const BLELib_Characteristics gap_device_name = {
    GATT_UID_GAP_DEVICE_NAME, 0x2a00, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ | BLELIB_PERMISSION_WRITE,
    bnmsg_gap_device_name, sizeof(bnmsg_gap_device_name),
    NULL, 0
};
const BLELib_Characteristics gap_appearance = {
    GATT_UID_GAP_APPEARANCE, 0x2a01, 0, BLELIB_UUID_16,
    BLELIB_PROPERTY_READ,
    BLELIB_PERMISSION_READ,
    bnmsg_gap_appearance, sizeof(bnmsg_gap_appearance),
    NULL, 0
};
const BLELib_Characteristics *const gap_characteristics[] = { &gap_device_name, &gap_appearance };
const BLELib_Service gap_service = {
    GATT_UID_GAP_SERVICE, 0x1800, 0, BLELIB_UUID_16,
    true, NULL, 0,
    gap_characteristics, 2
};

/* Service list */
const BLELib_Service *const hrgn_service_list[] = {
    &gap_service
};

/*- INDICATION data -*/
uint8_t bnmsg_advertising_data[] = {
    0x02, /* length of this data */
    0x01, /* AD type = Flags */
    0x06, /* LE General Discoverable Mode = 0x02 */
    /* BR/EDR Not Supported (i.e. bit 37
     * of LMP Extended Feature bits Page 0) = 0x04 */

    /* Eddystone(https://github.com/google/eddystone/blob/master/protocol-specification.md) */
    0x03, /* length of this data */
    0x03, /* AD type = Complete list of 16-bit UUIDs available */
    0xAA, /* Eddystone service FEAA */
    0xFE,
    
#if 0
    0x17, /* length of this data */
    0x16, /* AD type = Service Data type value */
    0xAA, /* Eddystone service FEAA */
    0xFE,
    /* Eddystone-UID(https://github.com/google/eddystone/tree/master/eddystone-uid) */
    0x00, /* Frame Type: UID */
    0xE7, /* Ranging Data */
    0x16, 0xF7, 0x42, 0xE6, 0xA8, 0x8C, 0x17, 0x5B, 0xA3, 0x22, /* Namespace:MSB 10Bytes of SHA1("blueninja.cerevo.com") */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, /* Instance */
    0x00, /* Reserved */
    0x00, /* Reserved */
#else
    0x14, /* length of this data */
    0x16, /* AD type = Service Data type value */
    0xAA, /* Eddystone service FEAA */
    0xFE,
    /* Eddystone-URL(https://github.com/google/eddystone/tree/master/eddystone-url) */
    0x10, /* Frame Type: URL */
    0xE7, /* Ranging Data */
    0x03, /* URL Scheme: https:// */
    'b', 'n', 'i', 'n', 'j', 'a', '.', 'c', 'e', 'r', 'e', 'v', 'o', 0x07,
#endif
};

uint8_t bnmsg_scan_resp_data[] = {
    0x02, /* length of this data */
    0x01, /* AD type = Flags */
    0x06, /* LE General Discoverable Mode = 0x02 */
    /* BR/EDR Not Supported (i.e. bit 37
     * of LMP Extended Feature bits Page 0) = 0x04 */

    0x02, /* length of this data */
    0x0A, /* AD type = TX Power Level (1 byte) */
    0x00, /* 0dB (-127...127 = 0x81...0x7F) */
};

static void periodic_callback(RTC_EVENT e)
{
    /* RTCäÑÇËçûÇ›ÉnÉìÉhÉâ */
}

static tz1em_t tz1em_eddystone;
static bool init_tz1em(void)
{
    tz1emRequirement_t gp;
    
    if (tz1emInitializeEntry(&tz1em_eddystone) != TZ1EM_STATUS_OK) {
        return false;
    }
    
    gp.pcd = TZ1EM_PCDG_URT2;
    gp.mode = TZ1EM_OP_WAIT_RETENTION;
    gp.sunshine_vf = TZ1EM_VF_UN;
    gp.wakeup = NULL;
    gp.permissible_time_lag_us = TZ1EM_PTL_SAFE_ASYNCHRONOUS_IO;
    
    gp.trigger[0].event = TZ1EM_WE_EP;
    gp.trigger[0].factor = TZ1EM_WF_RTC;
    gp.trigger[1].event = TZ1EM_WE_OFF;
    gp.trigger[1].factor = TZ1EM_WF_UN;
    gp.trigger[2].event = TZ1EM_WE_OFF;
    gp.trigger[2].factor = TZ1EM_WF_UN;
    if (TZ1EM_STATUS_OK != tz1emConfigureEntry(&tz1em_eddystone, &gp)) {
        return false;
    }

    return true;
}

/** Global **/

int BLE_init_dev(void)
{
    if (TZ1EM_STATUS_OK != tz1emInitializeSystem())
        return 1; /* Must not use UART for LOG before twicIfLeIoInitialize. */
        
    /* create random bdaddr */
    uint32_t randval;
    Driver_PMU.SetPowerDomainState(PMU_PD_ENCRYPT, PMU_PD_MODE_ON);
    Driver_RNG.Initialize();
    Driver_RNG.PowerControl(ARM_POWER_FULL);
    Driver_RNG.Read(&randval);
    Driver_RNG.Uninitialize();
    hrgn_bdaddr |= (uint64_t)randval;
    
    RTC_TIME now = {
        55, 33, 12, 8, 6, 15, 1 /* 15-06-08(Mon) 12:33:45 */
    };
    Driver_PMU.StartClockSource(PMU_CLOCK_SOURCE_OSC32K);
    Driver_PMU.SelectClockSource(PMU_CSM_RTC, PMU_CLOCK_SOURCE_OSC32K);
    Driver_PMU.SetPrescaler(PMU_CD_RTC, 1);
    Driver_RTC.Initialize();
    Driver_RTC.SetTime(&now); /* Set current date and time */

    Driver_RTC.SetPeriodicInterrupt(RTC_PERIOD_EVERY_SECOND, periodic_callback);

    if (init_tz1em() == false) {
        TZ01_console_puts("init_tz1em() failed\r\n");
        return 1;
    }

    return 0;
}

static bool is_adv = false;
static bool is_reg = false;
static uint8_t led_blink = 0;
static uint8_t cnt = 0;
extern twicConnIface_t* BLELib_Internal_ConnIface(void);

int BLE_main(void)
{
    int ret, res = 0;
    BLELib_State state;
    bool has_event;
    uint32_t pin;

    if (tz1emParticipateIn(&tz1em_eddystone) != TZ1EM_STATUS_OK) {
        return -1;
    }
    
    state = BLELib_getState();
    has_event = BLELib_hasEvent();

    switch (state) {
        case BLELIB_STATE_UNINITIALIZED:
            is_reg = false;
            is_adv = false;
            TZ01_console_puts("BLELIB_STATE_UNINITIALIZED\r\n");
            ret = BLELib_initialize(hrgn_bdaddr, BLELIB_BAUDRATE_2304, NULL, NULL, NULL, NULL);
            if (ret != BLELIB_OK) {
                TZ01_console_puts("BLELib_initialize() failed.\r\n");
                return -1;  //Initialize failed
            }
            break;
        case BLELIB_STATE_INITIALIZED:
            if (is_reg == false) {
                TZ01_console_puts("BLELIB_STATE_INITIALIZED\r\n");
                BLELib_setLowPowerMode(BLELIB_LOWPOWER_ON);
                if (BLELib_registerService(hrgn_service_list, 1) == BLELIB_OK) {
                    is_reg = true;
                } else {
                    return -1;  //Register failed
                }
            }
            break;
        case BLELIB_STATE_READY:
            if (is_adv == false) {
                TZ01_console_puts("BLELIB_STATE_READY\r\n");
                ret = BLELib_startAdvertising(bnmsg_advertising_data, sizeof(bnmsg_advertising_data), bnmsg_scan_resp_data, sizeof(bnmsg_scan_resp_data));
                if (ret == BLELIB_OK) {
                    is_adv = true;
                    cnt = 0;
                }
                
                if (twicIfLeCeLowPowerMode(BLELib_Internal_ConnIface(), true, true, false, false) != TWIC_STATUS_OK) {
                    TZ01_console_puts("twicIfLeCeLowPowerMode() failed.\r\n");
                }
            }
            break;
        case BLELIB_STATE_ADVERTISING:
            is_adv = false;
            break;
        default:
            break;
    }
    
    if (has_event) {
        ret = BLELib_run();
        if (ret != BLELIB_OK) {
            res = -1;
            sprintf(msg, "BLELib_run() ret: %d\r\n", ret);
            TZ01_console_puts(msg);
        }
    } else {
        while (!Driver_UART1.TxDone());
        Driver_GPIO.WritePin(11, 0);
        tz1emGoIntoTheShade(&tz1em_eddystone, true);
        Driver_GPIO.WritePin(11, 1);
        Driver_PMU.SelectClockSource(PMU_CSM_UART1, PMU_CLOCK_SOURCE_OSC12M);
        Driver_PMU.SetPrescaler(PMU_CD_UART1, 3);
    }

    return res;
}

void BLE_stop(void)
{
    switch (BLELib_getState()) {
        case BLELIB_STATE_ADVERTISING:
            BLELib_stopAdvertising();
            break;
        default:
            break;
    }
    BLELib_finalize();
}
