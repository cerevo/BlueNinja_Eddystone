/**
 * @file   main.c
 * @brief  Application main.
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
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "TZ10xx.h"
#include "PMU_TZ10xx.h"
#include "GPIO_TZ10xx.h"

#include "TZ01_system.h"
#include "TZ01_console.h"

#include "utils.h"
#include "ble.h"

extern TZ10XX_DRIVER_GPIO Driver_GPIO;

static void power_on(void)
{
    Driver_GPIO.WritePin(3, 1);
}

static void power_off(void)
{
    Driver_GPIO.WritePin(3, 0);
}

static bool check_uvd(void)
{
    uint32_t val;
    if (Driver_GPIO.ReadPin(4, &val) != GPIO_ERROR) {
        return (val == 0);
    } else {
        return false;
    }
}

static void init(void)
{
    //Power Hold
    Driver_GPIO.Configure(3, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);
    //UVDetect
    Driver_GPIO.Configure(4, GPIO_DIRECTION_INPUT_HI_Z, GPIO_EVENT_DISABLE, NULL);
    Driver_PMU.StandbyInputBuffer(PMU_IO_FUNC_GPIO_4, 0);   /* UVdetect */
    //LED
    Driver_GPIO.Configure(10, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);
    Driver_GPIO.Configure(11, GPIO_DIRECTION_OUTPUT_2MA, GPIO_EVENT_DISABLE, NULL);
    
    power_on();
}

int main(void)
{
    if (BLE_init_dev() == 1) {
        return 1;
    }
    /* Initialize */
    init();
    TZ01_system_init();
    TZ01_console_init();

    for (;;) {
        TZ01_system_run();
        if (check_uvd()) {
            Driver_GPIO.WritePin(10, 1);
        } else {
            Driver_GPIO.WritePin(10, 0);
        }
        BLE_main();
    }
term:
    BLE_stop();

    TZ01_console_puts("Program terminated.\r\n");
    return 0;
}
