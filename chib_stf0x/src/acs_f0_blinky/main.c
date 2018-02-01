/*! \file main.c
 *  app_blinky-just blink a light to test compilers, flash programming and serial terminal tools.
 *
 * Serial terminal setting are in SerialConfig structure.
 */

/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

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

/*!
 * \defgroup main app_blinky main
 *
 * @{
 */

#include <stdbool.h>
#include "ch.h"

#include "hal.h"
#include "chprintf.h"
#include "board.h"

#include "util_general.h"
#include "util_version.h"

static SerialConfig ser_cfg =
{
    115200,
    0,
    0,
    0,
};

#define DEBUG_SERIAL  SD2
#define DEBUG_CHP     ((BaseSequentialStream *) &DEBUG_SERIAL)

void set_status_led(bool g)
{
    if(g)
    {
        palSetLine(LINE_LED_GREEN);
    }
    else
    {
        palClearLine(LINE_LED_GREEN);
    }
}

static void app_init(void)
{
    // start up deboug output, chprintf(DEBUG_CHP,...)
    sdStart(&DEBUG_SERIAL, &ser_cfg);

    set_util_fwversion(&version_info);
    set_util_hwversion(&version_info);

    chprintf(DEBUG_CHP, "\r\n\r\n");
    chprintf(DEBUG_CHP, "FW HASH: %s\r\n", version_info.firmware);
    chprintf(DEBUG_CHP, "STF0x UNIQUE HW ID (H,C,L):\r\n0x%x\t0x%x\t0x%x\r\n"
             , version_info.hardware.id_high
             , version_info.hardware.id_center
             , version_info.hardware.id_low
            );
}

/*! \brief main application loop
 */
static void main_app(void)
{

    app_init();
    chprintf(DEBUG_CHP, "App start\r\n");
    set_status_led(true);

    uint8_t led_blank_count = 0;

    while (true)
    {
        if( (++led_blank_count) >= 5 )
        {
            set_status_led(false);

            // chprintf(DEBUG_CHP, "%d\r\n", led_blank_count);
            if(led_blank_count == 5 )
            {
                chprintf(DEBUG_CHP, "OFF\r\n", led_blank_count);
            }
            if(led_blank_count >= 10)
            {
                led_blank_count = 0;
            }
        }
        else
        {
            set_status_led(true);
            if(led_blank_count == 1 )
            {
                chprintf(DEBUG_CHP, "ON\r\n", led_blank_count);
            }
        }
        chThdSleepMilliseconds(300);
    }
}

int main(void)
{
    halInit();
    chSysInit();

    main_app();

    return(0);
}

//! @}

