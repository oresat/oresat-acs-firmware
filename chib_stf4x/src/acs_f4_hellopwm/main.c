/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

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

#include "ch.h"
#include "hal.h"

/*
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  chRegSetThreadName("pwm");
  while (true) {
    palClearPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(500);
  }
}
//*/


// PWM and PWMConfig structure
// http://chibios.sourceforge.net/html/struct_p_w_m_config.html

static void pwmpcb(PWMDriver *pwmp) { // period call back

  (void)pwmp;
  palClearPad(GPIOD, GPIOD_PIN5);
}

static void pwmc1cb(PWMDriver *pwmp) { // channel 1 callback

  (void)pwmp;
  palSetPad(GPIOD, GPIOD_PIN5);
}

static PWMConfig pwmcfg = {
  10000,                                    /* 10kHz PWM clock frequency.   */
  10000,                                    /* Initial PWM period 1S.       */
  pwmpcb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, pwmc1cb},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

// Input Capture Unit (ICU)
// http://chibios.sourceforge.net/html/group___i_c_u.html

icucnt_t last_width, last_period; // ICU counter type

static void icuwidthcb(ICUDriver *icup) { // ICU width callback

  palSetPad(GPIOD, GPIOD_PIN4);
  last_width = icuGetWidthX(icup);
}

static void icuperiodcb(ICUDriver *icup) { // ICU period callback

  palClearPad(GPIOD, GPIOD_PIN4);
  last_period = icuGetPeriodX(icup);
}

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_HIGH,
  10000,                                    /* 10kHz ICU clock frequency.   */
  icuwidthcb,
  icuperiodcb,
  NULL,
  ICU_CHANNEL_1,
  0
};

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit(); // http://chibios.sourceforge.net/html/group___h_a_l.html
  chSysInit(); // http://chibios.sourceforge.net/html/group__system.html

  /*
   * Initializes the PWM driver 2 and ICU driver 3.
   * GPIOA8 is the PWM output.
   * GPIOC6 is the ICU input.
   * The two pins have to be externally connected together.
   */

	// PWM driver: http://chibios.sourceforge.net/html/group___p_w_m.html
  pwmStart(&PWMD1, &pwmcfg);
  pwmEnablePeriodicNotification(&PWMD1);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
  icuStart(&ICUD3, &icucfg);
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(2));
  icuStartCapture(&ICUD3);
  icuEnableNotifications(&ICUD3);
  chThdSleepMilliseconds(2000);

  /*
   * Starts the PWM channel 0 using 75% duty cycle.
   */
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 7500));
  pwmEnableChannelNotification(&PWMD1, 0);
  chThdSleepMilliseconds(5000);

  /*
   * Changes the PWM channel 0 to 50% duty cycle.
   */
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
  chThdSleepMilliseconds(5000);

  /*
   * Changes the PWM channel 0 to 25% duty cycle.
   */
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 2500));
  chThdSleepMilliseconds(5000);

  /*
   * Changes PWM period to half second the duty cycle becomes 50%
   * implicitly.
   */
  pwmChangePeriod(&PWMD1, 5000);
  chThdSleepMilliseconds(5000);

  /*
   * Disables channel 0 and stops the drivers.
   */
  pwmDisableChannel(&PWMD1, 0);
  pwmStop(&PWMD1);
  icuStopCapture(&ICUD3);
  icuStop(&ICUD3);
  palClearPad(GPIOD, GPIOD_PIN4);
  palClearPad(GPIOD, GPIOD_PIN5);

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (true) {
    chThdSleepMilliseconds(500);
  }
  return 0;
}
