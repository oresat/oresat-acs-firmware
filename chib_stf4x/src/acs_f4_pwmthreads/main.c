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

// PWM and PWMConfig structure
// http://chibios.sourceforge.net/html/struct_p_w_m_config.html

static void pwmp1cb(PWMDriver *pwmp) { // period call back

  (void)pwmp;
  palClearPad(GPIOA, GPIOA_LED_GREEN);
}

static void pwmc1cb(PWMDriver *pwmp) { // channel 1 callback

  (void)pwmp;
  palSetPad(GPIOA, GPIOA_LED_GREEN);
}

static PWMConfig pwmcfg1 = {
  10000,                                    /* 10kHz PWM clock frequency.   */
  10000,                                    /* Initial PWM period 1S.       */
  pwmp1cb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, pwmc1cb},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

static void pwmp2cb(PWMDriver *pwmp) { // period call back

  (void)pwmp;
  palClearPad(GPIOA, GPIOA_LED_GREEN);
}

static void pwmc2cb(PWMDriver *pwmp) { // channel 1 callback

  (void)pwmp;
  palSetPad(GPIOA, GPIOA_LED_GREEN);
}

static PWMConfig pwmcfg2 = {
  10000,                                    /* 10kHz PWM clock frequency.   */
  10000,                                    /* Initial PWM period 1S.       */
  pwmp2cb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, pwmc2cb},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

static void pwm1(void){
	pwmStart(&PWMD1, &pwmcfg1);
  pwmEnablePeriodicNotification(&PWMD1);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
  chThdSleepMilliseconds(2000);

  /*
   * Starts the PWM channel 0 using 75% duty cycle.
   */
  //pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 7500));
  //pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 7500));
//	pwmChangePeriod (&PWMD1, 5000);
  pwmEnableChannelNotification(&PWMD1, 0);
//  chThdSleepMilliseconds(5000);
  /*
   * Changes PWM period to half second the duty cycle becomes 50%
   * implicitly.
   */
//  pwmChangePeriod(&PWMD1, 5000);
  chThdSleepMilliseconds(5000);

  /*
   * Disables channel 0 and stops the drivers.
   */
//  pwmDisableChannel(&PWMD1, 0);
//  pwmStop(&PWMD1);
//  icuStopCapture(&ICUD3);
//  icuStop(&ICUD3);
//////////////  palClearPad(GPIOD, GPIOD_PIN4);
//  palClearPad(GPIOD, GPIOD_PIN5);


//  while (true) {
//    chThdSleepMilliseconds(500);
// }
}

static void pwm2(void){
	pwmStart(&PWMD1, &pwmcfg2);
  pwmEnablePeriodicNotification(&PWMD1);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
  chThdSleepMilliseconds(2000);

  /*
   * Starts the PWM channel 0 using 75% duty cycle.
   */
  //pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 7500));
  //pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 7500));
//	pwmChangePeriod (&PWMD1, 5000);
  pwmEnableChannelNotification(&PWMD1, 0);
  chThdSleepMilliseconds(5000);
}

static THD_WORKING_AREA(pwmThread1_wa, 128);
static THD_FUNCTION(pwmThread1, arg) {
  (void)arg;
  chRegSetThreadName("pwm");
	pwm1();
	pwm2();
}

int main(void) {
  halInit(); // http://chibios.sourceforge.net/html/group___h_a_l.html
  chSysInit(); // http://chibios.sourceforge.net/html/group__system.html

	chThdCreateStatic(pwmThread1, sizeof(pwmThread1_wa), NORMALPRIO, pwmThread1, NULL);
  while (true) {
    chThdSleepMilliseconds(500);
  }
  return 0;
}
