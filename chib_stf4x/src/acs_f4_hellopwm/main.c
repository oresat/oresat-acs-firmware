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

static int cycle0 = 1000;
static int cycle1 = 1000;
static int cycle2 = 1000;

static void pwmpcb(PWMDriver *pwmp) { // period call back
  (void)pwmp;
  //palClearPad(GPIOA, GPIOA_LED_GREEN);
  //palClearPad(GPIOA, GPIOA_PIN15);
  //palClearPad(GPIOA, 9);
  //pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 1000));
  //pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 1000));
  //pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 1000));
}

static void pwmc1cb(PWMDriver *pwmp) { // channel 1 callback
  (void)pwmp;
  //palSetPad(GPIOA, 10);
  pwmEnableChannelI(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, cycle0));
  cycle0 += 10;
  if (cycle0 > 10000)
    cycle0 = 0;
}

static void pwmc2cb(PWMDriver *pwmp) { // channel 1 callback
  (void)pwmp;
  //palSetPad(GPIOA, 8);
  pwmEnableChannelI(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, cycle1));
  cycle1 += 100;
  if (cycle1 > 10000)
    cycle1 = 0;
}

static void pwmc3cb(PWMDriver *pwmp) { // channel 1 callback
  (void)pwmp;
  //palSetPad(GPIOA, 9);
  pwmEnableChannelI(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, cycle2));
  cycle2 += 1000;
  if (cycle2 > 10000)
    cycle2 = 0;
}

static PWMConfig pwmcfg = {
  40000,                                    /* 10kHz PWM clock frequency.   */
  1000,                                    /* Initial PWM period 1S.       */
  pwmpcb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, pwmc1cb},
   {PWM_OUTPUT_ACTIVE_HIGH, pwmc2cb},
   {PWM_OUTPUT_ACTIVE_HIGH, pwmc3cb},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

//*
static THD_WORKING_AREA(pwmThread1_wa, 128);
static THD_FUNCTION(pwmThread1, arg) {
  (void)arg;
  chRegSetThreadName("pwm");
	// initialize pwm driver
  pwmStart(&PWMD1, &pwmcfg);
  //pwmEnablePeriodicNotification(&PWMD1);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(1));
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(1));
  chThdSleepMilliseconds(2000);
  //Starts the PWM channel 0 using 50% duty cycle.
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 1000));
  pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
  pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 9000));
//	pwmChangePeriod (&PWMD1, 5000);
  pwmEnableChannelNotification(&PWMD1, 0);
  pwmEnableChannelNotification(&PWMD1, 1);
  pwmEnableChannelNotification(&PWMD1, 2);
  chThdSleepMilliseconds(500);
	/*
   * Disables channel 0 and stops the drivers.
   */
//  pwmDisableChannel(&PWMD1, 0);
//  pwmStop(&PWMD1);
//  palClearPad(GPIOA, GPIOA_LED_GREEN);

	
	while(true){
  	chThdSleepMilliseconds(500);
	}
}
//*/

int main(void) {
  halInit(); 
  chSysInit(); 
  palSetPadMode(GPIOA, 10, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 9, PAL_MODE_OUTPUT_PUSHPULL);

  /*
   * Initializes the PWM driver
   */
/*
  pwmStart(&PWMD1, &pwmcfg);
  pwmEnablePeriodicNotification(&PWMD1);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
  chThdSleepMilliseconds(2000);

  
  //Starts the PWM channel 0 using 75% duty cycle.
  //pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 7500));
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
 	//pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 2500));
	//pwmChangePeriod (&PWMD1, 5000);
  pwmEnableChannelNotification(&PWMD1, 0);
  chThdSleepMilliseconds(5000);
//*/
  chThdCreateStatic(pwmThread1_wa, sizeof(pwmThread1_wa), NORMALPRIO, pwmThread1, NULL);
	/*
   * Disables channel 0 and stops the drivers.
   */
//  pwmDisableChannel(&PWMD1, 0);
//  pwmStop(&PWMD1);
//  palClearPad(GPIOA, GPIOA_LED_GREEN);

  while (true) {
    chThdSleepMilliseconds(500);
  }
  return 0;
}
