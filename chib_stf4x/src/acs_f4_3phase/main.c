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

#define U_High
#define U_Low
#define V_High
#define V_Low
#define W_High
#define W_Low

static void pwm1pcb(PWMDriver *pwmp) { // period call back
  (void)pwmp;
  palClearPad(GPIOA, GPIOA_LED_GREEN);
}

static void pwm1c1cb(PWMDriver *pwmp) { // channel 1 callback
  (void)pwmp;
  palSetPad(GPIOA, GPIOA_LED_GREEN);
}

static PWMConfig pwm1cfg = {
  10000,                                    /* 10kHz PWM clock frequency.   */
  10000,                                    /* Initial PWM period 1S.       */
  pwm1pcb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, pwm1c1cb},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};

static void startBLDC(void){
  pwmStart(&PWMD1, &pwm1cfg);
  pwmEnablePeriodicNotification(&PWMD1);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
  chThdSleepMilliseconds(2000);
  //Starts the PWM channel 0 using 50% duty cycle implicitly
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
//	pwmChangePeriod (&PWMD1, 5000);
  pwmEnableChannelNotification(&PWMD1, 0);
  chThdSleepMilliseconds(5000);
}

static void stopBLDC(void){
// 	Disables channel 0 and stops the drivers.
  pwmDisableChannel(&PWMD1, 0);
  pwmStop(&PWMD1);
  palClearPad(GPIOA, GPIOA_LED_GREEN);
}

/*
static void pwm1(void){
  pwmStart(&PWMD1, &pwm1cfg);
  pwmEnablePeriodicNotification(&PWMD1);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
  chThdSleepMilliseconds(2000);
  //Starts the PWM channel 0 using 50% duty cycle implicitly
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
//	pwmChangePeriod (&PWMD1, 5000);
  pwmEnableChannelNotification(&PWMD1, 0);
  chThdSleepMilliseconds(5000);
// 	Disables channel 0 and stops the drivers.
//  pwmDisableChannel(&PWMD1, 0);
//  pwmStop(&PWMD1);
//  palClearPad(GPIOA, GPIOA_LED_GREEN);

}
*/

static THD_WORKING_AREA(pwmThread1_wa, 128);
static THD_FUNCTION(pwmThread1, arg) {
  (void)arg;
  chRegSetThreadName("pwm");
	startBLDC();
	while(true){
  	chThdSleepMilliseconds(500);
	}
}

int main(void) {
  halInit(); 
  chSysInit(); 
  palSetPadMode(GPIOA, GPIOA_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
  
	chThdCreateStatic(pwmThread1_wa, sizeof(pwmThread1_wa), NORMALPRIO, pwmThread1, NULL);
  
	while (true) {
    chThdSleepMilliseconds(500);
  }
  return 0;
}
