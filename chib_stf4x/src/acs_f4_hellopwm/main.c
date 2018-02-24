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

#define GPIOPort GPIOA
// Pin number
#define ACH1 8U
#define ACH2 9U
#define ACH3 10U

#define GPIOComp GPIOB
#define BCH1 13U
#define BCH2 14U
#define BCH3 15U


// 10000 = 100%
#define CH1_DC 2000
#define CH2_DC 4000
#define CH3_DC 8000

#define PWM_PERIOD 5
#define PWM_FREQ 32000

int stepU = 0;
int stepV = 0;
int stepW = 0;

int sinusoidArraySize;

int sinusoidArray[] = {
	4980,5068,5155,5242,5329,5416,5503,5590,5676,5763,5849,5934,6020,6105,6190,6274,
	6359,6442,6525,6608,6690,6772,6853,6934,7014,7093,7172,7250,7328,7404,7480,7556,7630,7704,7776,7848,
	7919,7989,8059,8127,8194,8261,8326,8390,8454,8516,8577,8637,8696,8754,8811,8866,8920,8974,9025,9076,
	9126,9174,9221,9266,9311,9353,9395,9435,9474,9512,9548,9583,9616,9648,9679,9708,9736,9762,9787,9810,
	9832,9852,9871,9889,9904,9919,9932,9943,9953,9961,9968,9974,9977,9980,9980,9980,9977,9974,9968,9961,
	9953,9943,9932,9919,9904,9889,9871,9852,9832,9810,9787,9762,9736,9708,9679,9648,9616,9583,9548,9512,
	9474,9435,9395,9353,9311,9266,9221,9174,9126,9076,9025,8974,8920,8866,8811,8754,8696,8637,8577,8516,
	8454,8390,8326,8261,8194,8127,8059,7989,7919,7848,7776,7704,7630,7556,7480,7404,7328,7250,7172,7093,
	7014,6934,6853,6772,6690,6608,6525,6442,6359,6274,6190,6105,6020,5934,5849,5763,5676,5590,5503,5416,
	5329,5242,5155,5068,4980,4893,4806,4719,4632,4545,4458,4371,4285,4198,4112,4026,3941,3856,3771,3686,
	3602,3519,3435,3353,3270,3189,3107,3027,2947,2867,2789,2710,2633,2556,2480,2405,2331,2257,2184,2113,
	2041,1971,1902,1834,1766,1700,1635,1570,1507,1445,1384,1324,1265,1207,1150,1095,1040,987,935,885,835,
	787,740,695,650,607,566,525,486,449,413,378,344,312,282,253,225,199,174,151,129,109,90,72,56,42,29,18,8,
	0,0,0,0,0,0,0,0,0,0,0,8,18,29,42,56,72,90,109,129,151,174,199,225,253,282,312,344,378,
	413,449,486,525,566,607,650,695,740,787,835,885,935,987,1040,1095,1150,1207,1265,1324,1384,1445,1507,
	1570,1635,1700,1766,1834,1902,1971,2041,2113,2184,2257,2331,2405,2480,2556,2633,2710,2789,2867,2947,
	3027,3107,3189,3270,3353,3435,3519,3602,3686,3771,3856,3941,4026,4112,4198,4285,4371,4458,4545,4632,
	4719,4806,4893,4980
};

static void pwmpcb(PWMDriver *pwmp) { // period call back
  (void)pwmp;
  //palClearPad(GPIOPort, ACH1);
  //palClearPad(GPIOPort, ACH2);
  //palClearPad(GPIOPort, ACH3);
  stepU++;
  stepV++;
  stepW++;

  if (stepU >= 360)
  {
    stepU = 0;
  }
  
  if (stepV >= 360)
  {
    stepV = 0;
  }
  
  if (stepW >= 360)
  {
    stepW = 0;
  }
}

static void pwmc1cb(PWMDriver *pwmp) { // channel 1 callback
  (void)pwmp;
  //palSetPad(GPIOPort, ACH1);
  pwmEnableChannelI(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, sinusoidArray[stepU]));
}

static void pwmc2cb(PWMDriver *pwmp) { // channel 2 callback
  (void)pwmp;
  //palSetPad(GPIOPort, ACH2);
  pwmEnableChannelI(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, sinusoidArray[stepV]));
}

static void pwmc3cb(PWMDriver *pwmp) { // channel 3 callback
  (void)pwmp;
  //palSetPad(GPIOPort, ACH3);
  pwmEnableChannelI(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, sinusoidArray[stepW]));
}

static PWMConfig pwmcfg = {
  PWM_FREQ,                                    /* 10kHz PWM clock frequency.   */
  PWM_PERIOD,                                    /* Initial PWM period 1S.       */
  pwmpcb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH|PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW, pwmc1cb},
   {PWM_OUTPUT_ACTIVE_HIGH|PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW, pwmc2cb},
   {PWM_OUTPUT_ACTIVE_HIGH|PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW, pwmc3cb},
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
  pwmEnablePeriodicNotification(&PWMD1);
  palSetPadMode(GPIOPort, ACH1, PAL_MODE_ALTERNATE(1));
  palSetPadMode(GPIOPort, ACH2, PAL_MODE_ALTERNATE(1));
  palSetPadMode(GPIOPort, ACH3, PAL_MODE_ALTERNATE(1));
  
	palSetPadMode(GPIOComp, BCH1, PAL_MODE_ALTERNATE(1));
  palSetPadMode(GPIOComp, BCH2, PAL_MODE_ALTERNATE(1));
  palSetPadMode(GPIOComp, BCH3, PAL_MODE_ALTERNATE(1));
  //palSetPadMode(GPIOB, 3, PAL_MODE_ALTERNATE(1));
  chThdSleepMilliseconds(2000);
  //Starts the PWM channel 0 using 50% duty cycle.
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, CH1_DC));
  pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, CH2_DC));
  pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, CH3_DC));
  //pwmEnableChannel(&PWMD1, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
//	pwmChangePeriod (&PWMD1, 5000);
  pwmEnableChannelNotification(&PWMD1, 0);
  pwmEnableChannelNotification(&PWMD1, 1);
  pwmEnableChannelNotification(&PWMD1, 2);
  chThdSleepMilliseconds(5000);
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

  sinusoidArraySize = sizeof(sinusoidArray)/sizeof(int);
  int phaseShift = sinusoidArraySize/3;
  stepU = 0;
  stepV = stepU + phaseShift;
  stepW = stepV + phaseShift;

  palSetPadMode(GPIOPort, ACH1, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOPort, ACH2, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOPort, ACH3, PAL_MODE_OUTPUT_PUSHPULL);

  palSetPadMode(GPIOComp, ACH1, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOComp, ACH2, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOComp, ACH3, PAL_MODE_OUTPUT_PUSHPULL);
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
