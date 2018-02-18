#include "ch.h"
#include "hal.h"
#include "acs_bldc.h"

static void pwmpcb(PWMDriver *pwmp) { // period call back
  (void)pwmp;
  palClearPad(GPIOA, GPIOA_LED_GREEN);
}

static void pwmc0cb(PWMDriver *pwmp) { // channel 1 callback
  (void)pwmp;
  palSetPad(GPIOA, GPIOA_LED_GREEN);
}

static PWMConfig pwmcfg = {
	PWM_CLOCK_FREQ, 
	PWM_FREQ,
	pwmpcb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, pwmc0cb},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0,
	0 // needed with advanced and newer ChibiOS
};

extern void bldcInit(void){
  palSetPadMode(GPIOA, GPIOA_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);

	pwmEnablePeriodicNotification(&PWMD1);
  palSetGroupMode(
			PWM_OUT_PORT,PWM_OUT_PORT_MASK,PWM_OUT_OFFSET,PAL_MODE_OUTPUT_PUSHPULL);
	palWriteGroup(PWM_OUT_PORT, PWM_OUT_PORT_MASK,PWM_OUT_OFFSET,PWM_OFF);

  chThdSleepMilliseconds(2000);

  pwmEnableChannel(&PWMD1,0,PWM_PERCENTAGE_TO_WIDTH(&PWMD1,5000));
  pwmEnableChannelNotification(&PWMD1, 0);
  chThdSleepMilliseconds(5000);
}

/*
extern void bldcStart(void){
  pwmStart(&PWMD1, &pwmcfg);
  pwmEnablePeriodicNotification(&PWMD1);
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
  chThdSleepMilliseconds(2000);
  //Starts the PWM channel 0 using 50% duty cycle implicitly
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
//	pwmChangePeriod (&PWMD1, 5000);
  pwmEnableChannelNotification(&PWMD1, 0);
  chThdSleepMilliseconds(5000);
}
//*/

extern void bldcStart(void){
  pwmStart(&PWMD1, &pwmcfg);
/*
	pwmEnablePeriodicNotification(&PWMD1);
  palSetGroupMode(
			PWM_OUT_PORT,PWM_OUT_PORT_MASK,PWM_OUT_OFFSET,PAL_MODE_OUTPUT_PUSHPULL);
	palWriteGroup(PWM_OUT_PORT, PWM_OUT_PORT_MASK,PWM_OUT_OFFSET,PWM_OFF);

  chThdSleepMilliseconds(2000);

  pwmEnableChannel(&PWMD1,0,PWM_PERCENTAGE_TO_WIDTH(&PWMD1,5000));
  pwmEnableChannelNotification(&PWMD1, 0);
  chThdSleepMilliseconds(5000);
//*/
}

extern void bldcStop(void){
// 	Disables channel 0 and stops the drivers.
  pwmStop(&PWMD1);
}

extern void bldcKill(void){
  pwmDisableChannel(&PWMD1, 0);
  palClearPad(GPIOA, GPIOA_LED_GREEN);
}
