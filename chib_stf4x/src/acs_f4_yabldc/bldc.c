/*
 * pwm_gen.c
 *
 *  Created on: Jul 22, 2013
 *      Author: kjell
 */


#include "ch.h"
#include "hal.h"
//#include "pwm.h"

#include "bldc.h"

static adcsample_t zeroCrossing;

static void cbAdcZeroSense(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void) adcp;
  (void) n;

  adcsample_t  res;

  res = *buffer;
}

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 1 sample of 1 channel, SW triggered.
 * Channels:    IN1.
 */
static const ADCConversionGroup adcZeroSense = {
  FALSE,
  1,
  &cbAdcZeroSense,
  NULL,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  0,
  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_15),                        /* SMPR2 */
  ADC_SQR1_NUM_CH(1),
  0,                        /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1)
};

/*
 * PWM Scheme:  High PWM, Lo ON
 * Time0 Time1
 */
static uint8_t pwmScheme[6][3] = {
	{PWM_UP|PWM_VN, PWM_VN, !PWM_EXPECT_ZERO},    //UP PWM, VN ON
	{PWM_UP|PWM_WN, PWM_WN, PWM_EXPECT_ZERO},     //UP PWM, WN ON
	{PWM_VP|PWM_WN, PWM_WN, !PWM_EXPECT_ZERO},    //VP PWM, WN ON
	{PWM_VP|PWM_UN, PWM_UN, PWM_EXPECT_ZERO},     //VP PWM, UN ON
	{PWM_WP|PWM_UN, PWM_UN, !PWM_EXPECT_ZERO},    //WP PWM, UN ON
	{PWM_WP|PWM_VN, PWM_VN, PWM_EXPECT_ZERO}      //WP PWM, VN ON
};

BldcConfig  bldc;

/* The PWM Counter Reset will put the PWM system in "ACTIVE" state, which
 * is defined as the state when the channel is active and a compare event
 * has not yet taken place.
 */
static void cdPwmCounterReset(PWMDriver *pwmp) {
  (void) pwmp;

  chSysLockFromISR();
  palWriteGroup (PWM_OUT_PORT, PWM_OUT_PORT_MASK, PWM_OUT_OFFSET,  bldc.pwmOutT0);

  // Calculate and initiate the state change
  // Consider moving this to a thread to further slim down the ISR callback
  if (!chSysIsCounterWithinX(chSysGetRealtimeCounterX(), bldc.prevStateChange, bldc.nextStateChange)) {
  //if (!halIsCounterWithin(bldc.prevStateChange, bldc.nextStateChange)) {

    // Prepare next state
    if (bldc.directionFwd) {
      bldc.nextState++;
    }
    else {
      bldc.nextState--;
    }

    // Wrap the state counter
    bldc.nextState = (bldc.nextState+bldc.stateCount)%bldc.stateCount;

    // Prepare the next state change.
    bldc.prevStateChange = bldc.nextStateChange;
    bldc.nextStateChange += bldc.stateChangeInterval;
  }
  chSysUnlockFromISR();
}

/* The PWM Channel compare will put the PWM system in "IDLE" state, which
 * is defined as the state when the channel is active and a compare event
 * has taken place.
 */
static void cbPwmCh0Compare(PWMDriver *pwmp) {
  (void) pwmp;

  chSysLockFromISR();
  palWriteGroup (PWM_OUT_PORT, PWM_OUT_PORT_MASK, PWM_OUT_OFFSET,  bldc.pwmOutT1);

  // Do the state change before the next cycle.
  // Consider moving this to a thread to further slim down the ISR callback
  bldc.state = bldc.nextState;
  bldc.pwmOutT0 = (*bldc.scheme)[bldc.state][0];
  bldc.pwmOutT1 = (*bldc.scheme)[bldc.state][1];
  chSysUnlockFromISR();
}

static void pcbPwmAdcTrigger(PWMDriver *pwmp) {
  (void) pwmp;
  adcStartConversion(&ADCD1, &adcZeroSense, &zeroCrossing, 1);
}

static PWMConfig pwmcfg = {
	PWM_CLOCK_FREQ,
	PWM_PERIOD,
	&cdPwmCounterReset,
	{
	{PWM_OUTPUT_ACTIVE_HIGH, &cbPwmCh0Compare},
	{PWM_OUTPUT_DISABLED, NULL},
	{PWM_OUTPUT_DISABLED, NULL},
	{PWM_OUTPUT_ACTIVE_HIGH, &pcbPwmAdcTrigger}
	},
	0,
	0
};

/* This function will start the PWM generator.
 */
extern void bldcInit(void) {
  bldc.scheme = &pwmScheme;
  bldc.state = 0;          //Default to first state
  bldc.nextState = 0;
  bldc.directionFwd = TRUE;
  //bldc.stateChangeInterval = OSAL_MS2RTC(STM32_HSECLK,20);
  //bldc.stateChangeInterval = MS2RTT(20);
  bldc.stateChangeInterval = MS2RTC(STM32_HSECLK,20);
  bldc.prevStateChange = chSysGetRealtimeCounterX();
  //bldc.prevStateChange = halGetCounterValue();
  bldc.nextStateChange = bldc.prevStateChange + bldc.stateChangeInterval;
  bldc.pwmOutT0 = 0;
  bldc.pwmOutT1 = 0;
  bldc.stateCount = sizeof(pwmScheme)/3;
  bldc.dutyCycle = 1800;

  palWriteGroup (PWM_OUT_PORT, PWM_OUT_PORT_MASK, PWM_OUT_OFFSET,  PWM_OFF);
  palSetGroupMode (PWM_OUT_PORT, PWM_OUT_PORT_MASK, PWM_OUT_OFFSET, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode (GPIOA, GPIOA_ADC123_IN1, PAL_MODE_INPUT_ANALOG);

  pwmStart(&PWMD1, &pwmcfg);

  // ADC trigger channel. This will trigger the ADC read at 95% of the cycle,
  // when all the PWM outputs are set to 0
  pwmEnableChannel (&PWMD1, PWM_ADCTRIG_CH, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, PWM_MAX_DUTY_CYCLE));

  // Start the ADC
  adcStart(&ADCD1, NULL);

}

extern void bldcKill(void) {
  palWriteGroup (PWM_OUT_PORT, PWM_OUT_PORT_MASK, PWM_OUT_OFFSET,  PWM_OFF);
  pwmStop(&PWMD1);
}

extern void bldcStart(void) {
  // The PWM output generator channel.
  pwmEnableChannel (&PWMD1, PWM_PULSE0_CH, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, bldc.dutyCycle));
}

// This function should probably be replaced by a Mailbox and a thread.
extern void bldcSetDutyCycle(uint32_t dutyCycle) {
  if (dutyCycle > PWM_MAX_DUTY_CYCLE) {
    dutyCycle = PWM_MAX_DUTY_CYCLE;
  }

  bldc.dutyCycle = dutyCycle;

  pwmEnableChannel (&PWMD1, PWM_PULSE0_CH, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, dutyCycle));
}

// This function is, in this form, only for debugging. Later, it may
// be used when the RPM is controlled by a PID.
extern void bldcSetRPM (uint32_t rpm) {
  uint32_t uspc;  // us pr Commutations

  uspc = (1000000*60 / (rpm*bldc.stateCount));
  bldc.stateChangeInterval = US2RTC(STM32_HSECLK,uspc);
  //bldc.stateChangeInterval = OSAL_US2RTC(uspc);
}
