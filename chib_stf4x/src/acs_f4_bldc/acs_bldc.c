#include "ch.h"
#include "hal.h"
#include "acs_bldc.h"

static adcsample_t zeroCrossing;

//*
static void cbAdcZeroSense(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void) adcp;
  (void) n;

  adcsample_t  res;
  res = *buffer;
}
//*/

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 1 sample of 1 channel, SW triggered.
 * Channels:    IN1.
 */

//*
static const ADCConversionGroup adcZeroSense = {
  FALSE,
  1,
  &cbAdcZeroSense,
  NULL,
  0,                        // CR1 
  ADC_CR2_SWSTART,          // CR2
  0,
  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_15), // SMPR2
  ADC_SQR1_NUM_CH(1),
  0,                        // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1)
};
//*/

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

//*
static void cdPwmCounterReset(PWMDriver *pwmp) {
  (void)pwmp;
  
	//palSetPad(GPIOA, GPIOA_LED_GREEN); // for fun!
//*  
	chSysLockFromISR();
  palWriteGroup(PWM_OUT_PORT1,PWM_OUT_PORT_MASK1,PWM_OUT_OFFSET,bldc.pwmOutT0);
  palWriteGroup(PWM_OUT_PORT2,PWM_OUT_PORT_MASK2,PWM_OUT_OFFSET,bldc.pwmOutT0);

  // Calculate and initiate the state change
  // Consider moving this to a thread to further slim down the ISR callback
  if(!chSysIsCounterWithinX(chSysGetRealtimeCounterX(),bldc.prevStateChange,bldc.nextStateChange)) {
    // Prepare next state
    if (bldc.directionFwd) {
      ++bldc.nextState;
    }
    else {
      --bldc.nextState;
    }

    // Wrap the state counter
    bldc.nextState = (bldc.nextState+bldc.stateCount)%bldc.stateCount;

    // Prepare the next state change.
    bldc.prevStateChange = bldc.nextStateChange;
    bldc.nextStateChange += bldc.stateChangeInterval;
  }
  chSysUnlockFromISR();
	//*/
}
//*/

/* 
 * The PWM Channel compare will put the PWM system in "IDLE" state, which
 * is defined as the state when the channel is active and a compare event
 * has taken place.
 */

//*
static void cbPwmCh0Compare(PWMDriver *pwmp){
  (void)pwmp;
  
	//palClearPad(GPIOA, GPIOA_LED_GREEN); // added for fun!
//*
  chSysLockFromISR();
  //palWriteGroup(PWM_OUT_PORT1,PWM_OUT_PORT_MASK1,PWM_OUT_OFFSET,bldc.pwmOutT1);
  //palWriteGroup(PWM_OUT_PORT2,PWM_OUT_PORT_MASK2,PWM_OUT_OFFSET,bldc.pwmOutT1);

  // Do the state change before the next cycle.
  // Consider moving this to a thread to further slim down the ISR callback
  bldc.state = bldc.nextState;
  bldc.pwmOutT0 = (*bldc.scheme)[bldc.state][0];
  bldc.pwmOutT1 = (*bldc.scheme)[bldc.state][1];
  chSysUnlockFromISR();
//*/
}

//*
static void pcbPwmAdcTrigger(PWMDriver *pwmp){
  (void)pwmp;
  adcStartConversion(&ADCD1,&adcZeroSense,&zeroCrossing, 1);
}
//*/

static PWMConfig pwmcfg = {
	//40000,
	PWM_CLOCK_FREQ,
	//1000,
	PWM_PERIOD,
	&cdPwmCounterReset,
	{
		{PWM_OUTPUT_ACTIVE_HIGH,&cbPwmCh0Compare},
		{PWM_OUTPUT_DISABLED,NULL},
		{PWM_OUTPUT_DISABLED,NULL},
		{PWM_OUTPUT_ACTIVE_HIGH,&pcbPwmAdcTrigger}
	},
	0,
	0
};
//*/

extern void bldcInit(){
	bldc.scheme = &pwmScheme;
  bldc.state = 0;          //Default to first state
  bldc.nextState = 0;
  bldc.directionFwd = TRUE;
  //bldc.stateChangeInterval = MS2RTT(20); // old call
  bldc.stateChangeInterval = MS2RTC(PWM_CLOCK_FREQ,20);
  //bldc.prevStateChange = halGetCounterValue(); // old
  bldc.prevStateChange = chSysGetRealtimeCounterX();
  bldc.nextStateChange = bldc.prevStateChange + bldc.stateChangeInterval;
  bldc.pwmOutT0 = 0;
  bldc.pwmOutT1 = 0;
  bldc.stateCount = sizeof(pwmScheme)/3;
 // bldc.dutyCycle = 1800;
  bldc.dutyCycle = 5000;

  //palSetPadMode(GPIOA, GPIOA_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);
	palWriteGroup(PWM_OUT_PORT1, PWM_OUT_PORT_MASK1,PWM_OUT_OFFSET,PWM_OFF);
  palSetGroupMode(PWM_OUT_PORT1,PWM_OUT_PORT_MASK1,PWM_OUT_OFFSET,PAL_MODE_OUTPUT_PUSHPULL);
	palWriteGroup(PWM_OUT_PORT2, PWM_OUT_PORT_MASK2,PWM_OUT_OFFSET,PWM_OFF);
  palSetGroupMode(PWM_OUT_PORT2,PWM_OUT_PORT_MASK2,PWM_OUT_OFFSET,PAL_MODE_OUTPUT_PUSHPULL);
//	pwmEnablePeriodicNotification(&PWMD1);
  pwmEnableChannel(&PWMD1,0,PWM_PERCENTAGE_TO_WIDTH(&PWMD1,PWM_MAX_DUTY_CYCLE));
  pwmStart(&PWMD1, &pwmcfg);
//  pwmEnableChannelNotification(&PWMD1, 0);

	// ADC trigger channel. This will trigger the ADC read at 95% of the cycle,
  // when all the PWM outputs are set to 0
  pwmEnableChannel(&PWMD1,PWM_ADCTRIG_CH,PWM_PERCENTAGE_TO_WIDTH(&PWMD1,PWM_MAX_DUTY_CYCLE));

  // Start the ADC
  adcStart(&ADCD1, NULL);
}

extern void bldcStart(void){
  pwmEnableChannel(&PWMD1,PWM_PULSE0_CH,PWM_PERCENTAGE_TO_WIDTH(&PWMD1,bldc.dutyCycle));
}

extern void bldcStop(void){
// 	Disables channel 0 and stops the drivers.
  pwmStop(&PWMD1);
}

/*
extern void bldcKill(void){
  pwmDisableChannel(&PWMD1, 0);
  palClearPad(GPIOA, GPIOA_LED_GREEN);
}
//*/

// This function should probably be replaced by a Mailbox and a thread.
/*
extern void bldcSetDutyCycle(uint32_t dutyCycle) {
  if (dutyCycle > PWM_MAX_DUTY_CYCLE) {
    dutyCycle = PWM_MAX_DUTY_CYCLE;
  }

  bldc.dutyCycle = dutyCycle;

  pwmEnableChannel (&PWMD1, PWM_PULSE0_CH, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, dutyCycle));
}
//*/

// This function is, in this form, only for debugging. Later, it may
// be used when the RPM is controlled by a PID.
/*
extern void bldcSetRPM (uint32_t rpm) {
  uint32_t uspc;  // us pr Commutations

  uspc = (1000000*60 / (rpm*bldc.stateCount));
  bldc.stateChangeInterval = US2RTC(STM32_HSECLK,uspc);
  //bldc.stateChangeInterval = US2RTT(uspc);
}
//*/
