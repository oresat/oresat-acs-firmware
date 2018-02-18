/* bldc.h
 *
 *  Created on: Jul 22, 2013
 *      Author: kjell
 */

#ifndef BLDC_H_
#define BLDC_H_

typedef struct {
  uint8_t   (*scheme)[][3];
  uint8_t   pwmOutT0;
  uint8_t   pwmOutT1;
  uint32_t  state;
  uint32_t  nextState;
  uint32_t  stateCount;
  uint32_t  prevStateChange;
  uint32_t  nextStateChange;
  uint32_t  stateChangeInterval;
  bool      directionFwd;
  uint32_t  dutyCycle;
} BldcConfig;

#define BLDC_COMM_STACK_SIZE    1024

/*
 * Defines the states for different signals in the commutation scheme
 */
#define PWM_OFF     0b000000
#define PWM_UN      0b000001
#define PWM_UP      0b000010
#define PWM_VN      0b000100
#define PWM_VP      0b001000
#define PWM_WN      0b010000
#define PWM_WP      0b100000
#define PWM_EXPECT_ZERO     TRUE


#define PWM_STACK_SIZE      1024

//#define PWM_CLOCK_FREQ        16800000
//#define PWM_FREQ              5000
#define PWM_CLOCK_FREQ        10000
#define PWM_FREQ              10000
#define PWM_PERIOD            PWM_CLOCK_FREQ/PWM_FREQ

#define PWM_MAX_DUTY_CYCLE    9500


#define STATE_CHANGE_LIMIT_US 10*1000000/PWM_FREQ


#define PWM_OUT_PORT_MASK   0x3F
//#define PWM_OUT_PORT        GPIOE
#define PWM_OUT_PORT        GPIOB
#define PWM_OUT_OFFSET      8


#define PWM_PULSE0_CH       0
#define PWM_ADCTRIG_CH      3

#define TIME1_LIMIT         100

extern void bldcInit(void);
extern void bldcKill(void);
extern void bldcStart(void);
extern void bldcSetDutyCycle(uint32_t dutyCycle);
extern void bldcSetRPM (uint32_t rpm);

#endif /* BLDC_H_ */
