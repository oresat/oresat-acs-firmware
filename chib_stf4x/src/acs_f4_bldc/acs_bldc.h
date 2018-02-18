#ifndef ADC_BLDC_H
#define ADC_BLDC_H

#define PWM_OFF     0b000000
#define PWM_UN      0b000001
#define PWM_UP      0b000010
#define PWM_VN      0b000100
#define PWM_VP      0b001000
#define PWM_WN      0b010000
#define PWM_WP      0b100000
#define PWM_EXPECT_ZERO     TRUE  

#define PWM_CLOCK_FREQ      10000 
#define PWM_FREQ            10000
#define PWM_PERIOD          PWM_CLOCK_FREQ/PWM_FREQ

#define PWM_OUT_PORT_MASK   0x3F
#define PWM_OUT_PORT        GPIOA
#define PWM_OUT_OFFSET      8


#define PWM_PULSE0_CH       0
#define PWM_ADCTRIG_CH      3

#define TIME1_LIMIT         100

extern void bldcInit(void);
extern void bldcStop(void);
extern void bldcStart(void);
extern void bldcKill(void);
//extern void bldcSetDutyCycle(uint32_t dutyCycle);
//extern void bldcSetRPM (uint32_t rpm);


#endif
