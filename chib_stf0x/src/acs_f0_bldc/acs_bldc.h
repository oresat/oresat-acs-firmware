#ifndef ACS_BLDC_H
#define ACS_BLDC_H

#include "ch.h"
#include "hal.h"

typedef struct{
	int stepU;
	int stepV;
	int stepW;


} BldcConfig;


#define PWM_CLOCK_FREQ				40000U
#define PWM_FREQ							1U
#define PWM_PERIOD						PWM_CLOCK_FREQ/PWM_FREQ

#define PWM_OUT_PORT_MASK			0x7
#define PWM_OUT_PORT					GPIOA
#define PWM_OUT_PORT_N				GPIOB // ~PWM_OUT_PORT
#define PWM_OUT_OFFSET				8U
#define PWM_OUT_OFFSET_N			13U		// ~PWM_OUT_OFFSET

#define ACH1 8U
#define ACH2 9U
#define ACH3 10U 

#define BCH1 13U
#define BCH2 14U
#define BCH3 15U 

#define PWM_CH1								0U
#define PWM_CH2								1U
#define PWM_CH3								2U

#define PWM_DC_CH1						2000
#define PWM_DC_CH2						4000
#define PWM_DC_CH3						8000

extern void bldcInit(void);
extern void bldcStart(void);
extern void bldcStop(void);

extern void bldcSinStart(void);

#endif
