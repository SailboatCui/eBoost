#ifndef MAIN_H_
#define MAIN_H_

#include "pil.h"

#define SYSCLK_HZ 60000000L
#define LSPCLK_HZ 15000000L
#define PWM_HZ 100000L

#define BAUD_RATE 115200L

typedef struct PIL_VARS {
	PIL_READ_PROBE(int32_t, ipkp, 0, 1, "");
	PIL_READ_PROBE(int32_t, ipk, 0, 1, "");
	PIL_READ_PROBE(int32_t, DACVAL, 0, 1, "");
	PIL_OVERRIDE_PROBE(int16_t, VoltagePeak_ADC, 0, 1, "");
} PilVars_t;

extern PilVars_t PilVars;

#endif /* MAIN_H_ */
