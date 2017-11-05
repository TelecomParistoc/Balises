#include "ch.h"
#include "hal.h"

// sample to battery voltage (in 0.01V) conversion coeff
#define PROBE_TO_VBAT 660/4096

// battery states
#define BATTERY_OK 0
#define BATTERY_LOW 1

// battery transition thresholds
#define BATTERY_OK_LTHRES 450
#define BATTERY_LOW_HTHRES 460

int batteryState = BATTERY_OK;

static void adcErrorCallback(ADCDriver *adcp, adcerror_t err) {
	(void)adcp;
	(void)err;
}

// conversion of one sample of channel 1
static const ADCConversionGroup adcconf = {
	FALSE,                   // linear buffer (not circular)
	1,                       // one channel
	NULL,                    // end of conversion callback
	adcErrorCallback,        // error callback
	0,                       // CFGR
	ADC_TR(0, 4095),         // TR1
	{                        // SMPR[2]
		ADC_SMPR1_SMP4_1 | ADC_SMPR1_SMP4_2, // sample time = 181.5 ADC clk cycles
		0
	},
	{                        // SQR[4]
		ADC_SQR1_SQ1_N(ADC_CHANNEL_IN4),
		0,
		0,
		0
	}
};

static void updateState(int voltage) {
	if(batteryState == BATTERY_OK && voltage < BATTERY_OK_LTHRES) {
		batteryState = BATTERY_LOW;
		palSetLine(LINE_LED_BATT);
	} else if(batteryState == BATTERY_LOW && voltage > BATTERY_LOW_HTHRES) {
		batteryState = BATTERY_OK;
		palClearLine(LINE_LED_BATT);
	}
}

static int mean_voltage(int nsamples) {
	int total = 0;
	for(int i = 0; i < nsamples; i++) {
		adcsample_t sample;
		adcConvert(&ADCD1, &adcconf, &sample, 1);
		total += sample;
		// wait between samples to filter out low frequencies of noise
		chThdSleepMilliseconds(600);
	}
	return total * PROBE_TO_VBAT / nsamples;
}

static THD_WORKING_AREA(waBattery, 128);
static THD_FUNCTION(batteryThread, th_data) {
	(void) th_data;
	chRegSetThreadName("Battery");

	while(1) {
		updateState(mean_voltage(8));
	}
}

void initBattery(void) {
	// power up ADC1
	adcStart(&ADCD1, NULL);

	// start battery probe thread
	chThdCreateStatic(waBattery, sizeof(waBattery), NORMALPRIO-2, batteryThread, NULL);
}
