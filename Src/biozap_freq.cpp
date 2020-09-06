/*
 * biozap_freq.cpp
 *
 *  Created on: Sep 6, 2020
 *      Author: zgorfol
 */

#include "main.h"
#include "biozap_freq.h"
#include <cmath>

#define PI 3.14159265
#define BIOZAP_SAMPLE_SIZE 2048	// 12bit sample (0-4095) size

int BIOZAP_Sample_Lgth = BIOZAP_SAMPLE_SIZE;
int BIOZAP_Sample_Max = BIOZAP_SAMPLE_SIZE;
#define BIOZAP_ONE_VOLT 349.24  	// Divider 12k/4.7k with AD811 OpAmp equals 1V = 349.24 bits  or 1bit = 0.0028634V
#define BIOZAP_VMIN_MAX 400			//  4.00 V
#define BIOZAP_VMAX_MAX 1100		// 11.00 V

uint16_t BIOZAP_SampleArray[ BIOZAP_SAMPLE_SIZE]; //One period sample array
uint16_t BIOZAP_DutyCycle = 50;

static uint8_t generate_sin_sample(uint16_t v_min, uint16_t v_max , uint16_t *sample_array) {
/*Generate sine sample - one period depends on v_min(0) and v_max(4095 equals 12V vmax)
 * return:
 * 	1 = OK
 * 	0 = Error: v_min is >= v_max  or  v_max > 4095.
 */

	float step = 2*PI / BIOZAP_Sample_Lgth;

	if ( (v_max>4095) || (v_min>=v_max) ) return 0;

	for (int i=0; i < BIOZAP_Sample_Lgth; i++ ){
		sample_array[i] = ( sin(i*step) + 1) * (v_max-v_min)/2.0 + v_min ;
	}

return 1;

}

static uint8_t generate_saw_sample(uint16_t v_min, uint16_t v_max, uint16_t *sample_array) {
/*Generate saw triangle sample - one period depends on v_min(0) and v_max(4095 equals 12V vmax)
 * return:
 * 	1 = OK
 * 	0 = Error: v_min is >= v_max  or  v_max > 4095.
 */

	float step = 1.0 / BIOZAP_Sample_Lgth;

	if ( (v_max>4095) || (v_min>=v_max)) return 0;

	for (int i=0; i < BIOZAP_Sample_Lgth; i++ ){
		sample_array[i] = ( 1-i * step) * (v_max-v_min) + v_min ;

	}

return 1;

}

static uint8_t generate_rec_sample (uint16_t v_min, uint16_t v_max, uint16_t *sample_array) {
/*Generate rectangle sample - one period depends on v_min(0) and v_max(4095 equals 12V vmax)
 * return:
 * 	1 = OK
 * 	0 = Error: v_min is >= v_max  or  v_max > 4095.
 */

	if ( (v_max>4095) || (v_min>=v_max)) return 0;

	for (int i=0; i < BIOZAP_Sample_Lgth; i++ ){

		if ( i < BIOZAP_Sample_Lgth * BIOZAP_DutyCycle / 100.0 ){
			sample_array[i] = v_min;
		} else {
			sample_array[i] = v_max;
		}

	}

return 1;

}

uint8_t generate_sample(uint16_t vmin, uint16_t vampl, uint8_t waveType, uint16_t *sample_array){
/* Generates the sample and scaling it to vmin and vampl
 * return:
 * 	1 = OK
 * 	0 = Error.
*/
	uint16_t _vmin  = vmin;
	uint16_t _vampl = vampl;
	uint16_t _v_min, _v_max;

	if ( _vmin > BIOZAP_VMIN_MAX)           _vmin  = BIOZAP_VMIN_MAX;
	if ( _vmin + _vampl > BIOZAP_VMAX_MAX ) _vampl = BIOZAP_VMAX_MAX - _vmin;

	_v_min = BIOZAP_ONE_VOLT * _vmin / 100.0;
	_v_max = _v_min + BIOZAP_ONE_VOLT * _vampl / 100.0;

	switch(waveType){

		case (BIOZAP_USR): 	return generate_sin_sample (_v_min, _v_max, sample_array); break; //TODO: User sample as in

		case (BIOZAP_REC): 	return generate_rec_sample (_v_min, _v_max, sample_array); break;

		case (BIOZAP_SAW): 	return generate_saw_sample (_v_min, _v_max, sample_array); break;

		//(BIOZAP_SIN) default
		default:	return generate_sin_sample (_v_min, _v_max, sample_array);

	}

}


// arr = sys_clock/prescaler/freq/samlenumber

uint16_t min_Sample(uint32_t freq)
{
	if(freq < 1000)
		return 50; //128
	else
		return 50;
}

freq_item find_time_freq(TIM_HandleTypeDef *htim, double freq)
{
	freq_item element;
   	element.psc = 1;
   	element.arr = 1;
	element.error = 100.0;
	element.freq = 0;
	double TOLERANCE = 0.001;
	double CLOCK_MCU = HAL_RCC_GetSysClockFreq(); //
	double clk = CLOCK_MCU / (freq/100);
	do{
		for (uint16_t psc = min_Sample(freq); psc < BIOZAP_SAMPLE_SIZE; psc++ ){
			double arr = clk / psc;
			uint16_t intarr = (uint16_t)arr;
			double error = abs((intarr - arr)/intarr);
			if( (error < TOLERANCE) && (error < element.error ) && (intarr > 12) && (intarr <= 0xFFFF) ){
				element.psc = psc;
				element.arr = intarr;
				element.error= error;
			}
		}
		TOLERANCE *= 10;
	}while((element.error > 1) && (TOLERANCE < 100));

	if(element.arr > element.psc && element.arr > 50 && element.arr < BIOZAP_SAMPLE_SIZE){
		uint16_t save = element.arr;
		element.arr = element.psc;
		element.psc = save;
	}

	element.freq = CLOCK_MCU / element.psc / element.arr;
	return element;
}


