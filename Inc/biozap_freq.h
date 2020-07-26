/*
 * biozap_freq.h
 *
 *  Created on: 30.12.2018
 *      Author: kczoba
 */

#ifndef BIOZAP_FREQ_H_
#define BIOZAP_FREQ_H_


#include <cmath>
#include <algorithm>

using namespace std;

#define PI 3.14159265
#define BIOZAP_SAMPLE_SIZE 1024	// 12bit sample (0-4095) size
#define BIOZAP_ONE_VOLT 349.24  	// Divider 12k/4.7k with AD811 OpAmp equals 1V = 349.24 bits  or 1bit = 0.0028634V
#define BIOZAP_VMIN_MAX 400			//  4.00 V
#define BIOZAP_VMAX_MAX 1100		// 11.00 V


//WavesTypes
#define BIOZAP_USR 0 // User defined wave by the sample
#define BIOZAP_SIN 1 // Sine wave (default)
#define BIOZAP_REC 2 // Square/rectangular wave
#define BIOZAP_SAW 3 // Saw triangle wave


//Global variables
int BIOZAP_Sample_Lgth = BIOZAP_SAMPLE_SIZE; // Change BIOZAP_SAMPLE_SIZE to Lg everywhere except the array definition
uint16_t BIOZAP_SampleArray[ BIOZAP_SAMPLE_SIZE ]; //One period sample array
uint16_t BIOZAP_DutyCycle = 50;

struct freq_item{
	uint16_t psc;
	uint16_t arr;
	float error;
	uint32_t freq;
	uint8_t gcd;
};

static uint8_t generate_sin_sample (uint16_t v_min, uint16_t v_max, uint16_t *sample_array);
static uint8_t generate_saw_sample (uint16_t v_min, uint16_t v_max, uint16_t *sample_array);
static uint8_t generate_rec_sample (uint16_t v_min, uint16_t v_max, uint16_t *sample_array);
static freq_item find_time_freq(TIM_HandleTypeDef *htim, double freq);
void set_time_freq(TIM_HandleTypeDef *htim, uint16_t psc, uint16_t arr);



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

	int j=0;
	for (int i=BIOZAP_Sample_Lgth; i < BIOZAP_SAMPLE_SIZE; i++ ){
		sample_array [i] = sample_array [j++];
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

// period = sys_clock/prescaler/freq/samlenumber
// prescaler 1..65536

static freq_item find_time_freq(TIM_HandleTypeDef *htim, double freq)
{
	double TOLERANCE = 0.001;
	uint32_t CLOCK_MCU = HAL_RCC_GetSysClockFreq(); //
	freq_item element;
   	element.psc = 1;
   	element.arr = 1;
	element.error = 100.0;
	element.freq = 0.0;
	element.gcd = 0;
	do{
		for (uint16_t psc = 1; psc < 0xFFFF; psc++){
			double arr = CLOCK_MCU / (freq * psc);
			double error = abs(((uint16_t)arr - arr)/(uint16_t)arr);
			if( (error < TOLERANCE) && ((uint16_t)arr > 0) && ((uint16_t)arr <= 0xFFFF) && (__gcd(psc, (uint16_t)arr) > element.gcd) ){
				element.psc = psc;
				element.arr = (uint16_t)arr;
				element.error= error;
				element.freq = CLOCK_MCU / psc / (uint16_t)arr;
				element.gcd = __gcd(element.psc, element.arr);
			}
		}
		TOLERANCE *= 10;
	}while((element.error > 1) && (element.gcd <= 1) && (element.arr*element.psc > 12));

/*
   	if (element.arr == 1){  // Sometimes psc=xxx and arr=1 combination not working on DAC, but working if exchanged.
   		element.arr = element.psc;
   		element.psc = 1;
   	}
*/
   	if ((element.psc < element.arr) && (element.arr < BIOZAP_SAMPLE_SIZE) ){
   		uint16_t save_arr = element.arr;
   		element.arr = element.psc;
   		element.psc = save_arr;
   	}

   	if (element.psc*element.gcd < BIOZAP_SAMPLE_SIZE ){
   		element.arr = element.arr/element.gcd;
   		element.psc = element.psc*element.gcd;
   	}

// TODO psc 1 jön, Arr < 12 jön.
// 50 < psc < BIOZAP_SAMPLE_SIZE
// 10..12 < arr < FFFF
// Ezt az összes ellenőrzést a while elé kellene berakni.
// psc páratlan legyen.

   	if (element.psc > BIOZAP_SAMPLE_SIZE ){
   		element.arr = element.arr*element.gcd;
   		element.psc = element.psc/element.gcd;
   		if (element.psc > BIOZAP_SAMPLE_SIZE )
   			element.error = 2;
   	}
/*
	if (element.arr*element.psc <= 12) { // DMA underrun !!!!!!
		element.error = 1;
	}
*/
	return element;
}



#endif /* BIOZAP_FREQ_H_ */
