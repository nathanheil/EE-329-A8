    /*
 * ADC.h
 *
 *  Created on: May 27, 2025
 *      Author: Jesus
 */
#include <stdint.h>


#ifndef INC_ADC_H_
#define INC_ADC_H_

extern volatile uint16_t ADC_Samples[20];
extern volatile uint8_t sample_count;
extern volatile uint8_t samples_ready;

void ADC_init(void);
void ADC1_2_IRQHandler(void);

uint32_t ADC_converts_count_to_mV (uint16_t count);
uint32_t ADC_convert_to_uA(uint16_t count, uint32_t resistor_milli_ohm);

uint16_t ADC_array_minVal(uint16_t array[], uint8_t size);
uint16_t ADC_array_maxVal(uint16_t array[], uint8_t size);
uint16_t ADC_array_avgVal(uint16_t array[], uint8_t size);

void ADC_int_to_string(const char *label, uint16_t count, const char *row);
void ADC_display_current(uint16_t count, uint32_t resistor_milli_ohm,
		const char* row);

#endif /* INC_ADC_H_ */


