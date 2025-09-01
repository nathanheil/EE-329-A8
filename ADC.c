    /*
 * ADC.c
 *
 *  Created on: May 27, 2025
 *      Author: Jesus
 */
#include "ADC.h"
#include "lpuart.h"
#include "delay.h"
#include "main.h"
#include "stm32l4a6xx.h"
#include <stdint.h>

//
volatile uint16_t ADC_Samples[20];
volatile uint8_t sample_count = 0;
volatile uint8_t samples_ready = 0;

/* -----------------------------------------------------------------------------
 * function : void ADC_init(void)
 * INs : none
 * OUTs : none
 * action :
 * authors :
 * version : 1
 * date : 250426
 * -------------------------------------------------------------------------- */
void ADC_init(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;         // turn on clock for ADC
	// power up & calibrate ADC
	ADC123_COMMON->CCR |= (1 << ADC_CCR_CKMODE_Pos); // clock source = HCLK/1
	ADC1->CR &= ~(ADC_CR_DEEPPWD);             // disable deep-power-down
	ADC1->CR |= (ADC_CR_ADVREGEN);             // enable V regulator - see RM 18.4.6
	delay_us(20);                              // wait 20us for ADC to power up
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);    // PA0=ADC1_IN5, single-ended
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF); // disable ADC, single-end calib
	ADC1->CR |= ADC_CR_ADCAL;                  // start calibration
	while (ADC1->CR & ADC_CR_ADCAL) {;}        // wait for calib to finish
	// enable ADC
	ADC1->ISR |= (ADC_ISR_ADRDY);              // set to clr ADC Ready flag
	ADC1->CR |= ADC_CR_ADEN;                   // enable ADC
	while(!(ADC1->ISR & ADC_ISR_ADRDY)) {;}    // wait for ADC Ready flag
	ADC1->ISR |= (ADC_ISR_ADRDY);              // set to clr ADC Ready flag

	// configure ADC sampling & sequencing
	ADC1->SQR1  |= (5 << ADC_SQR1_SQ1_Pos);    // sequence = 1 conv., ch 5
	//ADC1->SMPR1 |= (1 << ADC_SMPR1_SMP5_Pos);  // ch 5 sample time = 6.5 clocks
//	ADC1->SMPR1 &= ~(0b111 << ADC_SMPR1_SMP5_Pos);     // clear previous setting
//	ADC1->SMPR1 |=  (4 << ADC_SMPR1_SMP5_Pos);     // set 47.5 clocks
	ADC1->SMPR1 &= ~(0b111 << ADC_SMPR1_SMP5_Pos);     // clear previous setting
	ADC1->SMPR1 |=  (15 << ADC_SMPR1_SMP5_Pos);     // set 640.5 clocks

	ADC1->CFGR  &= ~( ADC_CFGR_CONT  |         // single conversion mode
	                  ADC_CFGR_EXTEN |         // h/w trig disabled for s/w trig
	                  ADC_CFGR_RES   );        // 12-bit resolution
	// configure & enable ADC interrupt
	ADC1->IER |= ADC_IER_EOCIE;                // enable end-of-conv interrupt
	ADC1->ISR |= ADC_ISR_EOC;                  // set to clear EOC flag
	NVIC->ISER[0] = (1<<(ADC1_2_IRQn & 0x1F)); // enable ADC interrupt service
	__enable_irq();                            // enable global interrupts
	// configure GPIO pin PA0
	RCC->AHB2ENR  |= (RCC_AHB2ENR_GPIOAEN);    // connect clock to GPIOA
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL0);
	GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL0_Pos);
	GPIOA->MODER  |= (GPIO_MODER_MODE0);       // analog mode for PA0 (set MODER last)

	ADC1->CR |= ADC_CR_ADSTART;                // start 1st conversion

}
/* -----------------------------------------------------------------------------
 * function : ADC1_2_IRQHandler(void)
 * INs : none
 * OUTs : none
 * action :
 * authors :
 * version : 1
 * date : 250426
 * -------------------------------------------------------------------------- */
void ADC1_2_IRQHandler(void) {
	if (ADC1->ISR & ADC_ISR_EOC) {
		ADC1->ISR &= ~(ADC_ISR_EOC);
		//ADC_value = ADC1->DR;
		ADC_Samples[sample_count++] = ADC1->DR;

		if (sample_count >= 20) {
			samples_ready = 1;
			sample_count = 0; // reset for next round
		} else {
			ADC1->CR |= ADC_CR_ADSTART; // start next conversion
		}

	}
}

/* -----------------------------------------------------------------------------
 * function : ADC_calibrated_mV(uint16_t counts)
 * INs : raw ADC counts
 * OUTs : calibrated millivolts based on regression
 * authors :
 * version : 1
 * date : 250528
 * -------------------------------------------------------------------------- */
uint32_t ADC_calibrated_mV(uint16_t counts) {
    if (counts < 16) return 0;
    return ((uint32_t)(counts - 16) * 1000) / 1180;
}



/* -----------------------------------------------------------------------------
 * function : uint32_t ADC_converts_count_to_mV() uint32_t ADC_convert_to_uA()
 * INs : none
 * OUTs : none
 * action : converts 12 bit ADC value to millivolts
 * authors :
 * version : 1
 * date : 250426
 * -------------------------------------------------------------------------- */
uint32_t ADC_converts_count_to_mV (uint16_t count){
	return ((uint32_t)((count * 3300) / 4095)) -2;
}

uint32_t ADC_convert_to_uA(uint16_t count, uint32_t resistor_milli_ohm) {
	uint32_t mV = ((uint32_t) count * 3300) / 4095;
	return (mV * 1000) / resistor_milli_ohm;  // uA = (mV * 1000) / mΩ
}

/* -----------------------------------------------------------------------------
 * function : uint16_t ADC_array_minVal()
 * INs : none
 * OUTs : none
 * action : scans through the array and returns the smallest ADC sample value
 * authors :
 * version : 1
 * date : 250426
 * ---------------------------------------------------------------------------*/
uint16_t ADC_array_minVal(uint16_t array[], uint8_t size) {
	uint16_t minVal = 0xFFFF;
	for (uint8_t i = 0; i < size; i++) {
		if (array[i] < minVal)
			minVal = array[i];
	}
	return minVal;
}
/* -----------------------------------------------------------------------------
 * function : uint16_t ADC_array_maxVal()
 * INs : none
 * OUTs : none
 * action : scans through the array and returns the largest ADC sample value
 * authors :
 * version : 1
 * date : 250426
 * ---------------------------------------------------------------------------*/
uint16_t ADC_array_maxVal(uint16_t array[], uint8_t size) {
	uint16_t maxVal = 0;
	for (uint8_t i = 0; i < size; i++) {
		if (array[i] > maxVal)
			maxVal = array[i];
	}
	return maxVal;
}
/* -----------------------------------------------------------------------------
 * function : uint16_t ADC_array_avgVal()
 * INs : none
 * OUTs : none
 * action : adds all ADC sample values together using a 32-bit accumulator and returns the average as a 16-bit result
 * authors :
 * version : 1
 * date : 250426
 * ---------------------------------------------------------------------------*/
uint16_t ADC_array_avgVal(uint16_t array[], uint8_t size){
    uint32_t avgVal = 0;
    for (uint8_t i = 0; i < size; i++) avgVal += array[i];
    return (uint16_t)(avgVal / size);
}
/* -----------------------------------------------------------------------------
 * function :
 * INs : none
 * OUTs : none
 * action : converts a 12-bit ADC count into millivolt and decimal voltage string
 * 			output, and prints it in a formatted row on the terminal using LPUART
 * 			because we can't use sprintf();
 * authors :
 * version : 1
 * date : 250426
 * -------------------------------------------------------------------------- */
void ADC_int_to_string(const char *label, uint16_t count, const char *row){
	// convert 12 bit ADC count to millivolts (0-3300)
	uint32_t millivolt = ADC_calibrated_mV(count);

	// convert millivolts to integer and fraction to display
	uint8_t voltage_int = millivolt / 1000;
	uint16_t voltage_fraction = millivolt % 1000;

	// move cursor to specified row
	LPUART_ESC_Print (row , NULL);

	// print label on row
	LPUART_Print(label);
	LPUART_Print("    ");

	// print ADC count as 4 digits
	for (int d = 1000; d >= 1; d /= 10){
		// take each digit from MSB to LSB
		char c = ((count / d ) % 10 ) + '0';	 // convert to ASCII digit
		while (!(LPUART1->ISR & USART_ISR_TXE)); // wait until TX ready
		LPUART1->TDR = c ;						 // transmit character
	}

	LPUART_Print("    ");

	// print integer of millivolts
	while (!(LPUART1->ISR & USART_ISR_TXE))
		;
	LPUART1->TDR = voltage_int + '0';             // Transmit digit before decimal

	while (!(LPUART1->ISR & USART_ISR_TXE))
		;
	LPUART1->TDR = '.';                     // Transmit decimal point

	// print fraction part of millivolts as 3 digits
	for (int d = 100; d >= 1; d /= 10){
		char c = ((voltage_fraction / d) % 10) + '0';	// get each digit
		while(!(LPUART1->ISR & USART_ISR_TXE));
		LPUART1->TDR = c ;						// transmit each char
	}

	LPUART_Print(" V");
}
/* -----------------------------------------------------------------------------
 * function :	void ADC_display_current(uint16_t count,
 * 				uint32_t resistor_mOhm, const char* row
 * INs : none
 * OUTs : none
 * action :
 * authors :
 * version : 1
 * date : 250426
 * -------------------------------------------------------------------------- */
void ADC_display_current(uint16_t count, uint32_t resistor_milli_ohm, const char* row) {
    uint32_t current_uA = ADC_convert_to_uA(count, resistor_milli_ohm);

    LPUART_ESC_Print(row, NULL);
    LPUART_Print("coil current = ");

    // Print current in mA as integer + 3 decimal digits (e.g., 0.123 A)
    uint32_t current_mA_int = current_uA / 1000;
    uint32_t current_mA_frac = current_uA % 1000;

    char c;

    // Print integer part
    c = current_mA_int + '0';
    while (!(LPUART1->ISR & USART_ISR_TXE));
    LPUART1->TDR = c;

    // Decimal point
    while (!(LPUART1->ISR & USART_ISR_TXE));
    LPUART1->TDR = '.';

    // Print 3-digit fractional part
    for (int d = 100; d >= 1; d /= 10) {
        c = ((current_mA_frac / d) % 10) + '0';
        while (!(LPUART1->ISR & USART_ISR_TXE));
        LPUART1->TDR = c;
    }

    LPUART_Print(" A");
}



