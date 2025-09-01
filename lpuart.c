    /*
 * lpuart.c
 *
 *  Created on: May 27, 2025
 *      Author: Jesus
 */

#include "lpuart.h"
#include "stm32l4a6xx.h"
#include <stddef.h>


/* -----------------------------------------------------------------------------
 * function : LPUART1_init(void)
 * INs : none
 * OUTs : none
 * action :	initializes clocks, sets modes and alt functions and
 * 			enables interrupts
 * authors : Jesus Martinez Kush Patel
 * version : 1
 * date : 250423
 * -------------------------------------------------------------------------- */
void LPUART1_init(void) {
	PWR->CR2 |= (PWR_CR2_IOSV); 				//power avail at PG[15:2]
	// (LPUART1)
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOGEN);  	//enable GPIOG clock
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; 	//enable LPUART clock bridge
	RCC->CCIPR &= ~(0b11 << RCC_CCIPR_LPUART1SEL_Pos);   // Clear bits
	RCC->CCIPR |= (0b01 << RCC_CCIPR_LPUART1SEL_Pos);  	 // Select MSI (01)
	GPIOG->MODER &= ~(GPIO_MODER_MODE7_Msk); 		// Clear mode bits for pin 7
	GPIOG->MODER |= GPIO_MODER_MODE7_1;				// alternate function mode
	GPIOG->AFR[0] &= ~(GPIO_AFRL_AFSEL7_Msk); 			// clear PG7 bits in
	// AFR[0]
	GPIOG->AFR[0] |= (0x0008 << GPIO_AFRL_AFSEL7_Pos);  // set AF8 for PG7
	// (LPUART1_TX)
	GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk); 		//No pull-up/pull-down
	GPIOG->OTYPER &= ~(GPIO_OTYPER_OT7_Msk); 		//Push-pull output
	GPIOG->OSPEEDR |= (GPIO_OSPEEDR_OSPEED7_1); 	//High speed
	GPIOG->MODER &= ~(GPIO_MODER_MODE8_Msk); 		//Clear mode bits for pin 8
	GPIOG->MODER |= GPIO_MODER_MODE8_1;
	GPIOG->AFR[1] &= ~(GPIO_AFRH_AFSEL8_Msk); 			// clear PG8 bits in
	// AFR[1]
	GPIOG->AFR[1] |= (0x0008 << GPIO_AFRH_AFSEL8_Pos);  // set AF8 for PG8
	// (LPUART1_RX)
	GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPD8_Msk); 			//No pull-up/pull-down
	GPIOG->OTYPER &= ~(GPIO_OTYPER_OT8_Msk); 			//Push-pull output
	GPIOG->OSPEEDR |= (GPIO_OSPEEDR_OSPEED8_1); 		//High speed
	LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); 	//8-bit data
	LPUART1->CR1 |= USART_CR1_UE; 						//enable LPUART1
	LPUART1->CR1 |= (USART_CR1_TE | USART_CR1_RE); 		//enable xmit & recv
	//enable LPUART1 recv interrupt
	LPUART1->CR1 |= USART_CR1_RXNEIE;
	LPUART1->ISR &= ~(USART_ISR_RXNE); 					//clear ISR flag
	LPUART1->BRR = 0x22B9;
	NVIC->ISER[2] = (1 << (LPUART1_IRQn & 0x1F)); 		//enable LPUART1 ISR
	__enable_irq();                       				// enable global
	// interrupts
}
/* -----------------------------------------------------------------------------
 * function : LPUART_ESC_Print(const char *esc_sequence, const char *text)
 * INs :
 * OUTs :
 * action : Similar to LPUART_Print() but for escape codes
 * authors : Jesus Martinez Kush Patel
 * version : 1
 * date : 250423
 * -------------------------------------------------------------------------- */
void LPUART_ESC_Print(const char *esc_sequence, const char *text) {
	LPUART_Print("\x1B[");       		// print ESC character (ESC = 0x1B)
	LPUART_Print(esc_sequence);        	// print the escape sequence after ESC
	if (text != NULL) {					// only if text is NOT null, print our
		// text we input
		LPUART_Print(text);
	}
}
/* -----------------------------------------------------------------------------
 * function : LPUART_Print( constant char* message )
 * INs :
 * OUTs :
 * action : Transmits a string of characters by repeating the TXE-TDR process
 * 			for each character in the string until a string-terminating
 * 			NULL character (numeric 0) is encountered.
 * authors : Professor Penvenne
 * version : 1
 * date : 250423
 * -------------------------------------------------------------------------- */
void LPUART_Print(const char *message) {
	uint16_t iStrIdx = 0;
	while (message[iStrIdx] != 0) {
		while (!(LPUART1->ISR & USART_ISR_TXE))
			// wait for empty xmit buffer
			;
		LPUART1->TDR = message[iStrIdx];       // send this character
		iStrIdx++;                             // advance index to next char
	}
}


