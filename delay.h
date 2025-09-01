    /*
 * delay.h
 *
 *  Created on: Apr 19, 2025
 *      Author: Jesus
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "main.h"

void SysTick_Init(void);
// delay in microseconds using SysTick timer to count CPU clock cycles
void delay_us(const uint32_t time_us);

#endif /* INC_DELAY_H_ */


