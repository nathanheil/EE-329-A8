    /*
 * lpuart.h
 *
 *  Created on: May 27, 2025
 *      Author: Jesus
 */

#ifndef INC_LPUART_H_
#define INC_LPUART_H_

void LPUART1_init(void);
void LPUART_ESC_Print(const char *esc_sequence, const char *text);
void LPUART_Print(const char *message);


#endif /* INC_LPUART_H_ */


