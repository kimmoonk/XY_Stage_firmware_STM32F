#ifndef   ___INIT_H__
#define   ___INIT_H__

#include "stm32f4xx.h"

extern void _GPIO_Init(void);

extern void TIMER1_OC_Init(void);   //  PE9    
extern void TIMER8_OC_Init(void);    //  PC6 
extern void TIMER4_PWM_Init(void);	// Timer4 PWM mode 


extern void USART1_Init(void);
extern void USART_BRR_Configuration(uint32_t USART_BaudRate);
extern void SerialSendChar3(uint8_t Ch); // 1문자 보내기 함수
extern void SerialSendString3(char* str); // 여러문자 보내기 함수

extern void DelayMS(unsigned short wMS);
extern void DelayUS(unsigned short wUS);

extern void x_stop(void);
extern void x_start(void);
extern void x_right(void);
extern void x_left(void);

extern void y_start(void);
extern void y_stop(void);
extern void y_up(void);
extern void y_down(void);
extern void uart_arr_clear(void);

void _EXTI_Init(void);   //EXTI 초기화 함수
void _GPIO_Init(void);


#endif
