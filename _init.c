#include "_init.h"
#include "stm32f4xx.h"
#include "default.h"
#include "math.h"



void _GPIO_Init(void)
{
    // OUTPUT(GPIO F) 
    RCC->AHB1ENR |= (1 << 5);	// RCC_AHB1ENR : GPIOF Clock Enable							
    GPIOF->MODER |= 0x55550000;	// GPIOF 8~15 : Output mode (using pin12~15)				
    GPIOF->OTYPER &= ~0xFF00;	// GPIOF 8~15 : Push-pull	
    GPIOF->OSPEEDR |= 0x55550000;	// GPIOF 8~15 : Output speed 25MHZ Medium speed 
 
    GPIOF->MODER |= 0x00005555;   // GPIOF 8~15 : Output mode (using pin12~15)            
    GPIOF->OTYPER &= ~0x00FF;   // GPIOF 8~15 : Push-pull   
    GPIOF->OSPEEDR |= 0x00005555;   // GPIOF 8~15 : Output speed 25MHZ Medium speed
    
    RCC->AHB1ENR    |=  0x00000040;   // RCC_AHB1ENR : GPIOG(bit#6) Enable                     
    GPIOG->MODER    &= ~0xFFFF0000;   // GPIOG 8~15 : Input mode (0b01)       
    GPIOG->PUPDR    &= ~0xFFFF0000;   //GPIOD 8~15 : Floating input (No Pull-up, pull-down) :reset state

    RCC->AHB1ENR    |=  0x00000008;	// RCC_AHB1ENR : GPIOD(bit#7) Enable							
    GPIOD->MODER 	&= ~0xFFFF0000;	// GPIOD 8~15 : Input mode (reset state)				
	GPIOD->PUPDR 	&= ~0xFFFF0000;	// GPIOD 8~15 : Floating input (No Pull-up, pull-down) :reset state    
        
}

void TIMER1_OC_Init(void)   //y축 모터 동작을 위한 PLUSE
{
  // PE9 : TIM1_CH1
  
  RCC->AHB1ENR |= (1 << 4);      // GPIOE
  RCC->APB2ENR |= (1 << 0);      // TIMER1
  
  GPIOE->MODER |= (2 << 18);           //  PE9
  GPIOE->OSPEEDR |= (3 << 18);
  GPIOE->OTYPER &= ~(1 << 9);           // ~0x1000, GPIOD PIN12 Output type push-pull (reset state)
  GPIOE->PUPDR |= (1 << 18);            // 0x01000000, GPIOD PIN12 Pull-up
  GPIOE->AFR[1] |= (1 << 4);            // Connect TIM1 pins(PE9) to AF1(TIM1.2)
  
  // Setting CR1 : 0x0000 
  TIM1->CR1 &= ~(1 << 4);       // DIR=0(Up counter)(reset state)
  TIM1->CR1 &= ~(1 << 1);       // UDIS=0(Update event Enabled): By one of following events
                                //  Counter Overflow/Underflow, 
                                //  Setting the UG bit Set,
                                //  Update Generation through the slave mode controller 
                                // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
  TIM1->CR1 &= ~(1 << 2);   // URS=0(Update event source Selection): one of following events
  //   Counter Overflow/Underflow, 
  // Setting the UG bit Set,
  //   Update Generation through the slave mode controller 
  // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
  TIM1->CR1 &= ~(1 << 3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
  TIM1->CR1 &= ~(1 << 7);   // ARPE=0(ARR is NOT buffered) (reset state)
  TIM1->CR1 &= ~(3 << 8);    // CKD(Clock division)=00(reset state)
  TIM1->CR1 &= ~(3 << 5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
  // Center-aligned mode: The counter counts Up and DOWN alternatively
  
 
  TIM1->PSC = 168 - 1;   // Prescaler=168, 168MHz/168 = 1MHz
  TIM1->ARR = 10 - 1;   // Auto reload  : 1us * 500 = 0.2ms(period) 
  
  TIM1->DIER |= (1 << 0);            // UIE: Enable Tim4 Update interrupt
  TIM1->EGR |= (1 << 1);            // CC1G(Capture/copare 1 generation) = 1 
  
  // CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
  TIM1->CCMR1 &= ~(3 << 0); // CC1S(CC1 channel) = '0b00' : Output 
  TIM1->CCMR1 &= ~(1 << 2); // OC1FE=0: Output Compare 1 Fast disable 
  TIM1->CCMR1 &= ~(1 << 3); // OC1PE=0: Output Compare 1 preload disable
  TIM1->CCMR1 |= (3 << 4);   // OC1M=0b011 (Output Compare 1 Mode : toggle)
  // OC1REF toggles when CNT = CCR1
  
  // CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
  TIM1->CCER |= (1 << 0);   // CC1E=1: CC1 channel Output Enable
  TIM1->CCER &= ~(1 << 1);   // CC1P=0: CC1 channel Output Polarity 
  
  TIM1->CCR1 = 1;   // TIM1 CCR1 TIM4_Pulse
  
  TIM1->DIER &= ~(1 << 0);   // UIE: Enable Tim4 Update interrupt
  TIM1->DIER |= (1 << 1);   // CC1IE: Enable the Tim4 CC1 interrupt
    
  NVIC->ISER[0] |= (1 << 27);   // Enable Timer1 Capture Compare interrupt
  
  TIM1->BDTR |= (1 << 15);
  TIM1->CR1&= ~(1<<0);
}


void TIMER8_OC_Init(void)   //x축 동작을 위한 pulse 출력 
{
  // PC6 : TIM8_CH1
  
  RCC->AHB1ENR |= (1 << 2);      // GPIOC
  RCC->APB2ENR |= (1 << 1);      // TIMER8
  
  GPIOC->MODER |= (2 << 12);     //  PE9
  GPIOC->OSPEEDR |= (3 << 12);
  GPIOC->OTYPER &= ~(1 << 6);     // ~0x1000, GPIOC PIN6 Output type push-pull (reset state)
  GPIOC->PUPDR |= (1 << 12);      // 0x01000000, GPIOC PIN6 Pull-up
  GPIOC->AFR[0] |= (3 << 4 * 6);  // Connect TIM1 pins(PE9) to AF1(TIM1.2)
  
  // Setting CR1 : 0x0000 
  TIM8->CR1 &= ~(1 << 4);   // DIR=0(Up counter)(reset state)
  TIM8->CR1 &= ~(1 << 1);   // UDIS=0(Update event Enabled): By one of following events
  //  Counter Overflow/Underflow, 
  //  Setting the UG bit Set,
  //  Update Generation through the slave mode controller 
  // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
  TIM8->CR1 &= ~(1 << 2);   // URS=0(Update event source Selection): one of following events
  //   Counter Overflow/Underflow, 
  // Setting the UG bit Set,
  //   Update Generation through the slave mode controller 
  // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
  TIM8->CR1 &= ~(1 << 3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
  TIM8->CR1 &= ~(1 << 7);   // ARPE=0(ARR is NOT buffered) (reset state)
  TIM8->CR1 &= ~(3 << 8);    // CKD(Clock division)=00(reset state)
  TIM8->CR1 &= ~(3 << 5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
  // Center-aligned mode: The counter counts Up and DOWN alternatively
    
  TIM8->PSC = 168 - 1;   // Prescaler=168, 168MHz/168 = 1MHz
  TIM8->ARR = 10 - 1;   // Auto reload  : (1us * 20) * 2(펄스주기)  = 40us(period) 
  
  TIM8->DIER &= ~(1 << 0);  // UIE: Enable Tim8 Update interrupt
  TIM8->DIER |= (1 << 1);   // CC1IE: Enable the Tim8 CC1 interrupt

  TIM8->EGR |= (1 << 1);   // CC1G(Capture/copare 1 generation) = 1 
  
  // CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
  TIM8->CCMR1 &= ~(3 << 0); // CC1S(CC1 channel) = '0b00' : Output 
  TIM8->CCMR1 &= ~(1 << 2); // OC1FE=0: Output Compare 1 Fast disable 
  TIM8->CCMR1 &= ~(1 << 3); // OC1PE=0: Output Compare 1 preload disable(CCR
  TIM8->CCMR1 |= (3 << 4);   // OC1M=0b011 (Output Compare 1 Mode : toggle)
  // OC1REF toggles when CNT = CCR1
  
  // CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
  TIM8->CCER |= (1 << 0);   // CC1E=1: CC1 channel Output Enable
  // OC1(TIM4_CH1) Active: 
  TIM8->CCER &= ~(1 << 1);   // CC1P=0: CC1 channel Output Polarity (OCPolarity_High : 
  
  TIM8->CCR1 = 1;   // TIM1 CCR1 TIM4_Pulse
      
  TIM8->DIER &= ~(1 << 0);   // UIE: Enable Tim4 Update interrupt
  TIM8->DIER |= (1 << 1);   // CC1IE: Enable the Tim4 CC1 interrupt
  
  NVIC->ISER[1] |= (1 << 46 - 32);   // Enable Timer8 Capture Compare interrupt
  
  TIM8->BDTR |= (1 << 15);
  TIM8->CR1 &= ~(1<<0);
}

void TIMER4_PWM_Init(void)     //laser 세기를 위한 pulse 
{   
// TIM CH3 : PB8 (167번 핀)
// Clock Enable : GPIOB & TIMER4
	RCC->AHB1ENR	|= (1<<1);	// GPIOB CLOCK Enable
	RCC->APB1ENR 	|= (1<<2);	// TIMER4 CLOCK Enable 
    						
// PB8을 출력설정하고 Alternate function(TIM4_CH3)으로 사용 선언 : PWM 출력
	GPIOB->MODER 	|= (2<<16);	// 0x00020000 PB8 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= (3<<16);	// 0x00030000 PB8 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<8);	// PB8 Output type push-pull (reset state)
	GPIOB->PUPDR	|= (1<<16);	// 0x00010000 PB8 Pull-up
 	GPIOB->AFR[1]	|= (2<<0);	// 0x00000002 (AFR[1].(3~0)=0b0010): Connect TIM4 pins(PB8) to AF2(TIM3..5)
    
// TIM4 Channel 3 : PWM 1 mode
	// Assign 'PWM Pulse Period'
	TIM4->PSC	= 840-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz(0.1ms)  (1~65536)
	TIM4->ARR	= 100-1;	// Auto reload  (0.1ms * 100 = 10ms : PWM Period)

	// Setting CR1 : 0x0000 (Up counting)
	TIM4->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM4->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled)
	TIM4->CR1 &= ~(1<<2);	// URS=0(Update event source Selection)g events
	TIM4->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM4->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enable 
	TIM4->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM4->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)
				
	// Define the corresponding pin by 'Output'  
	// CCER(Capture/Compare Enable Register) : Enable "Channel 3" 
	TIM4->CCER	|= (1<<8);	// CC3E=1: OC3(TIM4_CH3) Active(Capture/Compare 3 output enable)
					// 해당핀(167번)을 통해 신호출력
	TIM4->CCER	&= ~(1<<9);	// CC3P=0: CC3 Output Polarity (OCPolarity_High : OC3으로 반전없이 출력)

	// Duty Ratio 
	TIM4->CCR3	= 50;		// CCR3 value

	// 'Mode' Selection : Output mode, PWM 1
	// CCMR2(Capture/Compare Mode Register 2) : Setting the MODE of Ch3 or Ch4
	TIM4->CCMR2 &= ~(3<<0); // CC3S(CC3 channel)= '0b00' : Output 
	TIM4->CCMR2 |= (1<<3); 	// OC3PE=1: Output Compare 3 preload Enable
	TIM4->CCMR2	|= (6<<4);	// OC3M=0b110: Output compare 3 mode: PWM 1 mode
	TIM4->CCMR2	|= (1<<7);	// OC3CE=1: Output compare 3 Clear enable
	
	//Counter TIM5 enable
	TIM4->CR1	|= (1<<0);	// CEN: Counter TIM4 enable
}

void _EXTI_Init(void)   //원점 설정을 위한 EXTI
{
    RCC->AHB1ENR 	|= 0x00000008;	// RCC_AHB1ENR GPIOD Enable
     
	RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
	
	GPIOD->MODER 	&= ~0xFFFF0000;	// GPIOD8 ~ 15 INPUT MODE
	
	SYSCFG->EXTICR[2] |= 0x0033; 	// EXTI8, EXTI9
    
        
	EXTI->FTSR |= 0x000300;		// EXTI12,14: Falling Trigger  Enable 

   	EXTI->IMR  |= 0x000300;  		// EXTI8,9 인터럽트 mask (Interrupt Enable) 설정
		
	NVIC->ISER[0] |= (1<<23);  // Enable 'Global Interrupt EXTI8,9'
    
}


void USART1_Init(void)   //PC 통신을 위한 USART INIT
{
	// USART1 : TX(PA9)
	RCC->AHB1ENR	|= (1<<0);	// RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER	|= (2<<2*9);	// GPIOA PIN9 Output Alternate function mode					
	GPIOA->OSPEEDR	|= (3<<2*9);	// GPIOA PIN9 Output speed (100MHz Very High speed)
	GPIOA->AFR[1]	|= (7<<4);	// Connect GPIOA pin9 to AF7(USART1)
    
	// USART1 : RX(PA10)
	GPIOA->MODER 	|= (2<<2*10);	// GPIOA PIN10 Output Alternate function mode
	GPIOA->OSPEEDR	|= (3<<2*10);	// GPIOA PIN10 Output speed (100MHz Very High speed
	GPIOA->AFR[1]	|= (7<<8);	// Connect GPIOA pin10 to AF7(USART1)

	RCC->APB2ENR	|= (1<<4);	// RCC_APB2ENR USART1 Enable
    
	USART_BRR_Configuration(9600); // USART Baud rate Configuration
    
	USART1->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
	USART1->CR1	&= ~(1<<10);	// NO USART_Parity
	USART1->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	USART1->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
    
	USART1->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	USART1->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
	NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37번)
	USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
}

void SerialSendChar(uint8_t Ch) // 1문자 보내기 함수
{
        while((USART1->SR & USART_SR_TXE) == RESET); // USART_SR_TXE=(1<<7), 송신 가능한 상태까지 대기

	USART1->DR = (Ch & 0x1FF);	// 전송 (최대 9bit 이므로 0x01FF과 masking)
}

void SerialSendString(char *str) // 여러문자 보내기 함수
{
	while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
	{
		SerialSendChar(*str);	// 포인터가 가르키는 곳의 데이터를 송신
		str++; 			// 포인터 수치 증가
	}
}

// Baud rate  
void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;	//PCLK2_Frequency
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	// Determine the integer part 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
        {                                         // USART1->CR1.OVER8 = 1 (8 oversampling)
		// Computing 'Integer part' when the oversampling mode is 8 Samples 
		integerdivider = ((25*APB2clock) / (2*USART_BaudRate));    
	}
	else  // USART1->CR1.OVER8 = 0 (16 oversampling)
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = (( 25*APB2clock ) / (4*USART_BaudRate  ));    
	}
	tmpreg = (integerdivider/100) << 4;
  
	// Determine the fractional part 
	fractionaldivider = integerdivider - ( 100* (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)	// 8 oversampling
	{
		tmpreg |= ((( fractionaldivider*8 ) + 50 ) / 100) & ( 0x07 );
	}
	else 						// 16 oversampling
	{
		tmpreg |= ((( fractionaldivider*16  ) + 50) / 100) & ( 0x0F );
	}

	// Write to USART BRR register
	USART1->BRR = (uint16_t)tmpreg;
}

void DelayMS(unsigned short wMS)
{
  register unsigned short i;
  for (i = 0; i<wMS; i++)
    DelayUS(1000);   // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
  volatile int Dly = (int)wUS * 17;
  for (; Dly; Dly--);
}

void x_stop(void)
{  
  GPIOF->ODR |= (1 << 14);	       
  
  TIM8->CR1 &= ~(1 << 0);
  
}
void x_start(void)
{
  GPIOF->ODR &= ~(1 << 14);	        
  
  TIM8->CR1 |= (1 << 0);
}
void x_right(void)
{
  GPIOF->ODR |= (1 << 15);	        
}
void x_left(void)
{
  GPIOF->ODR &= ~(1 << 15);	        
}

void y_stop(void)
{
  GPIOF->ODR |= (1 << 12);	        
  TIM1->CR1 &= ~(1 << 0);
}

void y_start(void)
{  
  GPIOF->ODR &= ~(1 << 12);	        
  TIM1->CR1 |= (1 << 0);
  
}

void y_up(void)
{
  GPIOF->ODR &= ~(1 << 13);	            
}
void y_down(void)
{
  GPIOF->ODR |= (1 << 13);	       
}

//void uart_arr_clear(void)
//{
//	add = 0;
//	for (int i = 0; i < 10; i++)
//		save_data[i] = NULL;
//}
