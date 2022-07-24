//원그리기 함수
#include "stm32f4xx.h"
#include "default.h"
#include "math.h"
#include "_init.h"
#include "xy_figure.h"

void xy_Home_Search();
//x축 동작을 위한 변수들 
int x_cur ;
int x_goal ;
uint8_t x_dir = 3;     //오른쪽 방향 = 0 , 왼쪽 방향 = 1

//y축 동작을 위한 변수들 
int y_cur;
int y_goal ;
uint8_t y_dir = 3;     //위 방향 = 0;  ,  아래 방 향 1


//usart data를 위한 변수들
int save_data[10000];     //32000
char vector_data;       //pc로부터 데이터를 받는 변수 char형
int k;             //배열 +시키기 위한 변수 

//flag 변수
uint8_t move_flag;     //동작하기 위한 flag변수
uint8_t check_flag;     //data check flag 변수
int j = 0;         //flag 범위 지정
int usart_count ;     //범위 설정

int Count_Straight;

int laser_check[50];
int t_i = 0; 
int x_flag=0;
int y_flag=0;

int x_home = 0;
int y_home = 0;

int Move_Mode[1000];
int Is_Bezier_End=0;
int Is_Bezier_Make=0;
int b_i=0;

int main(void)
{
	DelayMS(1000);
	_EXTI_Init(); 
	_GPIO_Init();
	TIMER1_OC_Init();     //  PE9    
	TIMER8_OC_Init();     //  PC6    
	TIMER4_PWM_Init();	  // PB8    
	USART1_Init();
     GPIOF->ODR &= ~(1<<4);

     // 시스템 구동시 원점 탐색
    x_right();
    x_start();
    y_down();
    y_start();
  
	while (1)
	{
      xy_Home_Search();
       
     //데이터 수신 -> 데이터 저장 
      if(check_flag == 1)
      {        
        Data_Check();
        //XY_Data();
        
        DelayMS(500);
        
        move_flag = 1;
        check_flag = 0;
      }
      
      // XY Stage 이동 부분 
      if(move_flag == 1)
      {     
        
        if(j < Count_Straight)
        {
          
          if(laser_check[t_i] == j)
          {
            t_i++;
            GPIOF->ODR ^= (1<<4);            
          }          
                      
          //직선 모드 
          if(Move_Mode[j] == 1){
              j++;            
              Direction();
              Straight();            
          }
          
          //베지어 모드
          else if(Move_Mode[j] == 2) 
          {
              //처음에만 Bezier 배열 생성
              if(Is_Bezier_Make==0)
              {
                Bezier_Coordinate_Make();
                Is_Bezier_Make = 1;

              }
              
              Bezier_Direction();
              Bezier_Straight();

              
              if(Is_Bezier_End == 1)
              {
                j++;
                Is_Bezier_End = 0;
                Is_Bezier_Make = 0;
                b_i=0;
              }
          }
          move_flag = 0;          

          
        }
        
      }
        
      //동작 종료시 Laser OFF 
        if(j == Count_Straight && move_flag == 1)
        {
          GPIOF->ODR &= ~(1<<4);
          // 변수초기화
        }
      
	}// while(1)


}

void TIM1_CC_IRQHandler(void)                                                    //RESET: 0
{
	if ((TIM1->SR & 0x02) != 0)                                               // Capture/Compare 1 interrupt flag
	{
		TIM1->SR &= ~(1 << 1);                                         // CC 1 Interrupt Claer
        
        if(y_dir == 0)
        {
          y_cur++;
          if(y_cur >= y_goal)
          {            
            y_stop();            
            move_flag = 1;
          }
        }
        
        if(y_dir == 1)
        {
          y_cur--;
          if(y_cur <= y_goal)
          {
            y_stop();            
            move_flag = 1;
          }
        }

        if(y_dir == 2)
        {
            y_stop();
            move_flag = 1;
        }
        
	}
}


void TIM8_CC_IRQHandler(void)                                           //RESET: 0
{
	if ((TIM8->SR & 0x02) != 0)                                          // Capture/Compare 1 interrupt flag
	{
		TIM8->SR &= ~(1 << 1);                                                   // CC 1 Interrupt Claer

        if(x_dir == 0)
        {
          x_cur++;
          if(x_cur >= x_goal)
          {            
            x_stop();            
            move_flag = 1;
          }
        }
        
        if(x_dir == 1)
        {
          x_cur--;
           if(x_cur <= x_goal)
          {
            x_stop();            
            move_flag = 1;
          }
        }

        if(x_dir == 2)
        {
            x_stop();
            move_flag = 1;            
        }
	}

}

void EXTI9_5_IRQHandler(void)     //EXTI9_5 인터럽트 핸들러
{

	if (EXTI->PR & 0x0100) 			// EXTI10 Interrupt Pending(발생) 여부?
	{
		EXTI->PR |= 0x0100;  // Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
		
	}

	if (EXTI->PR & 0x0200) 			// EXTI10 Interrupt Pending(발생) 여부?
	{
		EXTI->PR |= 0x0200;  // Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
        
	}

}

//void uart_arr_clear(void)
//{
//	add = 0;
//	for (int i = 0; i < 10; i++)
//		save_data[i] = NULL;
//}

void USART1_IRQHandler(void)	
{       
	if ( (USART1->SR & USART_SR_RXNE) ) // USART_SR_RXNE= 1? RX Buffer Full?
    // #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
	{
		vector_data = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// 수신된 문자 저장
        
		if ((vector_data != 0x0A) && (vector_data != 0x0D) && (vector_data != 0x6E) )		// 받은 문자가 enter가 아니라면
        {
          save_data[k] = vector_data;       //data 저장

          
          
          if(save_data[k] == 'Z')          //직선동작 수행
          {
             check_flag = 1;
             usart_count = k;     
          }
          
          k++;
          
          
          
        }
	} 
        // DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다. 즉 clear 할 필요없음 
}


void xy_Home_Search()
{
        if((GPIOD->IDR & 0x100) == 0 && x_flag ==0)
        {
            DelayMS(50);
            if((GPIOD->IDR & 0x100) == 0)
            {
                x_stop();
                x_flag = 1;
            }
        }
      
        if((GPIOD->IDR & 0x200) == 0)
        {
            DelayMS(50);
            if((GPIOD->IDR & 0x200) == 0 && y_flag == 0)
            {
                y_stop();
                y_flag = 1;
            }
        }
      
        if(x_flag == 1 && y_flag == 1)
        {
          x_left();
          x_start();
          DelayMS(5000);
          x_stop();
          x_flag = 2;
          y_flag = 2;
          
          //X, Y Home 좌표 저장
          x_home = x_cur;
          y_home = y_cur;
        }
        
  
}

