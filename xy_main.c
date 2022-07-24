//���׸��� �Լ�
#include "stm32f4xx.h"
#include "default.h"
#include "math.h"
#include "_init.h"
#include "xy_figure.h"

void xy_Home_Search();
//x�� ������ ���� ������ 
int x_cur ;
int x_goal ;
uint8_t x_dir = 3;     //������ ���� = 0 , ���� ���� = 1

//y�� ������ ���� ������ 
int y_cur;
int y_goal ;
uint8_t y_dir = 3;     //�� ���� = 0;  ,  �Ʒ� �� �� 1


//usart data�� ���� ������
int save_data[10000];     //32000
char vector_data;       //pc�κ��� �����͸� �޴� ���� char��
int k;             //�迭 +��Ű�� ���� ���� 

//flag ����
uint8_t move_flag;     //�����ϱ� ���� flag����
uint8_t check_flag;     //data check flag ����
int j = 0;         //flag ���� ����
int usart_count ;     //���� ����

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

     // �ý��� ������ ���� Ž��
    x_right();
    x_start();
    y_down();
    y_start();
  
	while (1)
	{
      xy_Home_Search();
       
     //������ ���� -> ������ ���� 
      if(check_flag == 1)
      {        
        Data_Check();
        //XY_Data();
        
        DelayMS(500);
        
        move_flag = 1;
        check_flag = 0;
      }
      
      // XY Stage �̵� �κ� 
      if(move_flag == 1)
      {     
        
        if(j < Count_Straight)
        {
          
          if(laser_check[t_i] == j)
          {
            t_i++;
            GPIOF->ODR ^= (1<<4);            
          }          
                      
          //���� ��� 
          if(Move_Mode[j] == 1){
              j++;            
              Direction();
              Straight();            
          }
          
          //������ ���
          else if(Move_Mode[j] == 2) 
          {
              //ó������ Bezier �迭 ����
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
        
      //���� ����� Laser OFF 
        if(j == Count_Straight && move_flag == 1)
        {
          GPIOF->ODR &= ~(1<<4);
          // �����ʱ�ȭ
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

void EXTI9_5_IRQHandler(void)     //EXTI9_5 ���ͷ�Ʈ �ڵ鷯
{

	if (EXTI->PR & 0x0100) 			// EXTI10 Interrupt Pending(�߻�) ����?
	{
		EXTI->PR |= 0x0100;  // Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
		
	}

	if (EXTI->PR & 0x0200) 			// EXTI10 Interrupt Pending(�߻�) ����?
	{
		EXTI->PR |= 0x0200;  // Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
        
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
		vector_data = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
        
		if ((vector_data != 0x0A) && (vector_data != 0x0D) && (vector_data != 0x6E) )		// ���� ���ڰ� enter�� �ƴ϶��
        {
          save_data[k] = vector_data;       //data ����

          
          
          if(save_data[k] == 'Z')          //�������� ����
          {
             check_flag = 1;
             usart_count = k;     
          }
          
          k++;
          
          
          
        }
	} 
        // DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 
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
          
          //X, Y Home ��ǥ ����
          x_home = x_cur;
          y_home = y_cur;
        }
        
  
}

