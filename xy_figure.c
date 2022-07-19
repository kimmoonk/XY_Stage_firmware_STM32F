#include "stm32f4xx.h"
#include "_init.h"
#include "xy_figure.h"
#include "math.h"
#include "stdlib.h"

//x축 동작을 위한 변수(전역변수)
extern int x_cur;
extern int x_goal;
extern uint8_t x_dir;     //오른쪽 방향 = 0 , 왼쪽 방향 = 1

//y축 동작을 위한 변수(전역변수)
extern int y_cur;
extern int y_goal;
extern uint8_t y_dir;     //위 방향 = 0;  ,  아래 방향 = 0;

//usart data를 위한 변수들
extern int save_data[5000];     //32000
extern uint8_t move_flag;
extern int usart_count;     //범위 설정

int i = 0;

int loc_x[400];     //x좌표 위치 확인 변수
int loc_y[400];     //y좌표 위치 확인 변수
int x = 0;
int y = 0;

double x_num[400];
double y_num[400];
int m = 0;
int n = 0;
int o = 0;
int p = 0;

extern int laser_check[50];
int laser_power[50];
int l_i = 0;
int l_j = 0;

int motor_arr = 100;
int resolution= 1500;

long int velo_x;
long int velo_y;

extern int Count_Straight;

// double B_X[4]={10.0,10.0,60.0,60.0};
// double B_Y[4]={10.0,60.0,60.0,10.0};
// int t_count=0;

void Straight()
{
  
  switch(Direction())
  {
    case 0:
        
        x_goal = resolution*x_num[i++];         //x축 + 방향으로 이동
        TIM8->ARR = motor_arr - 1;   // Auto reload  : 1us * 500 = 0.5ms(period) 
        
        x_right();
        x_start();
        y_stop();
        
        break;
        
    case 1:
        
        x_goal = resolution*x_num[i++];         //x축 - 방향으로 이동 
        TIM8->ARR = motor_arr - 1;   // Auto reload  : 1us * 500 = 0.5ms(period) 
        
        x_left();
        x_start();
        y_stop();
        
                
        break;
        
    case 2:
        
        y_goal = resolution*y_num[i++];         //y축 + 방향으로 이동
        TIM1->ARR = motor_arr - 1;   // Auto reload  : 1us * 500 = 0.2ms(period) 
        
        y_up();
        y_start();
        x_stop();
                
        break;
        
    case 3:
        
        y_goal = resolution*y_num[i++];         //y축 - 방향으로 이동
        TIM1->ARR = motor_arr - 1;   // Auto reload  : 1us * 500 = 0.2ms(period) 
        
        y_down();
        y_start();
        x_stop();

                
        break;
        
    case 4:
        
        x_goal = resolution*x_num[i];
        y_goal = resolution*y_num[i];         //x축 + 방향, y축 + 방향으로 이동        

//        velo_x = x_num[i-1] - x_num[i];    // x_num[i-l] - x_num[i]
//        velo_y = y_num[i-1] - y_num[i];
        
        velo_x = (resolution*x_num[i-1]) - x_goal;    // x_num[i-l] - x_num[i]
        velo_y = (resolution*y_num[i-1]) - y_goal;
        i++;
        
        TIM1->ARR = (int)motor_arr*(sqrt(pow(velo_x, 2) + pow(velo_y, 2)) / abs(velo_y))-1;   // Auto reload  : 1us * 500 = 0.2ms(period) 
        TIM8->ARR = (int)motor_arr*(sqrt(pow(velo_x, 2) + pow(velo_y, 2)) / abs(velo_x))-1;   // Auto reload  : 1us * 500 = 0.2ms(period) 

        
        x_right();
        y_up();
        x_start();
        y_start();
                
        break;
  
    case 5:
        
        x_goal = resolution*x_num[i];
        y_goal = resolution*y_num[i];         //x축 + 방향, y축 - 방향으로 이동
        
//        velo_x = x_num[i-1] - x_num[i];    // x_num[i-l] - x_num[i]
//        velo_y = y_num[i-1] - y_num[i];
        velo_x = (resolution*x_num[i-1]) - x_goal;    // x_num[i-l] - x_num[i]
        velo_y = (resolution*y_num[i-1]) - y_goal;

        i++;
        
        TIM1->ARR = (int)motor_arr*(sqrt(pow(velo_x, 2) + pow(velo_y, 2)) / abs(velo_y))-1;   // Auto reload  : 1us * 500 = 0.2ms(period) 
        TIM8->ARR = (int)motor_arr*(sqrt(pow(velo_x, 2) + pow(velo_y, 2)) / abs(velo_x))-1;   // Auto reload  : 1us * 500 = 0.2ms(period) 

        
        x_right();
        y_down();
        x_start();
        y_start();
                
        break;
    
    case 6:
        
        x_goal = resolution*x_num[i];
        y_goal = resolution*y_num[i];         //x축 - 방향, y축 + 방향으로 이동
        
//        velo_x = x_num[i-1] - x_num[i];    // x_num[i-l] - x_num[i]
//        velo_y = y_num[i-1] - y_num[i];

        velo_x = (resolution*x_num[i-1]) - x_goal;    // x_num[i-l] - x_num[i]
        velo_y = (resolution*y_num[i-1]) - y_goal;

        i++;
        
        TIM1->ARR = (int)motor_arr*(sqrt(pow(velo_x, 2) + pow(velo_y, 2)) / abs(velo_y))-1;   // Auto reload  : 1us * 500 = 0.2ms(period) 
        TIM8->ARR = (int)motor_arr*(sqrt(pow(velo_x, 2) + pow(velo_y, 2)) / abs(velo_x))-1;   // Auto reload  : 1us * 500 = 0.2ms(period) 

        
        x_left();
        y_up();
        x_start();
        y_start();
                
        break;

    case 7:
        
        x_goal = resolution*x_num[i];
        y_goal = resolution*y_num[i];         //x축 - 방향, y축 - 방향으로 이동
        
//        velo_x = x_num[i-1] - x_num[i];    // x_num[i-l] - x_num[i]
//        velo_y = y_num[i-1] - y_num[i];

        velo_x = (resolution*x_num[i-1]) - x_goal;    // x_num[i-l] - x_num[i]
        velo_y = (resolution*y_num[i-1]) - y_goal;

        i++;
        
        
        TIM1->ARR = (int)motor_arr*(sqrt(pow(velo_x, 2) + pow(velo_y, 2)) / abs(velo_y))-1;   // Auto reload  : 1us * 500 = 0.2ms(period) 
        TIM8->ARR = (int)motor_arr*(sqrt(pow(velo_x, 2) + pow(velo_y, 2)) / abs(velo_x))-1;   // Auto reload  : 1us * 500 = 0.2ms(period) 
        
        
        x_left();
        y_down();
        x_start();
        y_start();
                
        break;
            
    case 8:
        i++;
                
        x_start();
        y_start();                
        
        break;
        
  }
  
}

uint8_t Direction(void)      //방향 설정 함수
{

///////////////////////직선 동작 수행////////////////////////////////////  
  if(x_num[i-1]<x_num[i] && y_num[i-1] == y_num[i])      //x축 +방향으로 이동 , y축 고정
  {
    x_dir = 0;
    return 0;
  }

  else if(x_num[i-1]>x_num[i] && y_num[i-1] == y_num[i])     //x축 -방향으로 이동 , y축 고정
  {
    x_dir = 1;
    return 1;
  }
  
  else if(x_num[i-1]==x_num[i] && y_num[i-1] < y_num[i])    //x축 고정 , y축 + 방향으로 이동
  {
    y_dir = 0;
    return 2;
  }
  
  else if(x_num[i-1]==x_num[i] && y_num[i-1] > y_num[i])    //x축 고정, y축 - 방향으로 이동
  {
    y_dir = 1;
    return 3;
  }

////////////////////대각선 동작 수행/////////////////////////////////////////  
  else if(x_num[i-1] < x_num[i] && y_num[i-1] < y_num[i])   //x축 +방향 이동 , y축 + 방향 이동
  {
    x_dir = 0;
    y_dir = 0;
    return 4;
  }
  
  else if(x_num[i-1] < x_num[i] && y_num[i-1] > y_num[i])   //x축 + 방향 이동, y축 - 방향 이동
  {
    x_dir = 0;
    y_dir = 1;
    return 5;
  }

  else if(x_num[i-1] > x_num[i] && y_num[i-1] < y_num[i])   //x축 - 방향 이동, y축 + 방향 이동
  {
    x_dir = 1;
    y_dir = 0;
    return 6;
  }

  else if(x_num[i-1] > x_num[i] && y_num[i-1] > y_num[i])   //x축 - 방향 이동, y축 - 방향 이동
  {
    x_dir = 1;
    y_dir = 1;
    return 7;
  }
  
   else if(x_num[i-1] == x_num[i] && y_num[i-1] == y_num[i])   //x 축 정지, y축 정지
  {
    x_dir = 2;
    y_dir = 2;
    return 8;
  }
  
}

int Bezier_number;
int Bezier_Move[1000];
int Bezier_Turn=0;
int Count_Bezier=0;
int B_X[1000];
int B_Y[1000];

int Move_Mode[1000];
int Move_num = 0;
//직선은 1 
//베지어는 2 Move

void Data_Check(void)     //좌표 위치 data check 함수
{
  for(int c_j = 0; c_j < usart_count; c_j++)
  {
      if(save_data[c_j] == 'L')   //동작 갯수 check
        {
          Count_Straight++;      
          x_num[o++] = (10*((int)(save_data[c_j+2]-0x30)))+((int)(save_data[c_j+3]-0x30))+(0.1*((int)(save_data[c_j+5]-0x30)));
          y_num[p++] = (10*((int)(save_data[c_j+7]-0x30)))+((int)(save_data[c_j+8]-0x30))+(0.1*((int)(save_data[c_j+10]-0x30)));
          Move_Mode[Move_num] = 1; // 직선은 1
          Move_num++;
        }
      
     else if(save_data[c_j] == 'S')  //laser position check
      {
          laser_check[l_i] = Count_Straight;    
          laser_power[l_i] = c_j;
           l_i++;
      }
            
      //베지어 곡선 Data 처리
     else if(save_data[c_j] == 'B')
     {
       Count_Bezier++;
       Bezier_Move[Bezier_Turn++] = Count_Straight;
       
       
     }
      
   
  }
}

void laser_control(void)
{
  if(save_data[laser_power[l_j++]+1]-0x30  == 1){
//       TIM4->CCR3 = 10;      // CCR3 value
       GPIOF->ODR |= (1<<4);    
  }
  
  else if(save_data[laser_power[l_j++]+1]-0x30  == 0){
//       TIM4->CCR3 = 0;      // CCR3 value
       GPIOF->ODR &= ~(1<<4);    
  }
  
}

void XY_Data(void)
{
  for(int c_i = 0; c_i < Count_Straight; c_i++)
  {
    x_num[o++] = (10*((int)(save_data[loc_x[m]+1]-0x30)))+((int)(save_data[loc_x[m]+2]-0x30))+(0.1*((int)(save_data[loc_x[m]+4]-0x30)));
    y_num[p++] = (10*((int)(save_data[loc_y[n]+1]-0x30)))+((int)(save_data[loc_y[n]+2]-0x30))+(0.1*((int)(save_data[loc_y[n]+4]-0x30)));
    m++;
    n++;
      
//    GPIOF -> ODR |= (1<<11);
  }
}


//void Bezier(void)
//{  
//    for (double t = 0.0; t <= 1.01; t+=0.01)
//    {
//     x_num[t_count] = (pow(1 - t, 3) * B_X[0] 
//        + 3 * t * pow(1 - t, 2) * B_X[1] 
//        + 3 * pow(t, 2) * (1 - t) * B_X[2]
//        + pow(t, 3) * B_X[3]);
//
//
//     y_num[t_count] = (pow(1 - t, 3) * B_Y[0]
//        + 3 * t * pow(1 - t, 2) * B_Y[1]
//        + 3 * pow(t, 2) * (1 - t) * B_Y[2]
//        + pow(t, 3) * B_Y[3]);
//     
//     t_count++;
//    }
//}