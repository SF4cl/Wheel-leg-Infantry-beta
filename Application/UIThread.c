#include "RefereeThread.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "RefereeBehaviour.h"
#include "Client_UI.h"
#include "CanPacket.h"
#include "CalculateThread.h"
#include "UIThread.h"
#include "dma.h"

Graph_Data imagex,imagey,x1,x2,x3,x13,x14;
String_Data aimbot,Mode,open3,noforce;
Graph_Data gun;
Graph_Data balance1,balance2,balance3,balance4;
int mode_flag=0;

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;


extern EulerSystemMeasure_t    Imu;

uint8_t last_PTZstate=0x0,now_state,aim_state=0;
void UISendThread(void * argument)
{
		osDelay(1000);
		while(1)
		{
				if(num>0)
				{
						__HAL_DMA_DISABLE(&hdma_uart5_tx);
						while(locked)	osDelay(1);
						locked=1;					
						SCB_InvalidateDCache_by_Addr(UIsend_buffer+head[num-1],head[num]-head[num-1]);
						HAL_UART_Transmit_DMA(&huart5,UIsend_buffer+head[num-1],head[num]-head[num-1]);
						num--;top=head[num];
						locked=0;
						osDelay(100);
						
				}
				else
						osDelay(1);
		}
}

void UIGenerateThread(void * argument)
{
		num=0;top=0;//__HAL_DMA_DISABLE(&hdma_uart5_tx);locked=0;
		osDelay(1000);
		Char_Draw(&aimbot,"aim",UI_Graph_ADD,0,UI_Color_Green,14,3,2,6750,710,"AIM");
		Char_ReFresh(aimbot);	
		//osDelay(100);
		Char_Draw(&Mode,"mod",UI_Graph_ADD,0,UI_Color_Green,14,4,2,6750,770,"MODE");
		Char_ReFresh(Mode);
		osDelay(100);
		Char_Draw(&noforce,"nof",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,6820,770,"Noforce");
		Char_ReFresh(noforce);
		//osDelay(100);
		Char_Draw(&open3,"oop",UI_Graph_ADD,1,UI_Color_Purplish_red,16,5,2,6820,710,"Closed");
		Char_ReFresh(open3);
		osDelay(100);
		Line_Draw(&balance1, "wd1", UI_Graph_ADD, 2, UI_Color_Orange, 5, Cal_rotation_x(930, 200, YawMotorMeasure.angle), Cal_rotation_y(930, 200, YawMotorMeasure.angle), Cal_rotation_x(990.0, 200.0, YawMotorMeasure.angle), Cal_rotation_y(990, 200, YawMotorMeasure.angle));
		Line_Draw(&balance2, "wd2", UI_Graph_ADD, 2, UI_Color_Orange, 5, Cal_rotation_x(930, 100, YawMotorMeasure.angle), Cal_rotation_y(930, 100, YawMotorMeasure.angle), Cal_rotation_x(990, 100, YawMotorMeasure.angle), Cal_rotation_y(990, 100, YawMotorMeasure.angle));
		Line_Draw(&balance3, "lg1", UI_Graph_ADD, 2, UI_Color_White, 1, Cal_rotation_x(930, 200, YawMotorMeasure.angle), Cal_rotation_y(930, 200, YawMotorMeasure.angle), Cal_rotation_x(930, 100, YawMotorMeasure.angle), Cal_rotation_y(930, 100, YawMotorMeasure.angle));
		Line_Draw(&balance4, "lg2", UI_Graph_ADD, 2, UI_Color_White, 1, Cal_rotation_x(990, 200, YawMotorMeasure.angle), Cal_rotation_y(990, 200, YawMotorMeasure.angle), Cal_rotation_x(990, 100, YawMotorMeasure.angle), Cal_rotation_y(990, 100, YawMotorMeasure.angle));
		Line_Draw(&gun, "gun", UI_Graph_ADD, 0, UI_Color_Pink, 3, 960, 150, 960, 230);
		UI_ReFresh(5, balance1, balance2, balance3, balance4,gun);	
		osDelay(100);
		Line_Draw(&x13,"x13",UI_Graph_ADD,0,UI_Color_Pink,6,6736,40,6940,340);
		Line_Draw(&x14,"x14",UI_Graph_ADD,0,UI_Color_Pink,6,7474,40,7270,340);
		Line_Draw(&x1,"pit",UI_Graph_ADD,0,UI_Color_Orange,6,150,600+(int)(Imu.PitchAngle*3),250,600+(int)(-Imu.PitchAngle*3));
		Line_Draw(&x2,"l01",UI_Graph_ADD,0,UI_Color_Pink,6,200,600,200,600+(int)(l0_left*500));
		Line_Draw(&x3,"l02",UI_Graph_ADD,0,UI_Color_Pink,6,200,600,200,600+(int)(-l0_right*500));	
		UI_ReFresh(5,x1,x2,x3,x13,x14);		
		osDelay(1000);
		uint8_t t=0;
		while(1)
    {
			now_state=Chassis.Mode;
			if(last_PTZstate!=now_state)
			{
					


					if(now_state==NOFORCE&&last_PTZstate!=NOFORCE)
					{
							osDelay(1000);
							//while(locked)	osDelay(1);
							num=0;top=0;//__HAL_DMA_DISABLE(&hdma_uart5_tx);locked=0;
							osDelay(100);
							Char_Draw(&aimbot,"aim",UI_Graph_ADD,0,UI_Color_Green,14,3,2,6750,710,"AIM");
							Char_ReFresh(aimbot);		
							//osDelay(100);
							Char_Draw(&Mode,"mod",UI_Graph_ADD,0,UI_Color_Green,14,4,2,6750,770,"MODE");
							Char_ReFresh(Mode);
							osDelay(100);
							Char_Draw(&noforce,"nof",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,6820,770,"Noforce");
							Char_ReFresh(noforce);
							//osDelay(100);
							Char_Draw(&open3,"oop",UI_Graph_ADD,1,UI_Color_Purplish_red,16,5,2,6820,710,"Closed");
							Char_ReFresh(open3);
							osDelay(100);
							Line_Draw(&x13,"x13",UI_Graph_ADD,0,UI_Color_Pink,6,6736,40,6940,340);
							Line_Draw(&x14,"x14",UI_Graph_ADD,0,UI_Color_Pink,6,7474,40,7270,340);
							Line_Draw(&x1,"pit",UI_Graph_ADD,0,UI_Color_Orange,6,150,600+(int)(Imu.PitchAngle*3),250,600+(int)(-Imu.PitchAngle*3));
							Line_Draw(&x2,"l01",UI_Graph_ADD,0,UI_Color_Pink,6,200,600,200,600+(int)(l0_left*500));
							Line_Draw(&x3,"l02",UI_Graph_ADD,0,UI_Color_Pink,6,200,600,200,600+(int)(-l0_right*500));	
							UI_ReFresh(5,x1,x2,x3,x13,x14);
							osDelay(100);
							Line_Draw(&balance1, "wd1", UI_Graph_ADD, 2, UI_Color_White, 5, Cal_rotation_x(930, 200, YawMotorMeasure.angle), Cal_rotation_y(930, 200, YawMotorMeasure.angle), Cal_rotation_x(990.0, 200.0, YawMotorMeasure.angle), Cal_rotation_y(990, 200, YawMotorMeasure.angle));
							Line_Draw(&balance2, "wd2", UI_Graph_ADD, 2, UI_Color_White, 5, Cal_rotation_x(930, 100, YawMotorMeasure.angle), Cal_rotation_y(930, 100, YawMotorMeasure.angle), Cal_rotation_x(990, 100, YawMotorMeasure.angle), Cal_rotation_y(990, 100, YawMotorMeasure.angle));
							Line_Draw(&balance3, "lg1", UI_Graph_ADD, 2, UI_Color_Orange, 8, Cal_rotation_x(930, 200, YawMotorMeasure.angle), Cal_rotation_y(930, 200, YawMotorMeasure.angle), Cal_rotation_x(930, 100, YawMotorMeasure.angle), Cal_rotation_y(930, 100, YawMotorMeasure.angle));
							Line_Draw(&balance4, "lg2", UI_Graph_ADD, 2, UI_Color_Cyan, 8, Cal_rotation_x(990, 200, YawMotorMeasure.angle), Cal_rotation_y(990, 200, YawMotorMeasure.angle), Cal_rotation_x(990, 100, YawMotorMeasure.angle), Cal_rotation_y(990, 100, YawMotorMeasure.angle));
							Line_Draw(&gun, "gun", UI_Graph_ADD, 0, UI_Color_Pink, 3, 960, 150, 960, 230);
							UI_ReFresh(5, balance1, balance2, balance3, balance4,gun);
							
							osDelay(1000);
							last_PTZstate=NOFORCE;
							aim_state=0;
					}
					if(now_state == ROTING&&last_PTZstate!=ROTING)
					{

						Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_Green,16,6,2,6820,770,"Rotate");
						Char_ReFresh(noforce);
						last_PTZstate=ROTING;
					}
					if(now_state == FALLOW&&last_PTZstate!=FALLOW)
					{

						Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_Cyan,16,6,2,6820,770,"Follow");
						Char_ReFresh(noforce);
						last_PTZstate=FALLOW;
					}
					if(now_state == STOP&&last_PTZstate!=STOP)
					{
						Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_Cyan,16,4,2,6820,770,"Stop");
						Char_ReFresh(noforce);
						last_PTZstate=STOP;
					}
					if(now_state == HIGHSPEED&&last_PTZstate!=HIGHSPEED)
					{
						Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_Cyan,16,9,2,6820,770,"HIGHSPEED");
						Char_ReFresh(noforce);
						last_PTZstate=HIGHSPEED;
					}									
											
			}				

					if(t==0)
					{
							Line_Draw(&balance1, "wd1", UI_Graph_Change, 2, UI_Color_White, 5, Cal_rotation_x(930, 200, YawMotorMeasure.angle), Cal_rotation_y(930, 200, YawMotorMeasure.angle), Cal_rotation_x(990.0, 200.0, YawMotorMeasure.angle), Cal_rotation_y(990, 200, YawMotorMeasure.angle));
							Line_Draw(&balance2, "wd2", UI_Graph_Change, 2, UI_Color_White, 5, Cal_rotation_x(930, 100, YawMotorMeasure.angle), Cal_rotation_y(930, 100, YawMotorMeasure.angle), Cal_rotation_x(990, 100, YawMotorMeasure.angle), Cal_rotation_y(990, 100, YawMotorMeasure.angle));
							Line_Draw(&balance3, "lg1", UI_Graph_Change, 2, UI_Color_Orange, 8, Cal_rotation_x(930, 200, YawMotorMeasure.angle), Cal_rotation_y(930, 200, YawMotorMeasure.angle), Cal_rotation_x(930, 100, YawMotorMeasure.angle), Cal_rotation_y(930, 100, YawMotorMeasure.angle));
							Line_Draw(&balance4, "lg2", UI_Graph_Change, 2, UI_Color_Cyan, 8, Cal_rotation_x(990, 200, YawMotorMeasure.angle), Cal_rotation_y(990, 200, YawMotorMeasure.angle), Cal_rotation_x(990, 100, YawMotorMeasure.angle), Cal_rotation_y(990, 100, YawMotorMeasure.angle));

							Line_Draw(&x13,"x13",UI_Graph_Change,0,UI_Color_Pink,6,6736,40,6940,340);
							Line_Draw(&x14,"x14",UI_Graph_Change,0,UI_Color_Pink,6,7474,40,7270,340);
							Line_Draw(&x1,"pit",UI_Graph_Change,0,UI_Color_Orange,6,150,600+(int)(Imu.PitchAngle*3),250,600+(int)(-Imu.PitchAngle*3));
							Line_Draw(&x2,"l01",UI_Graph_Change,0,UI_Color_Pink,6,200,600,200,600+(int)(l0_left*500));
							Line_Draw(&x3,"l02",UI_Graph_Change,0,UI_Color_Pink,6,200,600,200,600+(int)(-l0_right*500));	
							UI_ReFresh(7, balance1, balance2, balance3, balance4,x1,x2,x3);		
					}

						if(Aim.AimStatus!=0&&aim_state==0)
						{
								aim_state=1;
								Char_Draw(&open3,"oop",UI_Graph_Change,1,UI_Color_Purplish_red,16,4,2,6820,710,"Open");
								Char_ReFresh(open3);
								
						}
						if(Aim.AimStatus==0&&aim_state==1)
						{
								aim_state=0;
								Char_Draw(&open3,"oop",UI_Graph_Change,1,UI_Color_Purplish_red,16,6,2,6820,710,"Closed");
								Char_ReFresh(open3);														
								
						}
				t=(t+1)%150;
				osDelay(1);
    }

 }