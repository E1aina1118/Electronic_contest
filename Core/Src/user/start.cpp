#include "main.h"
#include "gpio.h"
#include "OLED.h"
#include "tim.h"
#include "user/Key.h"
#include "user/start.h"
#include "user/Motor.h"
#include "user/GraySensor.h"

using namespace std;

RobotControl rc;

// 状态标志位
uint8_t startMove = 0;
uint8_t arrivedGoal = 0;
uint8_t isMoving = 0;
uint8_t currentPoint = 0;
uint8_t currentWay = 1;

// void toggleFlag(uint8_t* flag)
// {
// 	if(flag == 0)
// 	{
// 		flag = (uint8_t)1;
// 	}
// 	else
// 	{
// 		flag = (uint8_t)0;
// 	}
// }

// 定时中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim4)
	{
		rc.mt1.enc.read_cnt();
		rc.mt2.enc.read_cnt();
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		rc.robotUpdate();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    UNUSED(GPIO_Pin);
    if(GPIO_Pin == DrugDetector_Pin)
    {
		if(isMoving == 0)
		{
			startMove = 1;
		}
    }
}

// main函数
void startup()
{
	HAL_Delay(200);
	OLED_Init();
	HAL_Delay(200);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	// 设置pid参数
    rc.mt1.pid.pid_setParam(1.3,0.1,0.1);
    rc.mt2.pid.pid_setParam(1.3,0.1,0.1);
    rc.setSpeed(0,0);

	while(startMove == 0)
	{

	}

	while(1)
	{
		OLED_ShowFNum(1,1,rc.mt1.enc.mt_speed,5,3);
		OLED_ShowFNum(2,1,rc.mt2.enc.mt_speed,5,3);
		if(startMove == 1)
		{
			if(CrossDetect() == 1)
			{
				rc.setSpeed(0,0);
			}
			else
			{
				rc.setSpeed(0.3,0+Gray_control());
			}
		}
	}
}
