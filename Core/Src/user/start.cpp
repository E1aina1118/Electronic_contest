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

	while(1)
	{
		if(getButtomState(1) == 1)
		{
			break;
		}
	}

	while(1)
	{
		OLED_ShowFNum(1,1,rc.mt1.enc.enc_getSpeed(),5,3);
		OLED_ShowFNum(2,1,rc.mt2.enc.enc_getSpeed(),5,3);
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
