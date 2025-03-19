#include "main.h"
#include "gpio.h"
#include "OLED.h"
#include "tim.h"

#include "mpu6050.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "IIC.h"

#include "user/Key.h"
#include "user/start.h"
#include "user/Motor.h"
#include "user/GraySensor.h"

using namespace std;

RobotControl rc;
float pitch1, roll1, yaw1;

// 计数
float time_cnt = 0.0f;
uint8_t sub_time_cnt = 0;
float start_time = 0;

// 状态标志位
uint8_t isRotating = 0;
uint8_t interrupt4 = 0;
int16_t task = -1;
uint8_t step = 0;
int8_t mission = 0;
// uint8_t startMove = 0;
// uint8_t arrivedGoal = 0;
// uint8_t isMoving = 0;
// uint8_t currentPoint = 0;
// uint8_t currentWay = 1;

// uint8_t debug_cnt = 0;

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
		rc.yaw = yaw1;
		sub_time_cnt ++;
		if(sub_time_cnt == 5)
		{
			time_cnt += 0.1;
			sub_time_cnt = 0;
		}
		if(isRotating == 1)
		{
			OLED_ShowFNum(4,1,rc.yaw,5,2);
			rc.rotatePid.pid_setCurrent(rc.yaw);
			rc.rotatePid.pid_update_rotate();
			rc.setSpeed(0,rc.rotatePid.pid_getOp());
			OLED_ShowFNum(2,1,rc.rotatePid.error,5,2);
			if(getAbs(rc.rotatePid.error) <= 5)
			{
				isRotating = 0;
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_10)
	{
		mpu_dmp_get_data(&pitch1,&roll1,&yaw1);
		rc.yaw = yaw1;
	}
    UNUSED(GPIO_Pin);
}

// void tim_interrupt4()
// {
// 	rc.mt1.enc.read_cnt();
// 	rc.mt2.enc.read_cnt();
// 	__HAL_TIM_SET_COUNTER(&htim1, 0);
// 	__HAL_TIM_SET_COUNTER(&htim2, 0);
// 	rc.robotUpdate();
// 	sub_time_cnt ++;
// 	if(sub_time_cnt == 5)
// 	{
// 		time_cnt += 0.1;
// 		sub_time_cnt = 0;
// 	}
// }

// void rotate_left()
// {
// 	while(1)
// 	{
// 		OLED_ShowFNum(3,1,yaw1,5,2);
// 	}
//     // float start_time = 0;
//     // time_cnt = 0;
//     // rc.rotatePid.pid_setParam(0.02,0.01,0);
//     // rc.rotatePid.pid_setLim(5);
//     // rc.rotatePid.pid_setGoal(rc.yaw+80);
//     // while(1)
//     // {
//     //     // rc.rotatePid.pid_setCurrent(rc.yaw);
//     //     // rc.rotatePid.pid_update_rotate();
//     //     // rc.setSpeed(0,rc.rotatePid.pid_getOp());
//     //     if(getAbs(rc.rotatePid.error) <= 2)
//     //     {
//     //         if(start_time == 0)
//     //         {
//     //             start_time = time_cnt;
//     //         }
//     //         else if((time_cnt - start_time) >= 0.3)
//     //         {
//     //             rc.setSpeed(0,0);
//     //             break;
//     //         }
//     //     }
//     //     else
//     //     {
//     //         start_time = 0;
//     //     }
//     // }
// }


// void rotate_right()
// {
// 	if(step == 0)
// 	{
// 		rc.rotatePid.pid_setParam(0.015,0,0);
// 		rc.rotatePid.pid_setLim(10);
// 		rc.rotatePid.pid_setGoal(rc.yaw-80);
// 		step = 1;
// 		time_cnt = 0;
// 		start_time = 0;
// 	}
// 	if(step == 1)
// 	{
// 		rc.rotatePid.pid_setCurrent(rc.yaw);
// 		rc.rotatePid.pid_update_rotate();
// 		rc.setSpeed(0,rc.rotatePid.pid_getOp());
// 		if(getAbs(rc.rotatePid.error) <= 2)
// 		{
// 			if(start_time >= 0.3)
// 			{
// 				start_time = time_cnt;
// 			}
// 			else if((time_cnt - start_time) >= 0.3)
// 			{
// 				step = 0;
// 				task = -1;
// 				rc.setSpeed(0,0);
// 			}
// 		}
// 	}
// }

// void rotate_left()
// {
// 	if(step == 0)
// 	{
// 		rc.rotatePid.pid_setParam(0.015,0,0);
// 		rc.rotatePid.pid_setLim(10);
// 		rc.rotatePid.pid_setGoal(rc.yaw+80);
// 		step = 1;
// 		time_cnt = 0;
// 		start_time = 0;
// 	}
// 	if(step == 1)
// 	{
// 		rc.rotatePid.pid_setCurrent(rc.yaw);
// 		rc.rotatePid.pid_update_rotate();
// 		rc.setSpeed(0,rc.rotatePid.pid_getOp());
// 		if(getAbs(rc.rotatePid.error) <= 2)
// 		{
// 			if(start_time >= 0.3)
// 			{
// 				start_time = time_cnt;
// 			}
// 			else if((time_cnt - start_time) >= 0.3)
// 			{
// 				step = 0;
// 				task = -1;
// 				rc.setSpeed(0,0);
// 			}
// 		}
// 	}
// }

// void rotate_turn()
// {
// 	if(step == 0)
// 	{
// 		rc.rotatePid.pid_setParam(0.015,0,0);
// 		rc.rotatePid.pid_setLim(10);
// 		rc.rotatePid.pid_setGoal(rc.yaw+160);
// 		step = 1;
// 		time_cnt = 0;
// 		start_time = 0;
// 	}
// 	if(step == 1)
// 	{
// 		rc.rotatePid.pid_setCurrent(rc.yaw);
// 		rc.rotatePid.pid_update_rotate();
// 		rc.setSpeed(0,rc.rotatePid.pid_getOp());
// 		if(getAbs(rc.rotatePid.error) <= 2)
// 		{
// 			if(start_time >= 0.3)
// 			{
// 				start_time = time_cnt;
// 			}
// 			else if((time_cnt - start_time) >= 0.3)
// 			{
// 				step = 0;
// 				task = -1;
// 				rc.setSpeed(0,0);
// 			}
// 		}
// 	}
// }

void rotate_left()
{
    rc.rotatePid.pid_setParam(0.02,0,0);
    rc.rotatePid.pid_setLim(5);
    rc.rotatePid.pid_setGoal(rc.yaw+80);
	isRotating = 1;
	while(isRotating == 1)
	{
		OLED_ShowNum(3,1,isRotating,5);
	}
	rc.setSpeed(0,0);
}

void rotate_right()
{
    rc.rotatePid.pid_setParam(0.02,0,0);
    rc.rotatePid.pid_setLim(5);
    rc.rotatePid.pid_setGoal(rc.yaw-80);
	isRotating = 1;
	while(isRotating == 1)
	{
		OLED_ShowNum(3,1,isRotating,5);
	}
	rc.setSpeed(0,0);
}

void rotate_turn()
{
    rc.rotatePid.pid_setParam(0.02,0,0);
    rc.rotatePid.pid_setLim(5);
    rc.rotatePid.pid_setGoal(rc.yaw-160);
	isRotating = 1;
	while(isRotating == 1)
	{
		OLED_ShowNum(3,1,isRotating,5);
	}
	rc.setSpeed(0,0);
}

// main函数
void startup()
{
	HAL_Delay(200);
	OLED_Init();
	HAL_Delay(200);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	// 设置pid参数
    rc.mt1.pid.pid_setParam(1.3,0.1,0.1);
    rc.mt2.pid.pid_setParam(1.3,0.1,0.1);

	while(MPU_Init()!=0);
	while(mpu_dmp_init()!=0);
	
	HAL_TIM_Base_Start_IT(&htim4);

	rotate_left();
	rotate_right();
	rotate_turn();

	while(1)
	{
		OLED_ShowFNum(1,1,rc.mt1.pid.op,5,2);
		OLED_ShowNum(2,1,isRotating,5);
		// OLED_ShowFNum(2,1,rc.mt2.pid.op,5,2);
		// OLED_ShowFNum(3,1,getAbs(rc.rotatePid.error),5,2);
		// OLED_ShowFNum(4,1,rc.yaw,5,2);
		
	}
}
