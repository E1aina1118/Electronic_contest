#include "main.h"
#include "gpio.h"
#include "OLED.h"
#include "tim.h"
#include "usart.h"

#include "mpu6050.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "IIC.h"

#include "user/Key.h"
#include "user/start.h"
#include "user/Motor.h"
#include "user/GraySensor.h"
#include "user/dfs.h"

RobotControl rc;
float pitch1, roll1, yaw1;

// 变量
float time_cnt = 0.0f;
uint8_t sub_time_cnt = 0;
float start_time = 0;
uint8_t mission = 0; // 任务编号
uint8_t goalNum = 0; // 目标数字
char route[10]; // 路径
char lnum = '*';
char rnum = '*';
uint8_t k210_goalNum;

// 状态标志位
volatile uint8_t isRotating = 0;
volatile uint8_t startFlag = 0;
volatile uint8_t crossFlag = 0;
volatile uint8_t currentWay = 1; // 1上 2左 3下 4右
volatile int8_t atCross10Flag = 0; // 1 向左走 2 向右走
volatile int8_t atCross10Way = 0; // 1 现在面朝左边 2右边

// 地图
int edge[14][14];
int8_t correspond[14];
uint8_t nextPosition = 0;

// 串口
char rxBuffer;
uint8_t rxState = 0;
uint8_t packetReceived = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		if(rxState == 0)
		{
			if(rxBuffer == 'a')
			{
				rxState = 1;
			}
			else
			{
				rxState = 0;
			}
		}
		else if(rxState == 1)
		{
			if(rxBuffer == 'l')
			{
				rxState = 2;
			}
			else if(rxBuffer == 'r')
			{
				rxState = 3;
			}
			else if(rxBuffer == 's')
			{
				rxState = 4;
			}
			else
			{
				rxState = 0;
			}
		}
		else if(rxState == 2)
		{
			lnum = rxBuffer;
			rxState = 5;
		}
		else if(rxState == 3)
		{
			rnum = rxBuffer;
			rxState = 5;
		}
		else if(rxState == 4)
		{
			k210_goalNum = rxBuffer - '0';
			rxState = 5;
		}
		else if(rxState == 5)
		{
			if(rxBuffer == 'f')
			{
				rxState = 0;
			}
		}
		HAL_UART_Receive_IT(&huart2,(uint8_t*)&rxBuffer,1);
	}
}

// 提前声明部分函数
void rotate_left();
void rotate_right();
void rotate_turn();
void waitStartSignal();
void fallowLine();
void goGivenDistance(float distance);
void rotateGivenDegrees(float degrees);
void detectGoalNum();


// 地图 & 路径规划
void map_init()
{
	// 正向
	edge[0][2] = 1;
	edge[2][1] = 2;
	edge[2][3] = 4;
	edge[2][5] = 1;
	edge[5][4] = 2;
	edge[5][6] = 4;
	edge[5][10] = 1;
	edge[10][9] = 2;
	edge[10][11] = 4;
	edge[9][12] = 1;
	edge[9][7] = 3;
	edge[11][13] = 1;
	edge[11][8] = 3;

	// 反向
	edge[2][0] = 3;
	edge[1][2] = 4;
	edge[3][2] = 2;
	edge[5][2] = 3;
	edge[4][5] = 4;
	edge[6][5] = 2;
	edge[10][5] = 3;
	edge[9][10] = 4;
	edge[11][10] = 2;
	edge[12][9] = 3;
	edge[7][9] = 1;
	edge[13][11] = 3;
	edge[8][11] = 1;

	for(uint8_t i = 0; i<14; i++)
	{
		correspond[i] = -1;
	}
	correspond[0] = 0;
	correspond[1] = 1;
	correspond[2] = 3;
}

void buildMap()
{
	if(lnum != '*' && rnum != '*')
	{
		if(nextPosition == 5)
		{
			correspond[lnum-'0'] = 4;
			correspond[rnum-'0'] = 6;
		}
		else if(nextPosition == 9)
		{
			correspond[lnum-'0'] = 7;
			correspond[rnum-'0'] = 12;
		}
		else if(nextPosition == 11)
		{
			correspond[lnum-'0'] = 13;
			correspond[rnum-'0'] = 8;
		}
		else if(nextPosition == 10)
		{
			if(lnum == goalNum || rnum == goalNum)
			{
				if(atCross10Way == 1)
				{
					atCross10Flag = 1;
				}
				else if(atCross10Way == -1)
				{
					atCross10Flag = -1;
				}
			}
		}
	}
}

void explainRoute()
{
	uint8_t i = 0;
	int8_t intRoute;
	int8_t wayError;
	while(route[i] != '\0')
	{
		intRoute = route[i] - '0';
		wayError = currentWay - intRoute;
		if(wayError == 0)
		{
			fallowLine();
		}
		else if(wayError == -1 || wayError == 3)
		{
			rotate_left();
			fallowLine();
		}
		else if(wayError == 1 || wayError == -3)
		{
			rotate_right();
			fallowLine();
		}
		else if(wayError == 2 || wayError == -2)
		{
			rotate_turn();
			fallowLine();
		}
		i++;
	}
}

void mapUpdateWay(uint8_t rotateWay)
{
	currentWay = (currentWay - 1 + rotateWay + 4) % 4 + 1;
}


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
			OLED_ShowFNum(4,1,rc.rotatePid.error,5,2);
			rc.rotatePid.pid_setCurrent(rc.yaw);
			rc.rotatePid.pid_update_rotate();
			rc.setSpeed(0,rc.rotatePid.pid_getOp());
			if(getAbs(rc.rotatePid.error) <= 3)
			{
				isRotating = 0;
				OLED_Clear();
			}
		}
		crossFlag = CrossDetect();
	}
}

// 外部中断
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_10)
	{
		mpu_dmp_get_data(&pitch1,&roll1,&yaw1);
		rc.yaw = yaw1;
	}
    UNUSED(GPIO_Pin);
}

// 原子动作
void rotate_left()
{
    rc.rotatePid.pid_setGoal(rc.yaw+80);
	isRotating = 1;
	while(isRotating == 1)
	{
	}
	rc.setSpeed(0,0);
	mapUpdateWay(1);
	rc.rotatePid.pid_clear();
}

void rotate_right()
{
    rc.rotatePid.pid_setGoal(rc.yaw-80);
	isRotating = 1;
	while(isRotating == 1)
	{
		OLED_ShowNum(2,1,isRotating,2);
		OLED_ShowFNum(3,1,time_cnt,5,2);
	}
	OLED_Clear();
	rc.setSpeed(0,0);
	mapUpdateWay(-1);
	rc.rotatePid.pid_clear();
}

void rotate_turn()
{
    rc.rotatePid.pid_setGoal(rc.yaw+160);
	isRotating = 1;
	while(isRotating == 1)
	{
		OLED_ShowNum(2,1,isRotating,2);
		OLED_ShowFNum(3,1,time_cnt,5,2);
	}
	rc.setSpeed(0,0);
	mapUpdateWay(2);
	rc.rotatePid.pid_clear();
}

void waitStartSignal()
{
	while(1)
	{
		if(startFlag == 0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET)
		{
			startFlag = 1;
			break;
		}
		else if(startFlag == 1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET)
		{
			startFlag = 0;
			break;
		}
	}
}

void fallowLine()
{
	rc.robotOdom = 0;
	while(1)
	{
		rc.setSpeed(0.3, Gray_control());
		if(rc.robotOdom >= 0.2)
		{
			if(crossFlag == 1)
			{
				break;
			}
			else if(crossFlag == 2)
			{
				break;
			}
		}
	}
	rc.setSpeed(0,0);
}

void goGivenDistance(float distance)
{
	rc.robotOdom = 0;
	while(1)
	{
		OLED_ShowFNum(2,1,rc.robotOdom,4,1);
		rc.setSpeed(0.3, Gray_control());
		if(rc.robotOdom >= distance)
		{
			break;
		}
	}
	rc.setSpeed(0,0);
}

void rotateGivenDegrees(float degrees)
{
	rc.rotatePid.pid_setGoal(rc.yaw+degrees);
	isRotating = 1;
	while(isRotating == 1)
	{
	}
	rc.setSpeed(0,0);
}

void detectGoalNum()
{
    uint8_t previousNum = k210_goalNum;
    double startTime = time_cnt;

    while (1) {
		OLED_ShowNum(2,1,k210_goalNum,2);
        // 如果 k210_goalNum 发生变化，重置计时
        if(k210_goalNum != previousNum || k210_goalNum == 0)
		{
            previousNum = k210_goalNum;
            startTime = time_cnt;
        }
		else
		{
			// 如果值保持不变，检查持续时间
			if ((time_cnt - startTime) >= 3.0 && k210_goalNum != 0)
			{
				goalNum = k210_goalNum;
				break;
			}
		}
		
    }
	OLED_ShowNum(2,5,goalNum,2);
	OLED_Clear();
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
	rc.rotatePid.pid_setParam(13,0.5,0.8);
	rc.rotatePid.pid_setLim(90);

	while(MPU_Init()!=0);
	while(mpu_dmp_init()!=0);
	
	HAL_TIM_Base_Start_IT(&htim4);

	map_init();
	mission = 1;
	HAL_UART_Receive_IT(&huart2,(uint8_t*)&rxBuffer,1);

	OLED_ShowNum(1,1,111,3);
	waitStartSignal();
	rotate_left();
	rotate_right();

	while(1)
	{
		// if(mission == 1)
		// {
		// 	OLED_Clear();
		// 	detectGoalNum();
		// 	OLED_ShowNum(1,1,111,3);
		// 	waitStartSignal();
		// 	OLED_ShowNum(1,1,222,3);
		// 	fallowLine();
		// 	OLED_ShowNum(1,1,333,3);
		// 	if(goalNum == 1)
		// 	{
		// 		rotate_left();
		// 		OLED_ShowNum(1,1,444,3);
		// 	}
		// 	else if(goalNum == 2)
		// 	{
		// 		rotate_right();
		// 		OLED_ShowNum(1,1,555,3);
		// 	}
		// 	fallowLine();
		// 	OLED_ShowNum(1,1,666,3);
		// 	waitStartSignal();
		// 	OLED_ShowNum(1,1,777,3);
		// 	rotate_turn();
		// 	OLED_ShowNum(1,1,888,3);
		// 	fallowLine();
		// 	OLED_ShowNum(1,1,999,3);
		// 	rotate_right();
		// 	OLED_ShowNum(1,1,122,3);
		// 	fallowLine();
		// 	mission = 2;
		// }
		// else if(mission == 2)
		// {
		// 	detectGoalNum();
		// 	waitStartSignal();
		// 	fallowLine();
		// 	nextPosition = 5;
		// 	goGivenDistance(0.6);
		// 	buildMap();
		// 	dfs(5,correspond[goalNum],route);
		// 	fallowLine();
		// 	explainRoute();
		// 	waitStartSignal();
		// 	dfs(correspond[goalNum], correspond[0], route);
		// 	explainRoute();
		// 	mission = 2;
		// }
		// else if(mission == 3)
		// {
		// 	detectGoalNum();
		// 	waitStartSignal();
		// 	fallowLine();
		// 	fallowLine();
		// 	goGivenDistance(0.75);
		// 	nextPosition = 10;
		// 	rotateGivenDegrees(20);
		// 	atCross10Way = 1;
		// 	buildMap();
		// 	rotateGivenDegrees(-40);
		// 	atCross10Way = -1;
		// 	buildMap();
		// 	rotateGivenDegrees(20);
		// 	fallowLine();
		// 	if(atCross10Flag == 1)
		// 	{
		// 		nextPosition = 9;
		// 		rotate_left();
		// 	}
		// 	else if(atCross10Flag == -1)
		// 	{
		// 		nextPosition = 11;
		// 		rotate_right();
		// 	}
		// 	goGivenDistance(60);
		// 	buildMap();
		// 	dfs(nextPosition,correspond[goalNum],route);
		// 	fallowLine();
		// 	OLED_ShowNum(1,1,133,3);
		// 	explainRoute();
		// 	OLED_ShowNum(1,1,144,3);
		// 	waitStartSignal();
		// 	dfs(correspond[goalNum],correspond[0],route);
		// 	explainRoute();
		// 	mission = 4;
		// }
	}
}
