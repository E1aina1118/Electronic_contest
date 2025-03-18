#include "main.h"
#include "user/Motor.h"

// PID
void PID::pid_update() {
    error = goal - current;
    integral += error;
    derivative = error - prevError;
    op = kp * error + ki * integral + kd * derivative;
    prevError = error;
}
void PID::pid_setParam(float kp_, float ki_, float kd_) {
    kp = kp_;
    ki = ki_;
    kd = kd_;
}
float PID::pid_getOp()
{
	if(op > limMax)
	{
		op = limMax;
	}
	else if (op < -limMax)
	{
		op = -limMax;
	}
	return op;
}
void PID::pid_setCurrent(float current_)
{
	current = current_;
}
void PID::pid_setGoal(float goal_)
{
	goal = goal_;
}

// Encoder
Encoder::Encoder(uint8_t motornum_)
{
    if(motornum_ == 1)
    {
        timx = &htim2;   
    }
    else if (motornum_ == 2)
    {
        timx = &htim1;
    }
}
void Encoder::read_cnt()
{
    cnt = (int16_t)__HAL_TIM_GET_COUNTER(timx);
}
void Encoder::enc_getSpeed()
{
    mt_speed = -(cnt * 0.065 * 3.14)/(0.02 * 897.6);
}

// Motor
Motor::Motor(uint8_t motornum_) : pid(), enc(motornum_), motornum(motornum_)
{
    if(motornum_ == 1)
    {
        PWMChannel = TIM_CHANNEL_2;
    }
    else if (motornum_ == 2)
    {
        PWMChannel = TIM_CHANNEL_1;
    }
}
void Motor::setMotorDirection(uint8_t direction)
{
    if(motornum == 1)
    {
        if(direction == 1)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
        else if(direction == 2)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        }
        else if (direction == 0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        }
    }
    else if (motornum == 2)
    {
        if(direction == 1)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        }
        else if(direction == 2)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        }
        else if (direction == 0)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        }
    }
}
void Motor::setMotorCCR(uint16_t ccr)
{
    __HAL_TIM_SET_COMPARE(&htim3, PWMChannel, ccr);
}

// RobotControl
RobotControl::RobotControl() : mt1(1), mt2(2){}
void RobotControl::speedToCCR(float speed1, float speed2)
{
    if (speed1 > 0)
    {
        mt1.setMotorDirection(1);
    }
    else if (speed1 < 0)
    {
        mt1.setMotorDirection(2);
        speed1 = -speed1;
    }
    else
    {
        mt1.setMotorDirection(0);
    }

    if (speed2 > 0)
    {
        mt2.setMotorDirection(1);
    }
    else if (speed2 < 0)
    {
        mt2.setMotorDirection(2);
        speed2 = -speed2;
    }
    else
    {
        mt2.setMotorDirection(0);
    }

    uint16_t ccr1 = (uint16_t)(speed1/1.0*7200-1);
    uint16_t ccr2 = (uint16_t)(speed2/1.0*7200-1);
    mt1.setMotorCCR(ccr1);
    mt2.setMotorCCR(ccr2);
}

void RobotControl::setSpeed(float v, float w)
{
    float v1,v2;
    v1 = v - w*L/2;
    v2 = v + w*L/2;
    mt1.pid.pid_setGoal(v1);
    mt2.pid.pid_setGoal(v2);
}

void RobotControl::robotUpdate()
{
    mt1.enc.enc_getSpeed();
    mt2.enc.enc_getSpeed();
    mt1.pid.pid_setCurrent(mt1.enc.mt_speed);
    mt2.pid.pid_setCurrent(mt2.enc.mt_speed);
    mt1.pid.pid_update();
    mt2.pid.pid_update();
    robotSpeed = (mt1.enc.mt_speed + mt2.enc.mt_speed) / 2;
    robotOdom += robotSpeed * 0.02;
    speedToCCR(mt1.pid.pid_getOp(), mt2.pid.pid_getOp());
}

void RobotControl::clearRobotOdom()
{
    robotOdom = 0;
}