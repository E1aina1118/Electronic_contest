#ifndef __MORTOR_H
#define __MORTOR_H

#include "main.h"
#include "tim.h"

class PID {
public:
    PID() : kp(0), ki(0), kd(0), integral(0), derivative(0), goal(0), current(0), error(0), prevError(0), limMin(0), limMax(1.0), op(0) {}
    void pid_update();
    void pid_update_rotate();
    void pid_setParam(float kp_, float ki_, float kd_);
    void pid_setGoal(float goal_);
    void pid_setLim(float max);
    void pid_setCurrent(float current_);
    float pid_getOp();

    void pid_clear();

// private:
    float kp;
    float ki;
    float kd;
    float integral;
    float derivative;
    float goal;
    float current;
    float error;
    float prevError;
    float limMin;
    float limMax;
    float op;
};

class Encoder {
public:
    Encoder(uint8_t motornum_);
    void read_cnt();
    void enc_getSpeed();
// private:
    int16_t cnt;
    float mt_speed;
    TIM_HandleTypeDef* timx;
};

class Motor {
    public:
        Motor(uint8_t motornum_);
        PID pid;
        Encoder enc;
        void setMotorDirection(uint8_t direction);
        void setMotorCCR(uint16_t ccr);
    // private:
        uint8_t motornum;
        uint16_t PWMChannel;

};

class RobotControl {
public:
    RobotControl();
    void setSpeed(float v, float w);
    void clearRobotOdom();
    float getRobotOdom();
    void robotUpdate();

    void rotate_turn();
    void rotate_left();
    void rotate_right();
    void speedToCCR(float speed1, float speed2);
    Motor mt1;
    Motor mt2;
    float roll;
    float pitch;
    float yaw;
    PID rotatePid;
    float robotOdom;
    float robotSpeed;
private:
    const float L = 0.2;
};

float getAbs(float value);

#endif