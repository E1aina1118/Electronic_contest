#include "main.h"

uint8_t L1_Val, L2_Val, M_Val, R1_Val, R2_Val;
float Line_Num;

/**
  * @brief:1:not white 0:white
  */
void Gray_Get_TTL(void)
{
	if (HAL_GPIO_ReadPin(GrayL2_GPIO_Port, GrayL2_Pin) == GPIO_PIN_SET) {L2_Val = 1;} else {L2_Val = 0;}
	if (HAL_GPIO_ReadPin(GrayL1_GPIO_Port, GrayL1_Pin) == GPIO_PIN_SET) {L1_Val = 1;} else {L1_Val = 0;}
	if (HAL_GPIO_ReadPin(GrayM_GPIO_Port,  GrayM_Pin)  == GPIO_PIN_SET) {M_Val = 1;}  else {M_Val = 0;}
	if (HAL_GPIO_ReadPin(GrayR1_GPIO_Port, GrayR1_Pin) == GPIO_PIN_SET) {R1_Val = 1;} else {R1_Val = 0;}
	if (HAL_GPIO_ReadPin(GrayR2_GPIO_Port, GrayR2_Pin) == GPIO_PIN_SET) {R2_Val = 1;} else {R2_Val = 0;}
}

float Gray_control(void)
{
	Gray_Get_TTL();
	  
	if(L2_Val == 0 &&  M_Val == 1 && R2_Val == 0 )  Line_Num =  0;//ֱstright
	if(L2_Val == 0 &&  M_Val == 0 && R2_Val == 1) Line_Num =  -20;//too left
    if(L2_Val == 1 &&  M_Val == 0 && R2_Val == 0) Line_Num = 20;//too right
	if(L1_Val == 1 && L2_Val == 0 &&  M_Val == 0 && R2_Val == 0 && R1_Val == 0) Line_Num = 50;
	if(L1_Val == 0 && L2_Val == 0 &&  M_Val == 0 && R2_Val == 0 && R1_Val == 1) Line_Num = -50;

	return Line_Num;
}

//检测十字，返回1表示十字，返回2表示到达，否则返回0
uint8_t CrossDetect(void)
{
	Gray_Get_TTL();
	// if (L1_Val == 1 && L2_Val == 1 && M_Val == 1 && R1_Val == 1 && R2_Val == 1) return 1;
	// if (L1_Val == 0 && L2_Val == 0 && M_Val == 0 && R1_Val == 0 && R2_Val == 0) return 2;
	if (L2_Val == 1 && M_Val == 1 && R2_Val == 1) return 1;
	if (L2_Val == 0 && M_Val == 0 && R2_Val == 0) return 2;
	else return 0;
}

uint8_t CrossDetect2(void)
{
	Gray_Get_TTL();
	// if (L1_Val == 1 && L2_Val == 1 && M_Val == 1 && R1_Val == 1 && R2_Val == 1) return 1;
	// if (L1_Val == 0 && L2_Val == 0 && M_Val == 0 && R1_Val == 0 && R2_Val == 0) return 2;
	if (M_Val == 1 && R2_Val == 1 && R1_Val == 1) return 1;
	if (L2_Val == 1 && M_Val == 1 && L1_Val == 1) return 1;
	else return 0;
}


