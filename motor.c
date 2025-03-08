#include "motor.h"

//逆时针为正转 
void MotorA_pos(void)
{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}
void MotorB_pos(void)
{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}
//顺时针为反转
void MotorA_neg(void)
{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
}
void MotorB_neg(void)
{
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}

//前进
void Move_forward(void)
{
		MotorA_neg();
		MotorB_pos();
}
//后退
void Move_backward(void)
{
		MotorA_pos();
		MotorB_neg();
}

//