#include "main.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"

// �ֶ������ַ���������λ������ʵʱ�ٶ�
void Send_Speed_Data(UART_HandleTypeDef *huart, float current_speed, float target_speed);

