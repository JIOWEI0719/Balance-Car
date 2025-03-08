#include "main.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"

// 手动构建字符串并向上位机发送实时速度
void Send_Speed_Data(UART_HandleTypeDef *huart, float current_speed, float target_speed);

