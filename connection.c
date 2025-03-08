#include "connection.h"

// �ֶ������ַ���������λ������ʵʱ�ٶ�
void Send_Speed_Data(UART_HandleTypeDef *huart, float current_speed, float target_speed) {
    char buffer[64];
    int len = sprintf(buffer, "%f,%f\n", current_speed, target_speed);
    HAL_UART_Transmit(huart, (uint8_t *)buffer, len, 1000);
}


