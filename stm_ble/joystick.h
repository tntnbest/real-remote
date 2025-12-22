#pragma once
#include "stm32f4xx_hal.h"

typedef enum {
    DIR_CENTER,
    DIR_UP,
    DIR_DOWN,
    DIR_LEFT,
    DIR_RIGHT,
		DIR_BTN
} Direction;

void joystick_init(ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart);
void joystick_start(void);
void joystick_task(void);  
void joystick_buttonIRQHandler(uint16_t GPIO_Pin); 
Direction getLastDirection(void);