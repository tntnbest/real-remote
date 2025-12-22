#include "joystick.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define CENTER_X 2730
#define CENTER_Y 2730
#define CENTER_THRESHOLD 800
#define RELEASE_THRESHOLD 400

#define INPUT_DELAY 250        // 버튼 입력 지연(ms)
#define REPEAT_DELAY 500       // 꾹 누르고 있을 때 반복 입력 간격(ms)

#define JOY_X_CHANNEL ADC_CHANNEL_0
#define JOY_Y_CHANNEL ADC_CHANNEL_1

static ADC_HandleTypeDef *hadc_joystick;
static UART_HandleTypeDef *huart_joystick;

static volatile uint16_t adc_x_val = 0;
static volatile uint16_t adc_y_val = 0;
static volatile uint8_t current_channel = 0;

static uint32_t lastInputTime = 0;
static Direction lastDirection = DIR_CENTER;
static volatile bool btn_flag = false;

static void Start_ADC_Conversion(uint32_t channel);
static Direction getDirection(int xVal, int yVal, Direction lastDir);

void joystick_init(ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart)
{
    hadc_joystick = hadc;
    huart_joystick = huart;
}

void joystick_start(void)
{
    current_channel = 0;
    Start_ADC_Conversion(JOY_X_CHANNEL);
}

void joystick_buttonIRQHandler(uint16_t GPIO_Pin)
{
    if (HAL_GetTick() - lastInputTime > INPUT_DELAY)
			{
					btn_flag     = true;              
					lastInputTime = HAL_GetTick();
			}
}

static void Start_ADC_Conversion(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(hadc_joystick, &sConfig);
    HAL_ADC_Start_IT(hadc_joystick);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        uint16_t val = HAL_ADC_GetValue(hadc);

        if (current_channel == 0)
        {
					adc_x_val = val;
					current_channel = 1;
					Start_ADC_Conversion(JOY_Y_CHANNEL);
        }
        else
        {
					adc_y_val = val;
					current_channel = 0;
					Start_ADC_Conversion(JOY_X_CHANNEL);
        }
    }
}

void joystick_task(void)
{
		uint32_t now = HAL_GetTick();
		Direction dir;

    if (btn_flag)
    {
        dir = DIR_BTN;
        btn_flag = false;  
    }
    else
    {
			dir = getDirection(adc_x_val, adc_y_val, lastDirection);
			
			if (dir != DIR_CENTER)
			{
					if (lastDirection == DIR_CENTER && (now - lastInputTime) > INPUT_DELAY)
					{
							lastInputTime = now;
					}
					
					else if (dir == lastDirection && (now - lastInputTime) > REPEAT_DELAY)
					{
							lastInputTime = now;
					}
			}
    }
    lastDirection = dir;
}

static Direction getDirection(int xVal, int yVal, Direction lastDir)
{
    if (abs(xVal - CENTER_X) < RELEASE_THRESHOLD && 
        abs(yVal - CENTER_Y) < RELEASE_THRESHOLD)
    {
        return DIR_CENTER;
    }

    Direction dir = DIR_CENTER;
    if (abs(xVal - CENTER_X) > abs(yVal - CENTER_Y)) 
    {
        if (xVal < (CENTER_X - CENTER_THRESHOLD)) 
            dir = DIR_LEFT;
        else if (xVal > (CENTER_X + CENTER_THRESHOLD)) 
            dir = DIR_RIGHT;
        else if (lastDir == DIR_LEFT && xVal < (CENTER_X - RELEASE_THRESHOLD)) 
            dir = DIR_LEFT;
        else if (lastDir == DIR_RIGHT && xVal > (CENTER_X + RELEASE_THRESHOLD)) 
            dir = DIR_RIGHT;
        else if (lastDir == DIR_LEFT || lastDir == DIR_RIGHT)
            dir = lastDir; 
    } 
    else 
    {
        if (yVal > (CENTER_Y + CENTER_THRESHOLD)) 
            dir = DIR_DOWN;
        else if (yVal < (CENTER_Y - CENTER_THRESHOLD)) 
            dir = DIR_UP;
        else if (lastDir == DIR_DOWN && yVal > (CENTER_Y + RELEASE_THRESHOLD)) 
            dir = DIR_DOWN;
        else if (lastDir == DIR_UP && yVal < (CENTER_Y - RELEASE_THRESHOLD)) 
            dir = DIR_UP;
        else if (lastDir == DIR_DOWN || lastDir == DIR_UP)
            dir = lastDir; 
    }
    return dir;
}

Direction getLastDirection(void) {
    return lastDirection;
}
