/*
 * API.c
 *
 *  Created on: Aug 17, 2024
 *      Author: Minh Tuan
 */


#include "API.h"
#include "MPU6050.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

extern void angle(int goc);


void forwardGPIO(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
}

void API_moveForward(){
	forwardGPIO();
}



void API_turnRight(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
    HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,SET);

	angle(89);
	stop();
}

void API_turnLeft(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,SET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,SET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
	angle(89);
	stop();
}
void stop(){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0); //left
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,0);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin,RESET);
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin,RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin,RESET);
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin,RESET);
}

