/**
  ******************************************************************************
  * @file		keypad.h
  * @author	based Yohanes Erwin Setiawan
  * @modified	GZ
  * @date		1 Aug 2020
  ******************************************************************************
  */

#ifndef __KEYPAD_H
#define __KEYPAD_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes ---------------------------------------------------------------- */
#include "main.h"

// Configure GPIO as output pull up for keypad columns
#define KEYPAD_GPIO_COL0			D6_GPIO_Port
#define KEYPAD_PIN_COL0				D6_Pin
#define KEYPAD_GPIO_COL1			D5_GPIO_Port
#define KEYPAD_PIN_COL1				D5_Pin
#define KEYPAD_GPIO_COL2			D4_GPIO_Port
#define KEYPAD_PIN_COL2				D4_Pin
#define KEYPAD_GPIO_COL3			D3_GPIO_Port
#define KEYPAD_PIN_COL3				D3_Pin

// Configure GPIO as input with pull-down resistor for keypad rows
// You have to edit stm32..it.c to write EXTI... handlers
#define KEYPAD_GPIO_ROW0			D10_GPIO_Port
#define KEYPAD_PIN_ROW0				D10_Pin
#define KEYPAD_PIN_ROW0_EXTI	EXTI15_10_IRQn
#define KEYPAD_GPIO_ROW1			D9_GPIO_Port
#define KEYPAD_PIN_ROW1				D9_Pin
#define KEYPAD_PIN_ROW1_EXTI	EXTI9_5_IRQn
#define KEYPAD_GPIO_ROW2			D8_GPIO_Port
#define KEYPAD_PIN_ROW2				D8_Pin
#define KEYPAD_PIN_ROW2_EXTI	EXTI15_10_IRQn
#define KEYPAD_GPIO_ROW3			D7_GPIO_Port
#define KEYPAD_PIN_ROW3				D7_Pin
#define KEYPAD_PIN_ROW3_EXTI	EXTI15_10_IRQn

#define KEYPAD_NO_PRESSED			0x00

uint8_t prev_key = KEYPAD_NO_PRESSED;

bool Keypad_withIT = true;
volatile uint8_t key = KEYPAD_NO_PRESSED; // Interrupt routine use it, this is the pressed key.
volatile uint8_t Tim7_Col_Idx = 0;

void init_Keypad(bool withIT);
uint8_t KeypadGetKey(bool withIT);

void init_Keypad(bool withIT) {
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_RESET);
	if (withIT) {
		HAL_NVIC_SetPriority(KEYPAD_PIN_ROW0_EXTI, 0, 0);
		HAL_NVIC_EnableIRQ(KEYPAD_PIN_ROW0_EXTI);
		HAL_NVIC_SetPriority(KEYPAD_PIN_ROW1_EXTI, 0, 0);
		HAL_NVIC_EnableIRQ(KEYPAD_PIN_ROW1_EXTI);
		HAL_NVIC_SetPriority(KEYPAD_PIN_ROW2_EXTI, 0, 0);
		HAL_NVIC_EnableIRQ(KEYPAD_PIN_ROW2_EXTI);
		HAL_NVIC_SetPriority(KEYPAD_PIN_ROW3_EXTI, 0, 0);
		HAL_NVIC_EnableIRQ(KEYPAD_PIN_ROW3_EXTI);

		HAL_TIM_Base_Start_IT(&htim7);  // Time7 gives the poll cycle
	}
}

void keypad_Poll_IT(void)
{ // Interrupt for Rising edge
	if (__NVIC_GetEnableIRQ( KEYPAD_PIN_ROW0_EXTI)) {
		HAL_NVIC_DisableIRQ(KEYPAD_PIN_ROW0_EXTI);
		HAL_NVIC_DisableIRQ(KEYPAD_PIN_ROW1_EXTI);
		HAL_NVIC_DisableIRQ(KEYPAD_PIN_ROW2_EXTI);
		HAL_NVIC_DisableIRQ(KEYPAD_PIN_ROW3_EXTI);
		Tim7_Col_Idx = (Tim7_Col_Idx + 1) % 4;
		key =  KEYPAD_NO_PRESSED;
		switch (Tim7_Col_Idx){
			case 0:
				  HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0, GPIO_PIN_SET);
				  break;
			case 1:
				  HAL_GPIO_WritePin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1, GPIO_PIN_SET);
				  break;
			case 2:
				  HAL_GPIO_WritePin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2, GPIO_PIN_SET);
				  break;
			case 3:
				  HAL_GPIO_WritePin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_SET);
				  break;
		}
 	}
 	else {
 		HAL_NVIC_EnableIRQ(KEYPAD_PIN_ROW3_EXTI);
 		HAL_NVIC_EnableIRQ(KEYPAD_PIN_ROW2_EXTI);
 		HAL_NVIC_EnableIRQ(KEYPAD_PIN_ROW1_EXTI);
 		HAL_NVIC_EnableIRQ(KEYPAD_PIN_ROW0_EXTI);
 	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7  ) {
		keypad_Poll_IT();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == D10_Pin) {
		   switch (Tim7_Col_Idx) {
		   	   case 0:
		   		   key = '1';
		   		   break;
		   	   case 1:
		   		   key = '2';
		   		   break;
		   	   case 2:
		   		   key = '3';
		   		   break;
		   	   case 3:
		   		   key = 'A';
		   		   break;
		   }
	}
	if (GPIO_Pin == D9_Pin) {
		   switch (Tim7_Col_Idx) {
		   	   case 0:
		   		   key = '4';
		   		   break;
		   	   case 1:
		   		   key = '5';
		   		   break;
		   	   case 2:
		   		   key = '6';
		   		   break;
		   	   case 3:
		   		   key = 'B';
		   		   break;
		   }
	}
	if (GPIO_Pin == D8_Pin) {
		   switch (Tim7_Col_Idx) {
		   	   case 0:
		   		   key = '7';
		   		   break;
		   	   case 1:
		   		   key = '8';
		   		   break;
		   	   case 2:
		   		   key = '9';
		   		   break;
		   	   case 3:
		   		   key = 'C';
		   		   break;
		   }
	}
	if (GPIO_Pin == D7_Pin) {
		   switch (Tim7_Col_Idx) {
		   	   case 0:
		   		   key = '*';
		   		   break;
		   	   case 1:
		   		   key = '0';
		   		   break;
		   	   case 2:
		   		   key = '#';
		   		   break;
		   	   case 3:
		   		   key = 'D';
		   		   break;
		   }
	}
}

uint8_t keypad_Getch()
{
	// Scan column 0 (column 0 pin is set, other column pins is grounded)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_RESET);
	HAL_Delay(1);
	// Read rows
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW0, KEYPAD_PIN_ROW0))
		return '1';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW1))
		return '4';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW2, KEYPAD_PIN_ROW2))
		return '7';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW3, KEYPAD_PIN_ROW3))
		return '*';

	// Scan column 1 (column 1 pin is set, other column pins is grounded)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_RESET);
	HAL_Delay(1);
	// Read rows
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW0, KEYPAD_PIN_ROW0))
		return '2';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW1))
		return '5';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW2, KEYPAD_PIN_ROW2))
		return '8';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW3, KEYPAD_PIN_ROW3))
		return '0';

	// Scan column 2 (column 2 pin is set, other column pins is grounded)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_RESET);
	HAL_Delay(1);
	// Read rows
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW0, KEYPAD_PIN_ROW0))
		return '3';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW1))
		return '6';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW2, KEYPAD_PIN_ROW2))
		return '9';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW3, KEYPAD_PIN_ROW3))
		return '#';

	// Scan column 3 (column 3 pin is set, other column pins is grounded)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_SET);
	HAL_Delay(1);
	// Read rows
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW0, KEYPAD_PIN_ROW0))
		return 'A';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW1))
		return 'B';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW2, KEYPAD_PIN_ROW2))
		return 'C';
	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW3, KEYPAD_PIN_ROW3))
		return 'D';

	return KEYPAD_NO_PRESSED;
}

uint8_t KeypadGetKey(bool withIT){

	uint8_t keych = KEYPAD_NO_PRESSED;
	if (!withIT) {
		keych = keypad_Getch();
		while ((keych != KEYPAD_NO_PRESSED) && (keych == keypad_Getch()))
			;
	}
	else {
		keych = key;
		while ((keych != KEYPAD_NO_PRESSED) && (keych == key))
			HAL_Delay(300);
	}
	return keych;
}

#ifdef __cplusplus
}
#endif

#endif

/********************************* END OF FILE ********************************/
/******************************************************************************/
