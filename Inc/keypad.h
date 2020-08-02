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

// Configure GPIO as output open drain for keypad columns
#define KEYPAD_GPIO_COL0			D6_GPIO_Port
#define KEYPAD_PIN_COL0				D6_Pin
#define KEYPAD_GPIO_COL1			D5_GPIO_Port
#define KEYPAD_PIN_COL1				D5_Pin
#define KEYPAD_GPIO_COL2			D4_GPIO_Port
#define KEYPAD_PIN_COL2				D4_Pin
#define KEYPAD_GPIO_COL3			D3_GPIO_Port
#define KEYPAD_PIN_COL3				D3_Pin

// Configure GPIO as input with pull-up resistor for keypad rows
#define KEYPAD_GPIO_ROW0			D11_GPIO_Port
#define KEYPAD_PIN_ROW0				D11_Pin
#define KEYPAD_GPIO_ROW1			D10_GPIO_Port
#define KEYPAD_PIN_ROW1				D10_Pin
#define KEYPAD_GPIO_ROW2			D9_GPIO_Port
#define KEYPAD_PIN_ROW2				D9_Pin
#define KEYPAD_GPIO_ROW3			D2_GPIO_Port
#define KEYPAD_PIN_ROW3				D2_Pin

#define KEYPAD_NO_PRESSED			0x00

uint8_t prev_key = KEYPAD_NO_PRESSED;

uint8_t keypad_GetKey(void);
uint8_t KeypadGetKey(void);

uint8_t keypad_GetKey()
{
	// Scan column 0 (column 0 pin is grounded, other column pins is open drain)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_SET);
	Delay(1);
	// Read rows
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW0, KEYPAD_PIN_ROW0))
		return '1';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW1))
		return '4';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW2, KEYPAD_PIN_ROW2))
		return '7';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW3, KEYPAD_PIN_ROW3))
		return '*';

	// Scan column 1 (column 1 pin is grounded, other column pins is open drain)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_SET);
	Delay(1);
	// Read rows
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW0, KEYPAD_PIN_ROW0))
		return '2';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW1))
		return '5';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW2, KEYPAD_PIN_ROW2))
		return '8';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW3, KEYPAD_PIN_ROW3))
		return '0';

	// Scan column 2 (column 2 pin is grounded, other column pins is open drain)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_SET);
	Delay(1);
	// Read rows
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW0, KEYPAD_PIN_ROW0))
		return '3';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW1))
		return '6';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW2, KEYPAD_PIN_ROW2))
		return '9';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW3, KEYPAD_PIN_ROW3))
		return '#';

	// Scan column 3 (column 3 pin is grounded, other column pins is open drain)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL0, KEYPAD_PIN_COL0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL1, KEYPAD_PIN_COL1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL2, KEYPAD_PIN_COL2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_RESET);
	Delay(1);
	// Read rows
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW0, KEYPAD_PIN_ROW0))
		return 'A';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW1, KEYPAD_PIN_ROW1))
		return 'B';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW2, KEYPAD_PIN_ROW2))
		return 'C';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW3, KEYPAD_PIN_ROW3))
		return 'D';

	return KEYPAD_NO_PRESSED;
}

uint8_t KeypadGetKey(){
	uint8_t key = keypad_GetKey();
	while ((key != KEYPAD_NO_PRESSED) && (key == keypad_GetKey()))
		;
	return key;
}

#ifdef __cplusplus
}
#endif

#endif

/********************************* END OF FILE ********************************/
/******************************************************************************/
