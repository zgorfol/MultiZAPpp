/*
 * biozap_comm.h
 *
 *  Created on: Sep 6, 2020
 *      Author: z_gorfol
 */

#ifndef BIOZAP_COMM_H_
#define BIOZAP_COMM_H_


void uart_TX_IT(UART_HandleTypeDef *huart, std::string inputString);
void start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel,TIM_HandleTypeDef *htim, uint16_t psc, uint16_t arr);
void stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel,TIM_HandleTypeDef *htim);
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);
void Cmd_Interpreter(UART_HandleTypeDef *huart, DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim);

#endif /* BIOZAP_COMM_H_ */
