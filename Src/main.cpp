/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <iostream>
#include <string>
#include <cstring>
#include <sstream>

using namespace std;

#include "biozap_freq.h"
#include "biozap_prog.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MAX_CMD_PARAMS 3

struct paramstruc{
    string param[MAX_CMD_PARAMS];

};

string EOL = "\r\n";
char LF = '\n';

volatile bool com_arrived = false;
string welcome_str = EOL+"Hello World !!!"+EOL+">";
string tx_buffer = "";
volatile bool uart_TX_busy = false;
volatile bool abort_Prog_Run = false;// Abort Command Interpreter running
uint8_t rx_data; 			// Serial receive char
string rx_buffer = ""; 		// Serial receive buffer
string command_Line = ""; 	// arrived command line
string user_Program = "";	// Store user program

void read_flash(string *data)
{
	volatile uint32_t read_data;
	volatile uint32_t read_cnt=0;
	do
	{
		read_data = *(uint32_t*)(FLASH_STORAGE + read_cnt);
		if(read_data != 0xFFFFFFFF)
		{
			*data += (char)read_data;
			*data += (char)(read_data >>  8);
			*data += (char)(read_data >> 16);
			*data += (char)(read_data >> 24);
			read_cnt += 4;
		}
	}while((read_data != 0xFFFFFFFF) && (read_cnt < PAGE_SIZE-1));
	while( (data->length() > 0) && (data->at(data->length()-1) != '\n') ) {
		data->pop_back();		// Delete any extra char before ending LF
	}
}

void save_to_flash(const program data)
{
	  volatile uint32_t data_length = (data.item.length() / 8) + (int)((data.item.length() % 8) != 0);
	  volatile uint64_t data_to_FLASH[data_length];
	  memset((uint8_t*)data_to_FLASH, 0, sizeof((char*)data_to_FLASH));
	  strcpy((char*)data_to_FLASH, data.item.c_str());

	  HAL_FLASH_Unlock();
	  FLASH_EraseInitTypeDef EraseInitStruct;
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.Page = (FLASH_STORAGE-FLASH_START)/PAGE_SIZE;
	  volatile uint16_t pages = (data.item.length()/PAGE_SIZE) + (int)((data.item.length()%PAGE_SIZE) != 0);
	  EraseInitStruct.NbPages = pages;
	  uint32_t PageError;
	  volatile HAL_StatusTypeDef status;
	  if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
		  Error_Handler();
		  return;
	  }

	  volatile uint32_t write_cnt=0, index=0;
	  while(index < data_length) {
		  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_STORAGE+write_cnt, data_to_FLASH[index]);
		  if(status == HAL_OK) {
			  write_cnt += 8;
			  index++;
		  }
		  else {
			  Error_Handler();
			  return;
		  }
	  }
	  HAL_FLASH_Lock();
}

void uart_TX_IT(string inputString){
	while (uart_TX_busy){ // Wait empty TX buffer
		;
	}
	tx_buffer = "";
	tx_buffer += inputString;
	HAL_UART_Transmit_IT(&huart2, (uint8_t *) tx_buffer.c_str(), tx_buffer.length());
	uart_TX_busy = true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		uart_TX_busy = false;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		if(EOL.find(rx_data) != std::string::npos){ // CR or LF
			if(rx_buffer.length() > 0){
				command_Line = "";
				command_Line += rx_buffer+EOL;
				rx_buffer = "";
				abort_Prog_Run = command_Line == "abort\r\n";
				if (abort_Prog_Run)
					command_Line = "";
				else
					com_arrived = true;  // Start Command Interpreter
			}
		}
		else {
			rx_buffer += rx_data;
		}
		HAL_UART_Receive_IT(&huart2,&rx_data, 1);
	}
}

void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac)
{
	__HAL_DAC_DISABLE(hdac, DAC_CHANNEL_1);
	hdac->ErrorCode = 0;
	hdac->State = HAL_DAC_STATE_READY;
	__HAL_DAC_ENABLE(hdac, DAC_CHANNEL_1);
}

void start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel,TIM_HandleTypeDef *htim, uint16_t psc, uint16_t arr)
{
	BIOZAP_Sample_Lgth = min(psc,(uint16_t)BIOZAP_SAMPLE_SIZE);
	generate_sample(0, 4095, BIOZAP_SIN, BIOZAP_SampleArray);
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)BIOZAP_SampleArray, BIOZAP_Sample_Lgth, DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(&htim6);
	__HAL_TIM_SET_AUTORELOAD(htim, arr);
}

void stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel,TIM_HandleTypeDef *htim)
{
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	HAL_TIM_Base_Stop(&htim6);
}


void getParams(string inputString, paramstruc* param){
	int j = 0;
	for (int i = 0; i < MAX_CMD_PARAMS; i++)
			param->param[i] = "";
	for (uint8_t i = 0; i < inputString.length(); i++){
		if((inputString[i] == 32) || (inputString [i] == 10) || (inputString[i] == 13)) {
			if (inputString[i] == 32) {
				if(++j == MAX_CMD_PARAMS) {
					uart_TX_IT("Bad command, too many space!!!"+EOL);
					return;
				}
			}
		}
		else {
			param->param[j] += inputString[i];
		}
	}
}

void findAndReplaceAll(std::string & data, std::string toSearch, std::string replaceStr)
{
	size_t pos = data.find(toSearch);	// Get the first occurrence
	while( pos != std::string::npos) {	// Repeat till end is reached
		data.replace(pos, toSearch.size(), replaceStr); // Replace Sub String
		// Get the next occurrence from the current position
		pos =data.find(toSearch, pos + replaceStr.size());
	}
}

void Delay(uint16_t Del_Time){
	uint32_t delay_end = HAL_GetTick() + Del_Time;
	while(HAL_GetTick() < delay_end) {
		HAL_Delay(10);
		if(abort_Prog_Run)
			return;
	}
}

void send_prg_to_uart(string inp_Str){
	findAndReplaceAll(inp_Str, "\r", "");	// Delete all CR in the inp_Str
	findAndReplaceAll(inp_Str, "\n", EOL);	// Change all LF to EOL in the inp_Str
	uart_TX_IT(inp_Str);
}

void send_ok_to_uart(){
	uart_TX_IT("Ok."+EOL);
}

void Command_Interpreter(string comm_Str)
{
	paramstruc param;
	size_t from = 0;
	size_t to = 0;
	if ( (comm_Str.find(LF, from) == string::npos) || (comm_Str[comm_Str.length()-1] != LF)) {
		uart_TX_IT("Missing LF in input string !"+EOL);
		return;
	}
	while( (to = comm_Str.find(LF, from)+1) >= from ) {  // Get a Command String with LF ending.
		string one_Str = comm_Str.substr(from, to-from); // from a string or a program.
		from = to;
		getParams(one_Str, &param); // set param[x]'s.
		send_prg_to_uart(one_Str);  // send to uart
		if (abort_Prog_Run) {
			uart_TX_IT("Aborting !"+EOL);
			abort_Prog_Run = false;
			return;
		}
		else if(param.param[0].length() == 0){  // Have to checked this, later param[0].at(0) chashed an empty string
			uart_TX_IT("empty command !"+EOL);
		}
		else if (param.param[0] == "freq") {
			uint32_t freq = std::stod(param.param[1]);
			freq_item element = find_time_freq(&htim6, freq);
			if(element.error < 1.0){
				start_DMA(&hdac1, DAC_CHANNEL_1, &htim6, element.psc - 1, element.arr - 1);
				uart_TX_IT("freq:" + to_string(element.freq) + " Sample:" + to_string(BIOZAP_Sample_Lgth) + " arr:" + to_string(element.arr-1) + " working ...  ");
				Delay(std::stol(param.param[2])*1000);
				stop_DMA(&hdac1, DAC_CHANNEL_1, &htim6);
				send_ok_to_uart();
			}
			else{
				uart_TX_IT("freq error :"+to_string((int) element.error)+EOL);
			}
		}
		else if (param.param[0] == "exe") {
			int e_idx = std::stoi(param.param[1]);
			if (e_idx > 0 && e_idx < 10) {
				Command_Interpreter(internalProgram[e_idx].item);
			}
			else if (e_idx == 0) {
				Command_Interpreter(user_Program);
			}
			else {
				uart_TX_IT("Param[1] error ->"+param.param[1]+"<-"+EOL);
			}
		}
		else if (param.param[0] == "ls") {
			send_prg_to_uart(user_Program);
		}
		else if (param.param[0] == "mem") { //
			send_ok_to_uart();
		}
		else if (param.param[0] == "wait") {
			uint32_t delay = std::stol(param.param[1]);
			Delay(delay);
			send_ok_to_uart();
		}
		else if (param.param[0] == "off") { //
			send_ok_to_uart();
		}
		else if (param.param[0] == "beep") { //
			send_ok_to_uart();
		}
		else if (param.param[0] == "pbar") { //
			send_ok_to_uart();
		}
		else if (param.param[0].at(0) == '#') { // comment
			;
		}
		else if (param.param[0].at(0) == ':') { // Label
			;
		}
		else {
			uart_TX_IT("unknown command -->"+param.param[0]+"<--"+EOL);
		}
	}
	if (param.param[0] != "exe")  // exe command recursive call this function
		uart_TX_IT(">");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  uart_TX_IT(welcome_str);  // Welcome string to Serial
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);  // Serial receive starting.

/*
    const program userProgram = {
    		"#Earth rhythm 8m\n"
  		"wait 3000\n"
  		"beep 100\n"
  		"freq 783 480\n"
  		"beep 500\n"
  		"off\n"
    };
    save_to_flash(userProgram); // If userProgram too big change the DATA location, now 2k the size !!!
*/
  read_flash(&user_Program);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//	Delay(1500);
  	if (com_arrived ) {
  		com_arrived = false;
	  	Command_Interpreter(command_Line);
  	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
