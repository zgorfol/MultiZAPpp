/*
 * biozap_comm.cpp
 *
 *  Created on: Sep 6, 2020
 *      Author: z_gorfol
 */

#include "main.h"
#include <string>
#include <cstring>
#include <sstream>
#include <functional>

#include "biozap_comm.h"
#include "biozap_freq.h"
#include "biozap_prog.h"
using namespace std;

#define MAX_CMD_PARAMS 3

struct paramstruc{
    string param[MAX_CMD_PARAMS];

};

uint16_t vout = 1200;
uint16_t vmin =    0;

volatile bool command_arrived = false;
volatile bool command1_arrived = false;
volatile bool uart_TX_busy = false;
volatile bool uart_TX1_busy = false;
volatile bool abort_Prog_Run = false;// Abort Command Interpreter running

char LF = '\n';
string EOL = "\r\n";

uint8_t rx_data; 			// Serial receive char
string tx_buffer = "";
string rx_buffer = ""; 		// Serial receive buffer
string command_Line = ""; 	// arrived command line

uint8_t rx1_data; 			// Serial receive char
string tx1_buffer = "";
string rx1_buffer = ""; 		// Serial receive buffer
string command_Line1 = ""; 	// arrived command line

string user_Program = "";	// Store user program


void uart_TX_IT(UART_HandleTypeDef *huart, string inputString){
	if (huart->Instance == USART2) {
		while (uart_TX_busy){ // Wait empty TX buffer
			;
		}
		tx_buffer = "";
		tx_buffer += inputString;
		HAL_UART_Transmit_IT(huart, (uint8_t *) tx_buffer.c_str(), tx_buffer.length());
		uart_TX_busy = true;
	}
	if (huart->Instance == USART1) {
		while (uart_TX1_busy){ // Wait empty TX buffer
			;
		}
		tx1_buffer = "";
		tx1_buffer += inputString;
		HAL_UART_Transmit_IT(huart, (uint8_t *) tx1_buffer.c_str(), tx1_buffer.length());
		uart_TX1_busy = true;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		uart_TX_busy = false;
	}
	if (huart->Instance == USART1) {
		uart_TX1_busy = false;
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
				abort_Prog_Run = command_Line == "abort"+EOL;
				if (abort_Prog_Run)
					command_Line = "beep 500"+EOL;
				command_arrived = true;  // Start Command Interpreter
			}
		}
		else {
			rx_buffer += rx_data;
		}
		HAL_UART_Receive_IT(huart,&rx_data, 1);
	}
	if (huart->Instance == USART1) {
		if(EOL.find(rx1_data) != std::string::npos){ // CR or LF
			if(rx1_buffer.length() > 0){
				command_Line1 = "";
				command_Line1 += rx1_buffer+EOL;
				rx1_buffer = "";
				abort_Prog_Run = command_Line1 == "abort"+EOL;
				if (abort_Prog_Run)
					command_Line1 = "beep 500"+EOL;
				command1_arrived = true;  // Start Command Interpreter
			}
		}
		else {
			rx1_buffer += rx1_data;
		}
		HAL_UART_Receive_IT(huart,&rx1_data, 1);
	}
}

void start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel,TIM_HandleTypeDef *htim, uint16_t psc, uint16_t arr)
{
	extern int BIOZAP_Sample_Max;
	extern int BIOZAP_Sample_Lgth;
	extern uint16_t BIOZAP_SampleArray[];

	BIOZAP_Sample_Lgth = min(psc,(uint16_t)BIOZAP_Sample_Max);
	generate_sample(vmin, vout-vmin, BIOZAP_SIN, BIOZAP_SampleArray);
	HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*)BIOZAP_SampleArray, BIOZAP_Sample_Lgth, DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(htim);
	__HAL_TIM_SET_PRESCALER(htim, 0);
	__HAL_TIM_SET_AUTORELOAD(htim, arr);
}

void stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel,TIM_HandleTypeDef *htim)
{
	HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_1);
	HAL_TIM_Base_Stop(htim);
}

void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac)
{
	__HAL_DAC_DISABLE(hdac, DAC_CHANNEL_1);
	hdac->ErrorCode = 0;
	hdac->State = HAL_DAC_STATE_READY;
	__HAL_DAC_ENABLE(hdac, DAC_CHANNEL_1);
}


void Delay(uint32_t Del_Time){
	uint32_t delay_end = HAL_GetTick() + Del_Time;
	while(HAL_GetTick() < delay_end) {
		HAL_Delay(1);
		if(abort_Prog_Run)
			return;
	}
}

void getParams(string inputString, paramstruc* param, UART_HandleTypeDef *huart, std::function<void (UART_HandleTypeDef *, string)> uart_TX_IT){
	int j = 0;
	for (int i = 0; i < MAX_CMD_PARAMS; i++)
			param->param[i] = "";
	for (uint8_t i = 0; i < inputString.length(); i++){
		if((inputString[i] == 32) || (inputString [i] == 10) || (inputString[i] == 13)) {
			if (inputString[i] == 32) {
				if(++j == MAX_CMD_PARAMS) {
					uart_TX_IT(huart, "Bad command, too many space!!!"+EOL);
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

void beep(uint32_t delay_time){
	HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_SET);
	Delay(delay_time);
	HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_RESET);
}

void Command_Interpreter(string comm_Str, UART_HandleTypeDef *huart, DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim, std::function<void (UART_HandleTypeDef *, string)> uart_TX_IT)
{
	extern int BIOZAP_Sample_Lgth;
//	extern uint16_t vout;
//	extern uint16_t vmin;

	paramstruc param;
	size_t from = 0;
	size_t to = 0;
	if ( (comm_Str.find(LF, from) == string::npos) || (comm_Str[comm_Str.length()-1] != LF)) {
		uart_TX_IT(huart, "Missing LF in input string !"+EOL);
		return;
	}
	while( (to = comm_Str.find(LF, from)+1) >= from ) {  // Get a Command String with LF ending.
		string one_Str = comm_Str.substr(from, to-from); // from a string or a program.
		from = to;
		getParams(one_Str, &param, huart, uart_TX_IT); // set param[x]'s.
		findAndReplaceAll(one_Str, "\r", "");	// Delete all CR in the inp_Str
		findAndReplaceAll(one_Str, "\n", EOL);	// Change all LF to EOL in the inp_Str
		uart_TX_IT(huart, one_Str);
		if (abort_Prog_Run) {
			uart_TX_IT(huart, "Aborting !"+EOL+">");
			abort_Prog_Run = false;
			return;
		}
		else if(param.param[0].length() == 0){  // Have to checked this, later param[0].at(0) chashed an empty string
			uart_TX_IT(huart, "empty command !"+EOL);
		}
		else if (param.param[0] == "freq") {
			double freq = std::stod(param.param[1]);
			freq_item element = find_time_freq(htim, freq);
			if(element.error < 1.0){
				start_DMA(hdac, DAC_CHANNEL_1, htim, element.psc - 1, element.arr - 1);

				string numstr = to_string((uint32_t)element.freq);
				string precstr = to_string((uint32_t)((element.freq-(uint32_t)element.freq)*100));
				uart_TX_IT(huart, "freq:" + numstr + "." + precstr + " Sample:" + to_string(BIOZAP_Sample_Lgth) + " arr:" + to_string(element.arr-1) + " working ...  ");
				Delay(std::stol(param.param[2])*1000);
				stop_DMA(hdac, DAC_CHANNEL_1, htim);
				uart_TX_IT(huart, "Ok."+EOL);
			}
			else{
				uart_TX_IT(huart, "freq error :"+to_string((int) element.error)+EOL);
			}
		}
		else if (param.param[0] == "exe") {
			int e_idx = std::stoi(param.param[1]);
			if (e_idx > 0 && e_idx < 10) {
				Command_Interpreter(internalProgram[e_idx].item, huart, hdac, htim, uart_TX_IT);
			}
			else if (e_idx == 0) {
				Command_Interpreter(user_Program, huart, hdac, htim, uart_TX_IT);
			}
			else {
				uart_TX_IT(huart, "Param[1] error ->"+param.param[1]+"<-"+EOL);
			}
		}
		else if (param.param[0] == "ls") {
			findAndReplaceAll(user_Program, "\r", "");	// Delete all CR in the inp_Str
			findAndReplaceAll(user_Program, "\n", EOL);	// Change all LF to EOL in the inp_Str
			uart_TX_IT(huart, user_Program);
		}
		else if (param.param[0] == "mem") {
			uart_TX_IT(huart, "Ok."+EOL);
		}
		else if (param.param[0] == "wait") {
			Delay(std::stol(param.param[1]));
			uart_TX_IT(huart, "Ok."+EOL);
		}
		else if (param.param[0] == "off") {
			uart_TX_IT(huart, "Ok."+EOL);
		}
		else if (param.param[0] == "beep") {
			if (param.param[1] == "")
				param.param[1] = "100";
			beep(std::stol(param.param[1]));
			uart_TX_IT(huart, "Ok."+EOL);
		}
		else if (param.param[0] == "vout") {
			if (param.param[1] == "")
				param.param[1] = to_string(vout);
			vout = std::stol(param.param[1]);
			uart_TX_IT(huart, param.param[1]+EOL+"Ok."+EOL);
		}
		else if (param.param[0] == "vmin") {
			if (param.param[1] == "")
				param.param[1] = to_string(vmin);
			vmin = std::stol(param.param[1]);
			uart_TX_IT(huart, param.param[1]+EOL+"Ok."+EOL);
		}
		else if (param.param[0] == "pbar") {
			uart_TX_IT(huart, "Ok."+EOL);
		}
		else if (param.param[0].at(0) == '#') { // comment
			;
		}
		else if (param.param[0].at(0) == ':') { // Label
			;
		}
		else {
			uart_TX_IT(huart, "unknown command -->"+param.param[0]+"<--"+EOL);
		}
	}
	if (param.param[0] != "exe")  // exe command recursive call this function
		uart_TX_IT(huart, ">");
}

void Cmd_Interpreter(UART_HandleTypeDef *huart, DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim)
{
	if (huart->Instance == USART2) {
		Command_Interpreter(command_Line, huart, hdac, htim , uart_TX_IT);
	}
	if (huart->Instance == USART1) {
		Command_Interpreter(command_Line1, huart, hdac, htim , uart_TX_IT);
	}
}
