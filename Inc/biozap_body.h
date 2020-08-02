

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

void uart1_TX_IT(string inputString){
	while (uart1_TX_busy){ // Wait empty TX buffer
		;
	}
	tx1_buffer = "";
	tx1_buffer += inputString;
	HAL_UART_Transmit_IT(&huart1, (uint8_t *) tx1_buffer.c_str(), tx1_buffer.length());
	uart1_TX_busy = true;
}

void uart2_TX_IT(string inputString){
	while (uart2_TX_busy){ // Wait empty TX buffer
		;
	}
	tx2_buffer = "";
	tx2_buffer += inputString;
	HAL_UART_Transmit_IT(&huart2, (uint8_t *) tx2_buffer.c_str(), tx2_buffer.length());
	uart2_TX_busy = true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		uart1_TX_busy = false;
	}
	if (huart->Instance == USART2) {
		uart2_TX_busy = false;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		if(EOL.find(rx1_data) != std::string::npos){ // CR or LF
			if(rx1_buffer.length() > 0){
				command1_Line = "";
				command1_Line += rx1_buffer+EOL;
				rx1_buffer = "";
				abort_Prog_Run = command1_Line == "abort"+EOL;
				if (abort_Prog_Run)
					command1_Line = "beep 500"+EOL;
				command1_arrived = true;  // Start Command Interpreter
			}
		}
		else {
			rx1_buffer += rx1_data;
		}
		HAL_UART_Receive_IT(&huart1,&rx1_data, 1);
	}
	if (huart->Instance == USART2) {
		if(EOL.find(rx2_data) != std::string::npos){ // CR or LF
			if(rx2_buffer.length() > 0){
				command2_Line = "";
				command2_Line += rx2_buffer+EOL;
				rx2_buffer = "";
				abort_Prog_Run = command2_Line == "abort"+EOL;
				if (abort_Prog_Run)
					command2_Line = "beep 500"+EOL;
				command2_arrived = true;  // Start Command Interpreter
			}
		}
		else {
			rx2_buffer += rx2_data;
		}
		HAL_UART_Receive_IT(&huart2,&rx2_data, 1);
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
	generate_sample(0, 1100, BIOZAP_SIN, BIOZAP_SampleArray);
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)BIOZAP_SampleArray, BIOZAP_Sample_Lgth, DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(&htim6);
	__HAL_TIM_SET_AUTORELOAD(htim, arr);
}

void stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel,TIM_HandleTypeDef *htim)
{
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	HAL_TIM_Base_Stop(&htim6);
}


void getParams(string inputString, paramstruc* param, std::function<void (string)> uart_TX_IT){
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

void Delay(uint32_t Del_Time){
	uint32_t delay_end = HAL_GetTick() + Del_Time;
	while(HAL_GetTick() < delay_end) {
		HAL_Delay(1);
		if(abort_Prog_Run)
			return;
	}
}

void beep(uint32_t delay_time){
	HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_SET);
	Delay(delay_time);
	HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_RESET);
}


void Command_Interpreter(string comm_Str, std::function<void (string)> uart_TX_IT)
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
		getParams(one_Str, &param, uart_TX_IT); // set param[x]'s.
		findAndReplaceAll(one_Str, "\r", "");	// Delete all CR in the inp_Str
		findAndReplaceAll(one_Str, "\n", EOL);	// Change all LF to EOL in the inp_Str
		uart_TX_IT(one_Str);
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
				uart_TX_IT("Ok."+EOL);
			}
			else{
				uart_TX_IT("freq error :"+to_string((int) element.error)+EOL);
			}
		}
		else if (param.param[0] == "exe") {
			int e_idx = std::stoi(param.param[1]);
			if (e_idx > 0 && e_idx < 10) {
				Command_Interpreter(internalProgram[e_idx].item, uart_TX_IT);
			}
			else if (e_idx == 0) {
				Command_Interpreter(user_Program, uart_TX_IT);
			}
			else {
				uart_TX_IT("Param[1] error ->"+param.param[1]+"<-"+EOL);
			}
		}
		else if (param.param[0] == "ls") {
			findAndReplaceAll(user_Program, "\r", "");	// Delete all CR in the inp_Str
			findAndReplaceAll(user_Program, "\n", EOL);	// Change all LF to EOL in the inp_Str
			uart_TX_IT(user_Program);
		}
		else if (param.param[0] == "mem") {
			uart_TX_IT("Ok."+EOL);
		}
		else if (param.param[0] == "wait") {
			Delay(std::stol(param.param[1]));
			uart_TX_IT("Ok."+EOL);
		}
		else if (param.param[0] == "off") {
			uart_TX_IT("Ok."+EOL);
		}
		else if (param.param[0] == "beep") {
			if (param.param[1] == "")
				param.param[1] = "100";
			beep(std::stol(param.param[1]));
			uart_TX_IT("Ok."+EOL);
		}
		else if (param.param[0] == "pbar") {
			uart_TX_IT("Ok."+EOL);
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



