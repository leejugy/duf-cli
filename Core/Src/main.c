/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
extern uint8_t usb_rx_val[5];
extern bool usb_send;
extern UART_HandleTypeDef huart1;
bool uart_flag=false;
bool toggle_do=false;
uint32_t toggle_val=100;
uint8_t buffer_tx[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sum_word(uint8_t *buf,uint8_t length,char *fmt,...);
bool is_same_string(uint8_t *buf, uint8_t start, uint8_t end,char *compare_string);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
void send_cdc(uint8_t* buffer);
void send_flash_val(uint8_t length,uint32_t add);

void usb_reset(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	  /*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void sum_word(uint8_t *buf,uint8_t length,char *fmt,...){
	va_list arg;
	va_start(arg,fmt);
	memset(buf,0,length);
	vsnprintf((char*)buf,length,fmt,arg);
	va_end(arg);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_flag=true;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}

bool is_same_string(uint8_t *buf, uint8_t start, uint8_t end,char *compare_string)
{
	uint8_t j=0;
	bool ret=false;
	for(uint8_t i=start;i<end;i++)
	{
		if(buf[i]==0)
		{
			break;
		}
		if(buf[i]==compare_string[j++]){
			ret=true;
		}
		else
		{
			ret=false;
			break;
		}
	}
	return ret;
}


void printf_select_area(uint8_t *buf,uint8_t start,uint8_t end)
{
	for(uint8_t i=start+1;i<end;i++){
		uint8_t you=buf[i];
		HAL_Delay(1);
		CDC_Transmit_FS(&you,1);
	}
	HAL_Delay(1);
}

void send_cdc(uint8_t* buffer){
	uint8_t length = strlen((char *)buffer);
	CDC_Transmit_FS(buffer,length);
}

void send_flash_val(uint8_t length,uint32_t add){
	uint8_t *flash_value=(uint8_t *)add;
	send_cdc((uint8_t *)"\n");
	memset(buffer_tx,0,64);
	for(uint8_t i=0;i<length;i++){
		sum_word(buffer_tx,64,"add : 0x%X, val : 0x%X\n",add+i,flash_value[i]);
		HAL_Delay(1);
		send_cdc(buffer_tx);
	}
}

void enter_press(uint8_t *buffer){
	if(is_same_string(buffer,0,6,(char*)"printf")){
		send_cdc((uint8_t*)"\n>>");
		printf_select_area(buffer,6,32);
		send_cdc((uint8_t*)"\n");
	}
	else if(is_same_string(buffer,0,4,(char*)"help")){
		send_cdc((uint8_t*)"\n\n");
		HAL_Delay(1);
		send_cdc((uint8_t*)"********************************************************************************\n");
		HAL_Delay(1);
		send_cdc((uint8_t*)"command list\n\n\n");
		HAL_Delay(1);
		send_cdc((uint8_t*)"1. led on - turn on led\n\n");
		HAL_Delay(1);
		send_cdc((uint8_t*)"2. led off - turn off led\n\n");
		HAL_Delay(1);
		send_cdc((uint8_t*)"3. led toggle [second] - toggle led at user set time\n\n");
		HAL_Delay(1);
		send_cdc((uint8_t*)"4. printf [value] - print value\n\n");
		HAL_Delay(1);
		send_cdc((uint8_t*)"5. flashread [value] - read flash memory data, start from 0x08014000 to value\n\n");
		HAL_Delay(1);
		send_cdc((uint8_t*)"********************************************************************************\n\n");
	}
	else if(is_same_string(buffer,0,3,(char*)"led")){
		if(is_same_string(buffer,4,6,(char*)"on")){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			toggle_do=false;
			send_cdc((uint8_t*)"\n");
		}
		if(is_same_string(buffer,4,7,(char*)"off")){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			toggle_do=false;
			send_cdc((uint8_t*)"\n");
		}
		if(is_same_string(buffer,4,10,(char*)"toggle")){
			toggle_do=true;
			uint8_t toggle_val_index=11;
			uint8_t pre_toggle_val=toggle_val;
			if(buffer[11]!=0){
				toggle_val=0;
				while(buffer[toggle_val_index]!=0){
					if(buffer[toggle_val_index]<48 || buffer[toggle_val_index]>57){
						send_cdc((uint8_t*)"\ninput value is not number!\n");
						toggle_val=pre_toggle_val;
						break;
					}
					toggle_val=toggle_val*10+(buffer[toggle_val_index++]-48);
				}
			}
			send_cdc((uint8_t*)"\n");
		}
	}
	else if(is_same_string(buffer,0,9,(char*)"flashread")){
		uint8_t flash_add_index=10;
		uint8_t flash_add_val=0;
		if(buffer[10]!=0){
			while(buffer[flash_add_index]!=0){
				if(buffer[flash_add_index]<48 || buffer[flash_add_index]>57){
					send_cdc((uint8_t*)"\ninput value is not number!\n");
					flash_add_val=0;
					break;
				}
				flash_add_val=flash_add_val*10+(buffer[flash_add_index++]-48);
			}
			if(flash_add_val!=0){
				send_flash_val(flash_add_val,0x08005000+(60*1024));
			}
		}
	}
	else
	{
		send_cdc((uint8_t*)"\ninvalid input! typing [help] to get information about cli.\n");
	}
	HAL_Delay(1);
	send_cdc((uint8_t*)"\nCLI >> ");
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  usb_reset();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uint32_t pretime=HAL_GetTick();
  uint8_t uart_rx_val=0;
  uint8_t uart1_rx[5]={0,0,0,0,0};
  uint8_t uart_index;
  uint8_t command_line[32];
  memset(command_line,0,5);
  uint8_t command_line_index=0;
  uint8_t current_cursor_index=0;
  uint32_t pre_time1=HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	   * ANSI escape sequence
	   * \x1B[1D:cursor left move
	   * \x1B[1D:cursor right move
	   * \x1B[1P:delete from cursor locate
	   * \x1B[4h(%c):insert mode set, %c is inserted of value at cursor locate
	   * \x1B[4l:replace mode
	   * \x1B[2J:clear terminal
	   * */
	  HAL_UART_Receive_DMA(&huart1,&uart_rx_val,1);
	  if(uart_flag){
		  uint32_t tick_val=HAL_GetTick();
		  if(tick_val-pre_time1>50){
		  	pre_time1=HAL_GetTick();
		  	memset(uart1_rx,0,5);
		  	uart_index=0;
		  }
		  uart_index%=5;
		  uart1_rx[uart_index++]=uart_rx_val;

		  sum_word(buffer_tx,64,"%X %X %X %X %X\n",uart1_rx[0],uart1_rx[1],uart1_rx[2],uart1_rx[3],uart1_rx[4]);
		  uart_flag=false;
		  HAL_UART_Transmit(&huart1,buffer_tx,64,100);
	  }
	  if(HAL_GetTick()-pretime>toggle_val && toggle_do){
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		  pretime=HAL_GetTick();
	  }
	  if(usb_send){
		  switch(usb_rx_val[0]){
		  case(0x1B)://when esc begin
				  switch(usb_rx_val[2]){
				  case(0x44)://left move
						  if(current_cursor_index>0){
							  sum_word(buffer_tx,64,"\x1B[1D");
							  current_cursor_index--;
							  CDC_Transmit_FS(buffer_tx,64);
						  }
						  break;
				  case(0x43)://right move
						  if(current_cursor_index<command_line_index){
							  sum_word(buffer_tx,64,"\x1B[1C");
							  current_cursor_index++;
							  CDC_Transmit_FS(buffer_tx,64);
						  }
						  break;
				  case(0x42)://down move
						  break;
				  case(0x41)://up move
						  break;
				  case(0x31)://home button
						  for(uint8_t i=0;i<current_cursor_index;i++){
							  sum_word(buffer_tx,64,"\x1B[1D");
							  CDC_Transmit_FS(buffer_tx,64);
							  HAL_Delay(1);
						  }
						  current_cursor_index=0;
						  break;
				  case(0x34)://end button
							 for(uint8_t i=current_cursor_index;i<command_line_index;i++){
								 sum_word(buffer_tx,64,"\x1B[1C");
								 CDC_Transmit_FS(buffer_tx,64);
								 HAL_Delay(1);
							 }
						  current_cursor_index=command_line_index;
						  break;
				  }
				  break;
		  case(0x08)://backspace
				  if(current_cursor_index==0 || command_line_index==0){
					  break;
				  }
				  else if(current_cursor_index>0){
					  command_line[--current_cursor_index]=0;
				  	  command_line_index--;
				  	  for(uint8_t i=current_cursor_index;i<command_line_index;i++){
				  		  if(i+1>31){
				  			command_line[i]=0;
				  			break;
				  		  }
				  		  command_line[i]=command_line[i+1];
				  		  command_line[i+1]=0;
				  	  }
				  	  sum_word(buffer_tx,64,"\x1B[1D\x1B[1P");
				  	  CDC_Transmit_FS(buffer_tx,64);
				  }
		  	  	  break;
		  case(0x0D)://case enter
				  enter_press(command_line);
				  memset(command_line,0,32);
		  	  	  command_line_index=0;
		  	  	  current_cursor_index=0;
		  	  	  break;
		  case(0x7F)://case delete
				  if(command_line_index==0||command_line_index==current_cursor_index){
					  break;
				  }
				  for(uint8_t i=current_cursor_index;i<command_line_index;i++){
					  if(i+1>31){
						  command_line[i]=0;
						  break;
					  }
					  command_line[i]=command_line[i+1];
					  command_line[i+1]=0;
				  }
				  command_line_index--;
				  sum_word(buffer_tx,64,"\x1B[1P");
				  CDC_Transmit_FS(buffer_tx,64);
				  break;
		  default://insertion
			  	  if(command_line_index>31){
			  		  break;
			  	  }
			  	  for(uint8_t i=command_line_index;i>current_cursor_index;i--){
			  		  if(i>31){
			  			  break;
			  		  }
			  		  command_line[i]=command_line[i-1];
			  	  }
			  	  command_line[current_cursor_index++]=usb_rx_val[0];
			  	  command_line_index++;
			  	  sum_word(buffer_tx,64,"\x1B[4h%c\x1B[4l",usb_rx_val[0]);
			  	  CDC_Transmit_FS(buffer_tx,64);
			  	  break;
		  }
		  usb_send=false;
		  sum_word(buffer_tx,64,"\x1B[2J\nsend val:%s\n",command_line);
		  HAL_UART_Transmit(&huart1,buffer_tx,64,100);
		  sum_word(buffer_tx,64,"send_command:0x%X %X %X %X %X\n",usb_rx_val[0],usb_rx_val[1], usb_rx_val[2],usb_rx_val[3],usb_rx_val[4]);
		  HAL_UART_Transmit(&huart1,buffer_tx,64,100);
		  sum_word(buffer_tx,64,"cursor:%d index:%d",current_cursor_index,command_line_index);
		  HAL_UART_Transmit(&huart1,buffer_tx,64,100);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
