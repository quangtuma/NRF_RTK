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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NRF_SIZE 32
#define MAX_SIZE 1000
#define GGA_SIZE 200

#define INTERVAL_MODE_5 2 // = 1s

#define RX
#define Quectel
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

#ifdef TX
uint8_t Send_RTCM(uint8_t* data, uint16_t size);
#endif

#ifdef RX
uint8_t Check_End_Index(uint8_t* packet, uint16_t size, uint16_t start);
void handleUARTError(UART_HandleTypeDef *huart);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef TX
uint8_t nrf_tx_data[MAX_SIZE] = {};
uint8_t ack_Tx[NRF_SIZE];
#endif

#ifdef RX
uint8_t nrf_rx_packet[NRF_SIZE];
uint8_t nrf_rx_data[MAX_SIZE];
uint16_t nrf_packet_id = 0;

uint8_t rover_gga_message[GGA_SIZE];
uint8_t rover_mode;
uint8_t interval_mode_5 = INTERVAL_MODE_5;

uint8_t ack_Rx[NRF_SIZE] = { "Received" };
volatile uint8_t gga_index = 0;      // Tracks the current index for incoming data
volatile uint8_t data_ready = 0;    // Flag to signal that a string is ready
#endif

#ifdef TEST_TX
uint8_t tx_test[NRF_SIZE] = {"Hello"};
#endif

#ifdef TEST_RX
uint8_t rx_test[NRF_SIZE] = {};
#endif

uint16_t rtcm_end_id = 0;

uint8_t GGA_set_command[18] = { 0x24, 0x50, 0x41, 0x49, 0x52, 0x30, 0x36, 0x32, 0x2C, 0x30, 0x2C, 0x30, 0x31, 0x2A, 0x30, 0x46, 0x0D, 0x0A }; //$PAIR062,0,01*0F
uint8_t GGA_start[6] = { 0x24, 0x47, 0x4E, 0x47, 0x47, 0x41 };
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  csn_high();
  nrf24_init();
  nrf24_tx_pwr(_0dbm);
  nrf24_data_rate(_1mbps);
  nrf24_set_channel(78);
  nrf24_set_crc(en_crc, _1byte);
  nrf24_pipe_pld_size(0, NRF_SIZE);
  uint8_t addr[5] = { 0x10, 0x21, 0x32, 0x43, 0x54 };
  nrf24_open_tx_pipe(addr);
  nrf24_open_rx_pipe(0, addr);

  HAL_Delay(1000); // waiting initial Station
#ifdef TEST_TX
  nrf24_stop_listen();
#endif

#ifdef TEST_RX
  nrf24_listen();
#endif

#ifdef TX
  nrf24_stop_listen();

#ifdef Quectel
  HAL_UART_Transmit(&huart2, GGA_set_command, sizeof(GGA_set_command) / sizeof(uint8_t), 100);
  HAL_Delay(500); // waiting initial GGA Command
#endif

#endif

#ifdef RX
  nrf24_listen();
  HAL_UART_Receive_IT(&huart2, &rover_gga_message[gga_index], 1);
  HAL_TIM_Base_Start_IT(&htim3);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef TEST_TX
	  nrf24_transmit(tx_test, sizeof(tx_test));
	  HAL_Delay(1);
#endif
#ifdef TEST_RX
	  nrf24_listen();
	  if (nrf24_data_available()){
		  nrf24_receive(rx_test, sizeof(rx_test));

		  char tmp[40];
		  sprintf(tmp, " %s .\r\n", rx_test);

		  HAL_UART_Transmit(&huart2, rx_test, NRF_SIZE, 100);

		  for (uint8_t i = 0; i < sizeof(rx_test); i++){
			  rx_test[i] = '\0';
		  }
		  HAL_Delay(5);
	  }
#endif
#ifdef TX

	  HAL_UART_Receive(&huart2, (uint8_t *)&nrf_tx_data, MAX_SIZE, 250);
	  // get end of data
	  uint8_t d3_count = 0;
	  for (int i = 0; i < MAX_SIZE; i++){
		 if (nrf_tx_data[i] == 0x00 && nrf_tx_data[i - 1] == 0xD3)
			  d3_count++;
		 if(d3_count >= 5 && nrf_tx_data[i] == 0x0A && nrf_tx_data[i-1] == 0x0D)
		 {
			 rtcm_end_id = i + 1;
			 break;
		 }
	  }
	  if (rtcm_end_id != 0){
		  // separate data and send chunk
		  uint8_t check[rtcm_end_id]; // to extend to send
		  memcpy(check, nrf_tx_data, sizeof(check));

		  //uint8_t ret = Send_RTCM(check, rtcm_end_id);

		  Send_RTCM(check, rtcm_end_id);
		  // clear buffer
		  for (int i = 0; i < rtcm_end_id; i++){
			  nrf_tx_data[i] = '\0';
		  }
		  d3_count = 0;
		  rtcm_end_id = 0;

		  HAL_Delay(1);
	  }
#endif
#ifdef RX
	  // NRF Receiver Processing
	  nrf24_listen();
	  if (nrf24_data_available()){
		  nrf24_receive(nrf_rx_packet, sizeof(nrf_rx_packet));

		  //HAL_UART_Transmit(&huart2, nrf_rx_packet, NRF_SIZE, 10);

		  memcpy(&nrf_rx_data[nrf_packet_id], nrf_rx_packet, sizeof(nrf_rx_packet) / sizeof(uint8_t));
		  nrf_packet_id += NRF_SIZE;

		  if (Check_End_Index(nrf_rx_data, MAX_SIZE, 0) == 1){

			  HAL_UART_Transmit(&huart2, nrf_rx_data, rtcm_end_id, 250);

			  for (uint16_t i = 0; i < rtcm_end_id; i++){
				  nrf_rx_data[i] = '\0';
			  }

			  nrf_packet_id = 0;
			  rtcm_end_id = 0;
		  }

		  for (uint8_t i = 0; i < NRF_SIZE; i++){
			  nrf_rx_packet[i] = '\0';
		  }
	  }

	  // UART (Rover GGA) Receiver Processing
	  if (data_ready) {

		  data_ready = 0;  // Reset flag

		  int comma_count = 0;
		  for(int j = 0; j < GGA_SIZE; j++){
			  if (rover_gga_message[j] == 0x2C){ // 2C is ','
				  comma_count++;
				  if (comma_count == 6) // from $GNGGA to Rover Mode
				  {
					  rover_mode = rover_gga_message[j + 1];
					  HAL_UART_Receive_IT(&huart2, &rover_gga_message[gga_index], 1);
					  break;
				  }
			  }
		  }
		  memset(rover_gga_message, 0, sizeof(rover_gga_message));
	  }

#endif
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef RX
uint8_t Check_End_Index(uint8_t* data, uint16_t size, uint16_t start)
{
	uint8_t d3_count = 0;
	for (int i = start; i < size; i++){
		if (data[i] == 0xD3)
			d3_count++;

		if(d3_count >=5 && data[i-1] == 0x0D && data[i] == 0x0A)
		{
			rtcm_end_id = i + 1;
			break;
		}
	}

	if (rtcm_end_id != 0)
		return 1;
	else
		return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		switch (rover_mode)
		{
			case '0':
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			break;
			case '4':
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				break;
			case '5':
				interval_mode_5--;
				if (interval_mode_5 == 0){
					interval_mode_5 = INTERVAL_MODE_5;
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				}
				break;
			default:
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				break;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART2){
    	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) ||
			__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) ||
			__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)) {

			// Handle UART errors (e.g., reset UART, alert user)
			handleUARTError(huart);
		}
		// Check if the buffer is full
		if (rover_gga_message[gga_index] == 0x0A
				&& rover_gga_message[gga_index - 1] == 0x0D) {
			data_ready = 1;  // Set flag to indicate data reception is complete
			gga_index = 0;
		}
		else {
	    	gga_index++;
			// Continue receiving next byte if buffer is not yet full
			HAL_UART_Receive_IT(&huart2, &rover_gga_message[gga_index], 1);
		}
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Handle error if UART has a problem (e.g., disconnection)
        handleUARTError(huart);
    }
}

void handleUARTError(UART_HandleTypeDef *huart) {
    // Disable UART interrupts to stop triggering further
    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);

    // Optionally, reset UART (if needed)
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    // Re-enable UART for reception
    HAL_UART_Receive_IT(huart, &rover_gga_message[gga_index], 1);
}
#endif

#ifdef TX
uint8_t Send_RTCM(uint8_t* data, uint16_t size)
{
	uint8_t ret = 0;
	uint8_t chunk[NRF_SIZE];

	for (uint16_t i = 0; i < size; i += NRF_SIZE) {

		uint16_t packet_size = 0;

		if (size - i < NRF_SIZE){
			packet_size = size - i;
		}
		else{
			packet_size = NRF_SIZE;
		}

		memcpy(chunk, data + i, packet_size * sizeof(uint8_t));

		// Send the 32-byte chunk

		//HAL_UART_Transmit(&huart2, chunk, NRF_SIZE, 5);
		ret = nrf24_transmit(chunk, sizeof(chunk));
		if (ret == 0){
			return ret;
		}

		for(uint8_t j = 0; j < NRF_SIZE; j++){
			chunk[j] = '\0';
		}

		// Short delay if necessary to avoid overload
		HAL_Delay(1);
	}
	return ret;
}
#endif
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
