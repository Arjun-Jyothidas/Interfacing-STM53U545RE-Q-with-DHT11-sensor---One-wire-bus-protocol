#include "main.h"
#include<stdio.h>
#include "string.h"

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
HCD_HandleTypeDef hhcd_USB_DRD_FS;

void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USB_DRD_FS_HCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void Set_Pin_Out();
void Set_Pin_In();
int Sensor_Initialize();
void us_delay(uint32_t us);
uint8_t read();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t tim_Cnt1 = 0;
uint8_t tim_Cnt2 = 0;
uint8_t int_RH;
uint8_t dec_RH;
uint8_t int_T;
uint8_t dec_T;
uint8_t Checksum;
char Temp[32];
char Humidity[32];
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  SystemPower_Config();

  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_USB_DRD_FS_HCD_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
	
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  int ret = Sensor_Initialize();
  /* USER CODE END 2 */
  if (ret == 1) {
	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1) {
		  int_RH = read();
		  dec_RH = read();
		  int_T = read();
		  dec_T = read();
		  Checksum= read();

		  sprintf(Temp, "Temperature = %d.%d\n\r", int_T, dec_T);
		  sprintf(Humidity, "Humidity = %d.%d\n\r", int_RH, dec_RH);
		  HAL_UART_Transmit(&huart1, (uint8_t*)Temp, strlen(Temp), 50);
		  HAL_UART_Transmit(&huart1, (uint8_t*)Humidity, strlen(Humidity), 50);
		  HAL_Delay(3000);
		  Sensor_Initialize();

  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
	  }
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE3) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void SystemPower_Config(void)
{
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ICACHE_Init(void)
{
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USB_DRD_FS_HCD_Init(void)
{
  hhcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hhcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hhcd_USB_DRD_FS.Init.Host_channels = 8;
  hhcd_USB_DRD_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_DRD_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hhcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hhcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hhcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hhcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DHT11_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void us_delay(uint32_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while((__HAL_TIM_GET_COUNTER(&htim2))<us){}
}

int Sensor_Initialize() {
	int Response = 0;
	Set_Pin_In(); // Low power consumption mode
	HAL_Delay(100);
	Set_Pin_Out(); // Running mode
	// MCU starts communicating with DHT11 sensor
	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
	HAL_Delay(18);
	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
	us_delay(40); // Waiting for response
	Set_Pin_In();

	__HAL_TIM_SET_COUNTER(&htim2, 0);
	tim_Cnt1 = __HAL_TIM_GET_COUNTER(&htim2); // Reading current value of timer
	// start of the response from DHT11 sensor
	while (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))) // waits until the DHT11 sets single bus voltage to high
		tim_Cnt2 = __HAL_TIM_GET_COUNTER(&htim2); // updating with the current value of timer
	if (tim_Cnt2-tim_Cnt1 <= 80) { // checks the duration of the single bus voltage signal
		__HAL_TIM_SET_COUNTER(&htim2, 0); // resets to 0
		tim_Cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
		// end of the response
		while(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)) // waits until the DHT11 sets single bus voltage to low again
			tim_Cnt2 =__HAL_TIM_GET_COUNTER(&htim2);
		if(tim_Cnt2-tim_Cnt1<=80)
			Response = 1;
		else
			Response = -1;
	}
	else
		Response = -1;
	return Response;
}

uint8_t read() {
	uint8_t data={0};
	Set_Pin_In();
	for (int i=0;i<8;i++) {
		__HAL_TIM_SET_COUNTER(&htim2,0);
		tim_Cnt1=__HAL_TIM_GET_COUNTER(&htim2);
		// start of the sensor response
		while (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))) // waits until the DHT11 sets single bus voltage to high
			tim_Cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
		if (tim_Cnt2-tim_Cnt1 <= 50) { // duration of high signal
			__HAL_TIM_SET_COUNTER(&htim2,0);
			tim_Cnt1=__HAL_TIM_GET_COUNTER(&htim2);
			// start of data transmission
			while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)) // waits until the DHT11 sets single bus voltage to low
				tim_Cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
			if (tim_Cnt2-tim_Cnt1 <= 28)
				data |= (0<<(7-i)); // Data bit transmitted is 0
			else if (tim_Cnt2-tim_Cnt1>28 && tim_Cnt2-tim_Cnt1<=80)
				data |= (1<<(7-i)); // Data bit transmitted is 1
			else
				Error_Handler();
		}
	}
	return data;
}

void Set_Pin_In() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = DHT11_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

void Set_Pin_Out() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	 GPIO_InitStruct.Pin = DHT11_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	 HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE END 4 */

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

void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
