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
#include "lcd.h"
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t uart1_rx_buffer[2048];// for uart1 receive buffer
uint8_t uart2_rx_buffer[2048];// for uart2 receive buffer

unsigned char a[25] = {0};
unsigned char b[25] = {0};
unsigned char c[25] = {0};
unsigned char d[25] = {0};
unsigned char e[25] = {0};
unsigned char f[25] = {0};
unsigned char g[25] = {0};
unsigned char h[25] = {0};
unsigned char i[25] = {0};

unsigned int aa = 0;
unsigned int LENGTH = 7;
unsigned int ba = 0;
unsigned int ca = 0;
unsigned int da = 0;
unsigned int ea = 0;
unsigned int fa = 0;
unsigned int ga = 0;
unsigned int ha = 0;
unsigned int ia = 0;


unsigned int al = 0;
unsigned int bl = 0;
unsigned int cl = 0;
unsigned int dl = 0;
unsigned int el = 0;
unsigned int fl = 0;
unsigned int gl = 0;
unsigned int hl = 0;
unsigned int il = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char uRx_Data[2048];
int uLength = 0;


//Call this method to clear all the information.
void reset(){
	for(int i = 0;i < 2048;i++){
		uRx_Data[i] = 0;
	}
	uLength = 0;
	for(int x = 0;x < 25;x++){
		a[x] = 0;
		b[x] = 0;
		c[x] = 0;
		d[x] = 0;
		e[x] = 0;
		f[x] = 0;
		g[x] = 0;
		h[x] = 0;
		i[x] = 0;
	}

	aa = 0;
	LENGTH = 7;
	ba = 0;
	ca = 0;
	da = 0;
	ea = 0;
	fa = 0;
	ga = 0;
	ha = 0;
	ia = 0;

	al = 0;
	bl = 0;
	cl = 0;
	dl = 0;
	el = 0;
	fl = 0;
	gl = 0;
	hl = 0;
	il = 0;
	changeState(0);
	showMessage();
}

// This is called when the connection status changes
void StateChange(int connected){
	changeState(connected);
	showMessage();
}

// This method is called when sending a message.
// usart : 1 -> The machine sends a message.
// usart : 2 -> The machine received the message.
void printOut(unsigned char newline[],unsigned char ul, int usart,int connected){
	changeState(connected);
	updateuD(newline,ul,usart);
	showMessage();
}

void rightDialog(
	  	int x0, int y0,int r,int L,uint16_t c){
	POINT_COLOR = c;
  	int a, b;
  	int di;
	a = 0;
	b = r;
	di = 3 - (r << 1);
	while (a <= b) {
		LCD_DrawPoint(x0 - a, y0 + b);             //1
		LCD_DrawPoint(x0 - b, y0 + a);
		LCD_DrawPoint(x0 - a, y0 - b);             //2
		LCD_DrawPoint(x0 - b, y0 - a);             //7
		a++;
		if (di < 0)
			di += 4 * a + 6;
		else {
			di += 10 + 4 * (a - b);
			b--;
		}
	}

	int x_1 = x0;
	int y_1 = y0 - r;
	int x_2 = x0 + L;
	int y_2 = y0 + r;
	LCD_DrawLine(x_1, y_1, x_2 + r, y_1);
	LCD_DrawLine(x_2 + r, y_1,x_2 + sqrt(2)/2 * r, y0 - sqrt(2)/2 * r);
	LCD_DrawLine(x_1, y_2, x_2, y_2);
	x0 = x_2;
		a = 0;
		b = r;
		di = 3 - (r << 1);
		while (a <= b) {
			LCD_DrawPoint(x0 + b, y0 - a);             //0
			LCD_DrawPoint(x0 + b, y0 + a);             //4
			LCD_DrawPoint(x0 + a, y0 + b);             //6
			a++;
			if (di < 0)
				di += 4 * a + 6;
			else {
				di += 10 + 4 * (a - b);
				b--;
			}
		}

}
void leftDialog(
	  	int x0, int y0,int r,int L,uint16_t c){
	POINT_COLOR = c;
  	int a, b;
  	int di;
	a = 0;
	b = r;
	di = 3 - (r << 1);
	while (a <= b) {
		LCD_DrawPoint(x0 - a, y0 + b);             //1
		LCD_DrawPoint(x0 - b, y0 + a);
		LCD_DrawPoint(x0 - b, y0 - a);             //7
		a++;
		if (di < 0)
			di += 4 * a + 6;
		else {
			di += 10 + 4 * (a - b);
			b--;
		}
	}

	int x_1 = x0;
	int y_1 = y0 - r;
	int x_2 = x0 + L;
	int y_2 = y0 + r;
	LCD_DrawLine(x_1 - r, y_1, x_2, y_1);
	LCD_DrawLine(x_1 - r, y_1,x_1 - sqrt(2)/2 * r, y0 - sqrt(2)/2 * r);
	LCD_DrawLine(x_1, y_2, x_2, y_2);
	x0 = x_2;
		a = 0;
		b = r;
		di = 3 - (r << 1);
		while (a <= b) {
					LCD_DrawPoint(x0 + a, y0 - b);             //5
					LCD_DrawPoint(x0 + b, y0 - a);             //0
					LCD_DrawPoint(x0 + b, y0 + a);             //4
					LCD_DrawPoint(x0 + a, y0 + b);             //6
			a++;
			if (di < 0)
				di += 4 * a + 6;
			else {
				di += 10 + 4 * (a - b);
				b--;
			}
		}

}

void changeState(int connected){
	LCD_Clear(WHITE);
	LCD_DrawRectangle(16, 20, 220, 300);
	if(connected == 2){
			POINT_COLOR = GREEN;
			LCD_Draw_Circle(30,10,4);
			LCD_Draw_Circle(30,10,3);
			LCD_Draw_Circle(30,10,2);
			LCD_Draw_Circle(30,10,1);
			LCD_ShowString(40, 4, 185, 10, 16, (uint8_t*) "CONNECTION");
			POINT_COLOR = BLACK;
	}
	else{
		POINT_COLOR = RED;
		LCD_Draw_Circle(30,10,4);
		LCD_Draw_Circle(30,10,3);
		LCD_Draw_Circle(30,10,2);
		LCD_Draw_Circle(30,10,1);
		LCD_ShowString(40, 4, 185, 10, 16, (uint8_t*) "CLOSED");
		POINT_COLOR = BLACK;
	}
}

//usart = 1 : From PCS
//usart = 2 : From WIFI
void updateuD(unsigned char newline[],unsigned char ul, int usart){
	int iii = 0;
	while(iii<ul){
		uRx_Data[iii] = newline[iii];
		iii++;
	}
	uLength = ul;
	addNewLine(usart);
}
//usart = 1 : From PC
//usart = 2 : From WIFI
void addNewLine(int usart){
	int p = uLength / 24;
	for(int m = 0; m <= p; m++){
		for(int k = 0; k < 25; k++){
			a[k] = b[k];
			b[k] = c[k];
			c[k] = d[k];
			d[k] = e[k];
			e[k] = f[k];
			f[k] = g[k];
			g[k] = h[k];
			h[k] = i[k];
			i[k] = uRx_Data[k + m * 24];
		}
		aa = ba;
		ba = ca;
		ca = da;
		da = ea;
		ea = fa;
		fa = ga;
		ga = ha;
		ha = ia;
		ia = 0;


		al = bl;
		bl = cl;
		cl = dl;
		dl = el;
		el = fl;
		fl = gl;
		gl = hl;
		hl = il;
		il = uLength;

	}

	if(usart == 1){
		int l = uLength % 24;
		for(int k = 0; k < 24-l; k++){
			i[k] = '\40';
		}
		for(int k = 0; k < l; k++){
			i[24-l+k] = uRx_Data[k+24*p];
		}
		i[24] = 0;
	}
	else if(usart == 2){
		ia = 1;
	}

}
void showMessage(){
	if (aa == 1){
		POINT_COLOR = RED;
		if(al!=0){
			LCD_ShowString(23, 35, 185, 10, 16, (uint8_t*) a);
			leftDialog(30,43,10,al * LENGTH,RED);
		}
	}
	else{
		POINT_COLOR = BLACK;
		if(al!=0){
			LCD_ShowString(23, 35, 185, 10, 16, (uint8_t*) a);
			rightDialog(205-(al * LENGTH),43,10,al * LENGTH,BLACK);
		}
	}

	if (ba == 1){
		POINT_COLOR = RED;

		if (bl!=0){
			LCD_ShowString(23, 65, 185, 10, 16, (uint8_t*) b);
			leftDialog(30,73,10,bl * LENGTH,RED);
		}

	}
	else{
		POINT_COLOR = BLACK;

		if (bl!=0){
			LCD_ShowString(23, 65, 185, 10, 16, (uint8_t*) b);
			rightDialog(205-(bl * LENGTH),73,10,bl * LENGTH,BLACK);
		}
	}
	if (ca == 1){
		POINT_COLOR = RED;

		if(cl!=0){
			LCD_ShowString(23, 95, 185, 10, 16, (uint8_t*) c);
			leftDialog(30,103,10,cl * LENGTH,RED);
		}
	}
	else{
		POINT_COLOR = BLACK;
		if(cl!=0){
			LCD_ShowString(23, 95, 185, 10, 16, (uint8_t*) c);
			rightDialog(205-(cl * LENGTH),103,10,cl * LENGTH,BLACK);
		}
	}

	if (da == 1){
		POINT_COLOR = RED;
		if(dl!=0){
			LCD_ShowString(23, 125, 185, 10, 16, (uint8_t*) d);
			leftDialog(30,133,10,dl * LENGTH,RED);
		}
	}
	else{
		POINT_COLOR = BLACK;
		if(dl!=0){
			LCD_ShowString(23, 125, 185, 10, 16, (uint8_t*) d);
			rightDialog(205-(dl * LENGTH),133,10,dl * LENGTH,BLACK);
		}
	}
	if (ea == 1){
		POINT_COLOR = RED;
		if(el!=0){
			LCD_ShowString(23, 155, 185, 10, 16, (uint8_t*) e);
			leftDialog(30,163,10,el * LENGTH,RED);
		}
	}
	else{
		POINT_COLOR = BLACK;

		if(el!=0){
			LCD_ShowString(23, 155, 185, 10, 16, (uint8_t*) e);
			rightDialog(205-(el * LENGTH),163,10,el * LENGTH,BLACK);
		}
	}
	if (fa == 1){
		POINT_COLOR = RED;
		if(fl!=0){
			LCD_ShowString(23, 185, 185, 10, 16, (uint8_t*) f);
			leftDialog(30,193,10,fl * LENGTH,RED);
		}
	}
	else{
		POINT_COLOR = BLACK;
		if(fl!=0){
			LCD_ShowString(23, 185, 185, 10, 16, (uint8_t*) f);
			rightDialog(205-(fl * LENGTH),193,10,fl * LENGTH,BLACK);
		}

	}
	if (ga == 1){
		POINT_COLOR = RED;
		if(gl!=0){
			LCD_ShowString(23, 215, 185, 10, 16, (uint8_t*) g);
			leftDialog(30,223,10,gl * LENGTH,RED);
		}
	}
	else{
		POINT_COLOR = BLACK;

		if(gl!=0){
			LCD_ShowString(23, 215, 185, 10, 16, (uint8_t*) g);
			rightDialog(205-(gl * LENGTH),223,10,gl * LENGTH,BLACK);
		}

	}
	if (ha == 1){
		POINT_COLOR = RED;
		if(hl!=0){
			LCD_ShowString(23, 245, 185, 10, 16, (uint8_t*) h);
			leftDialog(30,253,10,hl * LENGTH,RED);
		}
	}
	else{
		POINT_COLOR = BLACK;
		if(hl!=0){
			LCD_ShowString(23, 245, 185, 10, 16, (uint8_t*) h);
			rightDialog(205-(hl * LENGTH),253,10,hl * LENGTH,BLACK);
		}

	}
	if (ia == 1){
		POINT_COLOR = RED;
		if(il!=0){
			LCD_ShowString(23, 275, 185, 10, 16, (uint8_t*) i);
			leftDialog(30,283,10,il * LENGTH,RED);
		}
	}
	else{
		POINT_COLOR = BLACK;

		if(il!=0){
			LCD_ShowString(23, 275, 185, 10, 16, (uint8_t*) i);
			rightDialog(205-(il * LENGTH),283,10,il * LENGTH,BLACK);
		}

	}
	POINT_COLOR = BLACK;
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
    LCD_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, (uint8_t *) uart1_rx_buffer, 2048);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, (uint8_t *) uart2_rx_buffer, 2048);
    HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
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
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY0_Pin */
  GPIO_InitStruct.Pin = KEY0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
