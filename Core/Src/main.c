/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include"stdio.h"
#include"string.h"
#include"math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define WIFI_NAME "zjxnb"
#define WIFI_PWD "12345678"
#define SERVER_ADDRESS "10.17.29.111"
#define SERVER_PORT "8081"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char rxBuffer[20];// 中间缓存区要设置大一?

uint8_t aRxBuffer[2000];//接收缓冲
uint8_t bRxBuffer[2000];//发送缓冲
uint16_t USART2_RX_STA= 0;
uint16_t pt_w1=1,pt_w2=0,pt_r1=1,pt_r2=0;


char lcdInput[9][12];
int lcdInputLen[9];
//定义部分代码
char ap_mode[] = "AT+CWMODE=2\r\n";
char reset[] = "AT+RST\r\n";
char ap_args[] = "AT+CWSAP=\"ZMFL\",\"12345678\",1,4\r\n";
char ap_multi_connection[] = "AT+CIPMUX=1\r\n";
char ap_cip_server[] = "AT+CIPSERVER=1,8081\r\n";
char ap_mode[] = "AT+CIPSEND=0,25\r\n";

char Test[]="AT\r\n";
char JoinWifi[]="AT+CWJAP=\"gjq\",\"12345678\"\r\n";
char TestJoin[]="AT+CWJAP?\r\n";
char SimpleConnect[]="AT+CIPMUX=0\r\n";
char JoinServer[]="AT+CIPSTART=\"TCP\",\"192.168.43.13\",3456\r\n";
char Mode[]="AT+CIPMODE=1\r\n";
char ModeSend[]="AT+CIPSEND=\r\n";



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t send_AT_command(uint8_t* at_command, uint8_t* ack, uint16_t timeout);
uint8_t check_command(uint8_t* desired_result);
uint8_t check_connection(void);
void get_WAN_IP(uint8_t * ip_buffer);
uint8_t send_data(uint8_t* data, uint8_t* ack, uint16_t wait_time);
uint8_t atk_8266_quit_trans(void);
uint8_t atk_8266_get_ip(uint8_t x,uint8_t y);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int tmpOpen = 0;
char temperature[1024] = "temperature\0";
//char yes[1024] = "yes";
char uRx_Data[1024];
int uLength = 0;

void addOneLine(char *data, int len){
	for(int i=8;i>=1;--i){
		for(int j=0;j<lcdInputLen[i-1];++j)	lcdInput[i][j] = lcdInput[i-1][j];
		lcdInput[i][lcdInputLen[i-1]] = '\0';
		lcdInputLen[i] = lcdInputLen[i-1];
	}
	for(int i=0;i<len;++i)	lcdInput[0][i] = data[i];
	lcdInput[0][len] = '\0';
	lcdInputLen[0] = len;
}


// to accept serial port data from computer (huart1) and wifi data (huart2)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        HAL_UART_Receive_IT(&huart1, &my_re_buf1[++pt_w1], 1);
    }

    if (huart == &huart2) {
        HAL_UART_Receive_IT(&huart2, &my_re_buf2[++pt_w2], 1);
    }
}
/*
//	if(huart->Instance==USART1){
//		static unsigned char uRx_Data[1024] = {0};
//		static unsigned char uLength = 0;
//		static unsigned char a[25] = {0};
//		static unsigned char b[25] = {0};
//		static unsigned char c[25] = {0};
//		static unsigned char d[25] = {0};
//		static unsigned char e[25] = {0};
//		static unsigned char f[25] = {0};
//		static unsigned char g[25] = {0};
//		static unsigned char h[25] = {0};
//		static unsigned char i[25] = {0};
//
//		static unsigned int aa = 0;
//		static unsigned int ba = 0;
//		static unsigned int ca = 0;
//		static unsigned int da = 0;
//		static unsigned int ea = 0;
//		static unsigned int fa = 0;
//		static unsigned int ga = 0;
//		static unsigned int ha = 0;
//		static unsigned int ia = 0;
//
//		if(rxBuffer[0] == '\n'){
//			HAL_UART_Transmit(&huart1, uRx_Data, uLength, 0xffff);
//			HAL_UART_Transmit(&huart2,uRx_Data, uLength,0xffff);
//
//			int p = uLength / 24;
//
//			for(int m = 0; m <= p; m++){
//				for(int k = 0; k < 25; k++){
//					a[k] = b[k];
//					b[k] = c[k];
//					c[k] = d[k];
//					d[k] = e[k];
//					e[k] = f[k];
//					f[k] = g[k];
//					g[k] = h[k];
//					h[k] = i[k];
//					i[k] = uRx_Data[k + m * 24];
//				}
//				aa = ba;
//				ba = ca;
//				ca = da;
//				da = ea;
//				ea = fa;
//				fa = ga;
//				ga = ha;
//				ha = ia;
//				ia = 0;
//			}
//
//			int l = uLength % 24;
//			for(int k = 0; k < 24-l; k++){
//				i[k] = '\40';
//			}
//			for(int k = 0; k < l; k++){
//				i[24-l+k] = uRx_Data[k+24*p];
//			}
//			i[24] = 0;
//
//			if(uLength == 12 && uRx_Data[0] == 't' && uRx_Data[1] == 'e' && uRx_Data[2] == 'm' && uRx_Data[3] == 'p' && uRx_Data[4] == 'e'
//					&& uRx_Data[5] == 'r' && uRx_Data[6] == 'a' && uRx_Data[7] == 't' && uRx_Data[8] == 'u' && uRx_Data[9] == 'r' && uRx_Data[10] == 'e' && uRx_Data[11] == '\r'){
//				HAL_UART_Transmit(&huart1, uRx_Data, uLength, 0xffff);
//
//				HAL_ADC_Start(&hadc1);
//				// Wait for regular group conversion to be completed
//				HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//				// Get ADC value
//				uint16_t raw = HAL_ADC_GetValue(&hadc1); // the voltage should be raw * (3.3/4096)(12bits)
//				float voltage = (1.43 - (raw * (3.3/4096))) / 4.3 + 25;
//				// Convert to string and print
//				char msg[20];
////				sprintf(msg, "%.6f\r\n", voltage);
//				HAL_UART_Transmit(&huart1, msg, 9, 0xffff);
//
//				for(int k = 0; k < 25; k++){
//					a[k] = b[k];
//					b[k] = c[k];
//					c[k] = d[k];
//					d[k] = e[k];
//					e[k] = f[k];
//					f[k] = g[k];
//					g[k] = h[k];
//					h[k] = i[k];
//				}
//				aa = ba;
//				ba = ca;
//				ca = da;
//				da = ea;
//				ea = fa;
//				fa = ga;
//				ga = ha;
//				ha = ia;
//				for(int k = 0; k < 9; k++){
//					i[k] = msg[k];
//				}
//				ia = 1;
//				i[9] = 0;
//			}
//
//			LCD_Clear(WHITE);
//			BACK_COLOR = BLUE;
//			LCD_DrawRectangle(20, 20, 220, 300);
//			LCD_Fill(21, 21, 219, 299, BLUE);
//			if (aa == 1){
//				POINT_COLOR = RED;
//			}
//			else{
//				POINT_COLOR = BLACK;
//			}
//			LCD_ShowString(23, 35, 185, 10, 16, (uint8_t*) a);
//			if (ba == 1){
//				POINT_COLOR = RED;
//			}
//			else{
//				POINT_COLOR = BLACK;
//			}
//			LCD_ShowString(23, 65, 185, 10, 16, (uint8_t*) b);
//			if (ca == 1){
//							POINT_COLOR = RED;
//						}
//						else{
//							POINT_COLOR = BLACK;
//						}
//			LCD_ShowString(23, 95, 185, 10, 16, (uint8_t*) c);
//			if (da == 1){
//							POINT_COLOR = RED;
//						}
//						else{
//							POINT_COLOR = BLACK;
//						}
//			LCD_ShowString(23, 125, 185, 10, 16, (uint8_t*) d);
//			if (ea == 1){
//							POINT_COLOR = RED;
//						}
//						else{
//							POINT_COLOR = BLACK;
//						}
//			LCD_ShowString(23, 155, 185, 10, 16, (uint8_t*) e);
//			if (fa == 1){
//							POINT_COLOR = RED;
//						}
//						else{
//							POINT_COLOR = BLACK;
//						}
//			LCD_ShowString(23, 185, 185, 10, 16, (uint8_t*) f);
//			if (ga == 1){
//							POINT_COLOR = RED;
//						}
//						else{
//							POINT_COLOR = BLACK;
//						}
//			LCD_ShowString(23, 215, 185, 10, 16, (uint8_t*) g);
//			if (ha == 1){
//							POINT_COLOR = RED;
//						}
//						else{
//							if (ia == 1){
//								POINT_COLOR = YELLOW;
//							}
//							else{
//								POINT_COLOR = BLACK;
//							}
//						}
//			LCD_ShowString(23, 245, 185, 10, 16, (uint8_t*) h);
//			if (ia == 1){
//							POINT_COLOR = RED;
//						}
//						else{
//							POINT_COLOR = YELLOW;
//						}
//			LCD_ShowString(23, 275, 185, 10, 16, (uint8_t*) i);
//			POINT_COLOR = BLACK;
//
//			uLength = 0;
//		}
//		else{
//			uRx_Data[uLength] = rxBuffer[0];
////			HAL_UART_Transmit(&huart1, rxBuffer[0], 1, 0xffff);
//			uLength++;
//		}
//	}*/

/**
 * <p> �? ESP8266发�?�命�?
 * @param at_command 发�?�的命令字符�?
 * @param ack 期待的应答结果，如果为空，则不需要等待应�?
 * @param timeout 等待时间（单�?: 10ms�?
 * @return 0-> 发�?�成�?; 1->发�?�失�?
 */
uint8_t send_AT_command(uint8_t* at_command, uint8_t* ack, uint16_t timeout){
    uint8_t response = 0;
    USART2_RX_STA = 0;

    if (ack && timeout) {
        while (--timeout) {
            HAL_Delay(10);
//            delay_ms(10);
            if (USART2_RX_STA & 0x8000) {
                if (check_command(ack)) {
                    break;
                }
                USART2_RX_STA = 0;
            }
        }
        if (timeout == 0) {
            response = 1;
        }
    }
    return response;
}
/**
 * to check the answer from the wifi
 */

uint8_t check_command(uint8_t* desired_result){
    char* re_ptr = 0;
    if (USART2_RX_STA & 0x8000) {
        aRxBuffer[USART2_RX_STA & 0x7fff] = 0;
        re_ptr = strstr((const char* )aRxBuffer,(const char*) desired_result);
    }
    return (uint8_t *) re_ptr;
}
//to check the wifi connection
uint8_t check_connection(void)
{
   uint8_t *p;
   uint8_t response;
   if (atk_8266_quit_trans()) {return 0;}
   send_AT_command("AT+CIPSTATUS",":",50);
   p = check_command("+CIPSTATUS:");
   response = *p;
   return response;
}
/**
 * 获取模块STA模式或者AP模式下的IP地址以及MAC地址
 * @param ip_buffer 地址输出缓存区
 */
void get_WAN_IP(uint8_t * ip_buffer){
    uint8_t *p,*p1;
    if (send_AT_command("AT+CIFSR","OK",50)){
        ip_buffer[0] = 0;
        return;
    }
    p = check_command("\"");
    p1 = (uint8_t *) strstr((const char *)(p+1),"\"");
    *p1 = 0;
    sprintf((char*)ip_buffer,"%s",p+1);
}

uint8_t atk_8266_get_ip(uint8_t x,uint8_t y){
    uint8_t *p;
    uint8_t *p1;
    uint8_t *p2;
    uint8_t *ip_buffer;
    uint8_t *buf;
    return 0;
}






/**
 *
 * @param data
 * @param ack
 * @param wait_time
 * @return
 */
uint8_t send_data(uint8_t* data, uint8_t* ack, uint16_t wait_time){
    uint8_t response;
    USART2_RX_STA=0;
    u2_printf("%s",data);	//发送命令
    if(ack&&waittime)		//需要等待应答
    {
        while(--waittime)	//等待倒计时
        {
            delay_ms(10);
            if(USART2_RX_STA&0X8000)//接收到期待的应答结果
            {
                if(atk_8266_check_cmd(ack))break;//得到有效数据
                USART2_RX_STA=0;
            }
        }
        if(waittime==0)res=1;
    }
    return res;
}

uint8_t atk_8266_quit_trans(void)
{
    while((USART2->SR&0X40)==0);	//等待发�?�空
    USART2->DR='+';
    HAL_Delay(15);
//    delay_ms(15);					//大于串口组帧时间(10ms)
    while((USART2->SR&0X40)==0);	//等待发�?�空
    USART2->DR='+';
    HAL_Delay(15);
//    delay_ms(15);					//大于串口组帧时间(10ms)
    while((USART2->SR&0X40)==0);	//等待发�?�空
    USART2->DR='+';
    HAL_Delay(500);
//    delay_ms(500);					//等待500ms
    return atk_8266_send_cmd("AT","OK",20);//�?出�?�传判断.
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuffer, 1);
//  HAL_UART_Receive_IT(&huart2,(uint8_t *)rxBuffer,1);

  LCD_Init();
  for(int i=0;i<=8;++i)	lcdInput[i][0] = '\0', lcdInputLen[i] = 0;

	LCD_Clear(WHITE);
	BACK_COLOR = BLUE;
	LCD_DrawRectangle(20, 20, 220, 300);
	LCD_Fill(21, 21, 219, 299, BLUE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
//	  LCD_Clear(WHITE);
//	  BACK_COLOR = GREEN;
//	  LCD_DrawRectangle(20, 20, 220, 290);	//x1,y1,x2,y2
//	  LCD_Fill(21, 21, 219, 289, GREEN);	//+1,+1,-1,-1
//
//	  POINT_COLOR = BLACK;
//	  for(int i=0;i<=8;++i)	LCD_ShowString(210-lcdInputLen[i]*12-2, 280-24*(i+1), 200, 24, 24, (uint8_t*) lcdInput[i]);
//
//	  HAL_ADC_Start(&hadc1);
//	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	  raw = HAL_ADC_GetValue(&hadc1);
//	  ans = raw * (3.3/4096);
//	  ans = (1.43 - ans)/4.3 + 25;
//	  sprintf(msg, "%.2f", ans);
//
//	  POINT_COLOR = RED;
//	  if(tmpOpen == 1)	LCD_ShowString(32, 32, 200, 24, 24, (uint8_t*) msg);	//x,y,w,h,size,mes
//
//	  HAL_Delay(2000);
//
//
//  }*/
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
