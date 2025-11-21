/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_button.h"
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
uint8_t usart1_rx_buf[32] = {0};
uint8_t usart1_tx_buf[26] = {0};
uint8_t usart2_rx_buf[64] = {0};
uint8_t usart2_tx_buf[128] = {0};

uint8_t button_state_list[32] = {0};

uint16_t adc1_buf[20] = {0};
uint16_t adc2_buf[20] = {0};
uint16_t adc3_buf[20] = {0};
uint16_t adc5_buf[20] = {0};

uint8_t SENDEN = 0;
uint8_t SENDCNT = 0;

uint32_t cnttest = 0;

uint8_t encoder[2] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC5_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_OPAMP4_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  usart1_tx_buf[0] = 0X00;// 地址高
  usart1_tx_buf[1] = 0XAA;// 地址低
  usart1_tx_buf[2] = 0X01;// 信道
  usart1_tx_buf[3] = 0XAA;// 包头
  usart1_tx_buf[4] = 0XAA;
  usart1_tx_buf[24] = 0XAA;// 包尾
  usart1_tx_buf[25] = 0XAA;

  HAL_GPIO_WritePin(M0_GPIO_Port ,M0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(M1_GPIO_Port ,M1_Pin, GPIO_PIN_RESET);
  
  // 按钮注册
  BTN.SW0 = Button_Register(SW0_GPIO_Port, SW0_Pin);
  BTN.SW1 = Button_Register(SW1_GPIO_Port, SW1_Pin);
  BTN.SW2 = Button_Register(SW2_GPIO_Port, SW2_Pin);
  BTN.SW3 = Button_Register(SW3_GPIO_Port, SW3_Pin);
  BTN.SW4 = Button_Register(SW4_GPIO_Port, SW4_Pin);
  BTN.SW5 = Button_Register(SW5_GPIO_Port, SW5_Pin);
  BTN.SW6 = Button_Register(SW6_GPIO_Port, SW6_Pin);
  BTN.SW7 = Button_Register(SW7_GPIO_Port, SW7_Pin);
  BTN.SW8 = Button_Register(SW8_GPIO_Port, SW8_Pin);
  BTN.SW9 = Button_Register(SW9_GPIO_Port, SW9_Pin);
  BTN.SW10 = Button_Register(SW10_GPIO_Port, SW10_Pin);
  BTN.SW11 = Button_Register(SW11_GPIO_Port, SW11_Pin);
  BTN.Toggle1A = Button_Register(Toggle1A_GPIO_Port, Toggle1A_Pin);
  BTN.Toggle1B = Button_Register(Toggle1B_GPIO_Port, Toggle1B_Pin);
  BTN.Toggle2A = Button_Register(Toggle2A_GPIO_Port, Toggle2A_Pin);
  BTN.Toggle2B = Button_Register(Toggle2B_GPIO_Port, Toggle2B_Pin);
  BTN.Toggle3A = Button_Register(Toggle3A_GPIO_Port, Toggle3A_Pin);
  BTN.Toggle3B = Button_Register(Toggle3B_GPIO_Port, Toggle3B_Pin);
  BTN.Toggle4A = Button_Register(Toggle4A_GPIO_Port, Toggle4A_Pin);
  BTN.Toggle4B = Button_Register(Toggle4B_GPIO_Port, Toggle4B_Pin);
  BTN.EncoderLA = Button_Register(EncoderLA_GPIO_Port, EncoderLA_Pin);
  BTN.EncoderLB = Button_Register(EncoderLB_GPIO_Port, EncoderLB_Pin);
  BTN.EncoderLSW = Button_Register(EncoderLSW_GPIO_Port, EncoderLSW_Pin);
  BTN.EncoderRA = Button_Register(EncoderRA_GPIO_Port, EncoderRA_Pin);
  BTN.EncoderRB = Button_Register(EncoderRB_GPIO_Port, EncoderRB_Pin);
  BTN.EncoderRSW = Button_Register(EncoderRSW_GPIO_Port, EncoderRSW_Pin);
  BTN.RockerLSW = Button_Register(RockerLSW_GPIO_Port, RockerLSW_Pin);
  BTN.RockerRSW = Button_Register(RockerRSW_GPIO_Port, RockerRSW_Pin);
	Button_Init();
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buf, sizeof(usart1_rx_buf));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buf, sizeof(usart2_rx_buf));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	
	// 开启摇杆ADC校准、采样
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);
  HAL_OPAMP_Start(&hopamp4);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, 20);
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, 20);
	__HAL_DMA_DISABLE_IT(&hdma_adc2, DMA_IT_HT);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, 20);
	__HAL_DMA_DISABLE_IT(&hdma_adc3, DMA_IT_HT);
  HAL_ADC_Start_DMA(&hadc5, (uint32_t*)adc5_buf, 20);
  __HAL_DMA_DISABLE_IT(&hdma_adc5, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		cnttest++;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  if(huart->Instance == USART1){
  }
  if(huart->Instance == USART2){
    if(usart2_rx_buf[0]==0xBB && usart2_rx_buf[1]==0xBB){
      usart1_tx_buf[14] = usart2_rx_buf[2]<<4 | usart2_rx_buf[3];
      usart1_tx_buf[15] = usart2_rx_buf[4];
      usart1_tx_buf[16] = usart2_rx_buf[5];
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART1){
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buf, sizeof(usart1_rx_buf));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  }
  if(huart->Instance == USART2){
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buf, sizeof(usart2_rx_buf));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
  if(hadc->Instance == ADC1){
    uint32_t sum1 = 0;
    for(int i=0; i<20; i++){
      sum1 += adc1_buf[i];
    }
    uint16_t adc1_value = sum1 / 20;
    uint8_t mapped1 = (adc1_value <= 900) ? 255 : (adc1_value >= 3200) ? 0 : (uint8_t)(255 - (((adc1_value - 900) * 255) / (3200 - 900)));
    usart1_tx_buf[7] = mapped1;
  }
  if(hadc->Instance == ADC2){
    uint32_t sum2 = 0;
    for(int i=0; i<20; i++){
      sum2 += adc2_buf[i];
    }
    uint16_t adc2_value = sum2 / 20;
    uint8_t mapped2 = (adc2_value <= 900) ? 0 : (adc2_value >= 3200) ? 255 : (uint8_t)(((adc2_value - 900) * 255) / (3200 - 900));
    usart1_tx_buf[6] = mapped2;
  }
  if(hadc->Instance == ADC3){
    uint32_t sum3 = 0;
    for(int i=0; i<20; i++){
      sum3 += adc3_buf[i];
    }
    uint16_t adc3_value = sum3 / 20;
    uint8_t mapped3 = (adc3_value <= 900) ? 255 : (adc3_value >= 3200) ? 0 : (uint8_t)(255 - (((adc3_value - 900) * 255) / (3200 - 900)));
    usart1_tx_buf[9] = mapped3;
  }
  if(hadc->Instance == ADC5){
    uint32_t sum5 = 0;
    for(int i=0; i<20; i++){
      sum5 += adc5_buf[i];
    }
    uint16_t adc5_value = sum5 / 20;
    uint8_t mapped5 = (adc5_value <= 900) ? 0 : (adc5_value >= 3200) ? 255 : (uint8_t)(((adc5_value - 900) * 255) / (3200 - 900));
    usart1_tx_buf[8] = mapped5;
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(htim->Instance == TIM6){
		encoder[0] = (encoder[0]==0) ? Encoder_Scan(buttons[BTN.EncoderLA].port, buttons[BTN.EncoderLA].pin, buttons[BTN.EncoderLB].port, buttons[BTN.EncoderLB].pin) : encoder[0];
    encoder[1] = (encoder[1]==0) ? Encoder_Scan(buttons[BTN.EncoderRA].port, buttons[BTN.EncoderRA].pin, buttons[BTN.EncoderRB].port, buttons[BTN.EncoderRB].pin) : encoder[1];
  }
  if(htim->Instance == TIM7){
    SENDCNT ++;
    usart1_tx_buf[23] = SENDCNT;
    Button_Scan();
    for(int i=0;i<32;i++){
      button_state_list[i] = Button_GetEvent((&BTN.SW0)+i);
    }

    if(button_state_list[BTN.EncoderLSW]==1) usart1_tx_buf[5] = 0x0F;
    else if(button_state_list[BTN.EncoderRSW]==1) usart1_tx_buf[5] = 0xF0;
    else usart1_tx_buf[5] = 0x00;
    
    usart1_tx_buf[10] = (button_state_list[0]<<7) | (button_state_list[1]<<6) | (button_state_list[2]<<5) | (button_state_list[3]<<4)
                      | (button_state_list[4]<<3) | (button_state_list[5]<<2) | (button_state_list[6]<<1) | (button_state_list[7]);
    usart1_tx_buf[11] = (button_state_list[8]<<7) | (button_state_list[9]<<6) | (button_state_list[10]<<5) | (button_state_list[11]<<4)
                      | (button_state_list[12]<<3) | (button_state_list[13]<<2) | (button_state_list[14]<<1) | (button_state_list[15]);
		usart1_tx_buf[12] = (button_state_list[16]<<7) | (button_state_list[17]<<6) | (button_state_list[18]<<5) | (button_state_list[19]<<4)
                      | (button_state_list[BTN.EncoderLSW]<<3) | (button_state_list[BTN.EncoderRSW]<<2) | (button_state_list[BTN.RockerLSW]<<1) | (button_state_list[BTN.RockerRSW]);
    usart1_tx_buf[13] = (encoder[0]<<6) | (encoder[1]<<4);
    
    if(SENDEN >= 1){
			HAL_UART_Transmit_DMA(&huart1, usart1_tx_buf, sizeof(usart1_tx_buf));
			SENDEN = 0;
      encoder[0] = 0;
      encoder[1] = 0;
		}
    else if(HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == GPIO_PIN_SET){
			SENDEN ++;
    }
		else SENDEN = 0;

    int send_len2 = sprintf(usart2_tx_buf, "r2_chassis.lx.val=%d\xFF\xFF\xFFr2_chassis.ly.val=%d\xFF\xFF\xFFr2_chassis.rx.val=%d\xFF\xFF\xFFr2_chassis.ry.val=%d\xFF\xFF\xFF",
      usart1_tx_buf[6], usart1_tx_buf[7],
      usart1_tx_buf[8], usart1_tx_buf[9]);
    HAL_UART_Transmit_DMA(&huart2, usart2_tx_buf, send_len2);

  }
  /* USER CODE END Callback 1 */
}

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
