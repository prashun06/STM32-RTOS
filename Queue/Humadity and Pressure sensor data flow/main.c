
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
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
UART_HandleTypeDef huart1;

osThreadId HumidityTaskHandle;
osThreadId receiverTaskHandle;
osThreadId PressureTaskHandle;
osMessageQId dataQueueHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void humTaskHandler(void const * argument);
void receiverTaskHandler(void const * argument);
void presTaskHandler(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum 
{
	humidity_sensor,
	pressure_sensor
} dataSource_t;   //two element

//define two structure type to be passed to the queue
typedef struct
{
	char ucValue;
	dataSource_t DataSrc;
} Data_t;    //

//declare two "Data_t" variable that will be passed to the queue
static const Data_t xStructToSend[2] =
{
	{'H', humidity_sensor},  //used by humidity s
	{'P', pressure_sensor}  //used by pressure s
};


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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of dataQueue */
  osMessageQDef(dataQueue, 3, int32_t);
  dataQueueHandle = osMessageCreate(osMessageQ(dataQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of HumidityTask */
  osThreadDef(HumidityTask, humTaskHandler, osPriorityNormal, 0, 128);
  HumidityTaskHandle = osThreadCreate(osThread(HumidityTask), (void*) &(xStructToSend[0]));

  /* definition and creation of receiverTask */
  osThreadDef(receiverTask, receiverTaskHandler, osPriorityAboveNormal, 0, 128);
  receiverTaskHandle = osThreadCreate(osThread(receiverTask), NULL);

  /* definition and creation of PressureTask */
  osThreadDef(PressureTask, presTaskHandler, osPriorityNormal, 0, 128);
  PressureTaskHandle = osThreadCreate(osThread(PressureTask), (void*) &(xStructToSend[1]));

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void send_string(char* string)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string), 1000);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_humTaskHandler */
/**
  * @brief  Function implementing the HumidityTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_humTaskHandler */
void humTaskHandler(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	BaseType_t qstatus;
	Data_t data;
  for(;;)
  {
		qstatus = xQueueSend(dataQueueHandle, argument, 0);
		if(qstatus != pdPASS)   //pdPASS =TRUE in rtos
		{
			send_string("Error : data not send! \r\n");
		}
    osDelay(110);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_receiverTaskHandler */
/**
* @brief Function implementing the receiverTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_receiverTaskHandler */
void receiverTaskHandler(void const * argument)
{
  /* USER CODE BEGIN receiverTaskHandler */
  /* Infinite loop */
	BaseType_t qstatus;
	Data_t xReceiveStructure;
	const TickType_t wait_time = pdMS_TO_TICKS(100); 
  for(;;)
  {
		qstatus = xQueueReceive(dataQueueHandle, &xReceiveStructure, wait_time);
		if(qstatus != pdPASS)
		{
			if(xReceiveStructure.DataSrc == humidity_sensor){
				send_string("Humidity sensor value:  ");
			  send_string(&xReceiveStructure.ucValue);
				send_string("...\r\n");
			}
			if(xReceiveStructure.DataSrc == pressure_sensor){
				send_string("Pressure sensor value:  ");
			  send_string(&xReceiveStructure.ucValue);
				send_string("...\r\n");
			}
		}
		else
		{
			//send_string("data not received \r\n");
		}		
    osDelay(1);
  }
  /* USER CODE END receiverTaskHandler */
}

/* USER CODE BEGIN Header_presTaskHandler */
/**
* @brief Function implementing the PressureTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_presTaskHandler */
void presTaskHandler(void const * argument)
{
  /* USER CODE BEGIN presTaskHandler */
  /* Infinite loop */
  BaseType_t qstatus;
  for(;;)
  {
		qstatus = xQueueSend(dataQueueHandle, argument, 0);
		if(qstatus != pdPASS)   //pdPASS =TRUE in rtos
		{
			send_string("Error : data not send! \r\n");
		}
    osDelay(110);
  }
  /* USER CODE END presTaskHandler */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
