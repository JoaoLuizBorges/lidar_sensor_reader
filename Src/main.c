/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lidar.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_SIZE 		512
#define SAFE_DISTANCE  	200.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for LidarSensorTask */
osThreadId_t LidarSensorTaskHandle;
const osThreadAttr_t LidarSensorTask_attributes = {
  .name = "LidarSensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ObstacleTask */
osThreadId_t ObstacleTaskHandle;
const osThreadAttr_t ObstacleTask_attributes = {
  .name = "ObstacleTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */
uint8_t UART2_rxBuffer[RX_SIZE];
uint8_t UART3_rxBuffer[RX_SIZE];
uint8_t g_UART3_bufferCopy[RX_SIZE];
osMessageQueueId_t g_LidarSizeQueueHandle;
volatile uint16_t g_uart2_rx_size;
volatile uint16_t g_uart3_rx_size;

packet_parser_t lidar_parser;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void LidarSensor(void *argument);
void Obstacle(void *argument);

/* USER CODE BEGIN PFP */
int __io_putchar(int ch);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  g_LidarSizeQueueHandle = osMessageQueueNew(10, sizeof(uint16_t), NULL);

  /*HAL_UARTEx_ReceiveToIdle_DMA(&huart2,
          							 UART2_rxBuffer,
  									 RX_SIZE);

  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);*/


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LidarSensorTask */
  LidarSensorTaskHandle = osThreadNew(LidarSensor, NULL, &LidarSensorTask_attributes);

  /* creation of ObstacleTask */
  ObstacleTaskHandle = osThreadNew(Obstacle, NULL, &ObstacleTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xFFFF);
	return ch;
}*/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                uint16_t Size) {

    /*
     *
     * BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    HAL_UART_RxEventTypeTypeDef event;

    event = HAL_UARTEx_GetRxEventType(huart);

    printf("Event = %d, Size = %u\r\n", event, Size);*/

    if (huart->Instance == USART2) {

//    	g_uart3_rx_size = Size;

    	osMessageQueuePut(g_LidarSizeQueueHandle, &Size, 0U, 0U);
    	/*
        vTaskNotifyGiveFromISR(LidarSensorTaskHandle,
                               &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        */
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART2) {
        volatile uint32_t tmpreg;

#if defined(USART_SR_ORE)
        tmpreg = huart->Instance->SR;
        tmpreg = huart->Instance->DR;
#elif defined(USART_ISR_ORE)
        tmpreg = huart->Instance->ISR;
        tmpreg = huart->Instance->RDR;
#endif

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);

        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_rxBuffer, RX_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

void LidarSensorTask(void *argument)
{
    for (;;)
    {

    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_LidarSensor */
/**
* @brief Function implementing the LidarSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LidarSensor */
void LidarSensor(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint16_t old_pos = 0;
  uint16_t current_pos = 0;

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_rxBuffer, RX_SIZE);
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  /* Infinite loop */
  for(;;) {

      if (osMessageQueueGet(g_LidarSizeQueueHandle, &current_pos, NULL, osWaitForever) == osOK) {

    	  if (current_pos > old_pos) {

			  packet_parse_frame(&lidar_parser,
								  &UART2_rxBuffer[old_pos],
								  current_pos - old_pos);

			  if(lidar_parser.frame_ready) {

				  lidar_process_frame(&lidar_parser.frame);
				  //lidar_print_frame(&lidar_parser.frame);
				  lidar_update_map(&lidar_parser.frame);
				  packet_parser_reset(&lidar_parser);

			  }

    	  } else if (current_pos < old_pos) {

    		  packet_parse_frame(&lidar_parser,
			  					  &UART2_rxBuffer[old_pos],
			  					  RX_SIZE - old_pos);

    		  if(lidar_parser.frame_ready) {

    			  lidar_process_frame(&lidar_parser.frame);
//    			  lidar_print_frame(&lidar_parser.frame);
    			  lidar_update_map(&lidar_parser.frame);
				  packet_parser_reset(&lidar_parser);

    		  }

			  packet_parse_frame(&lidar_parser,
			  			  		  &UART2_rxBuffer[0],
								  current_pos);

			  if(lidar_parser.frame_ready) {

				  lidar_process_frame(&lidar_parser.frame);
//				  lidar_print_frame(&lidar_parser.frame);
				  lidar_update_map(&lidar_parser.frame);
				  packet_parser_reset(&lidar_parser);

			  }
          }
    	  old_pos = current_pos;

      } else {

    	  __HAL_UART_CLEAR_OREFLAG(&huart2);
    	  __HAL_UART_CLEAR_NEFLAG(&huart2);
    	  __HAL_UART_CLEAR_FEFLAG(&huart2);

    	  HAL_UART_DMAStop(&huart2);
    	  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_rxBuffer, RX_SIZE);
    	  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
      }
      /*
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	  for (uint16_t i = 0; i < g_uart3_rx_size; i++) {
		printf("%02X ", UART3_rxBuffer[i]);
	  }
	  printf("\r\n");*/
  }
  /*
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,
  								 UART3_rxBuffer,
  								 RX_SIZE);

  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);*/

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Obstacle */
/**
* @brief Function implementing the ObstacleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Obstacle */
void Obstacle(void *argument)
{
  /* USER CODE BEGIN Obstacle */
  /* Infinite loop */
	const polar_map_t *map;

	for(;;) {

		map = lidar_get_map();

		/*for(int i = 0; i < 360; i++)
		{
		    if(map->valid[i])
		    {
		        printf("%3d -> %.1f mm\r\n", i, map->distance[i]);
		    }
		}*/

		bool obstacle = false;

		for(int angle = 330; angle < 360; angle++) {

			if(map->valid[angle] &&
			   map->distance[angle] < SAFE_DISTANCE)
			{
				obstacle = true;
				break;
			}
		}

		if(!obstacle) {

			for(int angle = 0; angle <= 30; angle++) {

				if(map->valid[angle] &&
				   map->distance[angle] < SAFE_DISTANCE) {

					obstacle = true;
					break;
				}
			}
		}

		if(obstacle)
		{

		    HAL_GPIO_WritePin(LD2_GPIO_Port,
		                      LD2_Pin,
		                      GPIO_PIN_SET);
		}
		else
		{
		    HAL_GPIO_WritePin(LD2_GPIO_Port,
		                      LD2_Pin,
		                      GPIO_PIN_RESET);
		}

		vTaskDelay(pdMS_TO_TICKS(20));
	}

  /* USER CODE END Obstacle */
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
#ifdef USE_FULL_ASSERT
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
