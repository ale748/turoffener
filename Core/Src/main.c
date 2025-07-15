/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void USER_GPIO_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t timecount = 0;
uint32_t timeoutCount=0;
uint8_t swdtime=0;


//used with delay function (1ms)
#define OPEN_TIME 800 //800 ms


// used with timer (100us)
#define RING_TIME 40000 //4000 ms
#define RING_TIMEOUT 1000 //100 ms
#define WAIT_TIMEOUT 600000 //60000 ms
#define SWD_TIME 20000 //2000 ms
#define MIN_PERIOD 5 //0.5 mS
#define MAX_PERIOD 100 //100ms

#define MIN_COUNT 10


uint32_t last_ring_change_time = 0;
uint32_t timecount;

uint8_t state_machine = 0;
uint8_t updatevar = 0;


struct button_t ringDet;
struct signal_t signalPin;
uint8_t lastState = 100;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1);
  timecount = 0;
  swdtime = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (swdtime == 0){
		  if (timecount>SWD_TIME){
			  USER_GPIO_Init();

			  HAL_GPIO_WritePin(PICK_UP_OUT_GPIO_Port, PICK_UP_OUT_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(OPEN_OUT_GPIO_Port, OPEN_OUT_Pin, GPIO_PIN_SET);
			  ringDet.state = HAL_GPIO_ReadPin(RING_DET_GPIO_Port, RING_DET_Pin)==GPIO_PIN_SET?1:0;
			  swdtime = 1;
			  signalPin.state=0;
			  signalPin.last_time_change = 0;

		  }
	  }else{

		  switch (state_machine){
			case 0: {
                uint8_t pinstate = HAL_GPIO_ReadPin(PICK_UP_DET_GPIO_Port, PICK_UP_DET_Pin) == 1 ? 1 : 0;
                if (pinstate && !ringDet.state){
                    ringDet.first_time_change = timecount;
                    ringDet.state = 1;
                }else if(!pinstate && ringDet.state){
                    //if the ring detect falls start counter to time out
                    ringDet.last_time_change = timecount;

                }
            }
			if (ringDet.state){
			  if (timecount-ringDet.first_time_change > RING_TIME){
				state_machine = 1;
				ringDet.state=0;

				//pressing button for OPEN_TIME
				HAL_GPIO_WritePin(OPEN_OUT_GPIO_Port, OPEN_OUT_Pin, GPIO_PIN_RESET);
				HAL_Delay(OPEN_TIME);
				HAL_GPIO_WritePin(OPEN_OUT_GPIO_Port, OPEN_OUT_Pin, GPIO_PIN_SET);

				//starting the timer for reactivation
				timeoutCount=timecount;


			  }
			  break;
			case 1:
				//if time for reactivation has passed reset state_machine
				if (timecount-timeoutCount>WAIT_TIMEOUT){
					state_machine=0;
				}
			  default:
			  break;
			}
		  }

	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PICK_UP_OUT_GPIO_Port, PICK_UP_OUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SIGNAL_READ_Pin PICK_UP_DET_Pin */
  GPIO_InitStruct.Pin = SIGNAL_READ_Pin|PICK_UP_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PICK_UP_OUT_Pin */
  GPIO_InitStruct.Pin = PICK_UP_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PICK_UP_OUT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ring_det_isr(){
	uint8_t pinstate = HAL_GPIO_ReadPin(RING_DET_GPIO_Port, RING_DET_Pin)==1? 1:0;
	  switch (state_machine){
	    case 0:
	    	//if the ring detect is high and it was not high before
	    if (pinstate && !ringDet.state){
	      ringDet.first_time_change = timecount;
	      ringDet.state = 1;
	    }else if(!pinstate && ringDet.state){
	    	//if the ring detect falls start counter to time out
	      ringDet.last_time_change = timecount;

	    }
	    break;
	    default:
	    break;
	  }
}

void pick_up_det_isr(){
// do nothing dont care
}

void signal_read_isr(){
	//getting the period of the signal, if the signal is between the ringing signal
	//and the number of good periods is higher the count, set the state to 1
	signalPin.period = timecount - signalPin.last_time_change;
	signalPin.last_time_change = timecount;
	if (signalPin.period>MIN_PERIOD && signalPin.period<MAX_PERIOD){
		signalPin.count++;
		if (signalPin.count > MIN_COUNT){
			signalPin.state=1;
		}
	}

}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t pin){
	if (swdtime){
		switch(pin){
			case RING_DET_Pin:
				//ring_det_isr();
				break;
			case PICK_UP_DET_Pin:
				//pick_up_det_isr();
				break;
			case SIGNAL_READ_Pin:
				//signal_read_isr();
			default:
				break;

		}
	}
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin){
	if (swdtime){
		switch(GPIO_Pin){

			case RING_DET_Pin:
				//ring_det_isr();
				break;
			case PICK_UP_DET_Pin:
				//pick_up_det_isr();
				break;
			case SIGNAL_READ_Pin:
				//signal_read_isr();
			default:
				break;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim1){
		timecount++;
		if (swdtime){
			uint8_t ringPinState = HAL_GPIO_ReadPin(RING_DET_GPIO_Port, RING_DET_Pin)? 1:0;

			  if (state_machine == 0){
			    if (ringDet.state == 1){
			      if (!ringPinState && timecount - ringDet.last_time_change> RING_TIMEOUT){
			    	  //if the pin is low and has a timeout then restart its state
			        ringDet.state = 0;

			      }
			    }
			  }

			  if ((timecount - signalPin.last_time_change>RING_TIMEOUT)){ //timeout signal ringing
				  // if the last time signal change has timeout, reset signal pin
				  //and if the ringDet state was 1 then needs to be cleared
				  signalPin.state=0;
				  signalPin.count=0;
				  if (ringDet.state == 1){
					 // ringDet.state = 0;
				  }
			  }
		}
	}
}

static void USER_GPIO_Init(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, SIGNAL_WRITE_Pin|OPEN_OUT_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(PICK_UP_OUT_GPIO_Port, PICK_UP_OUT_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pin : SIGNAL_READ_Pin */
	  GPIO_InitStruct.Pin = SIGNAL_READ_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(SIGNAL_READ_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : SIGNAL_WIRTE_Pin OPEN_OUT_Pin */
	  GPIO_InitStruct.Pin = OPEN_OUT_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PICK_UP_DET_Pin */
	  GPIO_InitStruct.Pin = SIGNAL_READ_Pin|PICK_UP_DET_Pin;
 	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : PICK_UP_OUT_Pin */
	  GPIO_InitStruct.Pin = PICK_UP_OUT_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(PICK_UP_OUT_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : RING_DET_Pin */
	  GPIO_InitStruct.Pin = RING_DET_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(RING_DET_GPIO_Port, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

	  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

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
