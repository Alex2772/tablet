/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
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
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
    GPIO_TypeDef *gpiox;
    uint32_t pin;
    char works;
} pin_def_t;

pin_def_t pin_definitions[] = {
        {GPIOB, 12 }, // 0
        {GPIOB, 13 }, // 1
        {GPIOB, 14 }, // 2
        {GPIOB, 15 }, // 3
        {GPIOA, 8 },  // 4
        {GPIOA, 9 },  // 5
        {GPIOA, 10 }, // 6
        {GPIOA, 11 }, // 7
        {GPIOA, 12 }, // 8
        {GPIOB, 4 }, // 9
        {GPIOB, 5 }, // 10
        {GPIOB, 8 }, // 11
        {GPIOB, 9 }, // 12
        {GPIOB, 11 }, // 13
        {GPIOB, 10 }, // 14
        {GPIOB, 1 }, // 15
        {GPIOB, 0 }, // 16
        {GPIOA, 7 }, // 17
        {GPIOA, 6 }, // 18
        {GPIOA, 5 }, // 19
        {GPIOA, 4 }, // 20
        {GPIOA, 3 }, // 21
        {GPIOA, 2 }, // 22
        {GPIOA, 1 }, // 23
        {GPIOA, 0 }, // 24
        {0}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  GPIO_InitTypeDef input = {
          .Mode = GPIO_MODE_INPUT,
          .Pull = GPIO_PULLDOWN,
          .Speed = GPIO_SPEED_FREQ_LOW
  };
  GPIO_InitTypeDef output = {
          .Mode = GPIO_MODE_OUTPUT_PP,
          .Pull = GPIO_NOPULL,
          .Speed = GPIO_SPEED_FREQ_LOW
  };

  for (int i = 0; pin_definitions[i].gpiox; ++i) {
    input.Pin = 1u << pin_definitions[i].pin;
    pin_definitions[i].works = 0;
    HAL_GPIO_Init(pin_definitions[i].gpiox, &input);
  }

  for (int i = 0; i < 4; ++i) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(200);
  }

  unsigned keycode = 0;
  uint8_t keystates[470 / 8];
  memset(keystates, 0, sizeof(keystates));

  struct {
    uint8_t high : 1;
    uint8_t is_pressed : 1;
    uint16_t keycode : 14;
  } packet;
  packet.high = 1;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
    for (int i = 0; i < FIRST_DATA_INDEX; ++i) {
        uint16_t pin = 1u << pin_definitions[i].pin;
        HAL_GPIO_WritePin(pin_definitions[i].gpiox, pin, GPIO_PIN_SET);
        for (int j = FIRST_DATA_INDEX; pin_definitions[j].gpiox; ++j) {
            if (HAL_GPIO_ReadPin(pin_definitions[j].gpiox, 1 << pin_definitions[j].pin)) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
                HAL_Delay(200);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
                HAL_Delay(200);
            }
        }
        HAL_GPIO_WritePin(pin_definitions[i].gpiox, pin, GPIO_PIN_RESET);
    }*/
    keycode = 0;
    int f = 1;
    for (int i = 0; pin_definitions[i].gpiox; ++i) {
      output.Pin = 1u << pin_definitions[i].pin;
      input.Pin = output.Pin;
      HAL_GPIO_Init(pin_definitions[i].gpiox, &output);
      HAL_GPIO_WritePin(pin_definitions[i].gpiox, output.Pin, GPIO_PIN_SET);

      for (int j = i + 1; pin_definitions[j].gpiox; ++j) {
        int actualState = HAL_GPIO_ReadPin(pin_definitions[j].gpiox, 1u << pin_definitions[j].pin);
         int previousState = (keystates[keycode / 8] >> (keycode % 8)) & 1;
        if (actualState) {
            f = 0;
        }
        if (actualState != previousState) {
            if (i == 2) {
                __NOP();
            }
            pin_definitions[i].works = 1;
            packet.is_pressed = actualState;
            packet.keycode = keycode;


            // wait until previous transfer end
            if (huart1.gState != HAL_UART_STATE_READY) {
                continue;
            }
            if (actualState) {
                keystates[keycode / 8] |= 1u << (keycode % 8);
            } else {
                keystates[keycode / 8] &= ~(1u << (keycode % 8));
            }
            while (HAL_UART_Transmit_DMA(&huart1, &packet, sizeof(packet)) != HAL_OK) {
                __NOP();
            }

        }
        ++keycode;
      }
      HAL_GPIO_Init(pin_definitions[i].gpiox, &input);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, f);

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

  /** Initializes the CPU, AHB and APB busses clocks
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
  /** Initializes the CPU, AHB and APB busses clocks
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
  huart1.Init.BaudRate = 460800;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
      HAL_NVIC_SystemReset();
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
