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
#include <usbd_hid.h>
#include <map_keycode.h>
#include <math.h>
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
#include "usb_device.h"
#include "usb_hid_keys.h"
#include <stm32f1xx_hal_flash.h>
#include <stm32f1xx_hal_flash_ex.h>
#include <assert.h>
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

typedef struct {
    int8_t offsets[8];
} calibration_data_t;

typedef struct {
    calibration_data_t calibration;
} config_t;

/* USER CODE BEGIN PV */

extern USBD_HandleTypeDef hUsbDeviceFS;
int is_fn_down_v = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
struct {
    uint8_t is_keyboard_has_new_data;
    uint8_t is_consumer_has_new_data;
} state;

struct {
    uint8_t high : 1;
    uint8_t is_pressed : 1;
    uint16_t keycode : 14;
} from_keyboard_controller_packet;

struct {
    uint8_t report_id;
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keys[6];
} hid_keyboard_report;
struct {
    uint8_t report_id;
    uint8_t select_music : 1;
    uint8_t scan_next_track : 1;
    uint8_t scan_prev_track : 1;
    uint8_t volume_up : 1;
    uint8_t volume_down : 1;
    uint8_t play_pause : 1;
    uint8_t stop : 1;
    uint8_t mute : 1;
    uint8_t tracking_decr : 1;
    uint8_t tracking_inc : 1;
    uint8_t select_www : 1;
    uint8_t select_home : 1;
    uint8_t select_messages : 1;
} hid_consumer_report; */
struct {
    uint8_t report_id;
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t wheelY;
    int8_t wheelX;
} hid_mouse_report;


int is_fn_down() {
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
}

extern uint32_t sensor_measure(GPIO_TypeDef *gpiox, uint32_t pin);

typedef struct {
    GPIO_TypeDef *gpiox;
    uint32_t pin;

    uint32_t (*callback)(GPIO_TypeDef *, uint32_t);
} sensor_pin_t;



uint32_t simple_button(GPIO_TypeDef *gpiox, uint32_t pin) {
    return HAL_GPIO_ReadPin(gpiox, pin) == GPIO_PIN_SET ? 1 : 0;
}

sensor_pin_t stick_pin_data[] = {
        {GPIOA, 0,           sensor_measure }, // 0
        {GPIOA, 1,           sensor_measure }, // 1
        {GPIOA, 4,           sensor_measure }, // 2
        {GPIOA, 3,           sensor_measure }, // 3
//        {GPIOA, 4,           sensor_measure }, // 4
        {GPIOA, 5,           sensor_measure }, // 4
        {GPIOA, 6,           sensor_measure }, // 5
        {GPIOA, 7,           sensor_measure }, // 6
        {GPIOB, 0,           sensor_measure }, // 7
        {0}
};

int isign(int i) {
    if (i > 0)
        return 1;
    if (i < 0)
        return -1;
    return 0;
}

int8_t i8max(int8_t a, int8_t b) {
    if (a > b) {
        return a;
    }
    return b;
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //USBD_HID_SendReport(&hUsbDeviceFS, )
/*
    if (from_keyboard_controller_packet.high == 0) {
        for (int i = 0; i < 50; ++i) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(30);
        }
        Error_Handler();
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);

    uint16_t mapped_keycode = map_scancode_to_keycode(from_keyboard_controller_packet.keycode);
    if (mapped_keycode != KEY_CUSTOM_FN && is_fn_down_v) {
        state.is_consumer_has_new_data = 1;
        switch (mapped_keycode) {

            case KEY_F2:
                hid_consumer_report.scan_prev_track = from_keyboard_controller_packet.is_pressed;
                break;
            case KEY_F3:
                hid_consumer_report.scan_next_track = from_keyboard_controller_packet.is_pressed;
                break;

            case KEY_F4:
                hid_consumer_report.volume_up = from_keyboard_controller_packet.is_pressed;
                break;
            case KEY_F5:
                hid_consumer_report.volume_down = from_keyboard_controller_packet.is_pressed;
                break;

            case KEY_F6:
                hid_consumer_report.play_pause = from_keyboard_controller_packet.is_pressed;
                break;
            case KEY_F7:
                hid_consumer_report.stop = from_keyboard_controller_packet.is_pressed;
                break;
            case KEY_F8:
                hid_consumer_report.mute = from_keyboard_controller_packet.is_pressed;
                break;

            default:
                state.is_consumer_has_new_data = 0;
        }
    }
    if (!state.is_consumer_has_new_data) {
        switch (mapped_keycode) {
            case KEY_NONE:
                break;
            case KEY_CUSTOM_FN:
                is_fn_down_v = from_keyboard_controller_packet.is_pressed;
                if (!is_fn_down_v) {
                    // release all consumer buttons whe fn released
                    memset(&hid_consumer_report, 0, sizeof(hid_consumer_report));
                    hid_consumer_report.report_id = 3;
                    state.is_consumer_has_new_data = 1;
                }
                break;

            case KEY_LEFTMETA:
            case KEY_RIGHTMETA:
            case KEY_LEFTSHIFT:
            case KEY_RIGHTSHIFT:
            case KEY_LEFTALT:
            case KEY_RIGHTALT:
            case KEY_LEFTCTRL:
            case KEY_RIGHTCTRL: {
                uint8_t flag = 0;
                switch (mapped_keycode) {
                    case KEY_LEFTMETA:
                        flag = KEY_MOD_LMETA;
                        break;
                    case KEY_RIGHTMETA:
                        flag = KEY_MOD_RMETA;
                        break;
                    case KEY_LEFTSHIFT:
                        flag = KEY_MOD_LSHIFT;
                        break;
                    case KEY_RIGHTSHIFT:
                        flag = KEY_MOD_RSHIFT;
                        break;
                    case KEY_LEFTALT:
                        flag = KEY_MOD_LALT;
                        break;
                    case KEY_RIGHTALT:
                        flag = KEY_MOD_RALT;
                        break;
                    case KEY_LEFTCTRL:
                        flag = KEY_MOD_LCTRL;
                        break;
                    case KEY_RIGHTCTRL:
                        flag = KEY_MOD_RCTRL;
                        break;
                }
                if (from_keyboard_controller_packet.is_pressed) {
                    hid_keyboard_report.modifiers |= flag;
                } else {
                    hid_keyboard_report.modifiers &= ~flag;
                }
                state.is_keyboard_has_new_data = 1;
            }
                break;

            default:
                if (from_keyboard_controller_packet.is_pressed) {
                    // find zero spot to place a keycode there
                    for (int i = 0; i < sizeof(hid_keyboard_report.keys); ++i) {
                        if (hid_keyboard_report.keys[i] == 0) {
                            hid_keyboard_report.keys[i] = mapped_keycode;
                            break;
                        }
                    }
                } else {
                    // find a keycode to set zero there
                    for (int i = 0; i < sizeof(hid_keyboard_report.keys); ++i) {
                        if (hid_keyboard_report.keys[i] == mapped_keycode) {
                            hid_keyboard_report.keys[i] = 0;
                            break;
                        }
                    }
                }
                state.is_keyboard_has_new_data = 1;
        }
    }
    HAL_UART_Receive_IT(&huart1, &from_keyboard_controller_packet, sizeof(from_keyboard_controller_packet));*/
}
/**
  * @brief  UART error callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Error_Handler();
}

uint32_t gCalibrationModeSwitchPressTimestamp = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  memset(&hid_mouse_report, 0, sizeof(hid_mouse_report));
  hid_mouse_report.report_id = 1;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  // set gnd on usb data+ for a while to force the host's controller to think that device was reconnected

  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio;
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(1000);
    __HAL_RCC_GPIOA_CLK_DISABLE();
  }

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  typedef struct {
    int16_t x, y;
  } vec2;
  vec2 mouseAcc = {0, 0};
  vec2 scrollAcc = {0, 0};

  int isCalibrating = 0;
  volatile config_t* pConfig =
          0x8000000 + // flash beginning
          0x10000 / 2 // flash size
          - 1024      // one page
          ;

  calibration_data_t newCalibrationData;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


    for (uint32_t i = 0; stick_pin_data[i].gpiox; ++i) {
        /*
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, !(hid_keyboard_report.keys[0] || hid_keyboard_report.keys[1] || hid_keyboard_report.keys[2]));

      }*/
      {
        int8_t measureTemp = stick_pin_data[i].callback(stick_pin_data[i].gpiox, stick_pin_data[i].pin);

        if (isCalibrating) {
            newCalibrationData.offsets[i] = i8max(newCalibrationData.offsets[i], measureTemp);
        }

        if (pConfig->calibration.offsets[i] == -1 && !isCalibrating) {
            // looks like empty data; switch to calibration mode
            isCalibrating = 1;
            memset(&newCalibrationData, 0, sizeof(newCalibrationData));
            gCalibrationModeSwitchPressTimestamp = HAL_GetTick();
        }
        measureTemp -= pConfig->calibration.offsets[i];
        if (measureTemp <= 0) continue;
        switch (measureTemp) {
            case 0 : measureTemp = 00; break;
            case 1 : measureTemp = 10; break;
            case 2 : measureTemp = 11; break; // 1
            case 3 : measureTemp = 13; break; // 2
            case 4 : measureTemp = 16; break;
            case 5 : measureTemp = 20; break;
            case 6 : measureTemp = 25; break;
            case 7 : measureTemp = 31; break;
            case 8 : measureTemp = 38; break;
            case 9 : measureTemp = 46; break;
            case 10: measureTemp = 55; break;
            case 11: measureTemp = 65; break;
            case 12: measureTemp = 86; break;
            case 13: measureTemp = 98; break;
            case 14: measureTemp = 120; break;
            case 15: measureTemp = 150; break;
        }
        switch (i) {
          case 0:
              mouseAcc.x -= measureTemp;
              mouseAcc.y -= measureTemp;
            break;
          case 1:
          //acc.x -= measureTemp;
            mouseAcc.y -= measureTemp;
            break;
          case 2:
              mouseAcc.x += measureTemp;
              mouseAcc.y -= measureTemp;
            break;

          case 3:
              mouseAcc.x -= measureTemp;
            break;
          case 5-1:
              mouseAcc.x += measureTemp;
            break;

          case 6-1:
              mouseAcc.x -= measureTemp;
              mouseAcc.y += measureTemp;
            break;
          case 7-1:
            //acc.x -= measureTemp;
              mouseAcc.y += measureTemp;
            break;
          case 8-1:
              mouseAcc.x += measureTemp;
              mouseAcc.y += measureTemp;
            break;
        }
      }
    }

    hid_mouse_report.buttons = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) | (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) << 1);

    if (abs(hid_mouse_report.x) < 10) hid_mouse_report.x = 0;
    if (abs(hid_mouse_report.y) < 10) hid_mouse_report.y = 0;

    hid_mouse_report.x = mouseAcc.x / 10;
    hid_mouse_report.y = mouseAcc.y / 10;
    mouseAcc.x -= hid_mouse_report.x * 10;
    mouseAcc.y -= hid_mouse_report.y * 10;


    if (is_fn_down()) {
      scrollAcc.x += hid_mouse_report.x;
      scrollAcc.y -= hid_mouse_report.y;
        hid_mouse_report.x = 0;
        hid_mouse_report.y = 0;
      if (abs(scrollAcc.x) > abs(scrollAcc.y)) {
          scrollAcc.y = 0;
      } else {
          scrollAcc.x = 0;
      }

      hid_mouse_report.wheelX = scrollAcc.x / 4;
      hid_mouse_report.wheelY = scrollAcc.y / 4;
      scrollAcc.x -= hid_mouse_report.wheelX * 4;
      scrollAcc.y -= hid_mouse_report.wheelY * 4;

      hid_mouse_report.x = 0;
      hid_mouse_report.y = 0;
    } else {
      hid_mouse_report.wheelX = 0;
      hid_mouse_report.wheelY = 0;
    }

    // calibration mode
    if (hid_mouse_report.buttons == 0b11 && is_fn_down() && !isCalibrating) {
        if (gCalibrationModeSwitchPressTimestamp == -1) {
            gCalibrationModeSwitchPressTimestamp = HAL_GetTick();
        }
        if (HAL_GetTick() - gCalibrationModeSwitchPressTimestamp > 3000) {
            // switch to calibration mode
            isCalibrating = 1;
            mouseAcc.x = -1000; // report to user that calibration is started by moving cursor a little

            memset(&newCalibrationData, 0, sizeof(newCalibrationData));
        }
    } else if (isCalibrating) {
        if (HAL_GetTick() - gCalibrationModeSwitchPressTimestamp > 6000) {
            // switch off the calibration mode
            isCalibrating = 0;

            // write new config
            config_t newConfig = *pConfig;
            newConfig.calibration = newCalibrationData;

            // unlock memory
            if (HAL_FLASH_Unlock() != HAL_OK) {
                assert(0);
            }

            // erase page
            SET_BIT(FLASH->CR, FLASH_CR_PER);
            WRITE_REG(FLASH->AR, pConfig);
            SET_BIT(FLASH->CR, FLASH_CR_STRT);
            while ((FLASH->SR & FLASH_SR_BSY) != 0 );
            CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

            // write data by 32-bit words
            void* pNewConfigEnd = &newConfig + 1;
            SET_BIT(FLASH->CR, FLASH_CR_PG);   // we're going to program new data
            for (volatile uint16_t* pOldConfig = pConfig, *pNewConfig = &newConfig; pNewConfig < pNewConfigEnd; pNewConfig += 1, pOldConfig += 1) {
                *pOldConfig = *pNewConfig;
                while ((FLASH->SR & FLASH_SR_BSY) != 0 );
                assert(*pOldConfig == *pNewConfig);
            }
            CLEAR_BIT(FLASH->CR, FLASH_CR_PG); // we've finished writing
            HAL_FLASH_Lock();


            mouseAcc.x = -1000; // report to user that calibration is completed by moving cursor a little
            gCalibrationModeSwitchPressTimestamp = -1; // reset timer
        }
    } else {
        gCalibrationModeSwitchPressTimestamp = -1;
    }


      USBD_HID_SendReport(&hUsbDeviceFS, &hid_mouse_report, sizeof(hid_mouse_report));
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
