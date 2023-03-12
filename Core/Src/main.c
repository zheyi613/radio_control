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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "dwt_delay.h"
#include "stdio.h"
#include "string.h"
#include "nrf24l01p.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_MSG_LENGTH        200
#define ADC_CHANNEL_SIZE      4

enum adc_val_index {
  JOYSTICK_LX_VAL_INDEX,
  JOYSTICK_LY_VAL_INDEX,
  JOYSTICK_RX_VAL_INDEX,
  JOYSTICK_RY_VAL_INDEX
};

enum input_magnification {
  INPUT_16X,  /* value: -128 ~ 127 */
  INPUT_8X,   /* -64 ~ 63 */
  INPUT_4X,   /* -32 ~ 31 */
  INPUT_2X,   /* -16 ~ 15 */
};

#define INPUT_MAGNIFICATION   INPUT_2X
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t msg[100], len;
uint16_t adc_val[ADC_CHANNEL_SIZE];
uint8_t adc_finish;
uint8_t tx_finish;
uint8_t lock;

union {
  struct {
    int8_t duty_input;
    int8_t roll_input;
    int8_t pitch_input;
    int8_t yaw_input;
    uint8_t dummy[28];
  } data;
  uint8_t bytes[32];
} payload;

union {
  struct {
    float roll;
    float pitch;
    float yaw;
    uint16_t duty;
  } data;
  uint8_t bytes[14];
} ack_payload;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int8_t convert_val(uint16_t val);
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
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();

  adc_finish = 0;
  tx_finish = 0;
  lock = 0;
  memset(payload.bytes, 0, 32);
  memset(ack_payload.bytes, 0, 14);

  struct nrf24l01p_cfg nrf24l01_param = {
    .mode = PTX_MODE,
    .crc_len = CRC_TWO_BYTES,
    .air_data_rate = _2Mbps,
    .output_power = _0dBm,
    .channel = 2432,
    .address_width = 5,
    .auto_retransmit_count = 6,
    .auto_retransmit_delay = 500
  };

  if (nrf24l01p_init(&nrf24l01_param)) {
    while (1) {
      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
      HAL_Delay(1000);  
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, ADC_CHANNEL_SIZE)
                        == HAL_OK) {
        while (!adc_finish)
          ;
        adc_finish = 0;

        payload.data.duty_input = 
                  convert_val(adc_val[JOYSTICK_LY_VAL_INDEX]);
        payload.data.roll_input =
                  -convert_val(adc_val[JOYSTICK_RX_VAL_INDEX]);
        payload.data.pitch_input =
                  convert_val(adc_val[JOYSTICK_RY_VAL_INDEX]);
        payload.data.yaw_input =
                  -convert_val(adc_val[JOYSTICK_LX_VAL_INDEX]);

        nrf24l01p_transmit(payload.bytes, 32);
        while (!tx_finish)
          ;
        tx_finish = 0;

        len = snprintf((char *)msg, MAX_MSG_LENGTH,
                      "duty in: %d, roll in: %d, pitch in: %d, yaw in: %d\n\r"
                      "duty: %d, roll: %.2f, pitch: %.2f, yaw: %.2f\n\r"
                      "lock: %d\n\r",
                      payload.data.duty_input, payload.data.roll_input,
                      payload.data.pitch_input, payload.data.yaw_input,
                      ack_payload.data.duty, ack_payload.data.roll,
                      ack_payload.data.pitch, ack_payload.data.yaw,
                      lock);
        HAL_UART_Transmit(&huart1, msg, len, 100);
    }
  
    HAL_Delay(200);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  adc_finish = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == NRF_IRQ_Pin) {
    if (nrf24l01p_tx_irq(ack_payload.bytes))
      HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
    else
      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
    tx_finish = 1;
  } else if (GPIO_Pin == SW_L_Pin) {
    
  } else if (GPIO_Pin == SW_R_Pin) {
  
  }
}

int8_t convert_val(uint16_t val)
{
  int8_t new_val;

  if (val > 2300 || val < 1900)
    new_val = (2048 - (int16_t)val) >> (4 + INPUT_MAGNIFICATION);
  else
    new_val = 0;

  return new_val;
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
