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
#include "tim.h"
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
#define TIMER_FREQ            50
#define MAX_MSG_LENGTH        400
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

enum PID_tuning {
  TILT_MODE,
  P_MODE,
  I_MODE,
  D_MODE
};

enum {
  NORMAL_MODE,
  ALTITUDE_MODE
};

#define PAYLOAD_WIDTH         32
#define ACK_PAYLOAD_WIDTH     20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t tick;
uint32_t lost_package;
uint8_t adc_finish;
uint8_t tx_finish;
uint8_t unlock;

union {
  struct {
    uint16_t throttle;
    int16_t roll_target; /* +-180/32768 LSB */
    int16_t pitch_target;
    int16_t yaw_target;
    uint16_t height_target; /* 1mm LSB */
    uint8_t P;           /* 0.001 LSB */
    uint8_t I;
    uint8_t D;
    uint8_t mode;
    uint8_t unlock;
    uint8_t dummy[17];
  } data;
  uint8_t bytes[PAYLOAD_WIDTH];
} payload;

union {
  struct {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint16_t throttle;
    uint16_t motor[4];
    int16_t height;  /* 0.01 m LSB, range: +-327.68 m */
    uint8_t voltage; /* 0.1 V LSB, range: 0 ~ 17.5 V */
    uint8_t event;
  } data;
  uint8_t bytes[ACK_PAYLOAD_WIDTH];
} ack_payload;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int8_t convert_val(uint16_t val);
void tuning(uint8_t *param, int8_t tuning_val);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();

  tick = 0;
  uint8_t last_tick = 0;
  lost_package = 0;
  uint8_t msg[MAX_MSG_LENGTH];
  uint16_t len;
  uint16_t adc_val[ADC_CHANNEL_SIZE];
  /* payload variable */
  int16_t tmp = 0;
  float roll_target = 0;
  float pitch_target = 0;
  float yaw_target = 0;
  float P = 1.2f;
  float I = 0.1f;
  float D = 0.2f;
  uint8_t mode = NORMAL_MODE;
  /* ack payload variable */
  uint16_t get_throttle = 0;
  float roll = 0.f;
  float pitch = 0.f;
  float yaw = 0.f;
  uint16_t motor[4] = {0};
  float height = 0.f;
  float voltage = 0.f;
  uint8_t event = 0;

  int8_t joystick_RY_last_state = 0;
  int8_t tuning_val = 0;

  uint8_t SW_L = 0;
  uint8_t SW_R = 0;
  uint8_t SW_last_state = 0;
  uint8_t SW_L_last_state = 0;
  uint8_t SW_R_last_state = 0;
  uint8_t PID_tuning = 0;

  adc_finish = 0;
  tx_finish = 0;
  unlock = 0;

  memset(payload.bytes, 0, PAYLOAD_WIDTH);
  memset(ack_payload.bytes, 0, ACK_PAYLOAD_WIDTH);

  payload.data.P = (uint8_t)(P * 20.f);
  payload.data.I = (uint8_t)(I * 20.f);
  payload.data.D = (uint8_t)(D * 20.f);

  struct nrf24l01p_cfg nrf24l01_param = {
    .mode = PTX_MODE,
    .crc_len = CRC_TWO_BYTES,
    .air_data_rate = _2Mbps,
    .output_power = _0dBm,
    .channel = 2502,
    .address_width = 5,
    .auto_retransmit_count = 6,
    .auto_retransmit_delay = 750
  };

  if (nrf24l01p_init(&nrf24l01_param)) {
    while (1) {
      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
      HAL_Delay(1000);  
    }
  }
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    while (tick == last_tick)
      ;
    last_tick = tick;
    HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
    if (tick % 5) { /* 10 Hz */
      if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, ADC_CHANNEL_SIZE)
                        == HAL_OK) {
        while (!adc_finish)
          ;
        adc_finish = 0;
        tmp = (int16_t)payload.data.throttle;
        tmp += convert_val(adc_val[JOYSTICK_LY_VAL_INDEX]);
        if (tmp < 0)
          payload.data.throttle = 0;
        else if (tmp > 1999)
          payload.data.throttle = 1999;
        else
          payload.data.throttle = (uint16_t)tmp;

        if (PID_tuning == TILT_MODE) {
          roll_target = convert_val(adc_val[JOYSTICK_RX_VAL_INDEX]);
          payload.data.roll_target = roll_target * 182.04444f;
          pitch_target = -convert_val(adc_val[JOYSTICK_RY_VAL_INDEX]);
          payload.data.pitch_target = pitch_target * 182.04444f;
          yaw_target = convert_val(adc_val[JOYSTICK_LX_VAL_INDEX]);
          payload.data.yaw_target = yaw_target * 182.04444f;
        } else {
          tuning_val = convert_val(adc_val[JOYSTICK_RY_VAL_INDEX]);
          if (joystick_RY_last_state == 0) {
            if (tuning_val > 0)
              tuning_val = 1;
            else if (tuning_val < 0)
              tuning_val = -1;
            
            if (PID_tuning == P_MODE) {
              tuning(&payload.data.P, tuning_val);
              P = (float)payload.data.P * 0.05f;
            } else if (PID_tuning == I_MODE) {
              tuning(&payload.data.I, tuning_val);
              I = (float)payload.data.I * 0.05f;
            } else {
              tuning(&payload.data.D, tuning_val);
              D = (float)payload.data.D * 0.05f;
            }
          }
          joystick_RY_last_state = tuning_val;
        }
      }
      SW_L = !HAL_GPIO_ReadPin(SW_L_GPIO_Port, SW_L_Pin);
        SW_R = !HAL_GPIO_ReadPin(SW_R_GPIO_Port, SW_R_Pin);
        if (SW_L & !SW_L_last_state) {
          mode++;
          if (mode > 1)
            mode = 0;
          payload.data.mode = mode;
        }
        SW_L_last_state = SW_L;
        if (SW_R & !SW_R_last_state) {
          PID_tuning++;
          if (PID_tuning > 3)
            PID_tuning = 0;
        }
        SW_R_last_state = SW_R;
        if (SW_L & SW_R & !SW_last_state)
          unlock ^= 1;
        payload.data.unlock = unlock;
        SW_last_state = SW_L | SW_R;
    }
    nrf24l01p_transmit(payload.bytes, PAYLOAD_WIDTH);
    while (!tx_finish)
      ;
    tx_finish = 0;

    get_throttle = ack_payload.data.throttle;
    roll = (float)ack_payload.data.roll * 0.005493164f;
    pitch = (float)ack_payload.data.pitch * 0.005493164f;
    yaw = (float)ack_payload.data.yaw * 0.005493164f;
    motor[0] = ack_payload.data.motor[0];
    motor[1] = ack_payload.data.motor[1];
    motor[2] = ack_payload.data.motor[2];
    motor[3] = ack_payload.data.motor[3];
    height = (float)ack_payload.data.height * 0.01f;
    voltage = (float)ack_payload.data.voltage * 0.1f;
    event = ack_payload.data.event;

    len = snprintf((char *)msg, MAX_MSG_LENGTH,
                   "throttle: %d, r target: %.2f, "
                   "p target: %.2f, y target: %.2f\n\r"
                   "unlock: %d, set mode: %d, P: %.3f, I: %.3f, D: %.3f\n\r"
                   "get throttle: %d, r: %.2f, p: %.2f, y: %.2f\n\r"
                   "m1: %d, m2: %d, m3: %d, m4: %d\n\r"
                   "h: %.2f, vol: %.1f, ctrl mode: %d, crash: %d\n\r"
                   "lost package: %ld\n\r",
                   payload.data.throttle, roll_target, pitch_target, yaw_target,
                   payload.data.unlock, PID_tuning, P, I, D,
                   get_throttle, roll, pitch, yaw,
                   motor[0], motor[1], motor[2], motor[3],
                   height, voltage, mode, event,
                   lost_package);
    CDC_Transmit_FS(msg, len);
    len = snprintf((char *)msg, MAX_MSG_LENGTH, "0.0,0.0,0.0,0.0,0.0,0.0,"
                                                "%.2f,%.2f,%.2f\n",
                                                roll, pitch, yaw);
    HAL_UART_Transmit(&huart1, msg, len, 10);
    HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim3.Instance) {
    tick++;
    if (tick == TIMER_FREQ)
      tick = 0;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  adc_finish = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == NRF_IRQ_Pin) {
    if (nrf24l01p_tx_irq(ack_payload.bytes)) {
      HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
      lost_package++;
    } else {
      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
    }
    tx_finish = 1;
  }
}

int8_t convert_val(uint16_t val)
{
  int8_t new_val;

  if (val > 2200 || val < 1900)
    new_val = (2048 - (int16_t)val) >> (4 + INPUT_MAGNIFICATION);
  else
    new_val = 0;

  return new_val;
}

void tuning(uint8_t *param, int8_t tuning_val)
{
  int16_t tmp;

  if (tuning_val == 0)
    return;
  tmp = (int16_t)(*param) + (int16_t)tuning_val;
  if (tmp >= 0 && tmp < 250)
    *param = (int8_t)tmp;
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
