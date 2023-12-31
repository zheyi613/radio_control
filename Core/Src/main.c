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
#include "nrf_payload.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMER_FREQ              50

#define ADC_CHANNEL_SIZE        4
#define MOVING_AVG_SAMPLE       5

#define RADIO_CYCLE             1
#define JOYSTICK_CYCLE          1
#define PRINT_MSG_CYCLE         1

#define JOYSTICK_RANGE          2048
#define JOYSTICK_ZERO_RANGE     150

#define THROTTLE_AJUSTMENT_RANGE        10.F
#define ANGLE_AJUSTMENT_RANGE           20.F /* degree */
#define GAIN_AJUSTMENT_RANGE            0.05F
#define MOTOR_AJUSTMENT_RANGE           5
#define HEIGHT_AJUSTMENT_RANGE          0.2F

#define MAX_MOTOR_DUTY          1999

#define JOYSTICK_TASK           (1U << 0)
#define RADIO_TASK              (1U << 1)
#define DECODE_ACK_TASK         (1U << 2)
#define MSG_TASK                (1U << 3)
                                        
#define SET_TASK_TRIGGER(TASK)          task_trigger |= (TASK)
#define CLEAR_TASK_TRIGGER(TASK)        task_trigger &= ~(TASK)
#define IS_TASK_TRIGGER(TASK)           (task_trigger & (TASK))

#define JOYSTICK_LX             (0)
#define JOYSTICK_LY             (1)
#define JOYSTICK_RX             (2)
#define JOYSTICK_RY             (3)

#define CONVERT_JOYSTICK_LEVEL(data, level) \
  if (data > 0) {                           \
          level = 1;                        \
  } else if (data < 0) {                    \
          level = -1;                       \
  } else {                                  \
          level = 0;                        \
  }

#define PRESS_BUTTON_L          (1U << 0)
#define PRESS_BUTTON_R          (1U << 1)

#define PARAM_AJUSTMENT_NONE    0
#define PARAM_AJUSTMENT_P       1
#define PARAM_AJUSTMENT_I       2
#define PARAM_AJUSTMENT_D       3
#define PARAM_AJUSTMENT_FAULT_RATIO 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t tick;
uint16_t adc_val[4];
uint8_t task_trigger;
struct payload pl;
struct ack_payload ack_pl;
uint32_t lost_package;

struct joystick_raw {
  uint16_t LX;
  uint16_t LY;
  uint16_t RX;
  uint16_t RY;
} joystick_raw_data;

struct command {
  uint16_t throttle;
  float sp_roll;
  float sp_pitch;
  float sp_yaw_rate;
  float sp_height;
  float P;
  float I;
  float D;
  float fault_radio;
  uint8_t mode;
  uint8_t param_ajustment;
} cmd;

struct uav {
  float roll;
  float pitch;
  float yaw;
  uint16_t motor[4];
  uint16_t throttle;
  float height;
  float voltage;
  float current;
  uint8_t rec_status;
  uint8_t gps_sv_status;
  uint16_t gps_pAcc;
} uav_data;

volatile uint8_t tx_finish;
volatile uint8_t tx_timeout;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void joystick_task(void);
static void radio_task(void);
static void decode_ack_task(void);
static void msg_task(void);
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
  /* Initialize variable */
  tick = 0;
  memset(adc_val, 0, sizeof(adc_val));
  task_trigger = 0;
  memset(&pl, 0, sizeof(struct payload));
  memset(&ack_pl, 0, sizeof(struct ack_payload));
  memset(&joystick_raw_data, 0, sizeof(struct joystick_raw));
  memset(&cmd, 0, sizeof(struct command));
  cmd.P = 0.5F;
  cmd.I = 0.05F;
  cmd.D = 20.F;
  memset(&uav_data, 0, sizeof(struct uav));

  HAL_Delay(1000);
  
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
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, ADC_CHANNEL_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    while (1) {
      if (IS_TASK_TRIGGER(JOYSTICK_TASK))
        joystick_task();
                
      if (IS_TASK_TRIGGER(RADIO_TASK))
        radio_task();
                
      if (IS_TASK_TRIGGER(DECODE_ACK_TASK))
        decode_ack_task();

      if (IS_TASK_TRIGGER(MSG_TASK))
        msg_task();
    }
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
/* Interrupt callback */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim3.Instance) {
    tick++;
    if (tick == TIMER_FREQ)
      tick = 0;
    
    if (!(tick % RADIO_CYCLE))
      SET_TASK_TRIGGER(RADIO_TASK);
    
    if (!(tick % PRINT_MSG_CYCLE))
      SET_TASK_TRIGGER(MSG_TASK);
    tx_timeout++;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  static uint8_t adc_count = 0;
  static uint32_t total[4] = {0};

  adc_count++;

  total[0] += (uint32_t)adc_val[0];
  total[1] += (uint32_t)adc_val[1];
  total[2] += (uint32_t)adc_val[2];
  total[3] += (uint32_t)adc_val[3];

  if (adc_count == JOYSTICK_CYCLE) {
    /* Average temperory samples */
    joystick_raw_data.LX = total[0] / JOYSTICK_CYCLE;
    joystick_raw_data.LY = total[1] / JOYSTICK_CYCLE;
    joystick_raw_data.RX = total[2] / JOYSTICK_CYCLE;
    joystick_raw_data.RY = total[3] / JOYSTICK_CYCLE;
    adc_count = 0;
    memset(total, 0, sizeof(total));
    SET_TASK_TRIGGER(JOYSTICK_TASK);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == NRF_IRQ_Pin) {
    if (nrf24l01p_tx_irq((uint8_t *)&ack_pl)) {
      HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
      lost_package++;
    } else {
      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
      SET_TASK_TRIGGER(DECODE_ACK_TASK);
    }
    tx_finish = 1;
  }
}

/* Task function */
float normalize_joystick(uint16_t val)
{
  float tmp = (float)val;

  if ((tmp > (JOYSTICK_RANGE + JOYSTICK_ZERO_RANGE)) ||
      (tmp < (JOYSTICK_RANGE - JOYSTICK_ZERO_RANGE))) {
          tmp = JOYSTICK_RANGE - tmp;
          tmp /= JOYSTICK_RANGE;
  } else {
          tmp = 0;
  }
  return tmp;
}

static void joystick_task(void) {
  static float LX = 0.F;
  static float LY = 0.F;
  static float RX = 0.F;
  static float RY = 0.F;
  /* push: +1, pull: -1, zero: 0 */
  static int8_t last_LX_level = 0;
  static int8_t last_LY_level = 0;
  static int8_t last_RX_level = 0;
  static int8_t last_RY_level = 0;
  static uint8_t button = 0;
  static uint8_t last_botton = 0;
  static int32_t tmp;

  /* Normalize raw data */
  LX = normalize_joystick(joystick_raw_data.LX);
  LY = normalize_joystick(joystick_raw_data.LY);
  RX = normalize_joystick(joystick_raw_data.RX);
  RY = normalize_joystick(joystick_raw_data.RY);
  button = HAL_GPIO_ReadPin(SW_L_GPIO_Port, SW_L_Pin) ? 0 : PRESS_BUTTON_L;
  button |= HAL_GPIO_ReadPin(SW_R_GPIO_Port, SW_R_Pin) ? 0 : PRESS_BUTTON_R;
  
  /* Inplement botton data if it has been click once (failing edge) */
  if (!(button & PRESS_BUTTON_L) && (last_botton & PRESS_BUTTON_L)) {
    cmd.mode++;
    if (cmd.mode > 1) {
      cmd.mode = 0;
    }
  }
  if (!(button & PRESS_BUTTON_R) && (last_botton & PRESS_BUTTON_R)) { 
    cmd.param_ajustment++;
    if (cmd.param_ajustment > 4)
      cmd.param_ajustment = 0;
  }
  last_botton = (button & PRESS_BUTTON_L) ? PRESS_BUTTON_L : 0;
  last_botton |= (button & PRESS_BUTTON_R) ? PRESS_BUTTON_R : 0;
  /* Inplement command by joystick data */
  tmp = cmd.throttle;
  tmp += (int32_t)(LY * THROTTLE_AJUSTMENT_RANGE);

  if (tmp > MAX_MOTOR_DUTY)
    tmp = MAX_MOTOR_DUTY;
  else if (tmp < 0)
    tmp = 0;

  cmd.throttle = (uint16_t)tmp;

  switch (cmd.param_ajustment) {
  case PARAM_AJUSTMENT_NONE:
    cmd.sp_roll = RX * ANGLE_AJUSTMENT_RANGE;
    cmd.sp_pitch = -RY * ANGLE_AJUSTMENT_RANGE;
    /* Yaw setpoint(on uav) = yaw(on uav) + yaw_target(radio control) 
     * yaw_target = LX * ANGLE_AJUSTMENT_RANGE / TIMER_FREQ * RADIO_CYCLE
     */
    cmd.sp_yaw_rate = LX * ANGLE_AJUSTMENT_RANGE / TIMER_FREQ * RADIO_CYCLE;
    break;
  case PARAM_AJUSTMENT_P:
    if ((RY == 0) && (last_RY_level != 0)) {
      tmp = (int32_t)(cmd.P / GAIN_AJUSTMENT_RANGE);
      tmp += (int32_t)last_RY_level;
      cmd.P = tmp * GAIN_AJUSTMENT_RANGE;
    }
    if (cmd.P < 0)
      cmd.P = 0;
    break;
  case PARAM_AJUSTMENT_I:
    if ((RY == 0) && (last_RY_level != 0)) {
      tmp = (int32_t)(cmd.I / GAIN_AJUSTMENT_RANGE);
      tmp += (int32_t)last_RY_level;
      cmd.I = tmp * GAIN_AJUSTMENT_RANGE;
    }
    if (cmd.I < 0)
      cmd.I = 0;
    break;
  case PARAM_AJUSTMENT_D:
    if ((RY == 0) && (last_RY_level != 0)){
      tmp = (int32_t)cmd.D;
      tmp += (int32_t)last_RY_level;
      cmd.D = tmp;
    }
    if (cmd.D < 0)
      cmd.D = 0;
    break;
  case PARAM_AJUSTMENT_FAULT_RATIO:
    if ((RY == 0) && (last_RY_level != 0)) {
      tmp = (int32_t)(cmd.fault_radio / GAIN_AJUSTMENT_RANGE);
      tmp += (int32_t)last_RY_level;
      cmd.fault_radio = (float)tmp * GAIN_AJUSTMENT_RANGE;
    }
    if (cmd.fault_radio < 0)
      cmd.fault_radio = 0;
    else if (cmd.fault_radio > 1)
      cmd.fault_radio = 1;
    break;
}
  CONVERT_JOYSTICK_LEVEL(LX, last_LX_level);
  CONVERT_JOYSTICK_LEVEL(LY, last_LY_level);
  CONVERT_JOYSTICK_LEVEL(RX, last_RX_level);
  CONVERT_JOYSTICK_LEVEL(RY, last_RY_level);

  /* End task */
  CLEAR_TASK_TRIGGER(JOYSTICK_TASK);
}

static void radio_task(void) {
  ENCODE_PAYLOAD_THROTTLE(cmd.throttle, pl.throttle);
  ENCODE_PAYLOAD_DEGREE(cmd.sp_roll, pl.sp_roll);
  ENCODE_PAYLOAD_DEGREE(cmd.sp_pitch, pl.sp_pitch);
  ENCODE_PAYLOAD_DEGREE(cmd.sp_yaw_rate, pl.sp_yaw_rate);
  ENCODE_PAYLOAD_HEIGHT(cmd.sp_height, pl.sp_height);
  ENCODE_PAYLOAD_CTRL_PI_GAIN(cmd.P, pl.P);
  ENCODE_PAYLOAD_CTRL_PI_GAIN(cmd.I, pl.I);
  ENCODE_PAYLOAD_CTRL_D_GAIN(cmd.D, pl.D);
  ENCODE_PAYLOAD_MODE(cmd.mode, pl.mode);
  ENCODE_PAYLOAD_FAULT_RATIO(cmd.fault_radio, pl.fault_ratio);

  nrf24l01p_transmit((uint8_t *)&pl, PAYLOAD_WIDTH);
  tx_finish = 0;
  tx_timeout = 0;

  while ((tx_finish == 0) && (tx_timeout < 5));

  

  /* End task */
  CLEAR_TASK_TRIGGER(RADIO_TASK);
}

static void decode_ack_task(void) {
  DECODE_PAYLOAD_DEGREE(ack_pl.roll, uav_data.roll);
  DECODE_PAYLOAD_DEGREE(ack_pl.pitch, uav_data.pitch);
  DECODE_PAYLOAD_DEGREE(ack_pl.yaw, uav_data.yaw);
  DECODE_PAYLOAD_MOTOR(ack_pl.motor[0], uav_data.motor[0]);
  DECODE_PAYLOAD_MOTOR(ack_pl.motor[1], uav_data.motor[1]);
  DECODE_PAYLOAD_MOTOR(ack_pl.motor[2], uav_data.motor[2]);
  DECODE_PAYLOAD_MOTOR(ack_pl.motor[3], uav_data.motor[3]);
  DECODE_PAYLOAD_THROTTLE(ack_pl.throttle, uav_data.throttle);
  DECODE_PAYLOAD_HEIGHT(ack_pl.height, uav_data.height);
  DECODE_PAYLOAD_VOLTAGE(ack_pl.voltage, uav_data.voltage);
  DECODE_PAYLOAD_CURRENT(ack_pl.current, uav_data.current);
  DECODE_PAYLOAD_REC_STATUS(ack_pl.rec_status, uav_data.rec_status);
  DECODE_PAYLOAD_GPS_SV_STATUS(ack_pl.gps_sv_status, uav_data.gps_sv_status);
  DECODE_PAYLOAD_GPS_PACC(ack_pl.gps_pAcc, uav_data.gps_pAcc);

  /* End task */
  CLEAR_TASK_TRIGGER(DECODE_ACK_TASK);
}

static void msg_task(void) {
  static char msg[512];
  static uint16_t len;
  
  len = snprintf(msg, 512,
            "LX: %d, LY: %d, RX: %d, RY: %d\r\n"
            "command:\r\n"
            "throttle: %d, sp_r: %.2f, sp_p: %.2f, sp yaw rate: %.2f\r\n"
            "sp_h: %.2f, P1: %.2f, I1: %.2f, D1: %.2f, fault radio: %.2f\r\n"
            "mode: %d, param ajustment mode: %d\r\n"
            "uav data:\r\n"
            "r: %.2f, p: %.2f, y: %.2f\r\n"
            "throttle: %d, m0: %d, m1: %d, m2: %d, m3: %d\r\n"
            "height: %.2f, V: %.1f, I: %.1f\r\n"
            "sd_rec: %d, sv num: %d, gps pAcc: %d\r\n"
            "lost package: %ld\r\n",
            joystick_raw_data.LX, joystick_raw_data.LY,
            joystick_raw_data.RX, joystick_raw_data.RY,
            cmd.throttle, cmd.sp_roll, cmd.sp_pitch, cmd.sp_yaw_rate,
            cmd.sp_height, cmd.P, cmd.I, cmd.D, cmd.fault_radio,
            cmd.mode, cmd.param_ajustment,
            uav_data.roll, uav_data.pitch, uav_data.yaw,
            uav_data.throttle, uav_data.motor[0], uav_data.motor[1],
            uav_data.motor[2], uav_data.motor[3],
            uav_data.height, uav_data.voltage, uav_data.current,
            uav_data.rec_status, uav_data.gps_sv_status, uav_data.gps_pAcc,
            lost_package);
  CDC_Transmit_FS((uint8_t *)msg, len);
  len = snprintf(msg, 512, "0.0,0.0,0.0,0.0,0.0,0.0,%.2f,%.2f,%.2f\n",
                           uav_data.roll, uav_data.pitch, uav_data.yaw);
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 5);

  /* End task */
  CLEAR_TASK_TRIGGER(MSG_TASK);
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
