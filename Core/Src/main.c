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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define MOTOR_PWM_MAX  4199U    /* TIM2 period = 100 % duty */

/* Sensor */
#define ECHO_MIN_US     116U    /* < 2 cm: noise, ignore              */
#define ECHO_MAX_US     23200U  /* >= 4 m: nothing in range           */
#define D_CLEAR_MM      4000U   /* distance used for "nothing ahead"  */
#define LOST_TICKS      6U      /* 300 ms without any echo = lost     */

/* Speed law */
#define D_REVERSE_MM      200U   /* retreat at or below                  */
#define D_STOP_MM         250U   /* brake at or below; retreat ends here */
#define D_RELEASE_MM      300U   /* drive forward again only above       */
#define D_FAR_MM          450U   /* full follow speed beyond 45 cm       */
#define D_NEAR_LOST_MM    1000U  /* lead lost closer than this = careful */

#define CMD_SEARCH        400    /* nothing seen nearby: 40 %            */
#define CMD_FOLLOW        1000   /* full throttle while the lead is far  */
#define CMD_MIN           150    /* slowest crawl                        */
#define CMD_REVERSE       400    /* retreat at 40 %                      */
#define REVERSE_MAX_TICKS 40     /* safety cap: 2 s of blind reversing   */
#define BLIND_CRAWL_TICKS 40     /* lead lost nearby: crawl at most 2 s  */
#define CMD_RAMP          100    /* max speed-up per tick (10 %)         */
/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint32_t tick_count    = 0;  /* control ticks since start           */
static volatile uint8_t  tick_pending  = 0;  /* 1 = ISR fired, loop hasn't run yet  */
static volatile uint32_t tick_overruns = 0;  /* ticks that fired while one pending  */

static uint32_t tx_drops = 0;                /* lines skipped, UART still busy      */
static uint32_t work_us  = 0;                /* duration of the last loop pass      */
static char     tx_buf[96];                  /* telemetry line, read by the DMA     */

/* Ultrasonic echo, written by the TIM3 capture interrupt */
static volatile uint32_t echo_rise  = 0;     /* timer count at echo rising edge     */
static volatile uint32_t echo_us    = 0;     /* width of the last complete echo, µs */
static volatile uint8_t  echo_fresh = 0;     /* 1 = new echo since loop last looked */

/* Motor state, set by the motor_ functions, shown in telemetry */
static char    motor_mode = 'F';     /* F = float, B = brake, D = drive */
static int32_t motor_cmd  = 0;       /* drive command, -1000..+1000     */

/* Controller state, shown in telemetry */
static uint32_t d_filt_mm  = 0;      /* filtered distance               */
static char     ctrl_state = 'S';    /* C search, A follow, S stop, R retreat,
                                        B blind crawl, W wait, L sensor lost */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Motor on the Infineon shield:
   TIM2 CH1 (PA0) -> IN_U, TIM2 CH2 (PA1) -> IN_V, PA4 -> INH (both). */

/* Float: both half-bridges asleep, motor coasts freely. */
static void motor_float(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  HAL_GPIO_WritePin(MOTOR_INH_GPIO_Port, MOTOR_INH_Pin, GPIO_PIN_RESET);
  motor_mode = 'F';
  motor_cmd  = 0;
}

/* Brake: both outputs to GND, motor terminals shorted. */
static void motor_brake(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  HAL_GPIO_WritePin(MOTOR_INH_GPIO_Port, MOTOR_INH_Pin, GPIO_PIN_SET);
  motor_mode = 'B';
  motor_cmd  = 0;
}

/* Drive: cmd in per mille, -1000..+1000. Positive = forward. */
static void motor_drive(int32_t cmd)
{
  if (cmd >  1000) cmd =  1000;
  if (cmd < -1000) cmd = -1000;

  uint32_t mag = (uint32_t)(cmd < 0 ? -cmd : cmd);
  uint32_t ccr = mag * (MOTOR_PWM_MAX + 1U) / 1000U;

  if (cmd >= 0)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);   /* V side held low */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr); /* U side pulses   */
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr);
  }
  HAL_GPIO_WritePin(MOTOR_INH_GPIO_Port, MOTOR_INH_Pin, GPIO_PIN_SET);
  motor_mode = 'D';
  motor_cmd  = cmd;
}

/* Middle value of three: one bad echo can never win. */
static uint32_t median3(uint32_t a, uint32_t b, uint32_t c)
{
  uint32_t t;
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { t = a; a = b; b = t; }
  return b;
}

/* Turns raw echoes into d_filt_mm. Returns 1 if the reading is
   usable this tick, 0 if the sensor counts as lost. */
static uint8_t sensor_update(uint32_t e_us, uint8_t fresh)
{
  static uint32_t m[3];
  static uint8_t  primed   = 0;
  static uint8_t  no_echo  = 0;

  if (fresh && e_us >= ECHO_MIN_US)
  {
    uint32_t d = (e_us >= ECHO_MAX_US) ? D_CLEAR_MM : (e_us * 10U) / 58U;
    if (!primed) { m[0] = m[1] = m[2] = d; primed = 1; }
    m[0] = m[1];  m[1] = m[2];  m[2] = d;      /* shift in newest */
    d_filt_mm = median3(m[0], m[1], m[2]);
    no_echo = 0;
  }
  else if (no_echo < 255U)
  {
    no_echo++;
  }
  return (primed && no_echo < LOST_TICKS) ? 1U : 0U;
}

/* One control step: distance -> motor. Called once per tick. */
static void control_step(uint8_t sensor_ok)
{
  static int32_t  cmd       = 0;           /* last forward command      */
  static uint32_t reverse_t = 0;           /* ticks reversing this time */
  static uint32_t blind_t   = 0;           /* ticks since lead vanished */
  static uint32_t last_seen = D_CLEAR_MM;  /* last distance to the lead */

  if (!sensor_ok)                          /* sensor dead: stop */
  {
    ctrl_state = 'L';  cmd = 0;  reverse_t = 0;
    motor_brake();  return;
  }

  uint32_t d = d_filt_mm;

  /* 1. Retreat: lead within 20 cm, reverse until 25 cm (max 2 s) */
  if (d <= D_REVERSE_MM || (ctrl_state == 'R' && d < D_STOP_MM))
  {
    cmd = 0;  last_seen = d;  blind_t = 0;
    if (reverse_t < REVERSE_MAX_TICKS)
    {
      reverse_t++;
      ctrl_state = 'R';
      motor_drive(-CMD_REVERSE);
    }
    else
    {
      ctrl_state = 'S';
      motor_brake();
    }
    return;
  }
  reverse_t = 0;

  /* 2. Stop zone: 25 cm, released only above 30 cm */
  uint8_t stopped = (ctrl_state == 'S' || ctrl_state == 'L' ||
                     ctrl_state == 'R' || ctrl_state == 'W');
  if (d <= D_STOP_MM || (stopped && d <= D_RELEASE_MM))
  {
    ctrl_state = 'S';  cmd = 0;  last_seen = d;  blind_t = 0;
    motor_brake();  return;
  }

  /* 3. Forward */
  int32_t target;
  if (d >= D_CLEAR_MM)                     /* lead not in sight */
  {
    if (last_seen < D_NEAR_LOST_MM)        /* it vanished close by */
    {
      if (blind_t < BLIND_CRAWL_TICKS)
      {
        blind_t++;
        target = CMD_MIN;  ctrl_state = 'B';  /* blind crawl */
      }
      else
      {
        ctrl_state = 'W';  cmd = 0;           /* wait, braked */
        motor_brake();  return;
      }
    }
    else                                   /* far, or never seen */
    {
      target = CMD_SEARCH;  ctrl_state = 'C';
    }
  }
  else                                     /* lead in sight: follow */
  {
    last_seen = d;  blind_t = 0;
    if (d >= D_FAR_MM)
    {
      target = CMD_FOLLOW;
    }
    else
    {
      target = CMD_MIN + (CMD_FOLLOW - CMD_MIN) * (int32_t)(d - D_STOP_MM)
                                              / (int32_t)(D_FAR_MM - D_STOP_MM);
    }
    ctrl_state = 'A';
  }

  if (target > cmd + CMD_RAMP)             /* speed up gently ...   */
  {
    target = cmd + CMD_RAMP;
  }
  cmd = target;                            /* ... slow down at once */
  motor_drive(cmd);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM11_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* Turn on the CPU's cycle counter (DWT). It counts every clock
     cycle, 84 per microsecond, and we use it to time the loop. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  /* CSV header, sent once. A blocking send is fine here:
     the control loop isn't running yet. */
  int n = snprintf(tx_buf, sizeof tx_buf,
          "t_ms,tick,overruns,tx_drops,work_us,echo_us,d_mm,fresh,d_filt,state,mode,cmd\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, n, 100);

  /* Motor PWM running from the start, but with 0 % duty and INH low */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  motor_float();

  /* Ultrasonic: start capturing echo edges first, so the first
     echo can't arrive before we're listening, then the trigger. */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* Start the 50 ms control tick last, once everything is ready. */
  HAL_TIM_Base_Start_IT(&htim11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (tick_pending)
    {
      uint32_t t0 = DWT->CYCCNT;   /* start timing this pass   */
      uint32_t k  = tick_count;    /* one consistent copy      */

      /* Ultrasonic: take a consistent snapshot of the latest echo */
      __disable_irq();
      uint32_t e_us  = echo_us;
      uint8_t  fresh = echo_fresh;
      echo_fresh = 0;
      __enable_irq();

      uint32_t d_mm = (e_us * 10U) / 58U;   /* 58 µs per cm, round trip */

      /* Heartbeat: toggle LD2 every 10 ticks, so it blinks at 1 Hz */
      if (k % 10 == 0)
      {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }

      /* Controller: clean the reading, then decide the motor command */
      uint8_t sensor_ok = sensor_update(e_us, fresh);
      control_step(sensor_ok);

      /* Telemetry: one CSV line per tick, sent by DMA in the background */
      if (huart2.gState == HAL_UART_STATE_READY)
      {
          int len = snprintf(tx_buf, sizeof tx_buf,
                             "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%lu,%c,%c,%ld\r\n",
                             HAL_GetTick(), k, tick_overruns, tx_drops, work_us,
                             e_us, d_mm, (unsigned)fresh,
                             d_filt_mm, ctrl_state, motor_mode, motor_cmd);
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)tx_buf, len);
      }
      else
      {
        tx_drops++;                /* last line still being sent */
      }

      work_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000U);
      tick_pending = 0;            /* done with this tick       */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* HAL calls this from inside a timer interrupt whenever a timer's
   counter wraps. Every timer shares this one function, so first
   check which timer it was. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11)      /* the 50 ms control tick */
  {
    if (tick_pending)               /* last tick still not handled? */
    {
      tick_overruns++;              /* the loop is too slow */
    }
    tick_pending = 1;               /* tell the main loop: run now */
    tick_count++;
  }
}

/* HAL calls this from the TIM3 interrupt each time an input
   capture channel latches an edge. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uint32_t t = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if (HAL_GPIO_ReadPin(SONAR_ECHO_GPIO_Port, SONAR_ECHO_Pin) == GPIO_PIN_SET)
    {
      echo_rise = t;                       /* pin is high: echo just started */
    }
    else
    {
      if (t < echo_rise)                   /* counter wrapped mid-echo */
      {
        t += 50000;                        /* one timer period in µs */
      }
      echo_us    = t - echo_rise;          /* pin is low: echo just ended */
      echo_fresh = 1;
    }
  }
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
