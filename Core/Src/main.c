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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define MOTOR_PWM_MAX  4199U    /* TIM2 period = 100 % duty */

/* Sensor */
#define ECHO_MIN_US     116U    /* < 2 cm: counted as 2 cm            */
#define ECHO_MAX_US     23200U  /* >= 4 m: nothing in range           */
#define D_CLEAR_MM      4000U   /* distance used for "nothing ahead"  */
#define LOST_TICKS      6U      /* 300 ms without any echo = lost     */
#define SONAR_PERIOD_US 65000U  /* ping every 65 ms: late echoes from far
                                   walls die out before the next ping   */

/* Speed law */
#define D_REVERSE_MM      200U   /* retreat at or below                  */
#define D_STOP_MM         250U   /* brake at or below; retreat ends here */
#define D_RELEASE_MM      300U   /* drive forward again only above       */
#define D_FAR_MM          500U   /* follow speed reaches CMD_FOLLOW here */
#define D_NEAR_LOST_MM    1000U  /* lead lost closer than this = careful */

#define CMD_SEARCH        700    /* nothing seen nearby: 70 %           */
#define CMD_FOLLOW        700    /* max forward normally: 70 %          */
#define CMD_BOOST         1000   /* max forward while lead pulls away    */
#define BOOST_V_MMS       50     /* boost when gap grows over 5 cm/s ... */
#define BOOST_MIN_MM      400    /* ... and the gap is over 40 cm        */
#define CMD_MIN           450    /* slowest forward: train breaks away   */
#define CMD_REVERSE       1000   /* max reverse: 100 %                   */
#define CMD_REVERSE_MIN   350    /* retreat starts at 35 %               */
#define REVERSE_RAMP      75     /* +7.5 % per tick while retreating     */
#define REVERSE_BRAKE_TICKS 2    /* brake 100 ms before reversing        */
#define BLIND_CRAWL_TICKS 40     /* lead lost nearby: crawl at most 2 s  */
#define BLIND_WAIT_TICKS  40     /* ... then wait braked 2 s, then creep */
#define CMD_RAMP          300    /* max speed-up per tick (30 %)         */
#define CMD_FWD_START     450    /* first kick from standstill: 45 %     */
/* Calibration tests (CAL_MODE 0 = normal driving):
   1 = dead band sweep: +2.5 % every 1 s from 0 to 70 %, then brake.
       Note cmd when the train starts rolling steadily.
   2 = 3 s at 100 %, then brake.
   3 = stopping distance: drive at CAL_CMD towards a box, brake at
       CAL_BRAKE_MM. Stopping distance = CAL_BRAKE_MM - smallest d_filt. */
#define CAL_MODE          0
#define CAL_CMD           1000   /* test 3 speed: run 600, 800 and 1000  */
#define CAL_BRAKE_MM      600    /* test 3: brake when box is 60 cm away */

/* Gap holding */
#define D_HOLD_MM         250    /* target gap: 25 cm                    */
#define HOLD_BAND_MM      20     /* halt when within +-2 cm of target    */
#define HOLD_V_MMS        40     /* ... and gap changing under 4 cm/s    */
#define KP_NUM            40     /* gap error gain: 4.0 per mm           */
#define KP_DEN            10
#define KP_REV_NUM        48     /* gain when too close: 4.8 per mm      */
#define KD_NUM            3      /* closing speed gain: 3 per mm/s       */
#define KD_DEN            1
#define KI_NUM            1      /* how fast it learns the lead's speed  */
#define KI_DEN            1
#define I_ZONE_MM         100    /* learn only within 10 cm of target    */
#define V_WINDOW          6      /* closing speed over 6 ticks = 300 ms  */
#define HOLD_EXIT_MM      40     /* leave hold only beyond +-4 cm ...    */
#define HOLD_EXIT_V_MMS   100    /* ... or gap changing over 10 cm/s     */
#define REACQ_TICKS       3      /* target appeared: no forward 150 ms   */
#define JUMP_MM           150    /* ... or jumped more than 15 cm        */
#define APPEAR_TICKS      20     /* popped up close: brake, no reverse
                                    for 1 s unless clearly approaching   */
#define UNSTEADY_JUMPS    2      /* jumps in 0.5 s = readings unsteady   */
#define CMD_UNSTEADY      500    /* ... then roll on steadily, max 50 %  */
#define D_CLOSE_MM        600    /* ... but a target this close ...      */
#define CLOSE_AGREE_MM    50     /* ... in 2 agreeing readings is real   */
#define D_DFADE_NEAR_MM   600    /* closing speed brakes fully below this */
#define D_DFADE_FAR_MM    1500   /* ... and not at all beyond this        */

/* Flight recorder in flash: two 128 KB sectors. Hold the blue button
   while pressing RESET to print them. A log is never overwritten before
   it has been printed: if both are unprinted, nothing new is recorded. */
#define LOG_SECTOR_A      FLASH_SECTOR_6
#define LOG_ADDR_A        0x08040000U
#define LOG_SECTOR_B      FLASH_SECTOR_7
#define LOG_ADDR_B        0x08060000U
#define LOG_SIZE          0x20000U      /* 128 KB = 16382 ticks = 13.6 min */
#define LOG_MAGIC         0x4C4F4732U   /* "LOG2": log with yaw column     */
#define LOG_FLAG_OFS      (LOG_SIZE - 4U) /* last word: 0 = already printed */

/* Gyro: DFRobot BMI160 on I2C, PB8 = SCL (D15), PB9 = SDA (D14).
   I2C is done in software here, so CubeMX needs no change. */
#define BMI_ADDR_A        0x69U  /* default on the DFRobot board         */
#define BMI_ADDR_B        0x68U  /* if SDO is tied to GND                */
#define BMI_CHIP_ID       0xD1U
#define GYRO_LSB_X10      656    /* +-500 dps range: 65.6 LSB per deg/s  */
#define CURVE_ENTER_DPS   150    /* in 0.1 deg/s: 15 deg/s = in a curve  */
#define CURVE_EXIT_DPS    80     /* below 8 deg/s while driving = straight */
#define CURVE_ENTER_TICKS 2      /* 100 ms above the threshold to enter  */
#define CURVE_EXIT_TICKS  4      /* 0.2 s straight driving to leave      */
#define CMD_CURVE         450    /* max forward in a curve: 45 %         */
#define CURVE_RESTART_TICKS 20   /* after a stop in a curve: drive 1 s
                                    before "straight" may be judged      */
/* Sonar servo (MG90S) on D7 = PA8, TIM1 CH1, 50 Hz. Turns the sensor
   towards the inside of a curve: angle = yaw x gain, limited. */
#define SERVO_ENABLE      1      /* 0 = servo held straight ahead        */
#define SERVO_TEST        0      /* 1 = TEST: swing -30 / 0 / +30 deg every
                                    1 s, ignores the gyro. Set back to 0! */
#define SERVO_CENTER_US   1500   /* pulse for "straight ahead": trim it  */
#define SERVO_US_PER_DEG  10     /* MG90S: about 10 us per degree        */
#define SERVO_DIR         1      /* turns the wrong way? set to -1       */
#define SERVO_GAIN_X10    5      /* 0.5 deg of servo per deg/s of yaw    */
#define SERVO_MAX_DEG     35     /* never turn further than this         */
#define SERVO_STEP_DEG    8      /* max change per 50 ms tick (smooth)   */
#define SERVO_RETURN_STEP_DEG 25 /* back to straight: fast, 25 deg/tick  */
#define CURVE_STOP_MM     350    /* in a curve, ONE reading closer than
                                    this brakes at once (no filtering)  */

/* TEST ONLY: 1 = ignore the ultrasonic sensor completely, the train
   always "sees nothing ahead": 70 % on straights, 45 % in curves.
   For gyro laps without the lead. NO collision protection!
   Set back to 0 for normal driving. */
#define TEST_NO_SONAR     0
#define GYRO_FAIL_TICKS   10     /* 0.5 s of read errors = gyro lost     */
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
static char     ctrl_state = 'S';    /* C search, A forward, R reverse, H hold,
                                        S stop, B blind crawl, W wait,
                                        U unsteady readings, L sensor lost */

/* Flight recorder */
static uint32_t log_addr  = 0;       /* next free flash address, 0 = off */
static uint32_t log_limit = 0;       /* end of the sector in use         */

/* Why the board started, from RCC->CSR (bits 25..31), read at boot */
static uint8_t  reset_cause = 0;

/* Sonar servo */
static int32_t  servo_deg = 0;       /* current servo angle, + = left     */

/* Gyro and curve detection */
static uint8_t  bmi_addr  = 0;       /* I2C address found, 0 = no gyro    */
static int32_t  gyro_bias = 0;       /* raw zero offset, measured at boot */
static uint32_t gyro_restarts = 0;   /* times the gyro had to be revived  */
static int32_t  yaw_dps10 = 0;       /* filtered turn rate, 0.1 deg/s     */
static uint8_t  in_curve  = 0;       /* 1 = train is in a curve           */
static int32_t  fwd_cap   = 1000;    /* forward limit used by motor_drive */
static uint32_t i2c_half  = 0;       /* CPU cycles per half clock, 5 us   */
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
  if (cmd > fwd_cap) cmd = fwd_cap;   /* in a curve: slower forward */

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

/* Reset cause as text. Power-on = power was off (e.g. power bank cut
   out), reset pin = RESET button or debugger, brown-out = voltage dip. */
static const char *reset_name(uint8_t rc)
{
  if (rc & 0x40U) return "low-power reset";
  if (rc & 0x20U) return "window watchdog";
  if (rc & 0x10U) return "watchdog";
  if (rc & 0x08U) return "software reset";
  if (rc & 0x04U) return "power-on (power was off)";
  if (rc & 0x01U) return "brown-out (voltage dip)";
  if (rc & 0x02U) return "reset pin (button / debugger)";
  return "unknown";
}

/* ---- Flight recorder ------------------------------------------------ */

/* Sequence number of the log in a sector, 0 if there is none. */
static uint32_t log_seq(uint32_t base)
{
  const uint32_t *p = (const uint32_t *)base;
  return (p[0] == LOG_MAGIC) ? (p[1] & 0x00FFFFFFU) : 0U;  /* top byte: reset cause */
}

/* 1 if the sector holds a log that has not been printed yet. */
static uint8_t log_unread(uint32_t base)
{
  return (log_seq(base) != 0U &&
          *(const uint32_t *)(base + LOG_FLAG_OFS) == 0xFFFFFFFFU) ? 1U : 0U;
}

/* New run: erase the older of the two sectors and record there, unless
   both logs are still unprinted (then record nothing, to keep them).
   Returns 1 if recording. Erasing takes 1 to 2 s. */
static uint8_t log_start(void)
{
  uint32_t sa = log_seq(LOG_ADDR_A);
  uint32_t sb = log_seq(LOG_ADDR_B);
  uint32_t newest = (sa > sb) ? sa : sb;
  uint8_t  use_a  = (sa <= sb) ? 1U : 0U;          /* A is older (or empty) */
  uint32_t base   = use_a ? LOG_ADDR_A : LOG_ADDR_B;

  if (log_unread(LOG_ADDR_A) && log_unread(LOG_ADDR_B))
  {
    return 0U;                                     /* keep both, no recording */
  }

  FLASH_EraseInitTypeDef er = {0};
  uint32_t bad = 0;
  er.TypeErase    = FLASH_TYPEERASE_SECTORS;
  er.Sector       = use_a ? LOG_SECTOR_A : LOG_SECTOR_B;
  er.NbSectors    = 1;
  er.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase(&er, &bad) == HAL_OK)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base,      LOG_MAGIC);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base + 4U,
                      (newest + 1U) | ((uint32_t)reset_cause << 24));
    log_addr  = base + 8U;
    log_limit = base + LOG_FLAG_OFS;               /* keep the flag word free */
  }
  HAL_FLASH_Lock();
  return (log_addr != 0U) ? 1U : 0U;
}

/* One entry per tick, 8 bytes. Stops quietly when the sector is full. */
static void log_write(uint32_t d_raw, uint32_t d_f, char st, char md, int32_t c,
                      int32_t yaw)
{
  if (log_addr == 0U || log_addr + 8U > log_limit) return;
  if (d_raw > 0x3FFFU) d_raw = 0x3FFFU;            /* 14 bits, max 16 m   */
  if (yaw >  127) yaw =  127;                      /* deg/s, 1 byte,      */
  if (yaw < -128) yaw = -128;                      /* -128 = no gyro      */
  uint32_t mc = (md == 'D') ? 2U : (md == 'B') ? 1U : 0U;
  uint32_t w0 = (d_f & 0xFFFFU) | ((uint32_t)(uint16_t)(int16_t)c << 16);
  uint32_t w1 = d_raw | (mc << 14) | ((uint32_t)(uint8_t)st << 16)
                      | ((uint32_t)(uint8_t)(int8_t)yaw << 24);
  HAL_FLASH_Unlock();
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, log_addr,      w0);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, log_addr + 4U, w1);
  HAL_FLASH_Lock();
  log_addr += 8U;
}

/* Print one stored log as CSV. */
static void log_dump_one(uint32_t base)
{
  char line[128];
  uint32_t seq = log_seq(base);
  if (seq == 0U) return;
  uint8_t rc = (uint8_t)(*(const uint32_t *)(base + 4U) >> 24);
  int n = snprintf(line, sizeof line, "# log %lu, started by: %s\r\nt_ms,d_raw,d_filt,state,mode,cmd,yaw\r\n",
                   (unsigned long)seq, reset_name(rc));
  HAL_UART_Transmit(&huart2, (uint8_t *)line, n, 100);
  uint32_t i = 0;
  for (uint32_t a = base + 8U; a + 8U <= base + LOG_FLAG_OFS; a += 8U, i++)
  {
    uint32_t w0 = *(const uint32_t *)a;
    uint32_t w1 = *(const uint32_t *)(a + 4U);
    if (w0 == 0xFFFFFFFFU && w1 == 0xFFFFFFFFU) break;   /* end of log */
    uint32_t mc = (w1 >> 14) & 3U;
    n = snprintf(line, sizeof line, "%lu,%u,%u,%c,%c,%d,%d\r\n",
                 (unsigned long)(i * 50U), (unsigned)(w1 & 0x3FFFU),
                 (unsigned)(w0 & 0xFFFFU), (char)((w1 >> 16) & 0xFFU),
                 (mc == 2U) ? 'D' : (mc == 1U) ? 'B' : 'F',
                 (int)(int16_t)(w0 >> 16), (int)(int8_t)(w1 >> 24));
    HAL_UART_Transmit(&huart2, (uint8_t *)line, n, 100);
  }
}

/* Print both stored logs, older first, the most recent run last. */
static void log_dump(void)
{
  if (log_seq(LOG_ADDR_A) <= log_seq(LOG_ADDR_B))
  {
    log_dump_one(LOG_ADDR_A);  log_dump_one(LOG_ADDR_B);
  }
  else
  {
    log_dump_one(LOG_ADDR_B);  log_dump_one(LOG_ADDR_A);
  }
  HAL_UART_Transmit(&huart2, (uint8_t *)"# end\r\n", 7, 100);

  /* both printed now: mark them, so the next runs may overwrite them */
  HAL_FLASH_Unlock();
  if (log_unread(LOG_ADDR_A))
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, LOG_ADDR_A + LOG_FLAG_OFS, 0U);
  if (log_unread(LOG_ADDR_B))
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, LOG_ADDR_B + LOG_FLAG_OFS, 0U);
  HAL_FLASH_Lock();
}

/* Blue user button B1 on PC13, pressed = low. */
static uint8_t blue_button_held(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  g.Pin  = GPIO_PIN_13;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &g);
  HAL_Delay(5);
  return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) ? 1U : 0U;
}

/* ---- Gyro: software I2C + BMI160 ----------------------------------- */

/* Open-drain lines: writing 1 releases the line (pull-up makes it high),
   writing 0 pulls it low. */
#define I2C_SCL  GPIO_PIN_8
#define I2C_SDA  GPIO_PIN_9
#define SCL_H()  (GPIOB->BSRR = I2C_SCL)
#define SCL_L()  (GPIOB->BSRR = (uint32_t)I2C_SCL << 16)
#define SDA_H()  (GPIOB->BSRR = I2C_SDA)
#define SDA_L()  (GPIOB->BSRR = (uint32_t)I2C_SDA << 16)
#define SDA_IN() ((GPIOB->IDR & I2C_SDA) ? 1U : 0U)

static void i2c_delay(void)
{
  uint32_t t0 = DWT->CYCCNT;
  while ((DWT->CYCCNT - t0) < i2c_half) { }
}

static void i2c_start(void)
{
  SDA_H();  SCL_H();  i2c_delay();
  SDA_L();  i2c_delay();
  SCL_L();  i2c_delay();
}

static void i2c_stop(void)
{
  SDA_L();  i2c_delay();
  SCL_H();  i2c_delay();
  SDA_H();  i2c_delay();
}

/* Sends one byte, returns 1 if the chip answered (ACK). */
static uint8_t i2c_write(uint8_t b)
{
  for (int i = 7; i >= 0; i--)
  {
    if (b & (1U << i)) SDA_H(); else SDA_L();
    i2c_delay();  SCL_H();  i2c_delay();  SCL_L();
  }
  SDA_H();  i2c_delay();                   /* let the chip drive SDA */
  SCL_H();  i2c_delay();
  uint8_t ack = (SDA_IN() == 0U) ? 1U : 0U;
  SCL_L();  i2c_delay();
  return ack;
}

/* Reads one byte; ack = 1 asks for more, 0 ends the read. */
static uint8_t i2c_read(uint8_t ack)
{
  uint8_t b = 0;
  SDA_H();
  for (int i = 0; i < 8; i++)
  {
    i2c_delay();  SCL_H();  i2c_delay();
    b = (uint8_t)((b << 1) | SDA_IN());
    SCL_L();
  }
  if (ack) SDA_L(); else SDA_H();
  i2c_delay();  SCL_H();  i2c_delay();  SCL_L();
  SDA_H();  i2c_delay();
  return b;
}

static uint8_t bmi_write(uint8_t reg, uint8_t val)
{
  i2c_start();
  uint8_t ok = i2c_write((uint8_t)(bmi_addr << 1)) && i2c_write(reg) && i2c_write(val);
  i2c_stop();
  return ok;
}

static uint8_t bmi_read(uint8_t reg, uint8_t *buf, uint8_t n)
{
  i2c_start();
  if (!i2c_write((uint8_t)(bmi_addr << 1)) || !i2c_write(reg))
  {
    i2c_stop();  return 0;
  }
  i2c_start();                             /* repeated start: now read */
  if (!i2c_write((uint8_t)((bmi_addr << 1) | 1U)))
  {
    i2c_stop();  return 0;
  }
  for (uint8_t i = 0; i < n; i++) buf[i] = i2c_read(i + 1U < n);
  i2c_stop();
  return 1;
}

/* Raw gyro Z (turn left/right when mounted flat). Returns 1 if read. */
static uint8_t gyro_read_z(int32_t *raw)
{
  uint8_t b[2];
  if (!bmi_read(0x10, b, 2)) return 0;     /* GYR_Z low, high byte */
  *raw = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
  return 1;
}

/* Finds the BMI160, turns the gyro on and measures its zero offset.
   The train must stand still for this second. Returns 1 if found. */
static uint8_t gyro_init(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  i2c_half = SystemCoreClock / 200000U;    /* 5 us = 100 kHz clock */
  SCL_H();  SDA_H();
  g.Pin   = I2C_SCL | I2C_SDA;
  g.Mode  = GPIO_MODE_OUTPUT_OD;
  g.Pull  = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &g);

  for (int i = 0; i < 9; i++)              /* free a chip stuck mid-byte */
  {
    SCL_L();  i2c_delay();  SCL_H();  i2c_delay();
  }
  i2c_stop();

  /* look for the chip, several tries: after power-up it may need time */
  const uint8_t addrs[2] = { BMI_ADDR_A, BMI_ADDR_B };
  uint8_t id = 0;
  for (int t = 0; t < 10 && !bmi_addr; t++)
  {
    for (int k = 0; k < 2; k++)
    {
      bmi_addr = addrs[k];
      if (bmi_read(0x00, &id, 1) && id == BMI_CHIP_ID) break;
      bmi_addr = 0;
    }
    if (!bmi_addr) HAL_Delay(50);
  }
  if (!bmi_addr) return 0;

  bmi_write(0x7E, 0xB6);  HAL_Delay(20);   /* soft reset              */

  /* gyro to normal mode, and check that it really is (PMU_STATUS) */
  uint8_t pmu = 0;
  for (int t = 0; t < 5; t++)
  {
    bmi_write(0x7E, 0x15);  HAL_Delay(100);
    if (bmi_read(0x03, &pmu, 1) && ((pmu >> 2) & 3U) == 1U) break;
  }
  if (((pmu >> 2) & 3U) != 1U) { bmi_addr = 0;  return 0; }

  bmi_write(0x43, 0x02);                   /* range +-500 deg/s       */
  bmi_write(0x42, 0x28);                   /* 100 Hz, normal filter   */
  HAL_Delay(20);

  int32_t sum = 0, raw = 0, n = 0;
  for (int i = 0; i < 20; i++)             /* 1 s standing still      */
  {
    if (gyro_read_z(&raw)) { sum += raw;  n++; }
    HAL_Delay(50);
  }
  if (n < 10) { bmi_addr = 0;  return 0; }
  gyro_bias = sum / n;
  return 1;
}

/* ---- Sonar servo on PA8 (TIM1 CH1), set up here, no CubeMX ------- */

static void servo_write_deg(int32_t deg)
{
  if (deg >  SERVO_MAX_DEG) deg =  SERVO_MAX_DEG;
  if (deg < -SERVO_MAX_DEG) deg = -SERVO_MAX_DEG;
  TIM1->CCR1 = (uint32_t)(SERVO_CENTER_US + SERVO_DIR * deg * SERVO_US_PER_DEG);
}

static void servo_init(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  g.Pin       = GPIO_PIN_8;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &g);

  uint32_t clk = HAL_RCC_GetPCLK2Freq();          /* TIM1 runs on APB2 */
  if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) clk *= 2U;  /* x2 if divided */
  TIM1->PSC   = clk / 1000000U - 1U;               /* 1 us per count   */
  TIM1->ARR   = 20000U - 1U;                       /* 20 ms = 50 Hz    */
  TIM1->CCR1  = SERVO_CENTER_US;
  TIM1->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;  /* PWM 1 */
  TIM1->CCER  = TIM_CCER_CC1E;
  TIM1->BDTR  = TIM_BDTR_MOE;                      /* TIM1: outputs on */
  TIM1->EGR   = TIM_EGR_UG;
  TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;
}

/* Every tick: point the sonar towards the inside of the curve.
   Only follows the gyro while driving forward: stopped or reversing
   in a curve, it keeps its angle (yaw is then 0 or the wrong sign). */
static void servo_update(void)
{
  int32_t target = servo_deg;
#if SERVO_TEST
  {
    static const int8_t seq[4] = { 0, 30, 0, -30 };
    target = seq[(HAL_GetTick() / 1000U) % 4U];
  }
#elif SERVO_ENABLE
  /* In a curve the lead sits at the same angle all the way round, so
     the servo keeps the biggest angle seen in this curve. It does NOT
     follow the yaw back down when the train slows, stops or restarts;
     it only returns to straight ahead once the curve is over. */
  static int32_t peak = 0;
  if (!in_curve)
  {
    peak = 0;
    target = 0;                                    /* straight: ahead  */
  }
  else
  {
    if (motor_cmd > 0)                             /* learn only while */
    {                                              /* driving forward  */
      int32_t a  = yaw_dps10 * SERVO_GAIN_X10 / 100;   /* 0.1 dps -> deg */
      int32_t aa = (a < 0) ? -a : a;
      int32_t ap = (peak < 0) ? -peak : peak;
      if ((a > 0 && peak < 0) || (a < 0 && peak > 0))
      {
        if (aa >= 10) peak = a;                    /* S-bend: other way */
      }
      else if (aa > ap)
      {
        peak = a;                                  /* sharper: turn more */
      }
    }
    target = peak;
  }
#else
  target = 0;
#endif
  /* turning in: smooth; going back towards straight: fast */
  int32_t step = (target == 0) ? SERVO_RETURN_STEP_DEG : SERVO_STEP_DEG;
  if (target > servo_deg + step) target = servo_deg + step;
  if (target < servo_deg - step) target = servo_deg - step;
  if (target >  SERVO_MAX_DEG) target =  SERVO_MAX_DEG;
  if (target < -SERVO_MAX_DEG) target = -SERVO_MAX_DEG;
  servo_deg = target;
  servo_write_deg(servo_deg);
}

/* Every tick: read the gyro and decide straight or curve. */
static void curve_update(void)
{
  static uint8_t fails = 0, above = 0, straight = 0;
  int32_t raw;

  if (!bmi_addr) { in_curve = 0;  return; }     /* no gyro: drive as before */

  /* how long we have been driving forward without a stop. Starting
     again after a stop in a curve, the train is slow and the yaw is
     small, which must not be mistaken for "straight track" */
  /* Gyro watchdog. Two ways the BMI160 "dies" while still answering:
     a) a short power or wiring glitch resets it into sleep mode
        (PMU_STATUS no longer "normal"): checked every 0.5 s;
     b) its data freezes: the raw value never changes, while a live
        gyro always jitters by a few counts: checked every tick.
     Either way: switch the gyro on again (about 0.1 s). */
  static uint8_t watch = 0, frozen = 0;
  static int32_t last_raw = 0x7FFFFFFF;
  uint8_t revive = 0;
  if (++watch >= 10U)
  {
    uint8_t pmu = 0;
    watch = 0;
    if (bmi_read(0x03, &pmu, 1) && ((pmu >> 2) & 3U) != 1U) revive = 1;
  }
  if (frozen >= 20U) revive = 1;               /* 1 s the exact same value */
  if (revive)
  {
    bmi_write(0x7E, 0x15);                     /* gyro: normal mode again */
    bmi_write(0x43, 0x02);                     /* range +-500 deg/s       */
    bmi_write(0x42, 0x28);                     /* 100 Hz, normal filter   */
    gyro_restarts++;
    frozen = 0;
  }

  static uint8_t moving_t = 0;
  if (motor_cmd > 0) { if (moving_t < 255U) moving_t++; }
  else               { moving_t = 0; }

  if (!gyro_read_z(&raw))
  {
    if (fails < 255U) fails++;
    if (fails >= GYRO_FAIL_TICKS) in_curve = 1;  /* gyro lost: be careful   */
    return;
  }
  fails = 0;
  if (raw == last_raw) { if (frozen < 255U) frozen++; }
  else                 { frozen = 0; }
  last_raw = raw;

  int32_t now = (raw - gyro_bias) * 100 / GYRO_LSB_X10;  /* 0.1 deg/s */
  yaw_dps10 = (yaw_dps10 + now) / 2;                     /* smooth    */
  int32_t a = (yaw_dps10 < 0) ? -yaw_dps10 : yaw_dps10;

  if (!in_curve)
  {
    above = (a > CURVE_ENTER_DPS) ? (uint8_t)(above + 1U) : 0U;
    if (above >= CURVE_ENTER_TICKS) { in_curve = 1;  straight = 0; }
  }
  else if (motor_cmd > 0 && moving_t >= CURVE_RESTART_TICKS)
  {                                        /* only judge while rolling
                                              forward for 1 s: stopped or
                                              just restarted in a curve
                                              reads ~0 but is still in it */
    straight = (a < CURVE_EXIT_DPS) ? (uint8_t)(straight + 1U) : 0U;
    if (straight >= CURVE_EXIT_TICKS) { in_curve = 0;  above = 0; }
  }
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

  if (fresh)
  {
    if (e_us < ECHO_MIN_US)               /* closer than 2 cm: count as 2 cm */
    {
      e_us = ECHO_MIN_US;
    }
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

/* One control step: hold the gap at D_HOLD_MM, forward or backward. */
static void control_step(uint8_t sensor_ok)
{
  static int32_t  cmd       = 0;           /* command sent last tick          */
  static int32_t  i_sum     = 0;           /* integral, x10: learns lead speed */
  static uint32_t blind_t   = 0;           /* ticks since lead vanished       */
  static uint32_t last_seen = D_CLEAR_MM;  /* last distance to the lead       */
  static uint8_t  flip_t    = 0;           /* brake ticks left before a turn  */
  static uint32_t hist[V_WINDOW + 1];      /* recent distances, newest first  */
  static uint8_t  hist_n    = 0;           /* how many of them are valid      */
  static uint8_t  holding   = 0;           /* 1 = standing still at the gap   */
  static uint32_t prev_d    = D_CLEAR_MM;  /* distance last tick              */
  static uint8_t  reacq_t   = 0;           /* ticks of "no forward" left      */
  static uint8_t  appear_t  = 0;           /* ticks of "no reverse" left      */
  static uint32_t last_d    = D_CLEAR_MM;  /* any reading last tick           */
  static uint16_t jumps     = 0;           /* 1 bit per tick: reading jumped  */
  static uint8_t  close_n   = 0;           /* steady close readings in a row  */

  if (!sensor_ok)                          /* sensor dead: stop */
  {
    ctrl_state = 'L';  cmd = 0;  i_sum = 0;  hist_n = 0;
    holding = 0;  motor_brake();  return;
  }

  uint32_t d = d_filt_mm;

  /* Unsteady readings (curves: the sensor sweeps over walls, chairs,
     nothing): if the reading jumped more than JUMP_MM twice in the last
     0.5 s, ignore it and keep rolling steadily, unless a target is
     consistently close (two agreeing readings under D_CLOSE_MM). */
  uint32_t dd = (d > last_d) ? d - last_d : last_d - d;
  jumps = (uint16_t)(((jumps << 1) | (dd > JUMP_MM ? 1U : 0U)) & 0x3FFU);
  if (d < D_CLOSE_MM && dd <= CLOSE_AGREE_MM)
  {
    if (close_n < 255U) close_n++;
  }
  else
  {
    close_n = 0;
  }
  last_d = d;

  uint8_t n_jumps = 0;
  for (uint16_t m = jumps; m; m >>= 1) n_jumps += (uint8_t)(m & 1U);

  /* in a curve, a close reading is believed at once: it is most likely
     the lead appearing around the bend */
  uint8_t curve_close = (in_curve && d < CURVE_STOP_MM) ? 1U : 0U;

  if (n_jumps >= UNSTEADY_JUMPS && close_n < 2U && !curve_close)
  {
    hist_n = 0;  i_sum = 0;  holding = 0;  flip_t = 0;
    prev_d = D_CLEAR_MM;                   /* treat what follows as new  */
    if (cmd < 0)                           /* never roll on in reverse   */
    {
      cmd = 0;  ctrl_state = 'U';  motor_brake();  return;
    }
    int32_t target = (cmd > CMD_UNSTEADY) ? CMD_UNSTEADY : cmd;
    if (target < CMD_MIN) target = CMD_MIN;  /* keep rolling            */
    cmd = target;  ctrl_state = 'U';
    motor_drive(cmd);
    return;
  }

  /* A. Lead not in sight: forward only, as before */
  if (d >= D_CLEAR_MM)
  {
    hist_n = 0;  i_sum = 0;  holding = 0;  prev_d = D_CLEAR_MM;
    if (cmd < 0)                           /* never turn reverse into search */
    {
      cmd = 0;  ctrl_state = 'S';  motor_brake();  return;
    }
    int32_t target;
    if (last_seen < D_NEAR_LOST_MM)        /* it vanished close by */
    {
      if (blind_t < BLIND_CRAWL_TICKS)     /* 1. crawl on for 2 s       */
      {
        blind_t++;  target = CMD_MIN;  ctrl_state = 'B';
      }
      else if (blind_t < BLIND_CRAWL_TICKS + BLIND_WAIT_TICKS)
      {                                    /* 2. wait braked for 2 s    */
        blind_t++;  ctrl_state = 'W';  cmd = 0;
        motor_brake();  return;
      }
      else                                 /* 3. lead not back: creep on
                                                 at crawl speed, never wait
                                                 forever (it would lap us) */
      {
        target = CMD_MIN;  ctrl_state = 'C';
      }
    }
    else
    {                                      /* in a curve, "nothing ahead" may
                                              be the lead around the bend   */
      target = in_curve ? CMD_MIN : CMD_SEARCH;  ctrl_state = 'C';
    }
    if (target > cmd + CMD_RAMP) target = cmd + CMD_RAMP;
    cmd = target;
    motor_drive(cmd);
    return;
  }

  /* B. Lead in sight: hold the gap */
  last_seen = d;  blind_t = 0;

  /* target just appeared, or jumped much closer: we don't know yet how
     fast we are closing in, so no forward drive for a moment */
  if (prev_d >= D_CLEAR_MM)                /* popped up out of nothing */
  {
    appear_t = APPEAR_TICKS;
  }
  else if (appear_t)
  {
    appear_t--;
  }
  if (prev_d >= D_CLEAR_MM || d + JUMP_MM < prev_d || d > prev_d + JUMP_MM)
  {
    hist_n  = 0;                           /* old readings: other target */
    if (d < D_NEAR_LOST_MM && (prev_d >= D_CLEAR_MM || d < prev_d))
    {
      reacq_t = REACQ_TICKS;               /* new or closer, within 1 m: */
    }                                      /* no forward for a moment    */
  }
  else if (reacq_t)                        /* count down every tick, */
  {                                        /* also while holding     */
    reacq_t--;
  }
  prev_d = d;

  /* closing speed over the last 300 ms, mm/s (+ = gap growing) */
  for (int i = V_WINDOW; i > 0; i--) hist[i] = hist[i - 1];
  hist[0] = d;
  if (hist_n <= V_WINDOW) hist_n++;
  int32_t v = 0;
  uint8_t n = (hist_n > V_WINDOW) ? V_WINDOW : (uint8_t)(hist_n - 1);
  if (n >= 2)                              /* usable after 3 readings */
  {
    v = ((int32_t)hist[0] - (int32_t)hist[n]) * 1000 / ((int32_t)n * 50);
  }

  int32_t e = (int32_t)d - D_HOLD_MM;      /* + = too far, - = too close */

  /* integral: keeps adding while the gap is off, so it ends up
     matching the lead's speed (forward or backward) */
  if (abs(e) <= I_ZONE_MM)                 /* learn only near the target gap */
  {
    i_sum += e * KI_NUM / KI_DEN;
  }
  /* clearly on the wrong side of the target, but the integral still
     pushes the other way: forget it (anti-windup) */
  if ((e < -HOLD_BAND_MM && i_sum > 0) || (e > HOLD_BAND_MM && i_sum < 0))
  {
    i_sum = 0;
  }
  if (i_sum >  CMD_FOLLOW  * 10) i_sum =  CMD_FOLLOW  * 10;
  if (i_sum < -CMD_REVERSE * 10) i_sum = -CMD_REVERSE * 10;

  /* hold with hysteresis: enter when close to the target and calm,
     leave only on a clear change, so sensor jitter can't wake it up */
  if (holding)
  {
    if (abs(e) > HOLD_EXIT_MM || abs(v) > HOLD_EXIT_V_MMS)
    {
      holding = 0;                         /* real change: start moving */
    }
  }
  else if (abs(e) <= HOLD_BAND_MM && abs(v) <= HOLD_V_MMS &&
           abs(i_sum) < CMD_MIN * 10)
  {
    holding = 1;                           /* at the gap, lead still: halt */
  }
  if (holding)
  {
    ctrl_state = 'H';  cmd = 0;  i_sum = 0;  flip_t = 0;
    motor_brake();  return;
  }

  /* gap error + closing speed + learned speed -> command */
  int32_t kp = (e < 0) ? KP_REV_NUM : KP_NUM;   /* react harder when too close */
  /* closing speed: brakes fully when the target is near, fades out
     between D_DFADE_NEAR and D_DFADE_FAR. Far away, a quickly shrinking
     reading is mostly the sensor swinging across the room (curves). */
  int32_t kd_v = v * KD_NUM / KD_DEN;
  if (kd_v < 0 && d > D_DFADE_NEAR_MM)
  {
    kd_v = (d >= D_DFADE_FAR_MM) ? 0
         : kd_v * (int32_t)(D_DFADE_FAR_MM - d) / (int32_t)(D_DFADE_FAR_MM - D_DFADE_NEAR_MM);
  }
  int32_t want = e * kp / KP_DEN + kd_v + i_sum / 10;
  if (e < -HOLD_BAND_MM && want > 0)       /* too close: never forward */
  {
    want = 0;
  }
  if (e >= -HOLD_BAND_MM && want < 0)      /* not too close: brake at most, */
  {                                        /* never reverse                 */
    want = 0;
  }
  if (reacq_t && want > 0)                 /* just appeared: no forward yet */
  {
    want = 0;
  }
  /* popped up close out of nothing: could be a stray echo. Brake for it,
     but only reverse once it is clearly coming closer. */
  if (appear_t && want < 0 && v > -HOLD_EXIT_V_MMS)
  {
    want = 0;
  }
  /* top speed: CMD_FOLLOW normally; full boost only while the lead is
     visibly pulling away (gap growing), never towards a target that
     is not moving away, like a wall or a hidden, parked lead */
  int32_t top = (v > BOOST_V_MMS && d > BOOST_MIN_MM) ? CMD_BOOST : CMD_FOLLOW;
  if (want >  top)         want =  top;
  if (want < -CMD_REVERSE) want = -CMD_REVERSE;
  if (want > 0 && want <  CMD_MIN)         want =  CMD_MIN;          /* enough */
  if (want < 0 && want > -CMD_REVERSE_MIN) want = -CMD_REVERSE_MIN;  /* to move */

  /* direction change: brake 100 ms first, never slam into reverse */
  if ((want > 0 && cmd < 0) || (want < 0 && cmd > 0))
  {
    flip_t = REVERSE_BRAKE_TICKS;  cmd = 0;
  }
  if (flip_t)
  {
    flip_t--;  ctrl_state = (want > 0) ? 'A' : 'R';
    motor_brake();  return;
  }

  /* build up gently, ease off at once */
  if (want > 0 && want > cmd + CMD_RAMP)
  {
    want = (cmd < CMD_MIN) ? CMD_FWD_START : cmd + CMD_RAMP;
    if (want < CMD_MIN) want = CMD_MIN;
  }
  if (want < 0 && want < cmd - REVERSE_RAMP)
  {
    want = (cmd > -CMD_REVERSE_MIN) ? -CMD_REVERSE_MIN : cmd - REVERSE_RAMP;
  }

  cmd = want;
  if (cmd > 0)      { ctrl_state = 'A';  motor_drive(cmd); }
  else if (cmd < 0) { ctrl_state = 'R';  motor_drive(cmd); }
  else              { ctrl_state = 'H';  motor_brake(); }
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

  /* Why did we start? Read, then clear the flags for next time. */
  reset_cause = (uint8_t)((RCC->CSR >> 25) & 0x7FU);
  RCC->CSR |= RCC_CSR_RMVF;

  /* Flight recorder: blue button held during reset -> print the stored
     logs and stop (LD2 blinks fast). Otherwise start a new log. */
  motor_float();
  if (blue_button_held())
  {
    log_dump();
    while (1)
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(100);
    }
  }
  if (log_start())
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)"# recorder: on\r\n", 16, 100);
  }
  else
  {
    HAL_UART_Transmit(&huart2,
      (uint8_t *)"# recorder: OFF, 2 unprinted logs. Hold blue + RESET to print\r\n",
      63, 100);
  }

  /* CSV header, sent once. A blocking send is fine here:
     the control loop isn't running yet. */
  int n = snprintf(tx_buf, sizeof tx_buf,
          "t_ms,tick,overruns,tx_drops,work_us,echo_us,d_mm,fresh,d_filt,state,mode,cmd,yaw,servo,gyro_rst\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, n, 100);

  /* Motor PWM running from the start, but with 0 % duty and INH low */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  motor_float();

  /* Ultrasonic: start capturing echo edges first, so the first
     echo can't arrive before we're listening, then the trigger. */
  __HAL_TIM_SET_AUTORELOAD(&htim3, SONAR_PERIOD_US - 1U);  /* 65 ms pings */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* Servo: centre the sonar first, let it settle before the gyro
     measures its zero offset */
  servo_init();
  HAL_Delay(300);

  /* Gyro: train must stand still for 1 s now (zero offset) */
  if (gyro_init())
  {
    n = snprintf(tx_buf, sizeof tx_buf, "# gyro: BMI160 at 0x%02X, bias %ld\r\n",
                 (unsigned)bmi_addr, (long)gyro_bias);
  }
  else
  {
    n = snprintf(tx_buf, sizeof tx_buf, "# gyro: NOT FOUND, curve logic off\r\n");
  }
  HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, n, 100);

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
      if (k % (bmi_addr ? 10U : 2U) == 0)   /* no gyro: blinks fast, 5 Hz */
      {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }

      /* Controller: clean the reading, then decide the motor command */
      uint8_t sensor_ok = sensor_update(e_us, fresh);
      if (in_curve && fresh && d_mm < CURVE_STOP_MM)
      {
        d_filt_mm = d_mm;                  /* curve: skip the median, act now */
      }
      curve_update();                      /* gyro: straight or curve? */
      servo_update();                      /* point the sonar into the curve */
#if CAL_MODE == 1
      {
        int32_t c = (int32_t)(k / 20U) * 25;   /* +25 every 20 ticks = 1 s */
        if (c > 700) { ctrl_state = 'X';  motor_brake(); }    /* done   */
        else         { ctrl_state = 'K';  motor_drive(c); }
      }
#elif CAL_MODE == 2
      if (k < 60) { ctrl_state = 'K';  motor_drive(1000); }  /* 3 s full */
      else        { ctrl_state = 'X';  motor_brake(); }
#elif CAL_MODE == 3
      {
        static uint8_t braked = 0;
        if (k < 10 || braked || d_filt_mm <= CAL_BRAKE_MM)
        {
          if (k >= 10) braked = 1;             /* first 0.5 s: get readings */
          ctrl_state = 'X';  motor_brake();
        }
        else
        {
          ctrl_state = 'K';  motor_drive(CAL_CMD);
        }
      }
#else
      fwd_cap = in_curve ? CMD_CURVE : 1000;
#if TEST_NO_SONAR
      d_filt_mm = D_CLEAR_MM;              /* pretend: nothing ahead     */
      sensor_ok = 1;                       /* (raw d_mm is still logged) */
#endif
      control_step(sensor_ok);
#endif
      /* shown state: lower case while in a curve (a, c, h, ...) */
      char st = ctrl_state;
      if (in_curve && st >= 'A' && st <= 'Z') st = (char)(st + 32);
      int32_t yaw_log = bmi_addr ? yaw_dps10 / 10 : -128;   /* -128 = no gyro */
      log_write(d_mm, d_filt_mm, st, motor_mode, motor_cmd, yaw_log);

      /* Telemetry: one CSV line per tick, sent by DMA in the background */
      if (huart2.gState == HAL_UART_STATE_READY)
      {
          int len = snprintf(tx_buf, sizeof tx_buf,
                             "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%lu,%c,%c,%ld,%ld,%ld,%lu\r\n",
                             HAL_GetTick(), k, tick_overruns, tx_drops, work_us,
                             e_us, d_mm, (unsigned)fresh,
                             d_filt_mm, st, motor_mode, motor_cmd,
                             (long)yaw_log, (long)servo_deg,
                             (unsigned long)gyro_restarts);
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
        t += SONAR_PERIOD_US;              /* one timer period in µs */
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
