/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Balancing robot application
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <math.h>
#include <stdbool.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

/* Private define ------------------------------------------------------------*/
#define MPU_ADDR          (0x68u << 1)  // MPU6050, AD0 nối GND
#define CONTROL_MS        5u            // Vòng điều khiển mỗi 5 ms
#define TARGET_ANGLE      0.0f
#define ANGLE_LIMIT       35.0f
#define FILTER_ALPHA      0.98f

/* Giá trị khởi đầu, cần chỉnh theo robot thật */
#define KP                18.0f
#define KD                0.75f
#define MAX_DUTY          0.35f

/* Đổi thành -1 nếu motor tương ứng quay ngược chiều */
#define LEFT_SIGN         1.0f
#define RIGHT_SIGN        1.0f

/* Private variables ---------------------------------------------------------*/
static float angle_deg = 0.0f;
static bool angle_initialized = false;
static bool control_enabled = false;
static uint32_t last_control_ms = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

static bool MPU6050_Init(void);
static bool MPU6050_ReadAngle(float dt, float *angle, float *gyro_rate);
static void Motor_SetOne(float command, uint16_t in1, uint16_t in2,
                         uint32_t channel, float sign);
static void Motors_Stop(void);

void Error_Handler(void);

/* Private user code ---------------------------------------------------------*/
static bool MPU6050_WriteRegister(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &value, 1, 20) == HAL_OK;
}

static bool MPU6050_Init(void)
{
  /* Wake up MPU6050 */
  if (!MPU6050_WriteRegister(0x6B, 0x00))
    return false;

  HAL_Delay(100);

  /* Digital low-pass filter */
  if (!MPU6050_WriteRegister(0x1A, 0x03))
    return false;

  /* Gyro range: +/-250 degrees/second */
  if (!MPU6050_WriteRegister(0x1B, 0x00))
    return false;

  /* Accelerometer range: +/-2g */
  if (!MPU6050_WriteRegister(0x1C, 0x00))
    return false;

  angle_initialized = false;
  return true;
}

static bool MPU6050_ReadAngle(float dt, float *angle, float *gyro_rate)
{
  uint8_t data[14];

  if (angle == NULL || gyro_rate == NULL || dt <= 0.0f)
    return false;

  if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B,
                       I2C_MEMADD_SIZE_8BIT,
                       data, sizeof(data), 20) != HAL_OK)
    return false;

  int16_t ay = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  int16_t az = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  int16_t gx = (int16_t)(((uint16_t)data[8] << 8) | data[9]);

  /* Góc từ accelerometer và tốc độ quay từ gyro */
  float accel_angle = atan2f((float)ay, (float)az) * 57.29578f;
  float gyro_dps = (float)gx / 131.0f;

  if (!angle_initialized)
  {
    angle_deg = accel_angle;
    angle_initialized = true;
  }
  else
  {
    /* Complementary filter */
    angle_deg = FILTER_ALPHA * (angle_deg + gyro_dps * dt)
              + (1.0f - FILTER_ALPHA) * accel_angle;
  }

  *angle = angle_deg;
  *gyro_rate = gyro_dps;
  return true;
}

static void Motors_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_0 | GPIO_PIN_1 |
                    GPIO_PIN_10 | GPIO_PIN_12,
                    GPIO_PIN_RESET);
}

static void Motor_SetOne(float command,
                         uint16_t in1,
                         uint16_t in2,
                         uint32_t channel,
                         float sign)
{
  command *= sign;

  HAL_GPIO_WritePin(GPIOB, in1,
                    command >= 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, in2,
                    command >= 0.0f ? GPIO_PIN_RESET : GPIO_PIN_SET);

  float duty = fabsf(command);
  if (duty > MAX_DUTY)
    duty = MAX_DUTY;

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
  __HAL_TIM_SET_COMPARE(&htim2, channel, (uint32_t)(duty * arr));
}

/**
  * @brief  The application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();

  /* Start PWM for two motors */
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
    Error_Handler();

  Motors_Stop();

  /* Initialize MPU6050 */
  if (!MPU6050_Init())
    Error_Handler();

  last_control_ms = HAL_GetTick();
  control_enabled = true;

  while (1)
  {
    uint32_t now = HAL_GetTick();

    if (control_enabled &&
        (uint32_t)(now - last_control_ms) >= CONTROL_MS)
    {
      float dt = (float)(now - last_control_ms) / 1000.0f;
      last_control_ms = now;

      float angle;
      float gyro_rate;

      if (!MPU6050_ReadAngle(dt, &angle, &gyro_rate))
      {
        Motors_Stop();
        control_enabled = false;
      }
      else if (fabsf(angle - TARGET_ANGLE) > ANGLE_LIMIT)
      {
        /* Robot nghiêng quá nhiều thì dừng motor */
        Motors_Stop();
        control_enabled = false;
      }
      else
      {
        /* PD: góc nghiêng + tốc độ nghiêng */
        float output = KP * (angle - TARGET_ANGLE)
                     + KD * gyro_rate;

        if (output > MAX_DUTY)
          output = MAX_DUTY;
        if (output < -MAX_DUTY)
          output = -MAX_DUTY;

        Motor_SetOne(output,
                     GPIO_PIN_0, GPIO_PIN_1,
                     TIM_CHANNEL_1, LEFT_SIGN);

        Motor_SetOne(output,
                     GPIO_PIN_10, GPIO_PIN_12,
                     TIM_CHANNEL_2, RIGHT_SIGN);
      }
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    Error_Handler();
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    Error_Handler();
}

/**
  * @brief TIM2 Initialization Function
  * @note  Assumes TIM2 clock is 84 MHz.
  *        PSC=0, ARR=4199 gives approximately 20 kHz PWM.
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef clock_source = {0};
  TIM_MasterConfigTypeDef master = {0};
  TIM_OC_InitTypeDef pwm = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    Error_Handler();

  clock_source.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &clock_source) != HAL_OK)
    Error_Handler();

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    Error_Handler();

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master) != HAL_OK)
    Error_Handler();

  pwm.OCMode = TIM_OCMODE_PWM1;
  pwm.Pulse = 0;
  pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &pwm, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &pwm, TIM_CHANNEL_2) != HAL_OK)
    Error_Handler();

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_0 | GPIO_PIN_1 |
                    GPIO_PIN_10 | GPIO_PIN_12,
                    GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 |
                        GPIO_PIN_10 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
    /* Dừng tại đây nếu khởi tạo ngoại vi lỗi */
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif