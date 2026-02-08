/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  * ！！！！！这是黑底白线代码！！！！！
  * 
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 全局变量和宏定义 */
#define LEFT_SENSOR_PIN      GPIO_PIN_13      // 左传感器
#define MIDDLE_LEFT_SENSOR_PIN  GPIO_PIN_12   // 中间左传感器
#define MIDDLE_RIGHT_SENSOR_PIN GPIO_PIN_14   // 中间右传感器
#define RIGHT_SENSOR_PIN     GPIO_PIN_15      // 右传感器

#define MOTOR_IN1_PIN        GPIO_PIN_6
#define MOTOR_IN2_PIN        GPIO_PIN_7
#define MOTOR_IN3_PIN        GPIO_PIN_8
#define MOTOR_IN4_PIN        GPIO_PIN_9

// 电机控制宏
#define LEFT_MOTOR_FORWARD()  do { \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_PIN, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN2_PIN, GPIO_PIN_RESET); \
} while(0)

#define LEFT_MOTOR_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN2_PIN, GPIO_PIN_SET); \
} while(0)

#define LEFT_MOTOR_STOP() do { \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN2_PIN, GPIO_PIN_RESET); \
} while(0)

#define RIGHT_MOTOR_FORWARD() do { \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN3_PIN, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN4_PIN, GPIO_PIN_RESET); \
} while(0)

#define RIGHT_MOTOR_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN3_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN4_PIN, GPIO_PIN_SET); \
} while(0)

#define RIGHT_MOTOR_STOP() do { \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN3_PIN, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN4_PIN, GPIO_PIN_RESET); \
} while(0)

// 传感器状态读取宏（检测到白线返回1）
#define READ_LEFT_SENSOR()        (HAL_GPIO_ReadPin(GPIOB, LEFT_SENSOR_PIN) == GPIO_PIN_SET)
#define READ_MIDDLE_LEFT_SENSOR() (HAL_GPIO_ReadPin(GPIOB, MIDDLE_LEFT_SENSOR_PIN) == GPIO_PIN_SET)
#define READ_MIDDLE_RIGHT_SENSOR() (HAL_GPIO_ReadPin(GPIOB, MIDDLE_RIGHT_SENSOR_PIN) == GPIO_PIN_SET)
#define READ_RIGHT_SENSOR()       (HAL_GPIO_ReadPin(GPIOB, RIGHT_SENSOR_PIN) == GPIO_PIN_SET)

// PWM速度控制
#define BASE_SPEED 700     // 基础速度（全速前进）
#define TURN_SPEED 500     // 转向速度（左转/右转）
#define SLOW_SPEED 500     // 慢速（十字路口）
#define SPIN_SPEED 600     // 原地转圈速度
#define SHARP_TURN_SPEED 600  // 急转速度（只有一个外侧传感器检测到时）
#define MINOR_TURN_SPEED 340    // 微调速度

// 左轮补偿系数
#define LEFT_MOTOR_COMPENSATE 1.05f

// 方向记录
#define DIRECTION_NONE 0
#define DIRECTION_LEFT 1
#define DIRECTION_RIGHT 2
#define DIRECTION_HISTORY_SIZE 5  // 记录最近5次的方向

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables */
/* USER CODE BEGIN PV */
  uint8_t last_turn_direction = DIRECTION_NONE;  // 最近一次转弯方向
  uint8_t direction_history[DIRECTION_HISTORY_SIZE] = {0};  // 方向历史记录
  uint8_t history_index = 0;  // 历史记录索引
  uint8_t lost_line_timer = 0;  // 丢失白线计时器
  uint32_t last_line_time = 0;  // 上次检测到白线的时间

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes */
void motor_forward(void);
void motor_turn_left(void);
void motor_turn_right(void);
void motor_stop(void);
void motor_slow_forward(void);
void motor_slight_left(void);  //轻微左转
void motor_slight_right(void);
void motor_spin_left(void);    //原地左转
void motor_spin_right(void);
void motor_sharp_left(void);   //急左转
void motor_sharp_right(void);
void motor_minor_left(void);   //微调左转
void motor_minor_right(void);

// 方向记录函数
void update_direction_history(uint8_t direction);
uint8_t get_most_common_direction(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 更新方向历史记录
void update_direction_history(uint8_t direction)
{
    if(direction != DIRECTION_NONE) {
        direction_history[history_index] = direction;
        history_index = (history_index + 1) % DIRECTION_HISTORY_SIZE;
        last_turn_direction = direction;
        last_line_time = HAL_GetTick();  // 更新最后检测到白线的时间
    }
}

// 获取最常见的转弯方向
uint8_t get_most_common_direction(void)
{
    uint8_t left_count = 0, right_count = 0;

    for(int i = 0; i < DIRECTION_HISTORY_SIZE; i++) {
        if(direction_history[i] == DIRECTION_LEFT) {
            left_count++;
        } else if(direction_history[i] == DIRECTION_RIGHT) {
            right_count++;
        }
    }

    if(left_count > right_count) {
        return DIRECTION_LEFT;
    } else if(right_count > left_count) {
        return DIRECTION_RIGHT;
    } else {
        return last_turn_direction;  // 如果数量相等，返回最近一次的方向
    }
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // 启动PWM输出
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 左轮PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // 右轮PWM

  // 初始速度为0
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

  HAL_Delay(1000);

  // 停止
  motor_stop();
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 读取四个传感器的状态
      uint8_t left_sensor = READ_LEFT_SENSOR();
      uint8_t middle_left_sensor = READ_MIDDLE_LEFT_SENSOR();
      uint8_t middle_right_sensor = READ_MIDDLE_RIGHT_SENSOR();
      uint8_t right_sensor = READ_RIGHT_SENSOR();

      // 检查是否所有传感器都检测不到白线
      if (!left_sensor && !middle_left_sensor && !middle_right_sensor && !right_sensor) {
          // 增加丢失白线计时
          lost_line_timer++;

          // 如果连续丢失白线超过5次（约50ms），开始智能寻找
          if(lost_line_timer > 5) {
              // 根据历史方向选择寻找方向
              uint8_t common_direction = get_most_common_direction();

              if(common_direction == DIRECTION_LEFT || common_direction == DIRECTION_NONE) {
                  // 向左寻找
                  motor_spin_left();
              } else {
                  // 向右寻找
                  motor_spin_right();
              }

             // HAL_Delay(20);
              continue;
          }
      } else {
          // 检测到白线，重置计时器
          lost_line_timer = 0;
      }

      // 条件1：四个传感器都检测到白线 - 十字路口，慢速前进
      if (left_sensor && middle_left_sensor && middle_right_sensor && right_sensor) {
          motor_slow_forward();
          HAL_Delay(200);  // 慢速通过十字路口
          // 十字路口不记录方向
          continue;
      }

      // 条件2：左边+中间两个检测到白线 - 直角左转
      if (left_sensor && middle_left_sensor && middle_right_sensor) {
          motor_turn_left();
          update_direction_history(DIRECTION_LEFT);  // 记录左转方向
          //HAL_Delay(10);  // 转90度所需时间，根据实际调整
          continue;
      }

      // 条件2.5：左边+中左/中右检测到白线 - 直角左转
      if ((left_sensor && middle_left_sensor) || (left_sensor && middle_right_sensor)) {
          motor_turn_left();
          update_direction_history(DIRECTION_LEFT);  // 记录左转方向
          //HAL_Delay(10);  // 转90度所需时间，根据实际调整
          continue;
      }

      // 条件3：右边+中间两个检测到白线 - 直角右转
      if (right_sensor && middle_left_sensor && middle_right_sensor) {
          motor_turn_right();
          update_direction_history(DIRECTION_RIGHT);  // 记录右转方向
          //HAL_Delay(10);  // 转90度所需时间，根据实际调整
          continue;
      }

      // 条件3.5：右边+中左/中右检测到白线 - 直角右转
      if ((right_sensor && middle_left_sensor) || (right_sensor && middle_right_sensor)) {
          motor_turn_right();
          update_direction_history(DIRECTION_RIGHT);  // 记录右转方向
          //HAL_Delay(10);  // 转90度所需时间，根据实际调整
          continue;
      }

      // 条件4：只有中间两个传感器检测到白线 - 全速前进
      if (!left_sensor && middle_left_sensor && middle_right_sensor && !right_sensor) {
          motor_forward();
          // 直行不记录方向
          //HAL_Delay(3);
          continue;
      }

      // 条件5：只有中左检测到白线 - 稍微左转
      if (!left_sensor && middle_left_sensor && !middle_right_sensor && !right_sensor) {
          motor_slight_left();
          update_direction_history(DIRECTION_LEFT);  // 记录左转趋势
          //HAL_Delay(5);
          continue;
      }

      // 条件6：只有中右检测到白线 - 稍微右转
      if (!left_sensor && !middle_left_sensor && middle_right_sensor && !right_sensor) {
          motor_slight_right();
          update_direction_history(DIRECTION_RIGHT);  // 记录右转趋势
          //HAL_Delay(5);
          continue;
      }

      // 条件7：只有左传感器检测到白线 - 急左转
      if (left_sensor && !middle_left_sensor && !middle_right_sensor && !right_sensor) {
          motor_sharp_left();
          update_direction_history(DIRECTION_LEFT);  // 记录左转方向
          //HAL_Delay(10);
          continue;
      }

      // 条件8：只有右传感器检测到白线 - 急右转
      if (!left_sensor && !middle_left_sensor && !middle_right_sensor && right_sensor) {
          motor_sharp_right();
          update_direction_history(DIRECTION_RIGHT);  // 记录右转方向
          //HAL_Delay(10);
          continue;
      }

      // 条件9：只有中左传感器检测到白线 - 微调右转
      if (!left_sensor && middle_left_sensor && !middle_right_sensor && !right_sensor) {
    	    motor_minor_right();
          //HAL_Delay(10);
          continue;
      }

      // 条件10：只有右传感器检测到白线 - 微调左转
      if (!left_sensor && !middle_left_sensor && middle_right_sensor && !right_sensor) {
    	    motor_minor_left();
          //HAL_Delay(10);
          continue;
      }

      //其他情况：保持前进
        // motor_forward();
        // HAL_Delay(3);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : middle_left_sensor_Pin left_sensor_Pin middle_right_sensor_Pin right_sensor_Pin */
  GPIO_InitStruct.Pin = middle_left_sensor_Pin|left_sensor_Pin|middle_right_sensor_Pin|right_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// 电机控制函数
void motor_forward(void)
{
  LEFT_MOTOR_FORWARD();
  RIGHT_MOTOR_FORWARD();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(BASE_SPEED * LEFT_MOTOR_COMPENSATE));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BASE_SPEED);
}

void motor_turn_left(void)
{
  LEFT_MOTOR_STOP();
  RIGHT_MOTOR_FORWARD();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TURN_SPEED);
}

void motor_turn_right(void)
{
  LEFT_MOTOR_FORWARD();
  RIGHT_MOTOR_STOP();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(TURN_SPEED * LEFT_MOTOR_COMPENSATE));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

void motor_stop(void)
{
  LEFT_MOTOR_STOP();
  RIGHT_MOTOR_STOP();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

void motor_slow_forward(void)
{
  LEFT_MOTOR_FORWARD();
  RIGHT_MOTOR_FORWARD();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(SLOW_SPEED * LEFT_MOTOR_COMPENSATE));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SLOW_SPEED);
}

void motor_slight_left(void)
{
  LEFT_MOTOR_FORWARD();
  RIGHT_MOTOR_FORWARD();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(TURN_SPEED * LEFT_MOTOR_COMPENSATE));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BASE_SPEED);
}

void motor_slight_right(void)
{
  LEFT_MOTOR_FORWARD();
  RIGHT_MOTOR_FORWARD();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(BASE_SPEED * LEFT_MOTOR_COMPENSATE));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TURN_SPEED);
}

void motor_spin_left(void)
{
  LEFT_MOTOR_BACKWARD();
  RIGHT_MOTOR_FORWARD();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(SPIN_SPEED * LEFT_MOTOR_COMPENSATE));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SPIN_SPEED);
}

void motor_spin_right(void)
{
  LEFT_MOTOR_FORWARD();
  RIGHT_MOTOR_BACKWARD();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(SPIN_SPEED * LEFT_MOTOR_COMPENSATE));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SPIN_SPEED);
}

void motor_sharp_left(void)
{
  LEFT_MOTOR_BACKWARD();
  RIGHT_MOTOR_FORWARD();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(SHARP_TURN_SPEED * LEFT_MOTOR_COMPENSATE));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SHARP_TURN_SPEED);
}

void motor_sharp_right(void)
{
  LEFT_MOTOR_FORWARD();
  RIGHT_MOTOR_BACKWARD();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(SHARP_TURN_SPEED * LEFT_MOTOR_COMPENSATE));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SHARP_TURN_SPEED);
}

void motor_minor_left(void)
{
  LEFT_MOTOR_STOP();
  RIGHT_MOTOR_FORWARD();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(MINOR_TURN_SPEED * LEFT_MOTOR_COMPENSATE));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MINOR_TURN_SPEED);
}

void motor_minor_right(void)
{
  LEFT_MOTOR_FORWARD();
  RIGHT_MOTOR_STOP();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(MINOR_TURN_SPEED * LEFT_MOTOR_COMPENSATE));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MINOR_TURN_SPEED);
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
