/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   主程序入口（初始化外设、启动 TIM6 节拍与 UART 接收，中间件在各模块中运行）
  *
  * 运行逻辑概述：
  *  1) CubeMX 生成的外设初始化（GPIO/TIM/UART 等）
  *  2) 启动 PWM（舵机控制）与 TIM6 1ms 中断（Timer.c）
  *  3) 启动 USART1/USART2 1字节中断接收（UART.c）
  *  4) while(1) 主循环：按键切换模式/触发动作，按 g_data_update_flag 节拍刷新显示
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "sys.h"
#include "mpu_atk_ms6dsv.h"
#include "Timer.h"
#include "UART.h"
#include "Servo.h"
#include "Key.h"
#include "movement.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* g_data_update_flag：
 * 由 Timer.c 每 10ms 置 1，主循环消费后清零。
 * 这是典型“中断产生节拍 -> 主循环做耗时刷新”的设计模式。
 */
uint8_t g_data_update_flag = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUT_MODE        1   // 自动模式
#define MANUAL_MODE     2   // 手动模式
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ====== 舵机 PWM 初始值：设置为安全中位/停止 ======
   * 教材解释：
   *   上电后，舵机若处于未知脉宽，可能会瞬间乱动。
   *   因此先明确写入：
   *     - 360 底座：STOP
   *     - 270 关节：中位
   *     - 180 夹爪：关闭位（或安全位）
   */

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, STOP);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
                        (uint16_t)(ANGLE_270_CENTER * 2000.0f / 270.0f + 500.0f));

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
                        (uint16_t)(ANGLE_180_CLOSE * 2000.0f / 180.0f + 500.0f));

  /* 启动 TIM6 1ms 中断（Timer.c 的 HAL_TIM_PeriodElapsedCallback 会运行） */
  HAL_TIM_Base_Start_IT(&htim6);

  /* 启动 PWM 输出：舵机才能接收到 PWM 信号 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* PWM 预装载（Preload）相关寄存器设置：
   * 解释：
   *   - OCxPE：输出比较预装载使能（写 CCR 不会立刻生效，而是在更新事件生效）
   *   - ARPE ：自动重装载预装载使能
   * 这样可以避免在 PWM 周期中间更新导致的毛刺/瞬态异常。
   */
  SET_BIT(htim3.Instance->CCMR1, TIM_CCMR1_OC2PE);
  SET_BIT(htim3.Instance->CR1, TIM_CR1_ARPE);

  /* 启动串口 1字节中断接收链路（UART.c 回调里会重挂接） */
  HAL_UART_Receive_IT(&huart1, g_uart1_rx_buf, 1);
  HAL_UART_Receive_IT(&huart2, g_uart2_rx_buf, 1);

  /* 初始化关节到中位（通过 movement 层设置目标；Timer 平滑推进） */
  joint2_reset();
  HAL_Delay(1000);

  /* 启动信息打印（调试用） */
  UART2_Printf("\r\n=================================\r\n");
  UART2_Printf("  STM32 Robot Arm Control v1.0\r\n");
  UART2_Printf("=================================\r\n");
  UART2_Printf("Init done.\r\n");
  UART2_Printf("Press KEY to switch mode (1=Auto, 2=Manual)\r\n");
  UART2_Printf("=================================\r\n\r\n");

  UART2_Printf("Auto startup enabled\r\n");
  //movement_startup();
  UART2_Printf("Startup sequence finished\r\n");

  uint8_t KeyNum = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 读取按键编号（Key 模块负责消抖/边沿识别） */
    KeyNum = Key_GetNum();

    /* Key1：切自动模式（视觉帧驱动） */
    if (KeyNum == 1)
    {
      mode_flag = AUT_MODE;
    }

    /* Key2：切手动模式（蓝牙命令驱动） */
    if (KeyNum == 2)
    {
      mode_flag = MANUAL_MODE;
    }

    /* Key3：GPIO 输出翻转（风扇开关功能） */
    if (KeyNum == 3)
    {
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_3);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_4);
    }

    /* ====== 10ms 周期刷新：由 TIM6 置位 g_data_update_flag ======
     *   - 中断产生“节拍/事件”
     *   - 主循环消费事件并执行耗时任务（OLED 刷新、串口输出等）
     */
    if (g_data_update_flag == 1)
    {
      g_data_update_flag = 0;

      /* IMU 刷新 */
      // USER_MS6DSV_YAW_ROLL_PITCH();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
