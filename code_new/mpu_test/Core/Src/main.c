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
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "HAl_Add.h"
#include "mpu.h"
#include "moter.h"
#include <stdint.h>
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

/* USER CODE BEGIN PV */
MPU6050 MM;
moter_feedback g_motor_fb;    // 电机反馈结构体
moter_command g_motor_cmd;    // 电机指令结构体
char fault_desc[50];          // 故障描述缓冲区
char massage[100];

#define lf_rf_port &huart2  //左前右前
#define lr_rr_port &huart6  //左后右后

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin ); 
//	  MPU6050_Get_Angle(&MM);//(超低零点漂移)
//    HAL_UART_Transmit(&huart1, (uint8_t*)massage, sizeof(massage), HAL_MAX_DELAY);
	  //MPU6050_Get_Angle_Plus(&MM);//四元素法
  }
}


void Motor_LeftFront_CurrentLoop_1A_Demo(void)
{
    HAL_StatusTypeDef status;
    
    // 1. 初始化所有电机（默认电流环模式）
    status = Motor_Init_All(&g_motor_fb);

    
    // 2. 配置左前电机（ID1）电流指令：1A
    g_motor_cmd.id_lf = 1;                  // 左前电机ID设置为1
    g_motor_cmd.current_left_front_set = 0.1f; // 电流设定为1A
    g_motor_cmd.speed_left_front_set = 100.0f;    // 速度设定为0rpm（电流环无效）
    
    // 3. 发送电流环指令（左前电机，假设使用USART2通信）
    status = Motor_SendCurrentCmd(&huart2, &g_motor_cmd, 1, &g_motor_fb);
/*   if (status != HAL_OK) {
        // 指令发送失败处理
        return;
    }
*/    
    status = Motor_SendCurrentCmd(&huart2, &g_motor_cmd, 1, &g_motor_fb);
    Motor_SetMode(&huart2, 1, 2, &g_motor_fb); // 设置右前电机为电流环模式S
    status = Motor_SendCommand(&huart2, &g_motor_cmd, 1, &g_motor_fb);
     g_motor_cmd.speed_left_front_set = 2500.0f;    // 速度设定为0rpm（电流环无效）
    status = Motor_SendCommand(&huart2, &g_motor_cmd, 1, &g_motor_fb);
    // 4. 解析反馈结果
    uint8_t current = g_motor_fb.current_lf;
    uint8_t temp = g_motor_fb.temp_lf;

    // 解析故障码
//    Motor_ParseFaultCode(g_motor_fb.fault_left_front, fault_desc, sizeof(fault_desc));
//    printf("左前电机故障：%s\n", fault_desc);
}
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
// 左前：&huart2 右前：&huart2
// 左后：&uart4 右后：&uart5
  g_motor_cmd.id_lf = 1; // 左前电机ID
  g_motor_cmd.id_rf = 2; // 右前电机ID
  g_motor_cmd.id_lr = 1; // 左后电机ID
  g_motor_cmd.id_rr = 2; // 右后电机ID
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  MPU6050_init(&hi2c1);//初始化MPU6050(你使用的硬件iic)
  HAL_TIM_Base_Start_IT(&htim2);//定时器中断(确保每次读取的dt稳定)

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  Motor_LeftFront_CurrentLoop_1A_Demo(); // 电机电流环1A示例
  char msg[150];
  while (1)
  {

	  MPU6050_Get_Angle(&MM);//(超低零点漂移)
    HAL_Delay(5);
    sprintf(msg, "%.3f,%.3f,%.3f\n",MM.roll,MM.pitch,MM.yaw);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);    
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
