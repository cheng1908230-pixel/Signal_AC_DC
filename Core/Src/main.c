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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SOGI.h"
#include "arm_math.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*私有类型定义*/
//单相整流锁相环结构体
SinglePhasePLL_t PLL_rectify = {
    .theta = 0.0f,         /* 锁相角，rad */
    .omega = 2.0f * PI * 50.0f, /* 锁相角频率，rad/s */
    .omega_nominal = 2.0f * PI * 50.0f, /* 额定角频率，rad/s */
    .alpha = 0.0f,                      /* SOGI alpha分量 */
    .beta = 0.0f,                       /* SOGI beta分量 */
    .vin_last = 0.0f,                   /* 上一次输入电压值 */
    .integrator = 0.0f,                 /* PID积分器初始值 */
    .amplitude = 0.0f,                  /* 电压幅值初始值 */
    .vq = 0.0f,                         /* q轴分量初始值 */
    .vq_normalized = 0.0f,              /* q轴归一化分量初始值 */
    .voltage_valid = false ,             /* 电压有效标志初始为false */

    //实际需要修改的参数
    .kp = 5.6f,                       /* PID比例增益 */
    .ki = 10.4f,                     /* PID积分增益 */

    .omega_min = 2.0f * PI * 45.0f,     /* 最小角频率，rad/s */
    .omega_max = 2.0f * PI * 55.0f,     /* 最大角频率，rad/s */
    .integrator_max = 30.0f,            /* 积分器最大值 */
    .integrator_min = -30.0f,           /* 积分器最小值 */

    .Ts = 0.00002f,                     /*实际调用时间 50k*/
    .sogi_k = 1.4142f,                  /* SOGI增益系数，理论最佳阻尼比为squrt(2) */
    .voltage_min = 0.5f,                /* 最小电压阈值，V */
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*私有宏定义，常量*/
#define ADC_V_SCALE (3.3f / (4095.0f * 0.0355f))//电压系数
#define ADC_FILTER_LENGTH 10U//十点滑动滤波

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*私有函数式宏*/

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*私有变量*/

//ADC采集电压值
volatile uint16_t adc_buffer[2]; // ADC数据缓存
volatile float adc_V_value[2]; // ADC转换后的电压值

/*仅仅调试与测量相位差时用*/
/*
volatile uint8_t TX_ready = 0; // 串口发送标志位
static uint32_t tx_divider = 0U;//串口发送计时
//电平反转用于测量实际相位延迟
volatile uint32_t tog = 0u;
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/*私有函数原型*/

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
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  //adc启动
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // ADC1校准
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);// 软件启动从ADC1
  HAL_TIM_Base_Start(&htim6);//开启tim6（adc时钟, 50k采集电压,并且锁相）
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    #if 0/*----------主函数注释的开始----------*/
    float omega;
    float omega2;
    
    if (TX_ready)
      {
        /* vofa上位机，串口两参量回调打印*/
        __disable_irq();//关中断
        omega = PLL_rectify.theta;
        omega2 = adc_V_value[1];
        TX_ready = 0;// 清除串口发送标志位
        __enable_irq();//开中断

        float abs_omega = fabsf(omega);
        float abs_omega2 = fabsf(omega2);

        uint32_t omega_integer = (uint32_t)abs_omega;
        uint32_t omega_decimal =
            (uint32_t)((abs_omega - (float)omega_integer) * 10000.0f);

        uint32_t omega2_integer = (uint32_t)abs_omega2;
        uint32_t omega2_decimal =
            (uint32_t)((abs_omega2 - (float)omega2_integer) * 10000.0f);

        printf("%c%lu.%04lu ,%c%lu.%04lu\r\n",
               (omega < 0.0f) ? '-' : '+',
               (unsigned long)omega_integer,
               (unsigned long)omega_decimal,
               (omega2 < 0.0f) ? '-' : '+',
               (unsigned long)omega2_integer,
               (unsigned long)omega2_decimal);
        

        /* vofa上位机串口一参量打印*/
        __disable_irq();//关中断
        omega = PLL_rectify.theta;
        TX_ready = 0;// 清除串口发送标志位
        __enable_irq();//开中断

        float abs_omega = fabsf(omega);

        uint32_t omega_integer = (uint32_t)abs_omega;
        uint32_t omega_decimal =
            (uint32_t)((abs_omega - (float)omega_integer) * 10000.0f);

        printf("%c%lu.%04lu \r\n",
               (omega < 0.0f) ? '-' : '+',
               (unsigned long)omega_integer,
               (unsigned long)omega_decimal);

      }
    #endif/*----------主函数注释的结束----------*/
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 重定向printf函数到串口
int _write(int file, char *ptr, int len) {
    (void)file; // 避免未使用参数警告
    
    // 直接发送所有数据
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len,1000);
    return len;
}



/*注意十点滑动滤波存在群延迟*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  //注意打开dma的循环模式，单次转换也需要开启循环模式
 if (hadc->Instance == ADC1)
  {
    //储存单相电压
    const uint32_t adc_sample0 = adc_buffer[0];
    const uint32_t adc_sample1 = adc_buffer[1];

    //adc转换完成，准备处理数据
    adc_V_value[0] = (float)((uint16_t)adc_sample0 & 0xFFFFU) * ADC_V_SCALE;   
    //a相电压PB1
    adc_V_value[1] = -((float)((uint16_t)adc_sample1 & 0xFFFFU) * ADC_V_SCALE - adc_V_value[0]);
 
    //十点滑动均值滤波
    static float adc_V_history[2][ADC_FILTER_LENGTH] = {{0.0f}};
    static float adc_V_sum[2] = {0.0f};
    static uint8_t filter_index = 0U;
    static uint8_t filter_count = 0U;

    //十点滑动均值滤波
    if (filter_count < ADC_FILTER_LENGTH)
    {
      filter_count++;
    }

     //计算平均系数，避免没有历史数据，而拉低平均值
    const float filter_gain = 1.0f / (float)filter_count;

    for (uint32_t channel = 0U; channel < 2U; channel++)
    {  
      //电压进行计算
      //从电压和中移除即将被覆盖的旧历史值
      adc_V_sum[channel] -= adc_V_history[channel][filter_index];
      //存储新的电压采样值，覆盖旧值
      adc_V_history[channel][filter_index] = adc_V_value[channel];
      //将新值重新加回电压和
      adc_V_sum[channel] += adc_V_history[channel][filter_index];

      //滑动平均值的计算
      adc_V_value[channel] = adc_V_sum[channel] * filter_gain;
    }

      filter_index++;
      if (filter_index >= ADC_FILTER_LENGTH)
      {
        filter_index = 0U;
      }

      //锁相
      SinglePhasePLL_Update(&PLL_rectify, adc_V_value[1]);

      //串口发送调试
      /*
      tx_divider++;
      if(tx_divider >= 1000U)
      {
          tx_divider = 0U;
          TX_ready = 1; // 设置串口发送标志位，表示可以发送数据
      }
      */

      /*补足相位滞后，以及相位滞后的过零点检测*/
      float theta = PLL_rectify.theta + PI/2;
      if(theta > 2*PI)
      {
         theta -= 2.0f * PI;
      }

      /*
      //测量实际相位差
      if(theta >= -2e-2 && theta <= 2e-2 && tog == 0u)
      {
        HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0);
        tog = 1u;
      }
      if (theta > 2e-1)
      {
        tog = 0u;
      }
      */
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
