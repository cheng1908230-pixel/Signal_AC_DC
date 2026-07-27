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
#include "pfc_spwm.h"
#include "qpr.h"
#include "pid.h"
#include <stdbool.h>//使用bool类型变量


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*私有类型定义*/
//锁相环内部用了归一化，无需在意外部电压幅值
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
    .kp = 1.184f,                       /* PID比例增益 */
    .ki = 3.168f,                       /* PID积分增益 */

    .omega_min = 2.0f * PI * 45.0f,     /* 最小角频率，rad/s */
    .omega_max = 2.0f * PI * 55.0f,     /* 最大角频率，rad/s */
    .integrator_max = 30.0f,            /* 积分器最大值 */
    .integrator_min = -30.0f,           /* 积分器最小值 */

    .Ts = 0.00005f,                     /*实际调用时间 20k*/
    .sogi_k = 1.4142f,                  /* SOGI增益系数，理论最佳阻尼比为squrt(2) */
    .voltage_min = 0.5f,                /* 最小电压阈值，V */
};

PFC_PWM_t pfc_spwm; // PFC_PWM实例结构体
#define PFC_SPWM_duty_min 0.02f
#define PFC_SPWM_duty_max 0.98f
#define PFC_SPWM_Vdc_max 50.0f
#define PFC_SPWM_Vdc_min 0.0f
#define PFC_SPWM_modulation_min -0.96f
#define PFC_SPWM_modulation_max 0.96f

//QPR控制器实例结构体，内环
QPRController qpr_controller={

    //实际需要修改的参数
    .Kp = 0.2f,          // 比例增益
    .Kr = 0.7f,          // 谐振增益
    .omega_c = 2.0f * PI * 5.0f,   // 谐振角频率
    .omega_0 = 2.0f * PI * 50.0f,  // 电网基波角频率

    .T = 0.00005f,       // 内环采样周期,20k

    .output_max = 2.5f,  // 输出上限
    .output_min = -2.5f, // 输出下限

    //无须修改的参数
    .u_prev1 = 0.0f,
    .u_prev2 = 0.0f,
    .y_prev1 = 0.0f,
    .y_prev2 = 0.0f,

    .b0 = 0.0f,
    .b1 = 0.0f,
    .b2 = 0.0f,
    .a1 = 0.0f,
    .a2 = 0.0f,

    .output = 0.0f
};

//PID控制器实例结构体
PID_Controller pid_controller; // PID 控制器实例结构体
#define PID_AC_DC_Kp 0.05f
#define PID_AC_DC_Ki 0.00005f
#define PID_AC_DC_Kd 0.0f
#define PID_AC_DC_maxintegral 0.15f
#define PID_AC_DC_maxoutput 0.15f
#define PID_AC_DC_minooutput 0.0f

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*私有宏定义，常量*/
#define ADC_V_SCALE (3.3f / (4095.0f * 0.0355f))//交流电压系数
#define ADC_V_SCALE_DC (3.3f / (4095.0f / 20.0f))//直流电压系数
#define ADC_I_SCALE (3.3f / ((4095.0f) * 7.5f))//交流电流系数
#define ADC_FILTER_LENGTH 5U//五点滑动滤波
#define voltage_reference 2.8f //直流电压参考值

#define PF 1.0f  //功率因数    
#define PF_MAX   1.0f //功率因数最大值
#define PF_MIN   0.0f //功率因数最小值

//锁相环理论上不应该输出的最大与最小相角，输出即为异常
#define PLL_rectify_omega_max (2.0f * PI * 53.0f)
#define PLL_rectify_omega_min (2.0f * PI * 47.0f)
#define PLL_rectify_vq_normalized_min 0.01f//锁相环理论上vq不应持续低于某个值

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*私有函数式宏*/

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*私有变量*/

//ADC采集电压值
volatile uint32_t adc_buffer[3]; // ADC数据缓存
volatile uint8_t  adc_buffer_num = 3;
volatile float adc_V_value[3]; // ADC转换后的电压值，第一个是偏移电压值，第二个是单相交流电压值，第三个直流母线电压
volatile float adc_I_value[3]; // ADC转换后的电流值，第一个是偏移电流值，第二个是交流测电感电流值,第三个空电流采集

/*仅仅调试与测量相位差时用*/
volatile uint8_t TX_ready = 0; // 串口发送标志位
volatile uint32_t tx_divider = 0U;//串口发送计时

//电平反转用于测量实际相位延迟
/*
volatile uint32_t tog = 0u;
*/

/*用作单相逆变spwm的测试变量*/
/*
float theta_spwm_step = 2*PI*50.0f*0.00002f; //SPWM逆变步长 
float theta_spwm = 0.0f;     // SPWM逆变角度
uint8_t spwm_start = 0u;     //SPWM逆变调整时间
float spwm_amplitude = 1.0f; // SPWM逆变测量幅值
*/

/*不用自己调整的参数*/
volatile uint8_t PLL_stable = 0U;          //等待锁相环稳定再进行闭环操作，锁相环稳定标志位
volatile uint32_t PI_outdoor_f = 0u;//PI外环执行频率，达到20更行一次，即以1k频率更新
volatile uint32_t PI_outdoor_time = 0u; //  判断PI外环是否为首次执行
volatile float Pll_angle_step = 0.0f; // 锁相环PFC角度步进
static uint16_t pll_lock_count = 0U;//判断锁相环是否锁定，正常电压时，计数达到4000更新一次，表示0.2s完成锁相
static uint16_t pll_unlock_count = 0U;//判断锁相环是否异常，在异常状态计数达到400更新，表示0.02s锁相环失去稳定
static bool pll_lock_condition = false;

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
  /**
  * @brief  前行计算与初始化
  */
  float x = PF;
  x = fmaxf(PF_MIN, fminf(PF_MAX, x));       /* 功率因数限制至PF_MAX,PF_MIN */
  Pll_angle_step = acosf(x);                 /*最终的值会归一到0到PI*/

  /* QPR与PID初始化*/
  QPRController_Init(&qpr_controller); // 初始化 QPR 控制器
  PID_Init(&pid_controller,  PID_AC_DC_Kp, PID_AC_DC_Ki, PID_AC_DC_Kd, PID_AC_DC_maxintegral, PID_AC_DC_maxoutput, PID_AC_DC_minooutput);
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
  MX_TIM8_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /**
  * @brief  相关外设初始化
  */
  /* adc启动，由TIM8的TRGO事件触发，20k，包括锁相环 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // ADC1校准
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // ADC2校准
  HAL_ADC_Start(&hadc2);// 软件启动从ADC2
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_buffer, adc_buffer_num);// 多模式DMA启动，启动主ADC1


  /* spwm启动，需要手动更改通道与时钟*/
  PFC_PWM_Init(&pfc_spwm,
                  &htim8,
                  TIM_CHANNEL_1,
                  TIM_CHANNEL_2,
                  PFC_SPWM_duty_min, PFC_SPWM_duty_max,
                  PFC_SPWM_Vdc_min, PFC_SPWM_Vdc_max,
                  PFC_SPWM_modulation_min, PFC_SPWM_modulation_max);

  if(!PFC_SPWM_Start(&pfc_spwm))
  {
     Error_Handler(); // 调用错误处理函数
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    #if 0/*----------主函数锁相环检测串口调试的开始，更改变量既可以看----------*/
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
    #endif/*----------主函数锁相环检测串口调试的结束----------*/

    /*主函数串口调试观测直流侧电压*/
    if(TX_ready == 1)
    {   
        float omega;

        __disable_irq();//关中断
        omega = adc_V_value[2];
        TX_ready = 0;// 清除串口发送标志位
        __enable_irq();//开中断

        float abs_omega = fabsf(omega);

        uint32_t omega_integer = (uint32_t)abs_omega;
        uint32_t omega_decimal =
            (uint32_t)((abs_omega - (float)omega_integer) * 10000.0f);

        printf("%c%lu.%04lu\r\n",
               (omega < 0.0f) ? '-' : '+',
               (unsigned long)omega_integer,
               (unsigned long)omega_decimal);
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
  /*端口反转，用于测量是否阻塞中断（采用PA0端口）*/
  //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0);
  
  /**
  * @brief  第一部分：电压转换函数，并采用滑动滤波，采用五点滤波
  */
    //储存单相电压
    const uint32_t adc_sample0 = adc_buffer[0];
    const uint32_t adc_sample1 = adc_buffer[1];
    const uint32_t adc_sample2 = adc_buffer[2];

    //adc转换完成，准备处理数据
    adc_V_value[0] = (float)((uint16_t)adc_sample0 & 0xFFFFU) * ADC_V_SCALE; //偏置  
    adc_V_value[1] = -((float)((uint16_t)adc_sample1 & 0xFFFFU) * ADC_V_SCALE - adc_V_value[0]);//交流电压
    adc_V_value[2] = (float)((uint16_t)adc_sample2 & 0xFFFFU) * ADC_V_SCALE_DC;//直流母线电压
 
    adc_I_value[0] = (float)((uint16_t)(adc_sample0>>16) & 0xFFFFU) * ADC_I_SCALE; //偏置电流
    adc_I_value[1] = ((float)((uint16_t)(adc_sample1>>16) & 0xFFFFU) * ADC_I_SCALE - adc_I_value[0]);//交流测电感电流
    adc_I_value[2] = ((float)((uint16_t)(adc_sample2>>16) & 0xFFFFU) * ADC_I_SCALE - adc_I_value[0]);//空电流采集

    //五点滑动均值滤波
    static float adc_V_history[3][ADC_FILTER_LENGTH] = {{0.0f}};
    static float adc_I_history[3][ADC_FILTER_LENGTH] = {{0.0f}};
    static float adc_V_sum[3] = {0.0f};
    static float adc_I_sum[3] = {0.0f};
    static uint8_t filter_index = 0U;
    static uint8_t filter_count = 0U;

    //五点滑动均值滤波
    if (filter_count < ADC_FILTER_LENGTH)
    {
      filter_count++;
    }

     //计算平均系数，避免没有历史数据，而拉低平均值
    const float filter_gain = 1.0f / (float)filter_count;

    for (uint32_t channel = 0U; channel < 3U; channel++)
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

    for (uint32_t channel = 0U; channel < 3U; channel++)
    {  
      //电流进行计算
      //从电流和中移除即将被覆盖的旧历史值
      adc_I_sum[channel] -= adc_I_history[channel][filter_index];
      //存储新的电流采样值，覆盖旧值
      adc_I_history[channel][filter_index] = adc_I_value[channel];
      //将新值重新加回电流和
      adc_I_sum[channel] += adc_I_history[channel][filter_index];

      //滑动平均值的计算
      adc_I_value[channel] = adc_I_sum[channel] * filter_gain;
    }

      filter_index++;
      if (filter_index >= ADC_FILTER_LENGTH)
      {
        filter_index = 0U;
      }

  /**
  * @brief  第二部分：锁相环开始，并判断锁相是否稳定
  */
      //锁相
      SinglePhasePLL_Update(&PLL_rectify, adc_V_value[1]);

      //测量实际相位差，方法通过过零点进行检测
      /*
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

      /*补足公式中的相位以及1.8度群延迟*/
      float theta = PLL_rectify.theta + PI/2 + 0.0314f;
      if(theta > 2*PI)
      {
         theta -= 2.0f * PI;
      }

      pll_lock_condition =PLL_rectify.voltage_valid && 
                          (fabsf(PLL_rectify.vq_normalized) < PLL_rectify_vq_normalized_min) &&
                          (PLL_rectify.omega > PLL_rectify_omega_min) && (PLL_rectify.omega < PLL_rectify_omega_max);
     
      /* PLL锁定条件判断 */
      /*假设正常电压下锁相环在0.2s后稳定
        不正常电压下0.02s后失锁*/
      if (pll_lock_condition){ 
         pll_unlock_count = 0U;
         if (pll_lock_count < 4000U){pll_lock_count++;}
         if (pll_lock_count >= 4000U){PLL_stable = 1U;}
        }
      else{
         pll_lock_count = 0U;
         if (pll_unlock_count < 400U){pll_unlock_count++;}
         if (pll_unlock_count >= 400U){PLL_stable = 0U;}
        }
     
  /**
  * @brief  第三部分：单相逆变测试代码，检测spwm是否正常工作
  */
      /*
      theta_spwm += theta_spwm_step;
      if(theta_spwm > 2*PI)
      {
         theta_spwm -= 2.0f * PI;
      }
      float v_ref = spwm_amplitude * arm_sin_f32(theta_spwm);
      spwm_start++;
      if(spwm_start > 5u)
      {
        PFC_PWM_Update(&pfc_spwm, v_ref, 2); 
        spwm_start = 0u;
      }
     */

    /**
    * @brief  第四部分：双闭环操作的开始，进行电压环和电流环的控制
    *         电压环利用PID控制，电流环利用QPR控制，内环20k，外环1k
    */
     if(PLL_stable == 1U)
     {
      PI_outdoor_f++;
      //首次外环
      if(PI_outdoor_time == 0U)
      {
        //第一步：电压环的PID控制
        PID_Calc(&pid_controller, voltage_reference, adc_V_value[2]);
        PI_outdoor_time = 1u;
      }

      /*周期性外环控制*/
      if(PI_outdoor_f >= 20u && PI_outdoor_time == 1u)
      {
        PI_outdoor_f = 0u;
      //第一步：电压环的PID控制
        PID_Calc(&pid_controller, voltage_reference, adc_V_value[2]);
      }
      //第二步：并行进行角度控制，PFC矫正
      float angle = Pll_angle_step + theta;        
      if(angle > 2*PI)
      {
        angle -= 2.0f * PI;
      }
      //第三步：目标电流的生成
      float I_target = arm_sin_f32(angle) * pid_controller.output;
      //第四步：差值进行QPR控制
      float I_error = I_target - adc_I_value[1];
      QPRController_Update(&qpr_controller, I_error);
      //第五步：根据PR控制器的输出更新SPWM
      float v_ref = qpr_controller.output;
      if(PFC_PWM_Update(&pfc_spwm, v_ref, adc_V_value[2]) != true)
       {if(PFC_SPWM_Stop(&pfc_spwm) != true)
        {
          Error_Handler();//硬件中断
        }
       };
     }
     //锁相环失控，断掉所有的spwm
     else{
        if(PFC_SPWM_Stop(&pfc_spwm) != true)
        {
          Error_Handler();//硬件中断
        }

     }

    /**
    * @brief  第五部分：串口发送调试，设置发送标志位
    *         每1000次循环设置一次发送标志位，即0.05s发送一次
    */
      tx_divider++;
      if(tx_divider >= 1000U)
      {
          tx_divider = 0U;
          TX_ready = 1; // 设置串口发送标志位，表示可以发送数据
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
