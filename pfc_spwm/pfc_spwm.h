#ifndef __PFC_SPWM_H
#define __PFC_SPWM_H

#include "main.h"
#include <stdbool.h>//使用bool类型变量
#include "arm_math.h"

 /**
  * @brief PFC_SPWM的结构体变量
 */
typedef struct
{
    TIM_HandleTypeDef *htim;

    uint32_t channel_u;
    uint32_t channel_v;

    float duty_min;       /* 最小占空比*/
    float duty_max;       /* 最大占空比*/
    float vdc_min;        /* 允许调制的最低母线电压 */
    float vdc_max;        /* 允许调制的最高母线电压 */
    float modulation_max; /* 实际最高调制比 */
    float modulation_min; /* 实际最低调制比*/

    bool started;
} PFC_PWM_t;

void PFC_PWM_Init(PFC_PWM_t *pwm,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel_u,
                  uint32_t channel_v,
                  float duty_min,float duty_max,
                  float vdc_min,float vdc_max,
                  float modulation_min,float modulation_max);
void PFC_PWM_SetZeroVoltage(PFC_PWM_t *pwm);
bool PFC_PWM_Update(PFC_PWM_t *pwm,float v_conv_ref,float vdc);
bool PFC_SPWM_Start(PFC_PWM_t *pwm);
bool PFC_SPWM_Stop(PFC_PWM_t *pwm);

#endif