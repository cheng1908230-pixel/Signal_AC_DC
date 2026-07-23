#ifndef __SOGI_H
#define __SOGI_H

#include "main.h"
#include <stdbool.h>
#include "arm_math.h"

 /**
  * @brief 双线性离散变换锁相环的结构体变量、
  * 
  * @note Ts、omega_nominal、omega_min、omega_max、integrator_max、integrator_min、
  *       kp、ki、sogi_k、voltage_min、omega等参数需要根据实际应用进行配置。
  *       vin_last、alpha、beta、integrator、theta、amplitude、vq、vq_normalized
  *       等变量在初始化时应设置为0.0f。
  *       voltage_valid应初始化为false，表示电压无效。
 */
typedef struct
{
    float Ts;             /* 实际调用周期，单位s */

    float theta;          /* 锁相角，rad */
    float omega;          /* 锁相角频率，rad/s */

    float omega_nominal;  /* 额定角频率，rad/s */
    float omega_min;
    float omega_max;
    float integrator_max;
    float integrator_min;

    float sogi_k;
    float alpha;
    float beta;
    float vin_last;

    float kp;
    float ki;
    float integrator;

    float amplitude;
    float vq;
    float vq_normalized;

    float voltage_min;  /* 最小电压，V */
    bool voltage_valid;
} SinglePhasePLL_t;

void SinglePhasePLL_Update(SinglePhasePLL_t *pll,
                           float grid_voltage);

#endif