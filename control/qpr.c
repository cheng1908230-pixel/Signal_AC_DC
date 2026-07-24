/**
 * @file     qpr.c
 * @brief    准比例谐振控制器实现
 *
 * @note     执行软启动可以在输入环节或者最终输出环节进行，
 *           这里的qpr控制器是中间环节，不需要增加任何软启动
 */

#include "qpr.h"
#include <stdint.h>
#include <stddef.h>

/**
 * @brief  初始化 QPR 控制器
 *
 * @param  qpr            指向 qpr 实例结构体的指针
 * @param  T              采样周期，单位s
 * @param  kp             比例增益
 * @param  Kr             谐振增益
 * @param  omega_c        截止角频率
 * @param  omega_0        谐振角频率
 * 
 * @note  其中系数都对a0进行了归一化处理，实际使用时无需再除以a0
 */
void QPRController_Init(QPRController *qpr) {
    
    if (qpr == NULL) {
        return;
    }

    if (qpr->T <= 0.0f) {
        // 防止采样周期为零或负数
        return;
    }
    
    float T = qpr->T;
    float Kp = qpr->Kp;
    float Kr = qpr->Kr;
    float omega_c = qpr->omega_c;
    float omega_0 = qpr->omega_0;

    // 计算分母系数
    float a0 = (4.0f / (T*T)) + (4.0f * omega_c / T) + omega_0 * omega_0;
    float a1 = (-8.0f / (T*T)) + 2.0f * omega_0 * omega_0;
    float a2 = (4.0f / (T*T)) - (4.0f * omega_c / T) + omega_0 * omega_0;

    // 计算分子系数
    float b0 = (4.0f * Kp) / (T*T) + (4.0f * omega_c * (Kp + Kr) / T) + Kp * omega_0 * omega_0;
    float b1 = (-8.0f * Kp) / (T*T) + 2.0f * Kp * omega_0 * omega_0;
    float b2 = (4.0f * Kp) / (T*T) - (4.0f * omega_c * (Kp + Kr) / T) + Kp * omega_0 * omega_0;

    //避免除零错误
    if (a0 == 0.0f) {
        // 防止除以零
        a0 = 1e-6f; // 设置一个很小的值，避免除以零
    }

    // 归一化系数
    qpr->b0 = b0 / a0;
    qpr->b1 = b1 / a0;
    qpr->b2 = b2 / a0;
    qpr->a1 = a1 / a0;
    qpr->a2 = a2 / a0;

    // 初始化历史状态
    qpr->u_prev1 = 0.0f;
    qpr->u_prev2 = 0.0f;
    qpr->y_prev1 = 0.0f;
    qpr->y_prev2 = 0.0f;

    //初始化输出值
    qpr->output = 0.0f;
}

/**
 * @brief  QPR 控制计算函数
 *
 * @param  qpr            指向 qpr 实例结构体的指针
 * @param  input          当前输入值
 * 
 * @note  与pid相同，需要周期性的调用该函数进行控制计算
 */
void QPRController_Update(QPRController *qpr, float input) {
    // 计算当前输出
    float output = 
        qpr->b0 * input + 
        qpr->b1 * qpr->u_prev1 + 
        qpr->b2 * qpr->u_prev2 - 
        qpr->a1 * qpr->y_prev1 - 
        qpr->a2 * qpr->y_prev2;

    if (output < qpr->output_min) {
        output = qpr->output_min;
    } 
    else if (output > qpr->output_max) {
        output = qpr->output_max;
    }
    
    // 更新历史状态
    qpr->u_prev2 = qpr->u_prev1;
    qpr->u_prev1 = input;
    qpr->y_prev2 = qpr->y_prev1;
    qpr->y_prev1 = output;

    qpr->output = output;
}