/**
 * @file    SOGI.c
 * @brief  单相二阶广义积分器锁相环（SOGI-PLL）实现
 *
 * @details
 * 本模块实现了基于双线性变换（Tustin 法）的 SOGI 算法，用于从单相电网
 * 电压瞬时值中提取一对正交分量（alpha/beta），并通过 PI 控制器跟踪电网
 * 电压的相位和频率。主要功能包括：
 * - 频率自适应的 SOGI 正交信号生成（内部函数）
 * - 归一化相位误差的 PI 锁相环（外部调用接口）
 * - 条件积分抗饱和 + 积分器硬限幅 + 输出硬限幅三重保护
 * - 电网电压跌落自动检测与锁相暂停
 * - 角度归一化与三角运算（调用 CMSIS-DSP 库 arm_sin_f32 / arm_cos_f32）
 *
 * 外部只需调用 SinglePhasePLL_Update() 即可完成一次完整的 PLL 迭代，
 * 调用周期必须与 PLL 结构体中的 Ts 严格一致（通常由定时器中断保证）。
 *
 * @note
 * - SOGI 使用当前锁相频率 omega 作为谐振频率，因此具备天然的频率自适应性。
 * - 归一化误差 vq_normalized 使 PI 增益不随电网电压幅值变化。
 * - 电压低于阈值时直接返回额定频率并挂起更新，保证恢复时快速重锁。
 *
 * @warning
 * - 本模块所有函数均非重入，禁止在不同中断优先级中同时操作同一实例。
 */


#include "SOGI.h"

/**
 * @brief  双线性锁相环更新计算，得出alpha、beta的最新值，以及将本次电压作为历史积分值
 *
 * @param  pll          指向 PLL 实例结构体的指针
 * @param  grid_voltage 当前 ADC 采样的电网电压瞬时值，单位：V
 *
 * @note   内部自动调用 SOGI 更新函数，无需外部干预，定义为static，防止外部干扰。
 */
static void SinglePhasePLL_UpdateSOGI(SinglePhasePLL_t *pll,float grid_voltage)
{
    //将指针变量pll解引用，获取结构体变量，增加可读性
    //减少指针寻址开销
    //分离新旧变量
    float w;
    float h;
    float kw;

    float m11;
    float m12;
    float m21;
    float determinant;

    float rhs1;
    float rhs2;

    float alpha_old;
    float beta_old;

    w = pll->omega;//获取当前锁相环角频率，单位rad/s   
    h = 0.5f * pll->Ts;//半个离散周期，单位s，用于梯形积分
    kw = pll->sogi_k * w;//阻尼系数与角频率乘积

    alpha_old = pll->alpha;
    beta_old = pll->beta;

    //梯形离散化积分
    m11 = 1.0f + h * kw;
    m12 = h * w;
    m21 = -h * w;

    rhs1 = (1.0f - h * kw) * alpha_old
         - h * w * beta_old
         + h * kw * (grid_voltage + pll->vin_last);

    rhs2 = h * w * alpha_old + beta_old;

    //利用克拉默法则求解二阶矩阵
    determinant = m11 - m12 * m21;
    pll->alpha = (rhs1 - m12 * rhs2) / determinant;
    pll->beta  = (-m21 * rhs1 + m11 * rhs2) / determinant;

    //储存本次电压作为历史值，为下一次迭代使用
    pll->vin_last = grid_voltage;
}


/**
 * @brief  双线性锁相环迭代，供外部函数调用，最后计算出锁相环的角度与频率
 *         pid控制器完成对频率的锁相
 *
 * @param  pll          指向 PLL 实例结构体的指针
 * @param  grid_voltage 当前 ADC 采样的电网电压瞬时值，单位：V
 *
 * @note   该函数应在固定的实时中断中调用，调用周期必须与 pll->Ts 严格一致。
 * @note   当检测到电压幅值低于 pll->voltage_min 时，函数会
 *         自动将电压有效标志置 false，并暂停角度与频率的更新，
 *         仅维持 SOGI 状态。
 * @note  存在输出限幅、积分限幅、抗积分饱和处理、角度归一化处理
 * @note  pid针对的是归一化的q值
 *
 * @warning 此函数非重入，不可在多个中断优先级中同时调用同一个
 *          PLL 实例。
 */
void SinglePhasePLL_Update(SinglePhasePLL_t *pll,
                           float grid_voltage)
{
    float sin_theta;
    float cos_theta;

    float integrator_step;
    float integrator_candidate;
    float omega_candidate;

    SinglePhasePLL_UpdateSOGI(pll, grid_voltage);

    //交流电压幅值的计算
    pll->amplitude = sqrtf(pll->alpha * pll->alpha
                         + pll->beta * pll->beta);

    /*
     *当电压低于阈值，判断电网没电，停止积分    
    */
    if (pll->amplitude < pll->voltage_min)
    {
        pll->voltage_valid = false;
        pll->vq = 0.0f;
        //返回额定频率与q轴，为下次重启，有一个良好的初值
        pll->vq_normalized = 0.0f;
        pll->omega = pll->omega_nominal;
        return;
    }

    pll->voltage_valid = true;

    //计算q轴分量
    while (pll->theta > 2.0f * PI)  pll->theta -= 2.0f * PI;
    while (pll->theta < -2.0f * PI) pll->theta += 2.0f * PI;
    sin_theta = arm_sin_f32(pll->theta);
    cos_theta = arm_cos_f32(pll->theta);
    pll->vq = -pll->alpha * sin_theta
            + pll->beta * cos_theta;

    //归一化vq的变化并进行pid矫正
    pll->vq_normalized = pll->vq / pll->amplitude;

    integrator_step = pll->ki
                    * pll->vq_normalized
                    * pll->Ts;

    integrator_candidate = pll->integrator
                         + integrator_step;

    //预测输出
    omega_candidate = pll->omega_nominal
                    + pll->kp * pll->vq_normalized
                    + integrator_candidate;

    //条件一：简单的抗积分饱和
    if (!((omega_candidate > pll->omega_max &&
           integrator_step > 0.0f) ||
          (omega_candidate < pll->omega_min &&
           integrator_step < 0.0f)))
    {
        pll->integrator = integrator_candidate;
    }

    //条件二：积分限幅处理
    if (pll->integrator > pll->integrator_max)
    {
        pll->integrator = pll->integrator_max;
    }
    else if (pll->integrator < pll->integrator_min)
    {
        pll->integrator = pll->integrator_min;
    }

    //最终执行输出
    pll->omega = pll->omega_nominal
               + pll->kp * pll->vq_normalized
               + pll->integrator;

    //条件三：输出限幅处理
    if (pll->omega > pll->omega_max)
    {
        pll->omega = pll->omega_max;
    }
    else if (pll->omega < pll->omega_min)
    {
        pll->omega = pll->omega_min;
    }

    //更新锁相环角度
    pll->theta += pll->omega * pll->Ts;

    //角度归一化处理
    while (pll->theta > 2.0f * PI)  pll->theta -= 2.0f * PI;
    while (pll->theta < -2.0f * PI) pll->theta += 2.0f * PI;
}