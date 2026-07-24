/**
 * @file     pfc_spwm.c
 * @brief    单相整流pfc的spwm实现
 *
 * @details  对于单相整流的pfc控制，普遍使用单极性的spwm控制，而svpwm控制不存在优势
 *           这是由于svpwm利用的是三倍零序电流来增加电压的利用率，而这对于单相整流是不存在的
 *           考虑到电压的突变，会使得控制变差，推荐使用软启动进行PFC控制
 *
 * @note     PWM还没有启动时，桥臂内部的反并联二极管会构成不可控整流桥，会存在直流电压Vdc
 *           注意不允许在母线直流电压为0的时候启动控制
 *           直流母线电压设定值不能随便低于不控整流电压，
 *           低于不控整流电压，通常无法实现，因为即使所有功率管关闭，反并联二极管仍然会把母线充到接近不控整流电压
 *           在交流测电压幅值等于电网电压时，不控整流恰好有一个占空比达到100%的一点，这是要避免的，因此
 *           预设值一般要大于不控整流电压，同时需要对占空比进行限制措施
 */

 #include "pfc_spwm.h"

  /**
 * @brief  SPWM初始化函数
 *
 * @param  pwm            指向 PFC_PWM 实例结构体的指针
 * @param  htim           指向定时器句柄的指针
 * @param  channel_u      指向u相的PWM输出端口
 * @param  channel_v      指向v相的PWM输出端口
 * @param  duty_min       最小占空比
 * @param  duty_max       最大占空比
 * @param  vdc_min        允许调制的最低母线电压
 * @param  vdc_max        允许调制的最高母线电压
 * @param  modulation_min 调制比最小值
 * @param  modulation_max 调制比最大值
 *
 * @note   U相与V相pwm端口应该是U相上桥臂与V相的下桥臂
 *         最小占空比与最大占空比均在合理范围内
 *         允许调制的最低母线电压不应低于不控整流电压
 *         调制比理论上的最小值可以到-最大值
 * 
 * @warning 供外部初始化调用
 */
 void PFC_PWM_Init(PFC_PWM_t *pwm,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel_u,
                  uint32_t channel_v,
                  float duty_min,float duty_max,
                  float vdc_min,float vdc_max,
                  float modulation_min,float modulation_max)
{
    if (pwm == NULL)
    {
        return;
    }

    pwm->htim = htim;
    pwm->channel_u = channel_u;
    pwm->channel_v = channel_v;
    pwm->duty_min = duty_min;
    pwm->duty_max = duty_max;
    pwm->vdc_min = vdc_min;
    pwm->vdc_max = vdc_max;
    pwm->modulation_min = modulation_min;
    pwm->modulation_max = modulation_max;

    float duty_compare = 1 - duty_max;
    if (duty_compare < duty_min)
    {
        duty_compare = duty_min;
    }

    /*
     * dU = 0.5 + 0.5m,dV = 0.5 - 0.5m, m = vUV_ref / Vdc
     *
     * |m| <= 1 - 2*duty_compare
     * 由此可以计算出调制比的最大值
     */
    float modulation_max_ideal = 1.0f - 2.0f * duty_compare;
    if (pwm->modulation_max > modulation_max_ideal)
    {
        pwm->modulation_max = modulation_max_ideal;
    }

    float modulation_min_ideal = -modulation_max_ideal;
    if (pwm->modulation_min < modulation_min_ideal)
    {
        pwm->modulation_min = modulation_min_ideal;
    }
    
    pwm->started = false;

    PFC_PWM_SetZeroVoltage(pwm);
}

 /**
 * @brief  占空比的比较值计算函数并采取系列防范措施
 *
 * @param  pwm          指向 PFC_PWM 实例结构体的指针
 * @param  duty         实际应该输出的占空比
 *
 * @note   对占空比进行处理，应该注意到桥臂电压为差值电压，两占空比为0.5时，桥臂电压差值为0
 *         应当注意此时并不说明vdc等于0，因为uv=(du - dv)Vdc,即此时仅仅代表UV=0
 * 
 * @warning 内部函数，不供外部调用
 */
static uint32_t DutyToCompare(PFC_PWM_t *pwm, float duty)
{
    uint32_t arr;
    uint32_t compare;
    float period_counts;

    arr = __HAL_TIM_GET_AUTORELOAD(pwm->htim);
    period_counts = (float)(arr + 1U);

    //限幅处理
    if(duty < pwm->duty_min)
    {
        duty = pwm->duty_min;
    }
    else if(duty > pwm->duty_max)
    {
        duty = pwm->duty_max;
    }

    //此处的0.5f用于四舍五入保精度计算
    compare = (uint32_t)(duty * period_counts + 0.5f);

    //越界保护
    if (compare > arr)
    {
        compare = arr;
    }

    return compare;
}

 /**
 * @brief  将桥臂电压设置为零差模电压，占空比均为百分之50
 *
 * @param  pwm   指向 PFC_PWM 实例结构体的指针
 */
void PFC_PWM_SetZeroVoltage(PFC_PWM_t *pwm)
{
    uint32_t compare_50_percent;

    if ((pwm == NULL) || (pwm->htim == NULL))
    {
        return;
    }

    compare_50_percent = DutyToCompare(pwm, 0.5f);

    __HAL_TIM_SET_COMPARE(pwm->htim,pwm->channel_u,compare_50_percent);

    __HAL_TIM_SET_COMPARE(pwm->htim,pwm->channel_v,compare_50_percent);
}

/**
 * @brief  PFC对占空比进行计算并实时更新比较值，产生SPWM波形
 *         中间做了一系列的限幅处理，保证占空比在合理范围内
 *
 * @param  pwm          指向 PFC_PWM 实例结构体的指针
 * @param  v_conv_ref   输入的交流参考电压，单位：V
 * @param  vdc          预计输出的直流母线电压，单位为：V
 *
 * @warning 供外部调用使用
 */
bool PFC_PWM_Update(PFC_PWM_t *pwm,float v_conv_ref,float vdc)
{
    float modulation;
    float duty_u;
    float duty_v;

    uint32_t compare_u;
    uint32_t compare_v;

    //软件指针合法性检测，电压数值合法性检测以及输入直流母线电压检测
    if ((pwm == NULL) || (pwm->htim == NULL))
    {
        return false;
    }

    if ((!isfinite(vdc)) || (!isfinite(v_conv_ref)))
    {
        PFC_PWM_SetZeroVoltage(pwm);
        return false;
    }

    if (vdc < pwm->vdc_min  || vdc > pwm->vdc_max)
    {
        PFC_PWM_SetZeroVoltage(pwm);
        return false;
    }

    //调制比 modulation = vUV_ref / Vdc
    modulation = v_conv_ref / vdc;

    //提前对调制比限制
    if(modulation < pwm->modulation_min)
    {
        modulation = pwm->modulation_min;
    }
    else if(modulation > pwm->modulation_max)
    {
        modulation = pwm->modulation_max;
    }


    duty_u = 0.5f + 0.5f * modulation;
    duty_v = 0.5f - 0.5f * modulation;

    //占空比分别限幅处理
    if(duty_u < pwm->duty_min)
    {
        duty_u = pwm->duty_min;
    }
    else if(duty_u > pwm->duty_max)
    {
        duty_u = pwm->duty_max;
    }

    if(duty_v < pwm->duty_min)
    {
        duty_v = pwm->duty_min;
    }
    else if(duty_v > pwm->duty_max)
    {
        duty_v = pwm->duty_max;
    }

    //ccr计算
    compare_u = DutyToCompare(pwm, duty_u);
    compare_v = DutyToCompare(pwm, duty_v);


    __HAL_TIM_SET_COMPARE(pwm->htim,pwm->channel_u,compare_u);

    __HAL_TIM_SET_COMPARE(pwm->htim,pwm->channel_v,compare_v);

    return true;
}