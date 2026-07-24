/**
 * @file     pid.c
 * @brief    PID 控制器实现
 *
 * @note     执行软启动可以在输入环节或者最终输出环节进行，
 *           这里的PID控制器是中间环节，不需要增加任何软启动
 */

#include "pid.h"

/**
 * @brief  初始化 PID 控制器
 *
 * @param  pid            指向 PID 实例结构体的指针
 * @param  kp             比例增益
 * @param  ki             积分增益
 * @param  kd             微分增益
 * @param  maxintegral    积分限幅
 * @param  maxoutput      最大输出限幅
 * @param  minoutput      最小输出限幅
 * 
 * @note  初始化 PID 控制器时，所有内部状态（误差、积分、输出）都会被清零，确保控制器从初始状态开始工作。
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float maxintegral, float maxoutput,float minoutput) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->maxintegral = maxintegral;
    pid->maxoutput = maxoutput;
    pid->error = 0.0f;
    pid->lasterror = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->minoutput = minoutput;
}

/**
 * @brief  PID 控制计算函数
 *
 * @param  pid            指向 PID 实例结构体的指针
 * @param  reference      参考值（目标值）
 * @param  feedback       反馈值（实际值）
 * 
 * @note  与 QPR 控制器相同，需要周期性的调用该函数进行控制计算
 */
void PID_Calc(PID_Controller *pid, float reference, float feedback)
{
 	//更新数据
    pid->lasterror = pid->error; //将旧error存起来
    pid->error = reference - feedback; //计算新error
    //计算微分
    float dout = (pid->error - pid->lasterror) * pid->kd;
    //计算比例
    float pout = pid->error * pid->kp;
    //计算积分(更新积分结果)
    pid->integral += pid->error * pid->ki;
    //积分限幅
    if(pid->integral > pid->maxintegral) pid->integral = pid->maxintegral;
    else if(pid->integral < -pid->maxintegral) pid->integral = -pid->maxintegral;
    //计算输出
    pid->output = pout + dout + pid->integral;
    //输出限幅
    if(pid->output > pid->maxoutput) pid->output =   pid->maxoutput;
    else if(pid->output < pid->minoutput) pid->output = pid->minoutput;
}