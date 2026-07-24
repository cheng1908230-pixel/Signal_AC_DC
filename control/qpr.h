#ifndef __QPR_H
#define __QPR_H

 /**
  * @brief 准比例谐振控制器
  * 
  * @note 外部只需要输入控制器参数并进行初始化即可，其他变量在初始化中
  *       均进行了计算，使用时一定要记住初始化结构体
  *       初始化成员：Kp、Kr、omega_c、omega_0、T、output_max、output_min
 */
typedef struct {
    // 控制器状态变量
    float u_prev1, u_prev2;  // 历史输入 u[k-1], u[k-2]
    float y_prev1, y_prev2;  // 历史输出 y[k-1], y[k-2]
    // 归一化差分方程系数，对a0归一化
    float b0, b1, b2, a1, a2;

    // 控制器参数
    float Kp;      // 比例增益
    float Kr;      // 谐振增益
    float omega_c; // 截止角频率
    float omega_0; // 谐振角频率
    float T;       // 采样周期

    float output_max; // 输出上限
    float output_min; // 输出下限

    //输出值
    float output;
} QPRController;

void QPRController_Init(QPRController *qpr);
void QPRController_Update(QPRController *qpr, float input);

#endif