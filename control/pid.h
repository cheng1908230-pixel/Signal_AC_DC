#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

 /**
  * @brief PID 控制器
 */
typedef struct {
    float kp, ki, kd;// 比例、积分、微分系数
    float error, lasterror;// 误差，上次误差
    float integral, maxintegral;// 积分，最大积分限幅
    float output, maxoutput, minoutput;// 输出，最大输出限幅
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float maxintegral, float maxoutput, float minoutput);
void PID_Calc(PID_Controller *pid, float reference, float feedback);

#ifdef __cplusplus
}
#endif

#endif // __PID_H