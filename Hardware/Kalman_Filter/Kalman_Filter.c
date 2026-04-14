#include "Kalman_Filter.h"

Kalman_Filter_t Kalman_Filter_Data = {0.02, 0.0, 0.02, 0.0, 0.01, 0.1};
float Kalman_Filter(Kalman_Filter_t *ekf, float input)
{
    ekf->Now_P = ekf->Last_P + ekf->Q;
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
    ekf->OUT = ekf->OUT + ekf->Kg * (input - ekf->OUT);
    ekf->Last_P = (1 - ekf->Kg) * ekf->Now_P;
    
    return ekf->OUT;  // 返回滤波后的结果
}


