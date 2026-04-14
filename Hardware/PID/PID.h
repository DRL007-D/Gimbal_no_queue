#ifndef PID_H__
#define PID_H__

#include "system.h"

typedef struct
{
    //ÔÆÌ¨½Ç¶È»·
    float Down_Gimbal_Kp;
    float Down_Gimbal_Ki;
    float Down_Gimbal_Kd;
    float Down_Gimbal_sum;
    float Down_Gimbal_Kff_v;    // Velocity feedforward gain
    float Down_Gimbal_Kff_a;    // Acceleration feedforward gain
    
    float Up_Gimbal_Kp;
    float Up_Gimbal_Ki;
    float Up_Gimbal_Kd;
    float Up_Gimbal_sum;

    int Down_Gimbal_Angle_Out;
    int Up_Gimbal_Angle_Out;

    int Down_Gimbal_Angle_Target;
    int Up_Gimbal_Angle_Target;

}Gimbal_PID_params;

extern Gimbal_PID_params Gimbal_PID_Data;
int Gimbal_Down_Angle_PID(int gimbal_Pixel_Measure,int gimbal_Angel_Target);
int Gimbal_Up_Angle_PID(int gimbal_Pixel_Measure,int gimbal_Angel_Target);
void PID_Set_Gimbal_Target(int Down_Gimbal_Angle_Target,int Up_Gimbal_Angle_Target);

#endif
