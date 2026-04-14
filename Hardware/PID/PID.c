#include "PID.h"

Gimbal_PID_params Gimbal_PID_Data;

int Gimbal_Down_Angle_PID(int gimbal_Pixel_Measure, int gimbal_Angel_Target)
{
    float sum = 0;
    static float Angel_encoder_err = 0;
    static float last_err = 0;
    static float err_difference = 0;
    
    // Feedforward control variables
    //static float last_target = 0;
    static float target_velocity = 0;
    static float target_acceleration = 0;
    //static float last_target_velocity = 0;
    
    // Calculate target velocity and acceleration for feedforward
    // target_velocity = gimbal_Angel_Target - last_target;
    // target_acceleration = target_velocity - last_target_velocity;
    
    // Update previous values
    // last_target = gimbal_Angel_Target;
    // last_target_velocity = target_velocity;
    
    // PID error calculation
    Angel_encoder_err = gimbal_Pixel_Measure - gimbal_Angel_Target;
    
    // Integral term with windup protection
    Gimbal_PID_Data.Down_Gimbal_sum += Angel_encoder_err;
    if(Gimbal_PID_Data.Down_Gimbal_sum > 3000) 
        Gimbal_PID_Data.Down_Gimbal_sum = 3000;
    if(Gimbal_PID_Data.Down_Gimbal_sum < -3000) 
        Gimbal_PID_Data.Down_Gimbal_sum = -3000;
    
    // Derivative term
    err_difference = Angel_encoder_err - last_err;
    last_err = Angel_encoder_err;
    
    // Feedforward terms
    float feedforward_velocity = Gimbal_PID_Data.Down_Gimbal_Kff_v * target_velocity;
    float feedforward_acceleration = Gimbal_PID_Data.Down_Gimbal_Kff_a * target_acceleration;
    
    // PID + Feedforward calculation
    sum = Gimbal_PID_Data.Down_Gimbal_Kp * Angel_encoder_err +          // Proportional
          Gimbal_PID_Data.Down_Gimbal_Ki * Gimbal_PID_Data.Down_Gimbal_sum +  // Integral
          Gimbal_PID_Data.Down_Gimbal_Kd * err_difference +             // Derivative
          feedforward_velocity +                                        // Velocity feedforward
          feedforward_acceleration;                                     // Acceleration feedforward
    
    // Output saturation
    sum = sum > 2000 ? 2000 : sum < -2000 ? -2000 : sum;
    
    return sum;
}


int Gimbal_Up_Angle_PID(int gimbal_Pixel_Measure,int gimbal_Angel_Target)
{
	float sum = 0;
	static float Angel_encoder_err  = 0;
	static float last_err = 0;
	static float err_difference = 0;

	Angel_encoder_err =  gimbal_Pixel_Measure - gimbal_Angel_Target;
	Gimbal_PID_Data.Up_Gimbal_sum += Angel_encoder_err;
	if(Gimbal_PID_Data.Up_Gimbal_sum>800) Gimbal_PID_Data.Up_Gimbal_sum = 800;
	if(Gimbal_PID_Data.Up_Gimbal_sum<-800) Gimbal_PID_Data.Up_Gimbal_sum = -800;
	err_difference = Angel_encoder_err - last_err;
	last_err = Angel_encoder_err;

	sum=
    Gimbal_PID_Data.Up_Gimbal_Kp * Angel_encoder_err + 
    Gimbal_PID_Data.Up_Gimbal_Ki * Gimbal_PID_Data.Up_Gimbal_sum + 
    Gimbal_PID_Data.Up_Gimbal_Kd * err_difference;
	sum = sum > 400 ? 400 : sum<-400 ? -400 : sum;
	return sum;
}


void PID_Set_Gimbal_Target(int Down_Gimbal_Angle_Target,int Up_Gimbal_Angle_Target)
{
    Gimbal_PID_Data.Down_Gimbal_Angle_Target = Down_Gimbal_Angle_Target;
    Gimbal_PID_Data.Up_Gimbal_Angle_Target = Up_Gimbal_Angle_Target;
}

