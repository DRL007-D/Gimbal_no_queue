#include "system.h"

usart_params USART1_Data;

void sys_init(void) 
{
    #if DWT_Delay
    DWT_Init();//使用DWT更高精度的延时
    #elif SYS_Tick_Delay
    delay_init(72);
    #endif
    system_Params_Init();
    __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);	//使能空闲中断
    HAL_UART_Receive_DMA(&huart1,USART1_Data.rx_Buffer_Usart,1024);
//    OLED_Init();
//    OLED_ShowString(1,1,"come on");
}

void system_Params_Init(void) 
{
    Gimbal_PID_Data.Down_Gimbal_Kp = 2.0;
    Gimbal_PID_Data.Down_Gimbal_Ki = 0.0;
    Gimbal_PID_Data.Down_Gimbal_Kd = 0.0;
    Gimbal_PID_Data.Down_Gimbal_sum = 0;
    Gimbal_PID_Data.Down_Gimbal_Angle_Out  = 0;

    Gimbal_PID_Data.Up_Gimbal_Kp = 2.0;
    Gimbal_PID_Data.Up_Gimbal_Ki = 0.0;
    Gimbal_PID_Data.Up_Gimbal_Kd = 0.0;
    Gimbal_PID_Data.Up_Gimbal_sum = 0;
    Gimbal_PID_Data.Up_Gimbal_Angle_Out  = 0;

    memset(USART1_Data.rx_Buffer_Usart, 0, sizeof(USART1_Data.rx_Buffer_Usart));    
    memset(USART1_Data.rx_Buffer_Usart, 0, sizeof(USART1_Data.rx_Buffer_wholl_Usart));
    USART1_Data.rx_Buffer_Usart_Length = 0;
	USART1_Data.X_uint16_K230 = 0;
	USART1_Data.Y_uint16_K230 = 0;
}

