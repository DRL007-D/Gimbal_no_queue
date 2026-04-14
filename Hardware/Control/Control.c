#include "control.h"
#include "stm32f1xx.h"

__IO CAN_t can = {0};
uint8_t receive_num = 0;
uint32_t angle_Real_INT = 0;
float  angle_Real_Float = 0;
uint16_t usart_f_count = 0;

#define FULL_ANGLE_360_CLK 51200
#define Every_Angle_Pulse FULL_ANGLE_360_CLK/360.0f
static uint8_t state = 0;

// 360°目标搜索主函数
void Target_Search_360(void){
    if(USART1_Data.rx_Buffer_wholl_Usart[USART1_Data.rx_Buffer_Usart_Length] == false)//串口接收到数据的时候才执行PID控制，否则执行360°目标寻找
    {
		static float now_angle =0;
        //脉冲数51200（256细分下发送51200个脉冲电机转一圈），相对运动,相对运动相对是上一个状态的运动，不需要位置清零
		switch (state){
			case 0:
				Emm_V5_Read_Sys_Params(Gimbal_Down,S_CPOS);
				// 不阻塞等待，改为状态机等待
				state = 1;
				break;
			case 1:
				if(Gimbal_Control_Wait())
				{
					//delay_us(150);
					now_angle = angle_Real_Float;
					uint8_t control_speed = 10;//闭环的控制速度
					Emm_V5_Pos_Control(Gimbal_Down,0,control_speed,0,FULL_ANGLE_360_CLK,0,0);
					state = 2;
				}
				break;
			case 2:
				if(Gimbal_Control_Wait())
				{
					Emm_V5_Read_Sys_Params(Gimbal_Down,S_CPOS);
					state = 3;
				}
				break;
			case 3:
				if(Gimbal_Control_Wait())
				{
					uint8_t control_speed = 10;//闭环的控制速度
					Emm_V5_Pos_Control(Gimbal_Up,0,control_speed,0,100,0,0);
					state = 0;
				}
				break;			
			
			default:
			break;

		}
		printf("now_angle%.2f\r\n real_angle%.2f\r\nstate:%d\r\n",
		now_angle,
		angle_Real_Float,
		state);

    }
}

bool  Gimbal_Control_Wait(void)//等待CAN通信完成帧，这里没有使用超时等待会导致系统卡死
{
	if(can.rxFrameFlag == true)
    {
        can.rxFrameFlag = false;
        return true;
    }
    return false;
}

void Gimbal_while(void)//建议使用这个计时等待
{
	while(can.rxFrameFlag == false)
	{
		static uint8_t count = 0;
		if(++count>5){
			count = 0;
			break;
		}
	}
	can.rxFrameFlag = false;
}
#define FILTER_SIZE 8
uint16_t filtered_x = 0;  // X轴滤波状态
uint16_t filtered_y = 0;  // Y轴滤波状态
//X轴一阶滤波
uint16_t lowpass_filter_x(uint16_t new_value) {
    filtered_x = (filtered_x * 1 + new_value) >> 1;  // α = 1/2，响应更快
    return filtered_x;
}
//Y轴一阶滤波
uint16_t lowpass_filter_y(uint16_t new_value) {
    filtered_y = (filtered_y * 1 + new_value) >> 1;  // α = 1/2
    return filtered_y;
}
/*K230的读取坐标实行卡尔曼，实际测试效果不好，不要使用*/
#include "Kalman_Filter.h"
Kalman_Filter_t kalman_x, kalman_y;
void Kalman_Init(Kalman_Filter_t *ekf) {
    ekf->Last_P = 0.01f;  // 初始误差协方差
    ekf->Now_P = 0.0f;
    ekf->OUT = 0.02f;     // 初始输出值
    ekf->Kg = 0.0f;
    ekf->Q = 0.01f;      // 过程噪声协方差（可调）
    ekf->R = 0.01f;       // 测量噪声协方差（可调）
}

static uint16_t led_detect_state = 0;
static uint16_t led_none_state = 0;
uint8_t led_detect_FPS = 0;

void K230_Control(void)
{
	static uint32_t Now_time = 0,Last_time = 0,err = 0;
	Now_time = HAL_GetTick();
	err = Now_time - Last_time;
	if(err>=1000){
		Last_time = Now_time;
		led_detect_FPS = 0;
	}
//    Kalman_Init(&kalman_x);
//    Kalman_Init(&kalman_y);
    int x_angle_temp = 0,y_angle_temp = 0;
	//串口接收到数据的时候才执行PID控制，LED快闪说明串口通信正常，否则没有进入串口中断或者中断有问题
    if(USART1_Data.rx_Buffer_wholl_Usart[USART1_Data.rx_Buffer_Usart_Length] == true)
    {
		usart_f_count = 0;
		state = 0;
		led_detect_FPS++;//帧率检测
		led_detect_state++;//LED快闪
		(led_detect_state>=3)?(GPIOC -> BSRR = GPIO_PIN_13):(GPIOC -> BSRR = (GPIO_PIN_13<<16));
		led_detect_state%=3*2;
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,(led_state?GPIO_PIN_SET:GPIO_PIN_RESET));
	
        // (float)USART1_Data.X_uint16_K230 = Kalman_Filter(&kalman_x, (float)USART1_Data.X_uint16_K230);
        // (float)USART1_Data.Y_uint16_K230 = Kalman_Filter(&kalman_y, (float)USART1_Data.Y_uint16_K230);
        // USART1_Data.X_uint16_K230 = lowpass_filter_x( USART1_Data.X_uint16_K230);
        // USART1_Data.Y_uint16_K230 = lowpass_filter_y( USART1_Data.Y_uint16_K230);
        static uint16_t X_temp = 0 ;
        static uint16_t Y_temp = 0 ;
        X_temp = USART1_Data.X_uint16_K230;
        Y_temp = USART1_Data.Y_uint16_K230;

        x_angle_temp = Gimbal_Down_Angle_PID(X_temp,Gimbal_PID_Data.Down_Gimbal_Angle_Target);//400
        y_angle_temp = Gimbal_Up_Angle_PID(Y_temp,Gimbal_PID_Data.Up_Gimbal_Angle_Target);//190,427
        Gimbal_PID_Data.Down_Gimbal_Angle_Out = x_angle_temp;
        Gimbal_PID_Data.Up_Gimbal_Angle_Out = y_angle_temp;
		if(Gimbal_PID_Data.Down_Gimbal_Angle_Out>0)//张大头步进电机闭环下部云台控制命令
        {
			Emm_V5_Pos_Control(Gimbal_Down,1,5000,0,x_angle_temp,0,0);//速度最快，直接控制，不使用加减速
			Gimbal_while();
        }
        if(Gimbal_PID_Data.Down_Gimbal_Angle_Out<0)
        {
			Emm_V5_Pos_Control(Gimbal_Down,0,5000,0,-x_angle_temp,0,0);
			Gimbal_while();
        }
        if(Gimbal_PID_Data.Up_Gimbal_Angle_Out>0)//张大头步进电机闭环上部云台控制命令
        {  
            Emm_V5_Pos_Control(Gimbal_Up,1,5000,0,y_angle_temp,0,0);
            Gimbal_while();
        }
        if(Gimbal_PID_Data.Up_Gimbal_Angle_Out<0)
        {
            Emm_V5_Pos_Control(Gimbal_Up,0,5000,0,-y_angle_temp,0,0);
			Gimbal_while();
        }  
        USART1_Data.rx_Buffer_wholl_Usart[USART1_Data.rx_Buffer_Usart_Length] = false;//末尾标志位复位
    }
    else//没有串口数据的时候清空PID计算值，LED灯慢闪
    {		
		usart_f_count++;
		if(usart_f_count>=1000)//1s内没有识别到目标就旋转搜寻
		{
			usart_f_count = 0;
			Target_Search_360();//云台旋转搜索目标
			led_none_state = ~ led_none_state;//LED慢闪
			(led_none_state)?(GPIOC -> BSRR = GPIO_PIN_13):(GPIOC -> BSRR = (GPIO_PIN_13<<16));
			Gimbal_PID_Data.Down_Gimbal_sum = 0;
			Gimbal_PID_Data.Up_Gimbal_sum = 0;
		}

    }	

}

//K230的帧头帧尾固定2C、5B，可以自己简单结合其他功能的识别，比如23年比赛的色块识别
void K230_Header_Tail_Check(uint8_t* Rx_String_Buffer,size_t  packet_length) // K230包尾检查函数，本版本加入K230胜利信号
{   
    if((Rx_String_Buffer[0] == 0x2C) && (Rx_String_Buffer[packet_length-1] == 0x5B))//导数第二位为0x5B，最后一位已经赋值为'\0'
    {
        USART1_Data.X_uint16_K230 = (uint16_t)(Rx_String_Buffer[1] << 8) | (uint16_t)Rx_String_Buffer[2];//X轴字节组合
        USART1_Data.Y_uint16_K230 = (uint16_t)(Rx_String_Buffer[3] << 8) | (uint16_t)Rx_String_Buffer[4];//Y轴字节组合
        // Ring_Flag_K230 = Rx_String_Buffer[5];
        // printf("X:%d,Y:%d\r\n",USART1_Data.X_uint16_K230,USART1_Data.Y_uint16_K230);
        // 修正后的代码片段
        if(USART1_Data.X_uint16_K230 || USART1_Data.Y_uint16_K230)
        {
            Rx_String_Buffer[packet_length] = true;//末尾更新为接受完成标志位
            Rx_String_Buffer[packet_length+1] = '\0';//末尾之后添加字符串终止符
        }
        else//如果坐标值未识别到，则置为未完成标志位
        {
            Rx_String_Buffer[packet_length] = false;//末尾更新为接受未完成标志位
            Rx_String_Buffer[packet_length+1] = '\0';//末尾之后添加字符串终止符
        }
    }
    else {}

}


void USART1_IRQ(void)//USART1_Data.rx_Buffer_data_wholl
{
    #if USART1_DMA == 1
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))	// 1. 获得串口空闲中断标志	
        {                               
            __HAL_UART_CLEAR_IDLEFLAG(&huart1);	            // 2. 清除空闲中断标志
            HAL_UART_DMAStop(&huart1);	                    // 3. 清除->空闲->停止DMA接收		
            USART1_Data.rx_Buffer_Usart_Length = 1024 - hdma_usart1_rx.Instance->CNDTR;          
            // 4. 设定的传输长度-剩余传输数量（DMA_CNDTRx）=实际长度，NDTR寄存器为串口DMA已经接收的数据长度
            USART1_Data.rx_Buffer_Usart[USART1_Data.rx_Buffer_Usart_Length]='\0';//已经接收的数组末尾置'\0'
            memcpy(USART1_Data.rx_Buffer_wholl_Usart,USART1_Data.rx_Buffer_Usart,sizeof(USART1_Data.rx_Buffer_Usart));//复制接收缓冲区rx_Buffer_wholl_Usart3
            // 添加调试输出观察实际接收内容
//            printf("RX Data: [%s] Len:%d\n", 
//            USART1_Data.rx_Buffer_wholl_Usart, 
//            strlen((const char*)USART1_Data.rx_Buffer_wholl_Usart));
			/*K230的包头包尾参数检测*/
			K230_Header_Tail_Check(USART1_Data.rx_Buffer_wholl_Usart,USART1_Data.rx_Buffer_Usart_Length);

            memset(USART1_Data.rx_Buffer_Usart,0,sizeof(USART1_Data.rx_Buffer_Usart));//清空接收缓冲区
            HAL_UART_Receive_DMA(&huart1, USART1_Data.rx_Buffer_Usart, 1024);// 6. 重新开启DMA
        }
    #endif
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart -> Instance == USART1)
//	{
//		//USART1_IRQ();
//	}
//}

#define pack_analyze 0 //CAN通信解包分析

void CAN1_RX0_Data(uint8_t packnum,uint8_t *rxData)
{
    static uint8_t packnum_temp_Show = 0;
    /*多包检测复制*/
    packnum_temp_Show = packnum;
    memcpy((void *)&can.pack[packnum *8], (void *)rxData, 8);
    for (int j = 0; j < (packnum+1)*8; j++)
    {
        packnum_temp_Show++;
        //if(packnum_temp_Show)//多包检测复制
        //printf("can.pack[%d]: 0x%02X\r\n", j, can.pack[j]);
    }    
#if pack_analyze
    static uint8_t packnum_temp = 0,
    packnum_temp = packnum;
    if(packnum_temp ==0)
    {
        memcpy((void *)KP_INT_data.kp, (void *)&can.pack[packnum *8+1], 4);
        memcpy((void *)KI_INT_data.ki, (void *)&can.pack[packnum *8+5], 3);
    }
    else if (packnum_temp ==1)
    {
        memcpy((void *)&KI_INT_data.ki[3], (void *)&can.pack[packnum *8+1], 1);
        memcpy((void *)KD_INT_data.kd, (void *)&can.pack[packnum *8+2], 4);     
        packnum_temp++;     
    }
    if(packnum_temp>1)
    {
        KP_INT_data.Kp = __REV(KP_INT_data.Kp);
        KI_INT_data.Ki = __REV(KI_INT_data.Ki);
        KD_INT_data.Kd = __REV(KD_INT_data.Kd);
        // 打印整数数组内容（假设kp、ki、kd是uint8_t数组）
        for (int i = 0; i < 4; i++)
        {
            printf("KP_INT_data.kp[%d]: 0x%02X\r\n", i, KP_INT_data.kp[i]);
        }
        for (int i = 0; i < 4; i++)
        {
            printf("KI_INT_data.ki[%d]: 0x%02X\r\n", i, KI_INT_data.ki[i]);
        }                
        for (int i = 0; i < 4; i++)
        {
            printf("KD_INT_data.kd[%d]: 0x%02X\r\n", i, KD_INT_data.kd[i]);
        }
        printf("KP: %d\r\n", KP_INT_data.Kp);
        printf("KI: %d\r\n", KI_INT_data.Ki);
        printf("KD: %d\r\n", KD_INT_data.Kd);


    }
#endif
}
void Angle_Read(void)
{
    if(can.pack[0] == 0x36)
    {
        angle_Real_INT = can.pack[2] <<24 | can.pack[3] <<16 | can.pack[4] <<8 | can.pack[5];
        // 转换成角度
        angle_Real_Float = (float)angle_Real_INT * 360.0f / 65536.0f;
        if(can.pack[1]==1)
        {
            angle_Real_Float = -angle_Real_Float;
        }
    }
}
/**
	* @brief   CAN1_RX0接收中断
	* @param   无
	* @retval  无
	*/
void CAN1_RX0_IRQ(void)
{
    // 判断FIFO0是否有数据
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
    {
        // 接收消息结构体
        CAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8];
        // 获取接收数据
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        {
            CAN1_RX0_Data(can.packNum,rxData);
            Angle_Read();
            // printf("angle:%f\r\n",angle_Real_Float);
            // 更新包计数器和标志位
            //can.packNum++;
            receive_num++;
            receive_num%=2;
			
			can.rxFrameFlag = true;

        }
    }
}

