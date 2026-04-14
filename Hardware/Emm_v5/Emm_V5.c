#include "Emm_V5.h"
KP_INT_params KP_INT_data;
KI_INT_params KI_INT_data;
KD_INT_params KD_INT_data;

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/
/**
  * @brief  CAN过滤器配置
  * @note   配置CAN接收过滤器参数并启动CAN总线
  * @param  None
  * @retval None
  */
void CAN1_Filter_Bank(void)
{
	// 设置STM32的帧ID - 扩展帧格式 - 不过滤任何数据帧
	__IO uint8_t id_o, im_o; __IO uint16_t id_l, id_h, im_l, im_h;
	id_o = (0x00);
	id_h = (uint16_t)((uint16_t)id_o >> 5);										 // 高3位
	id_l = (uint16_t)((uint16_t)id_o << 11) | CAN_ID_EXT; // 低5位
	im_o = (0x00);
	im_h = (uint16_t)((uint16_t)im_o >> 5);
	im_l = (uint16_t)((uint16_t)im_o << 11) | CAN_ID_EXT;
    
    CAN_FilterTypeDef FilterConfig;
    /* 过滤器参数配置 */
    FilterConfig.FilterBank = 1;                     // 使用过滤器组1
    FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // 标识符掩码模式
    FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;// 32位过滤器尺度
    FilterConfig.FilterIdHigh = id_h;									// 过滤器标识符的高16位值
    FilterConfig.FilterIdLow = id_l;										// 过滤器标识符的低16位值
    FilterConfig.FilterMaskIdHigh = im_h;							// 过滤器屏蔽标识符的高16位值
    FilterConfig.FilterMaskIdLow = im_l;								// 过滤器屏蔽标识符的低16位值
    FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;// 使用接收FIFO0
    FilterConfig.FilterActivation = ENABLE;          // 使能过滤器
    
    /* 应用过滤器配置 */
    if (HAL_CAN_ConfigFilter(&hcan, &FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* 启动CAN通信并激活接收中断 */
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
}

/**
  * @brief  CAN命令发送函数
  * @note   发送多包CAN数据命令
  * @param  cmd: 命令数据指针（包含地址和有效数据）
  * @param  len: 命令数据总长度
  * @retval None
  */
#define MAX_CMD_QUEUE 5       // 最大命令队列长度
#define MAX_PACKETS 10        // 单条命令最大分包数
#define MAX_CMD_LEN 64        // 单条命令最大长度（可根据需求调整）

typedef struct {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
} CAN_Packet_t;

typedef struct {
    uint8_t data[MAX_CMD_LEN];
    uint8_t len;
} CAN_Cmd_t;

typedef struct {
    CAN_Cmd_t cmds[MAX_CMD_QUEUE];
    uint8_t head;            // 队头索引
    uint8_t tail;            // 队尾索引
    uint8_t count;           // 队列中命令数量
} CAN_CmdQueue_t;

typedef struct {
    CAN_Packet_t packets[MAX_PACKETS];
    uint8_t totalPackets;
    uint8_t currentPacket;
    bool sending;
} CAN_SendBuffer_t;

static CAN_CmdQueue_t canCmdQueue = {0};
static CAN_SendBuffer_t canSendBuffer = {0};
void can_SendNextCmd(void);
bool can_EnqueueCmd(__IO uint8_t *cmd, uint8_t len);
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    if(!can_EnqueueCmd( cmd, len))
    {
        // 入队失败处理，比如队列满了
        Error_Handler();
    }
}

// 入队命令，返回是否成功
bool can_EnqueueCmd(__IO uint8_t *cmd, uint8_t len)
{
    if(canCmdQueue.count >= MAX_CMD_QUEUE || len > MAX_CMD_LEN)
        return false; // 队列满或命令过长

    // 复制命令数据
    uint8_t idx = canCmdQueue.tail;
    memcpy(canCmdQueue.cmds[idx].data, (void *)cmd, len);
    canCmdQueue.cmds[idx].len = len;

    canCmdQueue.tail = (canCmdQueue.tail + 1) % MAX_CMD_QUEUE;
    canCmdQueue.count++;

    // 如果当前没有发送任务，启动发送
    if(!canSendBuffer.sending)
    {
        can_SendNextCmd();
    }

    return true;
}

// 从队列取出一条命令，准备分包发送
void can_SendNextCmd(void)
{
    if(canCmdQueue.count == 0)
    {
        // 队列空，无任务
        canSendBuffer.sending = false;
        return;
    }

    uint8_t idx = canCmdQueue.head;
    uint8_t *cmd = canCmdQueue.cmds[idx].data;
    uint8_t len = canCmdQueue.cmds[idx].len;

    // 分包准备（参考你原来的can_SendCmd逻辑）
    uint8_t remaining = len - 2;  // 排除地址和功能码
    uint8_t dataOffset = 0;
    uint8_t packetNum = 0;

    canSendBuffer.totalPackets = 0;
    canSendBuffer.currentPacket = 0;
    canSendBuffer.sending = true;

    while(remaining > 0 && canSendBuffer.totalPackets < MAX_PACKETS)
    {
        CAN_Packet_t *pkt = &canSendBuffer.packets[canSendBuffer.totalPackets];

        pkt->TxHeader.ExtId = ((uint32_t)cmd[0] << 8) | packetNum;
        pkt->TxHeader.IDE = CAN_ID_EXT;
        pkt->TxHeader.RTR = CAN_RTR_DATA;

        uint8_t payloadLen = (remaining < 8) ? remaining : 7;
        pkt->TxHeader.DLC = payloadLen + 1;

        pkt->TxData[0] = cmd[1];  // 功能码
        for(uint8_t i = 0; i < payloadLen; i++)
        {
            pkt->TxData[i+1] = cmd[dataOffset + 2 + i];
        }

        remaining -= payloadLen;
        dataOffset += payloadLen;
        packetNum++;
        canSendBuffer.totalPackets++;
    }

    // 发送第一包
    uint32_t TxMailbox;
    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        canSendBuffer.sending = false;
        Error_Handler();
        return;
    }

    if(HAL_CAN_AddTxMessage(&hcan, &canSendBuffer.packets[0].TxHeader, canSendBuffer.packets[0].TxData, &TxMailbox) != HAL_OK)
    {
        canSendBuffer.sending = false;
        Error_Handler();
        return;
    }
}

// 发送完成中断回调
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if(canSendBuffer.sending)
    {
        canSendBuffer.currentPacket++;
        if(canSendBuffer.currentPacket < canSendBuffer.totalPackets)
        {
            uint32_t TxMailbox;
            HAL_CAN_AddTxMessage(hcan,
                &canSendBuffer.packets[canSendBuffer.currentPacket].TxHeader,
                canSendBuffer.packets[canSendBuffer.currentPacket].TxData,
                &TxMailbox);
        }
        else
        {
            // 当前命令发送完成，出队
            canCmdQueue.head = (canCmdQueue.head + 1) % MAX_CMD_QUEUE;
            canCmdQueue.count--;

            // 发送下一条命令
            if(canCmdQueue.count > 0)
            {
                can_SendNextCmd();
            }
            else
            {
                canSendBuffer.sending = false;
                // 这里可以添加发送完成通知
            }
        }
    }
}


/**
  * @brief  CAN标准数据发送函数
  * @note   发送单包CAN标准数据帧
  * @param  TxHeader: 发送帧头配置结构体
  * @param  TxData: 待发送数据指针
  * @param  length: 数据长度（0-8）
  * @retval uint8_t: 发送状态 1-成功
  */
uint8_t CANx_SendNormalData(CAN_TxHeaderTypeDef TxHeader,uint8_t *TxData ,uint16_t length)
{
    uint32_t TxMailbox; 
    
    /* 帧头参数配置 */
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;          // 使用扩展帧
    TxHeader.DLC = length;              // 数据长度代码
    
    /* 等待可用发送邮箱 */
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
    
    /* 执行数据发送 */
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
       Error_Handler();
    }
    return 1;
}
/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 功能码
  cmd[2] =  0x52;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, i);
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 6);
}

/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0是直接启动
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 8);
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 13);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 功能码
  cmd[2] =  0x88;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 功能码
  cmd[2] =  0xAE;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
  cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
  cmd[19] =  0x6B;                      // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 功能码
  cmd[2] =  0x48;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 4);
}
