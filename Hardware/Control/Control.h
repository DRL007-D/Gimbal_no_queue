#ifndef __CONTROL_H
#define __CONTROL_H

#include "system.h"

typedef struct {
	// __IO CanRxMsg CAN_RxMsg;
	// __IO CanTxMsg CAN_TxMsg;
	__IO bool rxFrameFlag;
	__IO uint8_t packNum;
	__IO uint8_t pack_1[8];
	__IO uint8_t pack_2[8];
	__IO uint8_t pack[256];
}CAN_t;
extern __IO CAN_t can;

bool  Gimbal_Control_Wait(void);
void K230_Control(void);
void USART1_IRQ(void);
void CAN1_RX0_IRQ(void);

#endif
