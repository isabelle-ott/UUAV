#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "tim.h"

// 编码器计数值获取
int32_t Encoder_LeftFront_GetCounter(void);
int32_t Encoder_RightFront_GetCounter(void);
int32_t Encoder_RightRear_GetCounter(void);
int32_t Encoder_LeftRear_GetCounter(void);

// 编码器计数值重置
void Encoder_LeftFront_ResetCounter(void);
void Encoder_RightFront_ResetCounter(void);
void Encoder_RightRear_ResetCounter(void);
void Encoder_LeftRear_ResetCounter(void);

#endif
