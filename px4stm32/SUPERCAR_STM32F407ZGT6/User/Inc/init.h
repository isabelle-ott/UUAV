#pragma once
#include "encoder.h"

// 确保All_Init()能被C/C++文件调用（extern "C"处理）
#ifdef __cplusplus
extern "C"
{
#endif

    // 声明硬件初始化函数（在init.cpp中实现）
    void All_Init(void);

#ifdef __cplusplus
}
#endif
