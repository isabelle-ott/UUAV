#ifndef LIGHT_H
#define LIGHT_H

#include "stm32f4xx_hal.h"

// 灯光控制核心接口（仅控制PD9，无需结构体实例）
/**
 * @brief 灯光初始化（确认PD9状态，无实际GPIO初始化）
 * @note PD9需提前通过MX_GPIO_Init()配置为推挽输出
 */
void LightInit(void);

/**
 * @brief 打开灯光（PD9置高电平）
 */
void LightOn(void);

/**
 * @brief 关闭灯光（PD9置低电平）
 */
void LightOff(void);

/**
 * @brief 灯光测试（循环开关，验证功能）
 * @param test_time_ms：测试总时长（ms）
 * @param interval_ms：开关间隔（ms，建议500~2000）
 */
void LightTest(uint32_t test_time_ms, uint32_t interval_ms);

#endif // LIGHT_H
