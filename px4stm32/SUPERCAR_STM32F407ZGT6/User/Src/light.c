#include "light.h"
#include "stdio.h"

// 固定PD9引脚参数（无需外部配置）
#define LIGHT_PORT GPIOD
#define LIGHT_PIN GPIO_PIN_9

void LightInit(void)
{
    // 初始化默认关闭灯光（确保初始状态一致）
    LightOff();
    printf("[Light] PD9灯光模块初始化完成（默认关闭）\r\n");
}

void LightOn(void)
{
    // PD9置高电平
    HAL_GPIO_WritePin(LIGHT_PORT, LIGHT_PIN, GPIO_PIN_SET);
}

void LightOff(void)
{
    // PD9置低电平
    HAL_GPIO_WritePin(LIGHT_PORT, LIGHT_PIN, GPIO_PIN_RESET);
}

void LightTest(uint32_t test_time_ms, uint32_t interval_ms)
{
    if (test_time_ms == 0 || interval_ms == 0)
    {
        printf("[Light Test Error] 时长参数不能为0！\r\n");
        return;
    }

    uint32_t start_time = HAL_GetTick();
    uint32_t last_switch_time = start_time;
    uint8_t is_on = 0; // 0=关，1=开

    printf("[Light Test] 启动（总时长：%lu ms，间隔：%lu ms）\r\n", test_time_ms, interval_ms);

    // 循环切换灯光状态
    while (HAL_GetTick() - start_time < test_time_ms)
    {
        if (HAL_GetTick() - last_switch_time >= interval_ms)
        {
            is_on = !is_on;
            is_on ? LightOn() : LightOff();
            printf("[%lu ms] 灯光：%s\r\n", HAL_GetTick() - start_time, is_on ? "开" : "关");
            last_switch_time = HAL_GetTick();
        }
        HAL_Delay(10); // 降低CPU占用
    }

    // 测试结束：确保关闭灯光
    LightOff();
    printf("[Light Test] 结束（灯光已关闭）\r\n");
}
