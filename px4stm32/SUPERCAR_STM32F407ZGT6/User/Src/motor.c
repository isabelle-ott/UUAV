#include "motor.h"

// -------------------------- 静态工具函数（仅保留ID检查） --------------------------
/**
 * @brief 检查电机ID合法性
 * @param id：电机ID
 * @return 合法返回1，非法返回0
 */
static uint8_t Motor_CheckID(MotorID id)
{
    return (id >= LEFT_FRONT && id < MOTOR_MAX) ? 1 : 0;
}

// -------------------------- 核心接口实现 --------------------------
void Motor_Init(Motor *motor,
                TIM_HandleTypeDef *pwm_tim,
                uint32_t pwm_period,
                const MotorSingleConfig *lf_config,
                const MotorSingleConfig *rf_config,
                const MotorSingleConfig *rr_config,
                const MotorSingleConfig *lr_config)
{
    // 入参合法性检查（避免空指针访问）
    if (motor == NULL || pwm_tim == NULL ||
        lf_config == NULL || rf_config == NULL ||
        rr_config == NULL || lr_config == NULL)
        return;

    // 1. 初始化定时器与PWM参数（与CubeMX配置一致）
    motor->pwm_tim = pwm_tim;
    motor->pwm_period = pwm_period;
    motor->max_pwm_value = pwm_period; // 最大PWM=周期（占空比100%）
    motor->min_pwm_value = 0;          // 最小PWM=0（占空比0%）

    // 2. 绑定4个电机的硬件配置（仅存储端口/引脚/通道，不初始化）
    motor->motor_config[LEFT_FRONT] = *lf_config;
    motor->motor_config[RIGHT_FRONT] = *rf_config;
    motor->motor_config[RIGHT_REAR] = *rr_config;
    motor->motor_config[LEFT_REAR] = *lr_config;

    // 3. 初始状态：停止所有电机（PWM设为0，方向设为默认）
    Motor_StopAll(motor);
}

void Motor_StartPWM(Motor *motor)
{
    if (motor == NULL || motor->pwm_tim == NULL)
        return;

    // 启动PWM定时器的所有通道（根据电机配置，避免重复启动）
    uint8_t ch_flag[4] = {0};
    for (MotorID id = LEFT_FRONT; id < MOTOR_MAX; id++)
    {
        uint32_t ch = motor->motor_config[id].pwm_ch;
        if (ch == TIM_CHANNEL_1 && ch_flag[0] == 0)
        {
            HAL_TIM_PWM_Start(motor->pwm_tim, ch);
            ch_flag[0] = 1;
        }
        else if (ch == TIM_CHANNEL_2 && ch_flag[1] == 0)
        {
            HAL_TIM_PWM_Start(motor->pwm_tim, ch);
            ch_flag[1] = 1;
        }
        else if (ch == TIM_CHANNEL_3 && ch_flag[2] == 0)
        {
            HAL_TIM_PWM_Start(motor->pwm_tim, ch);
            ch_flag[2] = 1;
        }
        else if (ch == TIM_CHANNEL_4 && ch_flag[3] == 0)
        {
            HAL_TIM_PWM_Start(motor->pwm_tim, ch);
            ch_flag[3] = 1;
        }
    }
}

void Motor_SetVelocity(Motor *motor, MotorID id, float velocity)
{
    // 入参合法性检查
    if (motor == NULL || !Motor_CheckID(id) || motor->pwm_tim == NULL)
        return;

    // 1. 限制速度范围（-100.0~100.0，对应占空比0~100%）
    if (velocity > 100.0f)
        velocity = 100.0f;
    if (velocity < -100.0f)
        velocity = -100.0f;

    // 2. 获取当前电机的配置（端口、引脚、PWM通道）
    MotorSingleConfig *config = &motor->motor_config[id];

    // 3. 控制电机方向（依赖MX_GPIO_Init()配置的输出模式）
    GPIO_PinState dir_state = (velocity >= 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(config->dir_port, config->dir_pin, dir_state);

    // 4. 计算PWM比较值（速度绝对值→占空比→CCR值）
    float speed_abs = fabsf(velocity);
    uint32_t pwm_value = (uint32_t)(motor->min_pwm_value +
                                    (motor->max_pwm_value - motor->min_pwm_value) * (speed_abs / 100.0f));

    // 5. 写入PWM比较寄存器（控制转速）
    switch (config->pwm_ch)
    {
    case TIM_CHANNEL_1:
        motor->pwm_tim->Instance->CCR1 = pwm_value;
        break;
    case TIM_CHANNEL_2:
        motor->pwm_tim->Instance->CCR2 = pwm_value;
        break;
    case TIM_CHANNEL_3:
        motor->pwm_tim->Instance->CCR3 = pwm_value;
        break;
    case TIM_CHANNEL_4:
        motor->pwm_tim->Instance->CCR4 = pwm_value;
        break;
    default:
        break; // 无效通道，不操作
    }
}

void Motor_StopAll(Motor *motor)
{
    if (motor == NULL)
        return;

    // 停止所有电机（PWM设为0，方向保持默认）
    Motor_SetVelocity(motor, LEFT_FRONT, 0.0f);
    Motor_SetVelocity(motor, RIGHT_FRONT, 0.0f);
    Motor_SetVelocity(motor, RIGHT_REAR, 0.0f);
    Motor_SetVelocity(motor, LEFT_REAR, 0.0f);
}
