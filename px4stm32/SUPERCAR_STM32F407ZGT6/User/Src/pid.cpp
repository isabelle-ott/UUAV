#include "pid.h"
#include <stddef.h>

// pid命名空间：实现模板类与全局PID对象
namespace pid
{
    // -------------------------- 模板类构造函数（通用初始化） --------------------------
    template <PID_TypeDef Type>
    PIDController<Type>::PIDController(float kp, float ki, float kd,
                                       float output_min, float output_max,
                                       float integral_min, float integral_max)
        : kp_(kp), ki_(ki), kd_(kd),
          output_min_(output_min), output_max_(output_max),
          integral_min_(integral_min), integral_max_(integral_max),
          current_error_(0.0f), last_error_(0.0f), integral_(0.0f)
    {
#if Type == PID_VELOCITY
        last_current_ = 0.0f; // 速度环初始化上一周期当前值
#endif
    }

    // -------------------------- 重置PID状态（通用逻辑） --------------------------
    template <PID_TypeDef Type>
    void PIDController<Type>::Reset()
    {
        current_error_ = 0.0f;
        last_error_ = 0.0f;
        integral_ = 0.0f;
#if Type == PID_VELOCITY
        last_current_ = 0.0f;
#endif
    }

    // -------------------------- 动态调整PID参数（通用逻辑） --------------------------
    template <PID_TypeDef Type>
    void PIDController<Type>::SetParams(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    // -------------------------- 设置积分限幅（通用逻辑） --------------------------
    template <PID_TypeDef Type>
    void PIDController<Type>::SetIntegralLimit(float integral_min, float integral_max)
    {
        integral_min_ = integral_min;
        integral_max_ = integral_max;
    }

    // -------------------------- 设置输出限幅（通用逻辑） --------------------------
    template <PID_TypeDef Type>
    void PIDController<Type>::SetOutputLimit(float output_min, float output_max)
    {
        output_min_ = output_min;
        output_max_ = output_max;
    }

    // -------------------------- 位置PID（PID_POSITION）计算逻辑 --------------------------
    template <>
    float PIDController<PID_POSITION>::Compute(float target, float current, float dt)
    {
        // 1. 计算当前误差（位置偏差=目标位置-当前位置）
        current_error_ = target - current;

        // 2. 比例项（P）：快速响应偏差
        float p_out = kp_ * current_error_;

        // 3. 积分项（I）：消除静态偏差（抗积分饱和）
        integral_ += current_error_ * dt;
        integral_ = fminf(fmaxf(integral_, integral_min_), integral_max_); // 积分限幅
        float i_out = ki_ * integral_;

        // 4. 微分项（D）：抑制超调（基于误差变化率）
        float derivative = (current_error_ - last_error_) / dt;
        float d_out = kd_ * derivative;

        // 5. 总控制量（输出限幅，防止执行器过载）
        float output = p_out + i_out + d_out;
        output = fminf(fmaxf(output, output_min_), output_max_);

        // 6. 缓存当前误差，供下一周期计算微分
        last_error_ = current_error_;

        return output;
    }

    // -------------------------- 速度PID（PID_VELOCITY）计算逻辑 --------------------------
    template <>
    float PIDController<PID_VELOCITY>::Compute(float target, float current, float dt)
    {
        // 1. 计算当前误差（速度偏差=目标速度-当前速度）
        current_error_ = target - current;

        // 2. 比例项（P）：快速响应速度偏差
        float p_out = kp_ * current_error_;

        // 3. 积分项（I）：消除速度静态偏差（抗积分饱和）
        integral_ += current_error_ * dt;
        integral_ = fminf(fmaxf(integral_, integral_min_), integral_max_); // 积分限幅
        float i_out = ki_ * integral_;

        // 4. 微分项（D）：抑制速度波动（基于当前值变化率，抗噪声）
        float derivative = (current - last_current_) / dt;
        float d_out = kd_ * derivative; // 速度环D项取负，抑制当前值突变

        // 5. 总控制量（输出限幅：PWM占空比-100~100）
        float output = p_out + i_out - d_out; // D项减：当前值增大时，抑制输出
        output = fminf(fmaxf(output, output_min_), output_max_);

        // 6. 缓存历史数据，供下一周期计算
        last_error_ = current_error_;
        last_current_ = current;

        return output;
    }

    // -------------------------- 麦轮专用PID对象初始化（参数适配麦轮电机） --------------------------
    // 1. 电机速度PID（输出PWM-100~100，参数经实测优化）
    PIDController<PID_VELOCITY> LeftFrontSpeedPID(6.0f, 0.8f, 0.2f, -100.0f, 100.0f, -40.0f, 40.0f);
    PIDController<PID_VELOCITY> RightFrontSpeedPID(6.0f, 0.8f, 0.2f, -100.0f, 100.0f, -40.0f, 40.0f);
    PIDController<PID_VELOCITY> RightRearSpeedPID(6.0f, 0.8f, 0.2f, -100.0f, 100.0f, -40.0f, 40.0f);
    PIDController<PID_VELOCITY> LeftRearSpeedPID(6.0f, 0.8f, 0.2f, -100.0f, 100.0f, -40.0f, 40.0f);

    // 2. 底盘位置PID（输出目标速度，参数适配麦轮底盘）
    PIDController<PID_POSITION> ChassisXPID(1.0f, 0.15f, 0.08f, -200.0f, 200.0f, -80.0f, 80.0f); // X轴：-200~200 mm/s
    PIDController<PID_POSITION> ChassisYPID(1.0f, 0.15f, 0.08f, -200.0f, 200.0f, -80.0f, 80.0f); // Y轴：-200~200 mm/s
    PIDController<PID_POSITION> ChassisThetaPID(1.5f, 0.25f, 0.12f, -1.0f, 1.0f, -0.6f, 0.6f);   // 旋转：-1~1 rad/s
}

// -------------------------- 模板类显式实例化（解决链接错误） --------------------------
// 必须显式实例化用到的模板类型，否则编译器无法生成二进制代码
template class pid::PIDController<PID_POSITION>;
template class pid::PIDController<PID_VELOCITY>;
