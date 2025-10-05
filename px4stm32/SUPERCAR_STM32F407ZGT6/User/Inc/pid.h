#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <math.h>

// 1. PID控制类型枚举（区分位置环/速度环）
typedef enum
{
    PID_POSITION = 0, // 位置PID（输出：位置控制量，如目标速度）
    PID_VELOCITY = 1  // 速度PID（输出：速度控制量，如PWM占空比）
} PID_TypeDef;

// 2. 全局PID控制周期（与TIM6编码器采样周期一致：2ms=0.002s）
#define PID_CONTROL_PERIOD 0.002f

// 3. pid命名空间：封装PID类，避免全局命名污染
namespace pid
{
    // 模板类：通过Type参数区分位置/速度PID
    template <PID_TypeDef Type>
    class PIDController
    {
    public:
        /**
         * @brief 构造函数：初始化PID参数与限制
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         * @param output_min 输出下限（防止执行器过载）
         * @param output_max 输出上限
         * @param integral_min 积分下限（抗积分饱和）
         * @param integral_max 积分上限
         */
        PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
                      float output_min = -100.0f, float output_max = 100.0f,
                      float integral_min = -50.0f, float integral_max = 50.0f);

        /**
         * @brief PID核心计算：输入目标值/当前值，输出控制量
         * @param target 目标值（位置环=目标位置，速度环=目标速度）
         * @param current 当前值（位置环=当前位置，速度环=当前速度）
         * @param dt 控制周期（默认=PID_CONTROL_PERIOD=2ms，无需手动传参）
         * @return 控制量（位置环=目标速度，速度环=PWM占空比）
         */
        float Compute(float target, float current, float dt = PID_CONTROL_PERIOD);

        /**
         * @brief 重置PID状态（清零误差、积分、历史缓存）
         * @note 电机启停/目标值突变时调用，避免历史误差导致超调
         */
        void Reset();

        /**
         * @brief 动态调整PID参数（支持运行中优化）
         * @param kp 新比例系数
         * @param ki 新积分系数
         * @param kd 新微分系数
         */
        void SetParams(float kp, float ki, float kd);

        /**
         * @brief 设置积分限幅（单独调整积分饱和范围）
         * @param integral_min 积分下限
         * @param integral_max 积分上限
         */
        void SetIntegralLimit(float integral_min, float integral_max);

        /**
         * @brief 设置输出限幅（单独调整执行器控制范围）
         * @param output_min 输出下限
         * @param output_max 输出上限
         */
        void SetOutputLimit(float output_min, float output_max);

        /**
         * @brief 获取当前积分值（调试用，查看积分饱和情况）
         * @return 当前积分累加值
         */
        float GetIntegral() const { return integral_; }

        /**
         * @brief 获取当前误差值（调试用，查看偏差情况）
         * @return 当前误差（目标值-当前值）
         */
        float GetCurrentError() const { return current_error_; }

    private:
        // PID核心参数
        float kp_; // 比例系数
        float ki_; // 积分系数
        float kd_; // 微分系数

        // 限制值（抗饱和）
        float output_min_;   // 输出下限
        float output_max_;   // 输出上限
        float integral_min_; // 积分下限
        float integral_max_; // 积分上限

        // 状态缓存（历史数据）
        float current_error_; // 当前误差（目标值-当前值）
        float last_error_;    // 上一周期误差（位置环/速度环共用）
        float integral_;      // 积分累加值（位置环/速度环共用）
#if Type == PID_VELOCITY
        float last_current_; // 速度环专用：上一周期当前值（抑制速度噪声）
#endif
    };

    // -------------------------- 麦轮专用PID对象声明（4个电机速度环+3个底盘位置环） --------------------------
    // 1. 电机速度PID（输出：PWM占空比，范围-100~100）
    extern PIDController<PID_VELOCITY> LeftFrontSpeedPID;  // 左前电机
    extern PIDController<PID_VELOCITY> RightFrontSpeedPID; // 右前电机
    extern PIDController<PID_VELOCITY> RightRearSpeedPID;  // 右后电机
    extern PIDController<PID_VELOCITY> LeftRearSpeedPID;   // 左后电机

    // 2. 底盘位置PID（输出：目标速度，范围-200~200 mm/s；旋转-1~1 rad/s）
    extern PIDController<PID_POSITION> ChassisXPID;     // 底盘X轴（右为正）
    extern PIDController<PID_POSITION> ChassisYPID;     // 底盘Y轴（前为正）
    extern PIDController<PID_POSITION> ChassisThetaPID; // 底盘旋转（逆时针为正）
}

#endif // PID_H
