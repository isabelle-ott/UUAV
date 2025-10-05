#include "encoder.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "pid.h"
#include "motor.h"

// -------------------------- 全局变量新增：电机转速命令缓存和标志 --------------------------
// 命令标志：0=无命令，1=有新命令
static uint8_t motor_command_flag = 0;
// 命令缓存：保存目标转速命令（单位：mm/s）
static float target_speed_cmd_x = 0.0f;
static float target_speed_cmd_y = 0.0f;
static float target_speed_cmd_theta = 0.0f;

// -------------------------- 新增：设置电机转速命令的接口 --------------------------
// 外部可调用此函数设置目标速度，触发PID控制
void Motor_SetSpeedCommand(float x, float y, float theta)
{
    // 缓存命令
    target_speed_cmd_x = x;
    target_speed_cmd_y = y;
    target_speed_cmd_theta = theta;
    // 置位命令标志
    motor_command_flag = 1;
}

// -------------------------- 全局里程计数据定义（C/C++共用） --------------------------
float encoder_total_x = 0.0f;
float encoder_total_y = 0.0f;
float encoder_total_theta = 0.0f;
uint32_t last_encoder_time = 0;

// -------------------------- C++编码器类实现（仅C++环境编译） --------------------------
#ifdef __cplusplus
namespace encoder
{
    // 全局编码器对象定义
    Encoder global_encoder(false);

    // 构造函数：初始化计数缓存，可选重置里程计
    Encoder::Encoder(bool reset_on_init)
    {
        // 初始化上一周期计数（读取当前编码器初始值）
        last_lf_count = GetLeftFrontCount();
        last_rf_count = GetRightFrontCount();
        last_rr_count = GetRightRearCount();
        last_lr_count = GetLeftRearCount();

        // 若需要初始化时重置里程计
        if (reset_on_init)
        {
            ResetOdometry();
            last_encoder_time = HAL_GetTick();
        }
    }

    // -------------------------- 1. 获取各轮原始计数 --------------------------
    int32_t Encoder::GetLeftFrontCount() const
    {
        return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_left_front));
    }

    int32_t Encoder::GetRightFrontCount() const
    {
        return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_right_front));
    }

    int32_t Encoder::GetRightRearCount() const
    {
        return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_right_rear));
    }

    int32_t Encoder::GetLeftRearCount() const
    {
        return static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_left_rear));
    }

    // -------------------------- 2. 重置各轮原始计数 --------------------------
    void Encoder::ResetLeftFrontCount()
    {
        __HAL_TIM_SET_COUNTER(htim_left_front, 0);
        last_lf_count = 0; // 同步缓存，避免下次解算偏差
    }

    void Encoder::ResetRightFrontCount()
    {
        __HAL_TIM_SET_COUNTER(htim_right_front, 0);
        last_rf_count = 0;
    }

    void Encoder::ResetRightRearCount()
    {
        __HAL_TIM_SET_COUNTER(htim_right_rear, 0);
        last_rr_count = 0;
    }

    void Encoder::ResetLeftRearCount()
    {
        __HAL_TIM_SET_COUNTER(htim_left_rear, 0);
        last_lr_count = 0;
    }

    // -------------------------- 3. 重置里程计 --------------------------
    void Encoder::ResetOdometry()
    {
        // 清零类内里程数据
        total_x = 0.0f;
        total_y = 0.0f;
        total_theta = 0.0f;

        // 同步清零全局里程数据（确保C/C++访问的一致性）
        encoder_total_x = total_x;
        encoder_total_y = total_y;
        encoder_total_theta = total_theta;

        // 重置计数缓存（避免清零后出现大的变化量）
        last_lf_count = GetLeftFrontCount();
        last_rf_count = GetRightFrontCount();
        last_rr_count = GetRightRearCount();
        last_lr_count = GetLeftRearCount();
    }

    // -------------------------- 4. 获取当前里程计数据（C++外部调用） --------------------------
    void Encoder::GetOdometry(float *x, float *y, float *theta, uint32_t *time) const
    {
        // 空指针判断：避免解引用空指针导致崩溃
        if (x != nullptr)
            *x = total_x;
        if (y != nullptr)
            *y = total_y;
        if (theta != nullptr)
            *theta = total_theta;
        if (time != nullptr)
            *time = last_encoder_time;
    }

    // -------------------------- 5. 麦轮运动学解算（TIM6中断触发） --------------------------
    void Encoder::SampleAndUpdate()
    {
        // 1. 读取当前周期各轮计数
        int32_t curr_lf = GetLeftFrontCount();
        int32_t curr_rf = GetRightFrontCount();
        int32_t curr_rr = GetRightRearCount();
        int32_t curr_lr = GetLeftRearCount();

        // 2. 计算10ms内计数变化量（当前 - 上一周期）
        int32_t delta_lf = curr_lf - last_lf_count;
        int32_t delta_rf = curr_rf - last_rf_count;
        int32_t delta_rr = curr_rr - last_rr_count;
        int32_t delta_lr = curr_lr - last_lr_count;

        // 3. 保存当前计数为下一周期的“上一周期计数”
        last_lf_count = curr_lf;
        last_rf_count = curr_rf;
        last_rr_count = curr_rr;
        last_lr_count = curr_lr;

        // 4. 计数变化量 → 轮子角位移（rad）：角位移 = (计数/PPR) * 2π
        float theta_lf = (static_cast<float>(delta_lf) / ENCODER_PPR) * 2 * M_PI;
        float theta_rf = (static_cast<float>(delta_rf) / ENCODER_PPR) * 2 * M_PI;
        float theta_rr = (static_cast<float>(delta_rr) / ENCODER_PPR) * 2 * M_PI;
        float theta_lr = (static_cast<float>(delta_lr) / ENCODER_PPR) * 2 * M_PI;

        // 5. 角位移 → 轮子线速度（mm/s）：速度 = 角位移 * 半径 / 采样周期（0.002s）
        float v_lf = theta_lf * MECANUM_WHEEL_R / 0.002f;
        float v_rf = theta_rf * MECANUM_WHEEL_R / 0.002f;
        float v_rr = theta_rr * MECANUM_WHEEL_R / 0.002f;
        float v_lr = theta_lr * MECANUM_WHEEL_R / 0.002f;

        // 更新轮速缓存（与当前周期轮速同步）
        left_front_speed_ = v_lf;
        right_front_speed_ = v_rf;
        right_rear_speed_ = v_rr;
        left_rear_speed_ = v_lr;

        // 6. 麦轮运动学逆解（X型安装）→ 底盘速度（mm/s、rad/s）
        float v_x = (v_rf + v_lf + v_rr + v_lr) / 4.0f;                             // X轴速度（右正）
        float v_y = (-v_rf + v_lf - v_rr + v_lr) / 4.0f;                            // Y轴速度（前正）
        float omega = (-v_rf - v_lf + v_rr + v_lr) / (4.0f * HALF_WHEELBASE_TRACK); // 旋转角速度（逆时针正）

        // 7. 速度积分 → 里程位移（10ms内增量）
        total_x += v_x * 0.002f;
        total_y += v_y * 0.002f;
        total_theta += omega * 0.002f;

        // 8. 同步更新全局里程数据（C/C++均可访问）
        encoder_total_x = total_x;
        encoder_total_y = total_y;
        encoder_total_theta = total_theta;
        last_encoder_time = HAL_GetTick(); // 更新时间戳
    }

    // 实现获取轮速的接口
    float Encoder::GetLeftFrontWheelSpeed() const
    {
        return left_front_speed_;
    }

    float Encoder::GetRightFrontWheelSpeed() const
    {
        return right_front_speed_;
    }

    float Encoder::GetRightRearWheelSpeed() const
    {
        return right_rear_speed_;
    }

    float Encoder::GetLeftRearWheelSpeed() const
    {
        return left_rear_speed_;
    }

} // 结束namespace encoder
#endif

// -------------------------- C风格接口实现（供C/C++调用） --------------------------
void Encoder_ResetOdometry(void)
{
#ifdef __cplusplus
    // C环境下调用C++全局对象的ResetOdometry()
    encoder::global_encoder.ResetOdometry();
#endif
}

// -------------------------- TIM定时中断回调（2ms触发一次，核心控制逻辑） --------------------------
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 容错：若中断句柄为空，直接返回（避免空指针崩溃）
    if (htim == NULL)
        return;

    // -------------------------- 1. TIM6：编码器更新 + 底盘位置环 + 电机速度环 --------------------------
    if (htim->Instance == TIM6)
    {
        // -------------------------- 步骤1：更新编码器数据（无论是否有命令都执行） --------------------------
        encoder::global_encoder.SampleAndUpdate();

        // -------------------------- 步骤2：仅当有电机转速命令时，才执行PID控制 --------------------------
        if (motor_command_flag)
        {
            // 使用缓存的命令作为目标值（替代原遥控器/固定值）
            float target_speed_x = target_speed_cmd_x;
            float target_speed_y = target_speed_cmd_y;
            float target_speed_theta = target_speed_cmd_theta;

            // 底盘速度限幅（防止目标速度超出电机最大能力）
            target_speed_x = fminf(fmaxf(target_speed_x, -300.0f), 300.0f);     // X轴速度：-300~300 mm/s
            target_speed_y = fminf(fmaxf(target_speed_y, -300.0f), 300.0f);     // Y轴速度：-300~300 mm/s
            target_speed_theta = fminf(fmaxf(target_speed_theta, -2.0f), 2.0f); // 旋转角速度：-2~2 rad/s

            // -------------------------- 步骤3：麦轮运动学正解「底盘速度→各电机目标速度」 --------------------------
            const float half_sum = (CHASSIS_WHEELBASE + CHASSIS_TRACK) / 2.0f;
            float target_lf_speed = target_speed_x - target_speed_y - target_speed_theta * half_sum;
            float target_rf_speed = target_speed_x + target_speed_y + target_speed_theta * half_sum;
            float target_rr_speed = target_speed_x - target_speed_y + target_speed_theta * half_sum;
            float target_lr_speed = target_speed_x + target_speed_y - target_speed_theta * half_sum;

            // 电机目标速度限幅
            target_lf_speed = fminf(fmaxf(target_lf_speed, -500.0f), 500.0f);
            target_rf_speed = fminf(fmaxf(target_rf_speed, -500.0f), 500.0f);
            target_rr_speed = fminf(fmaxf(target_rr_speed, -500.0f), 500.0f);
            target_lr_speed = fminf(fmaxf(target_lr_speed, -500.0f), 500.0f);

            // -------------------------- 步骤4：电机速度PID计算「目标速度→PWM占空比」 --------------------------
            float current_lf_speed = encoder::global_encoder.GetLeftFrontWheelSpeed();
            float current_rf_speed = encoder::global_encoder.GetRightFrontWheelSpeed();
            float current_rr_speed = encoder::global_encoder.GetRightRearWheelSpeed();
            float current_lr_speed = encoder::global_encoder.GetLeftRearWheelSpeed();

            // 速度PID计算
            float pwm_lf = pid::LeftFrontSpeedPID.Compute(target_lf_speed, current_lf_speed);
            float pwm_rf = pid::RightFrontSpeedPID.Compute(target_rf_speed, current_rf_speed);
            float pwm_rr = pid::RightRearSpeedPID.Compute(target_rr_speed, current_rr_speed);
            float pwm_lr = pid::LeftRearSpeedPID.Compute(target_lr_speed, current_lr_speed);

            // -------------------------- 步骤5：输出PWM控制电机 --------------------------
            Motor_SetPWM(MOTOR_LEFT_FRONT, pwm_lf);  // 左前电机
            Motor_SetPWM(MOTOR_RIGHT_FRONT, pwm_rf); // 右前电机
            Motor_SetPWM(MOTOR_RIGHT_REAR, pwm_rr);  // 右后电机
            Motor_SetPWM(MOTOR_LEFT_REAR, pwm_lr);   // 左后电机

            motor_command_flag = 0;
        }
        else
        {
            // 无命令时可选择停止电机或保持当前状态
            // Motor_StopAll();  // 可选：无命令时停止电机
        }
    }
}
