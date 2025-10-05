#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "tim.h"
#include <stdint.h>
#include <math.h>

// 修复1：补充M_PI定义（math.h可能未默认定义，避免编译报错）
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 修复2：全局里程计数据和C接口声明（C/C++环境均可见）
#ifdef __cplusplus
extern "C"
{ // 让C编译器能识别这些声明
#endif

// 麦轮硬件参数配置（C/C++共用，故放在extern "C"外但在头文件顶部）
#define ENCODER_PPR 1000                                                  // 编码器分辨率（每圈脉冲数）
#define MECANUM_WHEEL_R 50.0f                                             // 麦轮有效半径（mm，实测）
#define CHASSIS_WHEELBASE 200.0f                                          // 底盘轴距（前后轮中心距，mm）
#define CHASSIS_TRACK 150.0f                                              // 底盘轮距（左右轮中心距，mm）
#define HALF_WHEELBASE_TRACK ((CHASSIS_WHEELBASE + CHASSIS_TRACK) / 2.0f) // 辅助参数

    // 全局里程计数据（单位：mm、rad、ms）——C/C++均可直接访问
    extern float encoder_total_x;      // X轴总位移（右为正）
    extern float encoder_total_y;      // Y轴总位移（前为正）
    extern float encoder_total_theta;  // 总旋转角度（逆时针为正，rad）
    extern uint32_t last_encoder_time; // 上次里程更新时间戳（ms）

    // 供C/C++调用的里程计重置接口（C风格接口）
    void Encoder_ResetOdometry(void);

#ifdef __cplusplus
} // 结束extern "C"块
#endif

// -------------------------- C++编码器类（仅C++环境可见） --------------------------
#ifdef __cplusplus
namespace encoder
{
    class Encoder
    {
    public:
        Encoder(bool reset_on_init = false);
        ~Encoder() = default; // 简化析构函数（无动态内存，默认即可）

        // 1. 获取各轮原始计数（只读，故加const）
        int32_t GetLeftFrontCount() const;
        int32_t GetRightFrontCount() const;
        int32_t GetRightRearCount() const;
        int32_t GetLeftRearCount() const;

        // 2. 重置各轮原始计数
        void ResetLeftFrontCount();
        void ResetRightFrontCount();
        void ResetRightRearCount();
        void ResetLeftRearCount();

        // 3. 重置里程计（X/Y/旋转清零，同步更新全局变量）
        void ResetOdometry();

        // 4. 获取当前里程计数据（供外部C++文件调用，支持按需获取）
        void GetOdometry(float *x, float *y, float *theta, uint32_t *time) const;

        // 5. TIM6中断调用：采样并解算里程计（核心函数）
        void SampleAndUpdate();

        // 6. 轮子速度接口
        float GetLeftFrontWheelSpeed() const;
        float GetRightFrontWheelSpeed() const;
        float GetRightRearWheelSpeed() const;
        float GetLeftRearWheelSpeed() const;

    private:
        // 绑定编码器定时器句柄（与CubeMX配置一致，const确保不被修改）
        TIM_HandleTypeDef *const htim_left_front = &htim1;  // 左前
        TIM_HandleTypeDef *const htim_right_front = &htim2; // 右前
        TIM_HandleTypeDef *const htim_right_rear = &htim4;  // 右后
        TIM_HandleTypeDef *const htim_left_rear = &htim3;   // 左后

        // 上一周期编码器计数（用于计算10ms内变化量，避免中断中丢失数据）
        int32_t last_lf_count = 0;
        int32_t last_rf_count = 0;
        int32_t last_rr_count = 0;
        int32_t last_lr_count = 0;

        // 类内里程计累加值（与全局变量同步，确保数据一致性）
        float total_x = 0.0f;
        float total_y = 0.0f;
        float total_theta = 0.0f;

        // 轮速缓存变量（2ms更新一次，与SampleAndUpdate()同步）
        float left_front_speed_ = 0.0f;
        float right_front_speed_ = 0.0f;
        float right_rear_speed_ = 0.0f;
        float left_rear_speed_ = 0.0f;
    };

    // 全局编码器对象声明（供外部C++文件访问，定义在encoder.cpp中）
    extern Encoder global_encoder;
}
#endif

#endif // ENCODER_H
