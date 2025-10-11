#include "init.h"
#include "tim.h"
#include "stdio.h"
#include <stdint.h>
#include <math.h>
#include "usart.h"
#include "dma.h"

// 已测试
Encoder encoder_;
Motor motor_;
EncoderOdom encoder_odom_;
PID_Controller lf_pid;
PID_Controller rf_pid;
PID_Controller rr_pid;
PID_Controller lr_pid;
PID_Controller base_position_pid;

// 未测试
MotorControl motor_control_;
PositionControl position_control_;

UartPi uart_pi_;

JY61P_Acc g_jy61p_acc = {0};
JY61P_Gyro g_jy61p_gyro = {0};
JY61P_Angle g_jy61p_angle = {0};
JY61P_Tim g_jy61p_Tim = {0};

uint8_t rx_buffer = 0;

#define WHEEL_RADIUS 0.034f // 34mm
#define WHEEL_TRACK 0.230f  // 230mm（左右轮距）
#define WHEEL_BASE 0.094f   // 94mm（前后轴距）
#define ENCODER_RES 17000   // 编码器分辨率（线数×减速比，需按实际修改）

#define MAX_LINEAR_SPEED 0.5f      // 最大线速度(m/s)
#define MAX_ANGULAR_SPEED M_PI / 4 // 最大角速度(rad/s)

// -------------------------- PID参数配置（根据实际硬件调整） --------------------------
// 速度PID参数（增量式，输出限幅±100对应电机PWM占空比）
#define PID_VELOCITY_KP 2.5f              // 比例系数
#define PID_VELOCITY_KI 0.8f              // 积分系数
#define PID_VELOCITY_KD 0.1f              // 微分系数
#define PID_VELOCITY_INTEGRAL_LIMIT 50.0f // 积分限幅
#define PID_VELOCITY_OUTPUT_LIMIT 100.0f  // 输出限幅（±100对应PWM±100%）

// 位置PID参数（位置式，输出限幅根据控制需求调整）
#define PID_POSITION_KP 5.0f               // 比例系数
#define PID_POSITION_KI 0.2f               // 积分系数
#define PID_POSITION_KD 0.3f               // 微分系数
#define PID_POSITION_INTEGRAL_LIMIT 200.0f // 积分限幅
#define PID_POSITION_OUTPUT_LIMIT 500.0f   // 输出限幅（根据位置控制范围调整）

// -------------------------- MotorControl参数配置（根据硬件调试调整） --------------------------
#define MC_PID_KP 3.0f              // PID比例系数（速度环核心参数）
#define MC_PID_KI 1.2f              // PID积分系数（消除静态误差）
#define MC_PID_KD 0.15f             // PID微分系数（抑制超调）
#define MC_PID_INTEGRAL_LIMIT 60.0f // PID积分限幅（防止积分饱和）
#define MC_PID_OUTPUT_LIMIT 100.0f  // PID输出限幅（±100对应PWM占空比）
#define MC_CONTROL_FREQ 500U        // 控制频率（500Hz=2ms周期）

// -------------------------- 初始化所有PID实例 --------------------------
static void PID_All_Init(void)
{
    // 1. 初始化4个轮子速度PID（增量式）
    PID_Init(&lf_pid, PID_VELOCITY,
             PID_VELOCITY_KP, PID_VELOCITY_KI, PID_VELOCITY_KD,
             PID_VELOCITY_INTEGRAL_LIMIT, PID_VELOCITY_OUTPUT_LIMIT);

    PID_Init(&rf_pid, PID_VELOCITY,
             PID_VELOCITY_KP, PID_VELOCITY_KI, PID_VELOCITY_KD,
             PID_VELOCITY_INTEGRAL_LIMIT, PID_VELOCITY_OUTPUT_LIMIT);

    PID_Init(&rr_pid, PID_VELOCITY,
             PID_VELOCITY_KP, PID_VELOCITY_KI, PID_VELOCITY_KD,
             PID_VELOCITY_INTEGRAL_LIMIT, PID_VELOCITY_OUTPUT_LIMIT);

    PID_Init(&lr_pid, PID_VELOCITY,
             PID_VELOCITY_KP, PID_VELOCITY_KI, PID_VELOCITY_KD,
             PID_VELOCITY_INTEGRAL_LIMIT, PID_VELOCITY_OUTPUT_LIMIT);

    // 2. 初始化base_position位置PID（位置式）
    PID_Init(&base_position_pid, PID_POSITION,
             PID_POSITION_KP, PID_POSITION_KI, PID_POSITION_KD,
             PID_POSITION_INTEGRAL_LIMIT, PID_POSITION_OUTPUT_LIMIT);
}

// 2. 全初始化函数
// -------------------------- 全初始化函数（整合PID初始化） --------------------------
void All_Init(void)
{
    // 电机初始化
    MotorSingleConfig lf_cfg = {GPIOB, GPIO_PIN_0, TIM_CHANNEL_1};  // 左前：方向引脚GPIOB_PIN_0，PWM通道1
    MotorSingleConfig rf_cfg = {GPIOD, GPIO_PIN_0, TIM_CHANNEL_3};  // 右前：方向引脚GPIOD_PIN_0，PWM通道3
    MotorSingleConfig rr_cfg = {GPIOE, GPIO_PIN_0, TIM_CHANNEL_2};  // 右后：方向引脚GPIOE_PIN_0，PWM通道2
    MotorSingleConfig lr_cfg = {GPIOB, GPIO_PIN_10, TIM_CHANNEL_4}; // 左后：方向引脚GPIOB_PIN_10，PWM通道4
    Motor_Init(&motor_, &htim5, 2099, &lf_cfg, &rf_cfg, &rr_cfg, &lr_cfg);
    Motor_StartPWM(&motor_);

    // 编码器初始化
    Encoder_Init(&encoder_, &htim1, &htim2, &htim4, &htim3);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // 编码器里程计初始化
    EncoderOdom_Init(&encoder_odom_,
                     WHEEL_RADIUS,
                     WHEEL_TRACK,
                     WHEEL_BASE,
                     ENCODER_RES);

    // PID初始化（调用PID_All_Init）
    PID_All_Init();

    // MotorControl初始化（核心：绑定电机+编码器+PID参数）
    MotorControl_Init(&motor_control_,
                      &motor_,               // 绑定电机实例
                      &encoder_,             // 绑定编码器实例
                      MC_PID_KP,             // PID比例系数
                      MC_PID_KI,             // PID积分系数
                      MC_PID_KD,             // PID微分系数
                      MC_PID_INTEGRAL_LIMIT, // PID积分限幅
                      MC_PID_OUTPUT_LIMIT,   // PID输出限幅
                      MC_CONTROL_FREQ);      // 控制频率

    LightInit();
    PositionControl_Init(&position_control_, &motor_control_);

    // 5. 采样定时器初始化
    HAL_TIM_Base_Start_IT(&htim6);

    UartPi_Init(&uart_pi_, &huart2);

    JY61P_Init(&huart3);
    HAL_UART_Receive_DMA(&huart3, &rx_buffer, 1);
}

void odom_test(const EncoderOdom *odom, uint32_t print_interval_ms)
{
    static uint32_t last_print_time = 0;
    uint32_t current_time = HAL_GetTick();

    if (odom == NULL)
        return;

    // 按设定间隔打印
    if ((current_time - last_print_time) < print_interval_ms)
        return;
    last_print_time = current_time;

    // 打印里程计数据
    printf("===================== 里程计测试数据 =====================\r\n");
    printf("位置: X=%.3f m | Y=%.3f m\r\n",
           encoder_odom_x(odom), encoder_odom_y(odom));
    printf("航向角: Theta=%.3f rad (%.1f°)\r\n",
           encoder_odom_theta(odom), encoder_odom_theta(odom) * 180.0f / M_PI);
    printf("时间戳: %lu ms\r\n", encoder_odom_time(odom));
    printf("==========================================================\r\n\r\n");
}

// -------------------------- 电机控制测试函数 --------------------------
void motor_control_test(MotorControl *mc, uint32_t test_duration_ms)
{
    if (mc == NULL || test_duration_ms == 0)
    {
        printf("[ERROR] motor_control_test: 入参无效（空指针或测试时长为0）！\r\n");
        return;
    }

    uint32_t start_time = HAL_GetTick();
    uint32_t last_print_time = start_time;
    // 测试阶段目标速度规划（分阶段测试不同速度，验证动态响应）
    const float target_vel_plan[3][4] = {
        {2.0f, 2.0f, 2.0f, 2.0f}, // 阶段1：4电机均2r/s（匀速测试）
        {3.0f, 3.0f, 3.0f, 3.0f}, // 阶段2：4电机均3r/s（加速响应测试）
        {1.0f, 1.0f, 1.0f, 1.0f}  // 阶段3：4电机均1r/s（减速响应测试）
    };
    uint8_t current_stage = 0;                      // 当前测试阶段（0~2）
    uint32_t stage_duration = test_duration_ms / 3; // 每个阶段时长（均分总测试时间）

    printf("===================== 电机控制测试启动 =====================\r\n");
    printf("测试总时长：%lu ms | 分3阶段，每阶段%lu ms\r\n",
           test_duration_ms, stage_duration);
    printf("阶段1目标：2r/s | 阶段2目标：3r/s | 阶段3目标：1r/s\r\n");
    printf("----------------------------------------------------------\r\n");
    printf("时间(ms) | 左前(目标/反馈) | 右前(目标/反馈) | 右后(目标/反馈) | 左后(目标/反馈) | 左前PID输出\r\n");
    printf("----------------------------------------------------------\r\n");

    // 2. 测试主循环（按总时长运行，分阶段更新目标速度）
    while (HAL_GetTick() - start_time < test_duration_ms)
    {
        // 2.1 分阶段更新目标速度（每阶段切换一次目标）
        uint32_t elapsed_time = HAL_GetTick() - start_time;
        if (elapsed_time < stage_duration)
        {
            current_stage = 0; // 阶段1：0 ~ stage_duration ms
        }
        else if (elapsed_time < 2 * stage_duration)
        {
            current_stage = 1; // 阶段2：stage_duration ~ 2*stage_duration ms
        }
        else
        {
            current_stage = 2; // 阶段3：2*stage_duration ~ 测试结束
        }
        // 应用当前阶段的目标速度到所有电机
        for (MotorControlID id = MOTOR_LF; id < MOTOR_NUM; id++)
        {
            MotorControl_SetTargetVel(mc, id, target_vel_plan[current_stage][id]);
        }

        // 2.2 定期打印测试状态（500ms打印一次，避免串口阻塞）
        if (HAL_GetTick() - last_print_time >= 500)
        {
            // 获取各电机的目标速度、反馈速度和PID输出
            float lf_target = target_vel_plan[current_stage][MOTOR_LF];
            float lf_feedback = MotorControl_GetFeedbackVel(mc, MOTOR_LF);
            float rf_target = target_vel_plan[current_stage][MOTOR_RF];
            float rf_feedback = MotorControl_GetFeedbackVel(mc, MOTOR_RF);
            float rr_target = target_vel_plan[current_stage][MOTOR_RR];
            float rr_feedback = MotorControl_GetFeedbackVel(mc, MOTOR_RR);
            float lr_target = target_vel_plan[current_stage][MOTOR_LR];
            float lr_feedback = MotorControl_GetFeedbackVel(mc, MOTOR_LR);
            float lf_pid_output = MotorControl_GetPIDOutput(mc, MOTOR_LF);

            // 格式化打印（对齐显示，便于观察）
            printf("%8lu |   %.1f/%.1f    |   %.1f/%.1f    |   %.1f/%.1f    |   %.1f/%.1f    |   %.1f\r\n",
                   HAL_GetTick() - start_time, // 已运行时间
                   lf_target, lf_feedback,     // 左前电机（目标/反馈）
                   rf_target, rf_feedback,     // 右前电机（目标/反馈）
                   rr_target, rr_feedback,     // 右后电机（目标/反馈）
                   lr_target, lr_feedback,     // 左后电机（目标/反馈）
                   lf_pid_output);             // 左前电机PID输出（参考）

            last_print_time = HAL_GetTick();
        }

        // 2.3 短延时（10ms，与电机控制频率匹配，避免CPU占用过高）
        HAL_Delay(10);
    }

    // 3. 测试结束：安全处理
    printf("----------------------------------------------------------\r\n");
    printf("===================== 电机控制测试结束 =====================\r\n");
    // 3.1 停止所有电机（避免测试后电机持续运行）
    MotorControl_StopAll(mc);
    printf("测试后已停止所有电机，PID控制器已重置\r\n\r\n");

    // 3.2 打印测试总结（计算平均速度误差，评估闭环效果）
    float avg_error[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t sample_count = 0;
    // 重新获取测试最后1秒的速度数据，计算平均误差
    uint32_t summary_start = HAL_GetTick() - 1000;
    while (HAL_GetTick() - summary_start < 1000 && sample_count < 100)
    {
        for (MotorControlID id = MOTOR_LF; id < MOTOR_NUM; id++)
        {
            float target = target_vel_plan[2][id]; // 取最后阶段目标速度
            float feedback = MotorControl_GetFeedbackVel(mc, id);
            avg_error[id] += fabsf(target - feedback); // 累加绝对误差
        }
        sample_count++;
        HAL_Delay(10);
    }
    // 计算平均误差
    if (sample_count > 0)
    {
        for (MotorControlID id = MOTOR_LF; id < MOTOR_NUM; id++)
        {
            avg_error[id] /= sample_count;
        }
        // 打印误差总结
        printf("测试总结：最后1秒平均速度误差\r\n");
        printf("左前电机：%.3f r/s | 右前电机：%.3f r/s | 右后电机：%.3f r/s | 左后电机：%.3f r/s\r\n",
               avg_error[MOTOR_LF], avg_error[MOTOR_RF], avg_error[MOTOR_RR], avg_error[MOTOR_LR]);
        // 误差评估
        float max_error = 0.0f;
        for (MotorControlID id = MOTOR_LF; id < MOTOR_NUM; id++)
        {
            if (avg_error[id] > max_error)
                max_error = avg_error[id];
        }
        if (max_error < 0.1f)
            printf("评估结果：优秀（最大平均误差<0.1r/s，闭环精度高）\r\n\r\n");
        else if (max_error < 0.3f)
            printf("评估结果：良好（最大平均误差<0.3r/s，闭环精度满足需求）\r\n\r\n");
        else
            printf("评估结果：需优化（最大平均误差≥0.3r/s，建议调整PID参数）\r\n\r\n");
    }
}

void position_control_test(PositionControl *pc, uint32_t total_test_time_ms)
{
    if (pc == NULL)
    {
        printf("[Position Control Test] 无效的位置控制实例指针\r\n");
        return;
    }

    printf("========================================\r\n");
    printf("开始位置控制测试，总时长: %lu ms\r\n", total_test_time_ms);
    printf("测试将执行一系列移动和旋转动作\r\n");
    printf("========================================\r\n");

    uint32_t start_time = HAL_GetTick();
    uint32_t current_time = start_time;
    float x, y, angle;

    // 测试步骤计数器
    uint8_t test_step = 0;

    // 测试主循环
    while (current_time - start_time < total_test_time_ms)
    {
        // 获取当前位置并打印
        PositionControl_GetPosition(pc, &x, &y, &angle);
        printf("[%lu ms] 位置: X=%.2f m, Y=%.2f m, 角度=%.2f° | 测试步骤: %d\r\n",
               current_time - start_time,
               x, y, angle * 180.0f / M_PI,
               test_step);

        // 根据测试步骤执行不同动作
        switch (test_step)
        {
        case 0:
            // 步骤0: 移动到(0.5, 0)位置（x正方向移动0.5米）
            printf("步骤0: 移动到X=0.5m, Y=0m位置\r\n");
            PositionControl_SetPosition(pc, 0.5f, 0.0f, 0.3f);
            test_step++;
            break;

        case 1:
            // 步骤1: 等待到达位置或超时(5秒)
            if (!pc->is_moving || current_time - start_time > 5000)
            {
                printf("步骤1: 到达X=0.5m位置或超时\r\n");
                test_step++;
            }
            break;

        case 2:
            // 步骤2: 移动到(0.5, 0.5)位置（y正方向移动0.5米）
            printf("步骤2: 移动到X=0.5m, Y=0.5m位置\r\n");
            PositionControl_SetPosition(pc, 0.5f, 0.5f, 0.3f);
            test_step++;
            break;

        case 3:
            // 步骤3: 等待到达位置或超时(5秒)
            if (!pc->is_moving || current_time - start_time > 10000)
            {
                printf("步骤3: 到达X=0.5m, Y=0.5m位置或超时\r\n");
                test_step++;
            }
            break;

        case 4:
            // 步骤4: 旋转90度（顺时针）
            printf("步骤4: 顺时针旋转90度\r\n");
            PositionControl_SetAngle(pc, M_PI / 2, M_PI / 4);
            test_step++;
            break;

        case 5:
            // 步骤5: 等待旋转完成或超时(3秒)
            if (!pc->is_moving || current_time - start_time > 13000)
            {
                printf("步骤5: 旋转完成或超时\r\n");
                test_step++;
            }
            break;

        case 6:
            // 步骤6: 移动到(0, 0.5)位置（x负方向移动0.5米）
            printf("步骤6: 移动到X=0m, Y=0.5m位置\r\n");
            PositionControl_SetPosition(pc, 0.0f, 0.5f, 0.3f);
            test_step++;
            break;

        case 7:
            // 步骤7: 等待到达位置或超时(5秒)
            if (!pc->is_moving || current_time - start_time > 18000)
            {
                printf("步骤7: 到达X=0m, Y=0.5m位置或超时\r\n");
                test_step++;
            }
            break;

        case 8:
            // 步骤8: 旋转-90度（逆时针）
            printf("步骤8: 逆时针旋转90度\r\n");
            PositionControl_SetAngle(pc, -M_PI / 2, M_PI / 4);
            test_step++;
            break;

        case 9:
            // 步骤9: 等待旋转完成或超时(3秒)
            if (!pc->is_moving || current_time - start_time > 21000)
            {
                printf("步骤9: 旋转完成或超时\r\n");
                test_step++;
            }
            break;

        case 10:
            // 步骤10: 回到原点(0, 0)
            printf("步骤10: 返回原点(0, 0)\r\n");
            PositionControl_SetPosition(pc, 0.0f, 0.0f, 0.3f);
            test_step++;
            break;

        case 11:
            // 步骤11: 等待到达位置或超时(5秒)
            if (!pc->is_moving || current_time - start_time > 26000)
            {
                printf("步骤11: 到达原点或超时\r\n");
                test_step++;
            }
            break;

        default:
            // 所有步骤完成，保持停止状态
            PositionControl_Stop(pc);
            break;
        }

        // 延时一小段时间，降低CPU占用
        HAL_Delay(100);
        current_time = HAL_GetTick();
    }

    // 测试结束，停止所有运动
    PositionControl_Stop(pc);
    printf("========================================\r\n");
    printf("位置控制测试结束\r\n");
    printf("最终位置: X=%.2f m, Y=%.2f m, 角度=%.2f°\r\n",
           x, y, angle * 180.0f / M_PI);
    printf("========================================\r\n");
}
