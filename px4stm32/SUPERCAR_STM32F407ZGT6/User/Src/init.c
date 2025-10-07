#include "init.h"
#include "tim.h"   // 定时器句柄定义
#include "stdio.h" // 串口打印函数（printf）依赖（需确保串口已初始化）
#include <stdint.h>
#include <math.h>

// 1. 原有全局实例定义
Encoder encoder_;
Motor motor_;
EncoderOdom encoder_odom_;

// 2. PID实例定义（直接声明，与lf_pid等名称对应）
PID_Controller lf_pid;
PID_Controller rf_pid;
PID_Controller rr_pid;
PID_Controller lr_pid;
PID_Controller base_position_pid;

MotorControl motor_control_;

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
#define MC_CONTROL_FREQ 100U        // 控制频率（100Hz=10ms周期）

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

// 2. 全初始化函数（原逻辑保留）
// -------------------------- 全初始化函数（整合PID初始化） --------------------------
void All_Init(void)
{
    // 1. 编码器初始化（原逻辑）
    Encoder_Init(&encoder_, &htim1, &htim2, &htim4, &htim3);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // init.c 中的电机初始化（无需修改，仅参数绑定）
    MotorSingleConfig lf_cfg = {GPIOB, GPIO_PIN_0, TIM_CHANNEL_1};         // 左前：方向引脚GPIOB_PIN_0，PWM通道1
    MotorSingleConfig rf_cfg = {GPIOD, GPIO_PIN_0, TIM_CHANNEL_3};         // 右前：方向引脚GPIOD_PIN_0，PWM通道3
    MotorSingleConfig rr_cfg = {GPIOE, GPIO_PIN_0, TIM_CHANNEL_2};         // 右后：方向引脚GPIOE_PIN_0，PWM通道2
    MotorSingleConfig lr_cfg = {GPIOB, GPIO_PIN_10, TIM_CHANNEL_4};        // 左后：方向引脚GPIOB_PIN_10，PWM通道4
    Motor_Init(&motor_, &htim5, 1999, &lf_cfg, &rf_cfg, &rr_cfg, &lr_cfg); // 绑定参数（ARR=1999，与CubeMX一致）
    Motor_StartPWM(&motor_);                                               // 启动PWM（依赖CubeMX初始化的定时器）

    // 3. 编码器里程计初始化（原逻辑）
    EncoderOdom_Init(&encoder_odom_, 0.035f, 0.25f, 0.30f, 17000);

    // 4. PID初始化（新增，调用PID_All_Init）
    PID_All_Init();

    // 4. MotorControl初始化（核心：绑定电机+编码器+PID参数）
    MotorControl_Init(&motor_control_,
                      &motor_,               // 绑定电机实例
                      &encoder_,             // 绑定编码器实例
                      MC_PID_KP,             // PID比例系数
                      MC_PID_KI,             // PID积分系数
                      MC_PID_KD,             // PID微分系数
                      MC_PID_INTEGRAL_LIMIT, // PID积分限幅
                      MC_PID_OUTPUT_LIMIT,   // PID输出限幅
                      MC_CONTROL_FREQ);      // 控制频率

    // 5. 采样定时器初始化（原逻辑）
    HAL_TIM_Base_Start_IT(&htim6);
}

// 3. 编码器测试函数实现（新增）
void encoder_test(uint32_t print_interval_ms)
{
    // 局部变量：记录上次打印时间（避免高频打印）
    static uint32_t last_print_time = 0;
    // 当前时间（基于HAL_GetTick()，与编码器采样时间同步）
    uint32_t current_time = HAL_GetTick();

    // 按设定间隔打印（避免占用过多CPU资源）
    if ((current_time - last_print_time) < print_interval_ms)
    {
        return;
    }
    last_print_time = current_time; // 更新上次打印时间

    // -------------------------- 1. 获取编码器关键数据 --------------------------
    // 1.1 各轮上次采样计数差值（用于判断正反转）
    int32_t lf_diff = Encoder_GetLastLeftFrontDiff(&encoder_);
    int32_t rf_diff = Encoder_GetLastRightFrontDiff(&encoder_);
    int32_t rr_diff = Encoder_GetLastRightRearDiff(&encoder_);
    int32_t lr_diff = Encoder_GetLastLeftRearDiff(&encoder_);

    // 1.2 采样时间信息（判断采样是否正常）
    uint32_t last_sample_t = Encoder_GetLastSampleTime(&encoder_);
    uint32_t curr_sample_t = Encoder_GetCurrentSampleTime(&encoder_);
    uint32_t time_diff = Encoder_GetSampleTimeDiff(&encoder_);

    // 1.3 各轮实时速度（单位：圈/秒，可后续转换为m/s）
    float lf_vel = Encoder_GetLeftFrontVel(&encoder_);
    float rf_vel = Encoder_GetRightFrontVel(&encoder_);
    float rr_vel = Encoder_GetRightRearVel(&encoder_);
    float lr_vel = Encoder_GetLeftRearVel(&encoder_);

    // -------------------------- 2. 串口打印测试数据 --------------------------
    // 格式说明：清晰区分各轮数据，标注单位，便于调试
    printf("===================== 编码器测试数据 =====================\r\n");
    // 时间信息
    printf("采样时间：上次=%lu ms | 当前=%lu ms | 间隔=%lu ms\r\n",
           last_sample_t, curr_sample_t, time_diff);
    // 计数差值（正=正转，负=反转，0=静止）
    printf("计数差值：左前=%ld | 右前=%ld | 右后=%ld | 左后=%ld\r\n",
           lf_diff, rf_diff, rr_diff, lr_diff);
    // 实时速度（保留2位小数，直观查看转速）
    printf("实时速度：左前=%.2f r/s | 右前=%.2f r/s | 右后=%.2f r/s | 左后=%.2f r/s\r\n",
           lf_vel, rf_vel, rr_vel, lr_vel);
    printf("==========================================================\r\n\r\n");
}

// -------------------------- 电机测试函数实现 --------------------------
void motor_test(Motor *motor, uint32_t test_step_duration_ms, float test_speed)
{
    // 1. 入参合法性检查（异常保护，避免空指针/参数越界）
    if (motor == NULL)
    {
        printf("[ERROR] motor_test: Motor实例指针为空！\r\n");
        return;
    }
    if (test_step_duration_ms < 500) // 最小持续时间500ms，确保肉眼可观测
        test_step_duration_ms = 500;
    if (test_speed > 100.0f) // 速度限制在±100%占空比
        test_speed = 100.0f;
    if (test_speed < -100.0f)
        test_speed = -100.0f;

    // 2. 打印测试配置信息（串口输出，便于调试）
    printf("===================== 电机测试启动 =====================\r\n");
    printf("测试参数：步骤时长=%lu ms | 测试速度=%.1f（正=正转，负=反转）\r\n",
           test_step_duration_ms, test_speed);
    printf("测试流程：1.单电机正转→2.单电机反转→3.多电机协同→4.全部停止\r\n");
    printf("==========================================================\r\n");

    // 3. 步骤1：单个电机正转测试（按左前→右前→右后→左后顺序）
    printf("[步骤1/4] 单个电机正转测试（速度=%.1f）...\r\n", fabsf(test_speed));

    // 左前电机正转
    printf("  - 左前电机正转...\r\n");
    Motor_SetLeftFrontVel(motor, fabsf(test_speed)); // 取绝对值=正转
    HAL_Delay(test_step_duration_ms);
    Motor_SetLeftFrontVel(motor, 0.0f); // 停止当前电机
    HAL_Delay(300);                     // 间隔300ms，避免电机切换过快

    // 右前电机正转
    printf("  - 右前电机正转...\r\n");
    Motor_SetRightFrontVel(motor, fabsf(test_speed));
    HAL_Delay(test_step_duration_ms);
    Motor_SetRightFrontVel(motor, 0.0f);
    HAL_Delay(300);

    // 右后电机正转
    printf("  - 右后电机正转...\r\n");
    Motor_SetRightRearVel(motor, fabsf(test_speed));
    HAL_Delay(test_step_duration_ms);
    Motor_SetRightRearVel(motor, 0.0f);
    HAL_Delay(300);

    // 左后电机正转
    printf("  - 左后电机正转...\r\n");
    Motor_SetLeftRearVel(motor, fabsf(test_speed));
    HAL_Delay(test_step_duration_ms);
    Motor_SetLeftRearVel(motor, 0.0f);
    HAL_Delay(500); // 步骤间间隔延长，区分测试阶段

    // 4. 步骤2：单个电机反转测试（按左前→右前→右后→左后顺序）
    printf("[步骤2/4] 单个电机反转测试（速度=%.1f）...\r\n", fabsf(test_speed));

    // 左前电机反转
    printf("  - 左前电机反转...\r\n");
    Motor_SetLeftFrontVel(motor, -fabsf(test_speed)); // 负号=反转
    HAL_Delay(test_step_duration_ms);
    Motor_SetLeftFrontVel(motor, 0.0f);
    HAL_Delay(300);

    // 右前电机反转
    printf("  - 右前电机反转...\r\n");
    Motor_SetRightFrontVel(motor, -fabsf(test_speed));
    HAL_Delay(test_step_duration_ms);
    Motor_SetRightFrontVel(motor, 0.0f);
    HAL_Delay(300);

    // 右后电机反转
    printf("  - 右后电机反转...\r\n");
    Motor_SetRightRearVel(motor, -fabsf(test_speed));
    HAL_Delay(test_step_duration_ms);
    Motor_SetRightRearVel(motor, 0.0f);
    HAL_Delay(300);

    // 左后电机反转
    printf("  - 左后电机反转...\r\n");
    Motor_SetLeftRearVel(motor, -fabsf(test_speed));
    HAL_Delay(test_step_duration_ms);
    Motor_SetLeftRearVel(motor, 0.0f);
    HAL_Delay(500);

    // 5. 步骤3：多电机协同测试（前进/后退，模拟实际运动场景）
    printf("[步骤3/4] 多电机协同测试...\r\n");

    // 所有电机正转=前进
    printf("  - 所有电机正转（前进）...\r\n");
    Motor_SetLeftFrontVel(motor, fabsf(test_speed) * 0.8f); // 可微调速度，避免跑偏
    Motor_SetRightFrontVel(motor, fabsf(test_speed) * 0.8f);
    Motor_SetRightRearVel(motor, fabsf(test_speed) * 0.8f);
    Motor_SetLeftRearVel(motor, fabsf(test_speed) * 0.8f);
    HAL_Delay(test_step_duration_ms * 1.5f); // 持续时间延长1.5倍

    // 所有电机反转=后退
    printf("  - 所有电机反转（后退）...\r\n");
    Motor_SetLeftFrontVel(motor, -fabsf(test_speed) * 0.6f); // 后退速度降低，提高安全性
    Motor_SetRightFrontVel(motor, -fabsf(test_speed) * 0.6f);
    Motor_SetRightRearVel(motor, -fabsf(test_speed) * 0.6f);
    Motor_SetLeftRearVel(motor, -fabsf(test_speed) * 0.6f);
    HAL_Delay(test_step_duration_ms * 1.5f);

    // 停止所有电机
    Motor_StopAll(motor);
    HAL_Delay(500);

    // 6. 步骤4：测试结束，强制停止所有电机
    printf("[步骤4/4] 测试结束，强制停止所有电机！\r\n");
    Motor_StopAll(motor);
    printf("==========================================================\r\n\r\n");
}

// 新增里程计测试函数
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

// -------------------------- PID速度测试函数（可选） --------------------------
void pid_velocity_test(PID_Controller *pid, float target_speed, uint32_t test_time_ms)
{
    if (pid == NULL || test_time_ms == 0)
        return;

    uint32_t start_time = HAL_GetTick();
    printf("PID速度测试开始：目标速度=%.1f r/s，持续时间=%lu ms\r\n", target_speed, test_time_ms);

    while (HAL_GetTick() - start_time < test_time_ms)
    {
        // 1. 假设从编码器获取速度反馈（此处用模拟反馈，实际需替换为真实编码器速度）
        float feedback_speed = 0.8f * pid->target + 0.2f * sinf(HAL_GetTick() / 1000.0f); // 模拟扰动

        // 2. 设置PID目标值与反馈值
        PID_SetTarget(pid, target_speed);
        PID_SetFeedback(pid, feedback_speed);

        // 3. PID计算
        float output = PID_Calculate(pid);

        // 4. 打印PID状态
        printf("目标:%.1f | 反馈:%.1f | 输出:%.1f\r\n",
               pid->target, pid->feedback, output);

        HAL_Delay(50); // 50ms周期计算
    }

    // 测试结束，重置PID
    PID_Reset(pid);
    printf("PID速度测试结束！\r\n\r\n");
}

// -------------------------- 电机控制测试函数（验证闭环控制） --------------------------
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
