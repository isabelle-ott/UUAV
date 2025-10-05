cmake_minimum_required(VERSION 3.16)

# 启用C、C++、汇编语言支持（User目录含.cpp文件，需添加CXX）
enable_language(C CXX ASM)

# ========================== 1. 芯片与编译宏定义 ==========================
set(MX_Defines_Syms 
    USE_HAL_DRIVER          # 启用STM32 HAL库
    STM32F407xx             # 目标芯片型号（STM32F407ZGT6）
    $<$<CONFIG:Debug>:DEBUG> # Debug模式下定义DEBUG宏（用于条件编译）
    # __FPU_PRESENT=1U         # 启用Cortex-M4浮点单元（F407支持硬浮点）
    # ARM_MATH_CM4             # 适配DSP库（若使用arm_math.h需添加）
)

# ========================== 2. 头文件路径配置 ==========================
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../User/Inc
)

# ========================== 3. 源文件配置 ==========================
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_it.c       # 中断服务函数
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_hal_msp.c # HAL底层驱动配置
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c            # 内存分配（new/delete支持）
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c          # 系统调用（printf支持）
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32f407xx.s
    ${CMAKE_CURRENT_SOURCE_DIR}/../../User/Src/ekf2position.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../User/Src/encoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../User/Src/imu.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../User/Src/light.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../User/Src/pid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../User/Src/position_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../User/Src/init.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../User/Src/motor.cpp


)

# 3.2 STM32 HAL驱动源文件（按需添加，避免冗余）
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# ========================== 4. 链接配置 ==========================
# 链接目录（暂无额外库目录，预留）
set(MX_LINK_DIRS
)

# 链接库（STM32 HAL驱动库 + 工具链默认库）
set(MX_LINK_LIBS 
    STM32_Drivers 
    ${TOOLCHAIN_LINK_LIBRARIES}
)

# ========================== 5. 库与目标配置 ==========================
# 5.1 接口库：统一管理头文件和宏定义（供其他库依赖）
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})


# 配置C/C++标准（确保兼容性）
# target_compile_features(stm32cubemx INTERFACE 
#     c_std_11  # C标准：C11（CubeMX生成代码要求）
#     cxx_std_11 # C++标准：C11（适配User目录.cpp文件）
# )


# 5.2 HAL驱动对象库（将HAL源文件编译为对象库，避免重复编译）
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx) # 依赖接口库的头文件和宏

# 5.3 关联应用源文件到目标项目（需确保${CMAKE_PROJECT_NAME}已在主CMake定义）
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# 5.4 目标链接配置
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# ========================== 6. 附加配置 ==========================
# 1. 生成Map文件（用于调试：查看内存分布、函数地址）
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LINK_FLAGS "-Wl,-Map=${CMAKE_PROJECT_NAME}.map" # GCC链接选项生成Map文件
    ADDITIONAL_CLEAN_FILES "${CMAKE_PROJECT_NAME}.map" # clean时删除Map文件
)

# 2. 检查C标准兼容性（CubeMX生成代码不支持C90/C99）
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(FATAL_ERROR "STM32F407 HAL代码要求C11或更高标准！请设置CMAKE_C_STANDARD=11")
endif()

