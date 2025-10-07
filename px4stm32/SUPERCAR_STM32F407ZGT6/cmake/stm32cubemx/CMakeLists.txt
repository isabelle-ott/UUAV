cmake_minimum_required(VERSION 3.16)

# -------------------------- STM32CubeMX生成的宏定义（硬件相关） --------------------------
set(MX_Defines_Syms 
    USE_HAL_DRIVER        # 启用HAL库
    STM32F407xx           # 芯片型号（必须与实际一致）
    $<$<CONFIG:Debug>:DEBUG>  # Debug模式下定义DEBUG宏
)

# -------------------------- 头文件路径（CubeMX生成的核心+驱动头文件） --------------------------
set(MX_Include_Dirs
    # Core/Inc：CubeMX生成的主头文件（如main.h、stm32f4xx_hal_conf.h）
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../User/Inc
    # HAL库头文件
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    # CMSIS核心头文件（芯片内核、寄存器定义）
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
)

# -------------------------- CubeMX生成的核心源文件（基础初始化） --------------------------
set(MX_Application_Src
    # Core/Src：CubeMX生成的主文件（main.c、硬件初始化等）
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/gpio.c     # GPIO初始化
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/can.c      # CAN初始化
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/tim.c      # 定时器初始化（PWM、编码器）
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/usart.c    # 串口初始化
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_it.c  # 中断服务函数
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_hal_msp.c  # HAL MSP回调
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c   # 内存管理（堆/栈）
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c # 系统调用（如printf重定向）
    # 启动文件（汇编，必须添加，解决Reset_Handler缺失问题）
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32f407xx.s
)

# -------------------------- STM32 HAL驱动源文件（硬件驱动实现） --------------------------
set(STM32_Drivers_Src
    # CMSIS系统文件（时钟初始化、内核配置）
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32f4xx.c
    # HAL库核心驱动
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
)

# -------------------------- 接口库（供主工程链接） --------------------------
# 1. 创建stm32cubemx接口库（包含头文件和宏定义）
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# 2. 创建STM32_Drivers库（HAL驱动和核心源文件）
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src} ${MX_Application_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)

# -------------------------- 将CubeMX源文件添加到主工程 --------------------------
target_sources(${CMAKE_PROJECT_NAME} PRIVATE $<TARGET_OBJECTS:STM32_Drivers>)
