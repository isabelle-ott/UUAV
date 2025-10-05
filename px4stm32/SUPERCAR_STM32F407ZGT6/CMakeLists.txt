cmake_minimum_required(VERSION 3.16)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 添加 C++ 标准支持
set(CMAKE_CXX_STANDARD 11) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME SUPERCAR_STM32F407ZGT6)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM) 

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # User目录下的源文件
    User/Src/ekf2position.cpp
    User/Src/encoder.cpp
    User/Src/imu.cpp
    User/Src/light.cpp
    User/Src/motor_control.cpp
    User/Src/motor_drive.cpp
    User/Src/pid.cpp
    User/Src/position_control.cpp
    
    # 核心源文件
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/stm32f4xx_it.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Src/system_stm32f4xx.c
    
    # 汇编启动文件
    startup_stm32f407xx.s
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # User目录下头文件
    User/Inc
    
    # 核心头文件目录
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    STM32F407xx           # 芯片型号
    USE_HAL_DRIVER        # 启用HAL库
    __FPU_PRESENT=1U      # 启用浮点单元
    ARM_MATH_CM4          # 告诉DSP库当前内核是Cortex-M4
    ARM_MATH_MATRIX_CHECK # 可选：启用DSP矩阵检查(调试用)
    ARM_MATH_ROUNDING     # 可选：启用DSP舍入功能
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 添加生成bin和hex文件的命令
# add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
#     COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
#     COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
#     COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin ${CMAKE_PROJECT_NAME}.hex
#     COMMENT "Generated: ${CMAKE_PROJECT_NAME}.bin ${CMAKE_PROJECT_NAME}.hex"
# )
