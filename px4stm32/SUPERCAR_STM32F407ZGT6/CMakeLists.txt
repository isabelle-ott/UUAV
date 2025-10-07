cmake_minimum_required(VERSION 3.16)

# -------------------------- 基础配置 --------------------------
# 项目名称
set(CMAKE_PROJECT_NAME SUPERCAR_STM32F407ZGT6)
project(${CMAKE_PROJECT_NAME})

# 编译标准（C11，兼容HAL库）
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 构建类型（默认Debug，支持Release）
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose build type: Debug/Release" FORCE)
endif()
message("Build type: ${CMAKE_BUILD_TYPE}")

# 启用汇编和C语言支持（STM32启动文件为汇编）
enable_language(C ASM)

# 生成compile_commands.json，方便IDE索引（如VSCode、Clangd）
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# -------------------------- 工具链配置（若未单独配置需添加） --------------------------
# 若使用ARM交叉编译工具链，需指定工具链路径（根据实际安装路径修改）
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake" CACHE PATH "Toolchain file")
endif()

# -------------------------- 生成可执行文件 --------------------------
# 创建ELF可执行文件（STM32的目标文件）
add_executable(${CMAKE_PROJECT_NAME} "")

# -------------------------- 添加子模块（STM32CubeMX生成的驱动） --------------------------
add_subdirectory(cmake/stm32cubemx)

# -------------------------- 添加用户自定义源文件（关键：User/Src下的所有文件） --------------------------
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # User/Src下的所有用户文件，按实际文件补充
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/init.c          # 包含All_Init函数
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/motor.c         # 电机驱动实现
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/motor_control.c # 电机控制逻辑
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/pid.c           # PID算法实现
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/encoder.c       # 编码器实现
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/encoder_odom.c  # 编码器里程计
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/imu.c           # IMU驱动
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/imu_odom.c      # IMU里程计
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/odom.c          # 里程计融合
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/position_control.c # 位置控制
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/light.c         # 灯光控制
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/it_cb.c         # 中断回调
)

# -------------------------- 用户头文件路径（User/Inc） --------------------------
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Inc  # 让编译器找到User/Inc下的.h文件
)

# -------------------------- 链接配置 --------------------------
# 移除无用的ob库依赖（避免C++编译冲突，若用C则可选）
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 链接核心库（stm32cubemx包含HAL库和基础配置）
target_link_libraries(${CMAKE_PROJECT_NAME}
    PRIVATE
    stm32cubemx  # 来自cmake/stm32cubemx的接口库
    -lm
)

# -------------------------- 生成HEX和BIN文件（下载到STM32用） --------------------------
# 从ELF生成HEX文件
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex"
)

# 从ELF生成BIN文件
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin"
)

# -------------------------- 清理目标（可选） --------------------------
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    ADDITIONAL_CLEAN_FILES "${CMAKE_PROJECT_NAME}.hex;${CMAKE_PROJECT_NAME}.bin;${CMAKE_PROJECT_NAME}.map"
)
