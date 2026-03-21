cmake_minimum_required(VERSION 3.8)
project(fastlio2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 开启调试打印
message(STATUS "------------------------------------------------")
message(STATUS "DEBUG: Starting configuration for FAST-LIO2")

# -----------------------------------------------------------
# 修复 1: 必须先寻找 OpenMP 包
# -----------------------------------------------------------
find_package(OpenMP REQUIRED)

# -----------------------------------------------------------
# 修复 2: 把 OpenMP 的编译参数加入 CMAKE_CXX_FLAGS
# (如果不加这个，#pragma omp 可能会被忽略)
# -----------------------------------------------------------
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "DEBUG: OpenMP Found. Flags: ${OpenMP_CXX_FLAGS}")
endif()

# 基础编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)
add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=2)

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 关键：寻找 Sophus
find_package(Sophus REQUIRED)
message(STATUS "DEBUG: Sophus Found: ${Sophus_FOUND}")

# -----------------------------------------------------------
# 关键修改：包含所有头文件路径 (包括 OpenMP)
# -----------------------------------------------------------
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenMP_CXX_INCLUDE_DIRS}  # <--- 加上 OpenMP 头文件路径
  
  # 这里是“救命”的硬编码路径
  /opt/ros/humble/include      
  /usr/include
  /usr/local/include
)

set(SRC_LIST src/map_builder/commons.cpp
             src/map_builder/ieskf.cpp
             src/map_builder/imu_processor.cpp
             src/map_builder/ikd_Tree.cpp
             src/map_builder/lidar_processor.cpp
             src/map_builder/map_builder.cpp
             src/utils.cpp)

add_executable(lio_node src/lio_node.cpp ${SRC_LIST})

ament_target_dependencies(lio_node 
  rclcpp std_msgs tf2 tf2_ros nav_msgs 
  sensor_msgs livox_ros_driver2 pcl_conversions geometry_msgs
)

# -----------------------------------------------------------
# 关键修改：正确链接 OpenMP
# -----------------------------------------------------------
target_link_libraries(lio_node
  yaml-cpp
  ${PCL_LIBRARIES}
  ${Sophus_LIBRARIES} 
  OpenMP::OpenMP_CXX      # <--- 使用这个现代写法，它最稳！
)

install(TARGETS lio_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
