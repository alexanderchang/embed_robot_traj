cmake_minimum_required(VERSION 3.12)
project(EmbedRobotTraj CXX)

# 嵌入式全局配置：C++17、Release、裁剪优化
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -Wall -Wextra -ffast-math -march=native)

# 查找依赖
find_package(Boost REQUIRED COMPONENTS thread chrono system)
find_package(SOEM REQUIRED)

# 头文件路径
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${SOEM_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src
)

# 所有源码文件
set(SRCS
    src/common.cpp
    src/ec_dev.cpp
    src/kinematics.cpp
    src/trajectory.cpp
    src/traj_opt.cpp
    src/traj_blend.cpp
    src/obstacle.cpp
    src/ml_traj.cpp
    src/multi_robot.cpp
    src/robot_ctrl.cpp
    src/main.cpp
)

# 可执行文件
add_executable(robot_app ${SRCS})

# 链接库（嵌入式最小依赖）
target_link_libraries(robot_app
    ${Boost_LIBRARIES}
    ${SOEM_LIBRARIES}
    pthread
    rt
)

# 安装规则（部署到嵌入式系统）
install(TARGETS robot_app RUNTIME DESTINATION /usr/local/bin)