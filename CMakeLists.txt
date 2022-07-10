cmake_minimum_required(VERSION 3.10.2)
project(main LANGUAGES CXX VERSION 0.0.1)

# -fPIC：生成动态库，-fopenmp 开启多线程，-O3 对代码进行优化，-g 打印调试信息，-Wall 打印所有警告信息, pthread 支持多线程
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++17 -g -O3 -fopenmp -pthread")

# 不生成警告信息: debug阶段尽量不要使用
add_definitions(-w)

option(BUILD_TEST "Build all tests." ON)
if(BUILD_TEST)
  find_package(GTest REQUIRED)
  enable_testing()
endif()

find_package(Boost REQUIRED COMPONENTS thread filesystem program_options system)

link_directories(
    interface
    sensor_data
    ${Boost_INCLUDE_DIRS}
)

#必须为绝对路径，如有多个文件使用空格隔开
set(EZCFG_STRUCT_HEADER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/config/config.h)

add_subdirectory(third_party/backward-cpp)
add_subdirectory(third_party/easy_config)
add_subdirectory(common)
add_subdirectory(math_utils)
add_subdirectory(interface)
add_subdirectory(sensor_data)

if(BUILD_TEST)
    add_subdirectory(gtest)
endif()

# 输出可执行文件到指定目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(${PROJECT_NAME} main.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME} PRIVATE
    common
    interface
    sensor_data
    ezcfg::ezcfg
    ${Boost_LIBRARIES}
    ${BACKWARD_ENABLE}
)
add_backward(${PROJECT_NAME})
