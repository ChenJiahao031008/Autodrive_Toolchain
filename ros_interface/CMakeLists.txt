cmake_minimum_required(VERSION 3.10.2)
project(Autodrive_Toolchain LANGUAGES CXX VERSION 0.0.1)

# -fPIC：生成动态库，-fopenmp 开启多线程，-O3 对代码进行优化，-g 打印调试信息，-Wall 打印所有警告信息, pthread 支持多线程
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++17 -g -O3 -fopenmp -pthread -Wall")

# 不生成警告信息: debug阶段尽量不要使用
add_definitions(-w)

find_package(Boost REQUIRED COMPONENTS thread filesystem program_options system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    message_generation
    std_srvs
    cv_bridge
    image_transport
    roslib
)

## Generate services in the 'srv' folder
add_service_files(FILES
  saveOdometry.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    roscpp
    sensor_msgs
    cv_bridge
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    /usr/local/include/eigen3
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../bin)

add_library(${PROJECT_NAME}_lib SHARED
    # ros
    src/message_flow.cc
    src/imu_subscriber.cc
    src/image_subscriber.cc
)

#必须为绝对路径，如有多个文件使用空格隔开
set(EZCFG_STRUCT_HEADER_FILE ${PROJECT_SOURCE_DIR}/../config/config.h)

# 指定外部文件夹时需要显式的指定输出文件夹位置
add_subdirectory(../third_party/backward-cpp external_library/backward-cpp)
add_subdirectory(../third_party/easy_config external_library/easy_config)
add_subdirectory(../common external_library/common)
add_subdirectory(../math_utils external_library/math_utils)
add_subdirectory(../interface external_library/interface)
add_subdirectory(../sensor_data external_library/sensor_data)

target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ezcfg::ezcfg
)

add_executable(ros_interface ros_interface.cpp)
target_compile_features(ros_interface PRIVATE cxx_std_17)
add_dependencies(ros_interface ${catkin_EXPORTED_TARGETS})
target_include_directories(ros_interface PRIVATE
    PUBLIC ${PROJECT_SOURCE_DIR}
)
target_link_libraries(ros_interface PRIVATE
    yaml-cpp
    common
    interface
    sensor_data
    math_utils
    ${BACKWARD_ENABLE}
    ${PROJECT_NAME}_lib
)
add_backward(ros_interface)




#############
## Install ##
#############

