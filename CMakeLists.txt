cmake_minimum_required(VERSION 3.10.2)
project(DataReader LANGUAGES CXX VERSION 0.0.1)

# -fPIC：生成动态库，-fopenmp 开启多线程，-O3 对代码进行优化，-g 打印调试信息，-Wall 打印所有警告信息, pthread 支持多线程
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++17 -g -O3 -fopenmp -pthread -Wall" CACHE STRING "" FORCE)

find_package(Boost REQUIRED COMPONENTS thread filesystem program_options system)

link_directories(
    interface
    sensor_data
    ${Boost_INCLUDE_DIRS}
)

add_subdirectory(common)
add_subdirectory(interface)
add_subdirectory(sensor_data)

add_executable(${PROJECT_NAME} data_reader_test.cc)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME}
    common
    interface
    sensor_data
    ${Boost_LIBRARIES}
)
