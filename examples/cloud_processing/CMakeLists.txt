cmake_minimum_required(VERSION 3.10.2)
project(cloud_processing_example LANGUAGES CXX VERSION 0.0.1)

set(CMAKE_BUILD_TYPE Release)
# -fPIC：生成动态库，-fopenmp 开启多线程，-O3 对代码进行优化，-g 打印调试信息，-Wall 打印所有警告信息, pthread 支持多线程
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++17 -g -O3 -fopenmp -pthread")

find_package(PCL 1.2 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../../bin)
add_executable(${PROJECT_NAME} pca_example.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PRIVATE ${PROJECT_SOURCE_DIR}/../../
${PCL_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME} PRIVATE
    common
    interface
    ${BACKWARD_ENABLE}
    ${PCL_LIBRARIES}
)
add_backward(${PROJECT_NAME})
