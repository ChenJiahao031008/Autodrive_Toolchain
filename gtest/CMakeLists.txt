project(func_test)

# 输出可执行文件到指定目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../bin)

add_executable(func_test gtest_all.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(func_test gtest gtest_main
    common
    interface
    sensor_data
    ${Boost_LIBRARIES}
)
add_test(NAME func_test COMMAND func_test)
