project(unit_testing)

# 输出可执行文件到指定目录
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../bin)

add_executable(common_test common_test.cpp)
target_compile_features(common_test PRIVATE cxx_std_17)
target_link_libraries(common_test
    gtest gtest_main
    common
    ${Boost_LIBRARIES}
)
add_test(NAME common_test COMMAND common_test)

add_executable(interface_test interface_test.cpp)
target_compile_features(interface_test PRIVATE cxx_std_17)
target_link_libraries(interface_test
    gtest gtest_main
    common
    interface
    sensor_data
    ${Boost_LIBRARIES}
)
add_test(NAME interface_test COMMAND interface_test)

add_executable(senser_data_test senser_data_test.cpp)
target_compile_features(senser_data_test PRIVATE cxx_std_17)
target_link_libraries(senser_data_test
    gtest gtest_main
    common
    sensor_data
    ${Boost_LIBRARIES}
)
add_test(NAME senser_data_test COMMAND senser_data_test)


add_executable(math_kalman_filter_test math_kalman_filter_test.cpp)
target_compile_features(math_kalman_filter_test PRIVATE cxx_std_17)
target_link_libraries(math_kalman_filter_test
    gtest gtest_main
    common
    math_utils
    ${Boost_LIBRARIES}
)
add_test(NAME math_kalman_filter_test COMMAND math_kalman_filter_test)


add_executable(math_utils_test math_utils_test.cpp)
target_compile_features(math_utils_test PRIVATE cxx_std_17)
target_link_libraries(math_utils_test
    gtest gtest_main
    common
    math_utils
    ${Boost_LIBRARIES}
)
add_test(NAME math_utils_test COMMAND math_utils_test)
