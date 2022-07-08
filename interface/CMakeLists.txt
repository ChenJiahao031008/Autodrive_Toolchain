project (interface)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV 3 REQUIRED)
message(STATUS "Using opencv version ${OpenCV_VERSION}")

file(GLOB _SRCS "*.cc" "*.cpp" "*.h")
add_library(${PROJECT_NAME} OBJECT ${_SRCS})


target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE
    PUBLIC ${PROJECT_SOURCE_DIR}
    ${YAML_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
    common
    sensor_data
    yaml-cpp
)

