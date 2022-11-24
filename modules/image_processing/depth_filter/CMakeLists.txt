project (depth_filter)

file(GLOB _SRCS "*.cc" "*.cpp" "*.hpp")
add_library(${PROJECT_NAME} OBJECT ${_SRCS})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE
    PUBLIC ${PROJECT_SOURCE_DIR}
)

