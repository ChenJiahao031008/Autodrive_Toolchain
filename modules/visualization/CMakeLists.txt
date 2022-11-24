project (visualization)

file(GLOB _SRCS "*.cc" "*.cpp" "*.hpp")
add_library(${PROJECT_NAME} OBJECT ${_SRCS})

find_package(OpenCV 3 REQUIRED)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE
    PUBLIC ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
)
