project (image_processing)

file(GLOB _SRCS "*.cc" "*.cpp" "*.hpp")
add_library(${PROJECT_NAME} OBJECT ${_SRCS})

find_package(OpenCV 3 REQUIRED)
link_directories(${OpenCV_LIBS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE
    PUBLIC ${PROJECT_SOURCE_DIR}
)

add_subdirectory(depth_filter)
