cd build/
cmake ..
make -j4
cd gtest/
./senser_data_test
./interface_test
./common_test
./math_utils_test
./math_kalman_filter_test
