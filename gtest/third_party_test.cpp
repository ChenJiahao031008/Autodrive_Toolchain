#include <gtest/gtest.h>
#include <interpreter.hpp>
#include "../config/config.h"
#include "../common/logger.hpp"

TEST(TestThirdParty, TestConfig){
    ezcfg::Interpreter itp("../../gtest/config_test/config_test.txt", true);
    Config_Test conf;
    itp.parse(conf);
    EXPECT_EQ(conf.example_name, "Hello, World!");
    EXPECT_EQ(conf.example_vector.size(), 5);
    EXPECT_NEAR(conf.example_number, 1e-6, 1e-8);
}
