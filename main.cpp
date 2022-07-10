#include <iostream>
#include <iomanip>

#include "common/logger.hpp"
#include "common/time_base.hpp"
#include "common/time_utils.hpp"
#include "interface/DataReader.h"
#include "interface/DataProcessor.h"
#include "interface/DataConverter.h"

#include "config/config.h"
#include <interpreter.hpp>

#ifndef LOG_CORE_DUMP_CAPTURE
    #define BACKWARD_HAS_DW 1
#endif

int main(int argc, char** argv)
{
    common::Logger logger(argc, argv);
    ezcfg::Interpreter itp("../../config/config.txt", true);
    Config conf;
    itp.parse(conf);
    AINFO << conf.example_name;

    // 测试backward-cpp模块的core dump捕获功能
    // char* c = "hello world";
    // c[1] = 'H';

    return 0;
}
