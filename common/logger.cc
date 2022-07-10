#include <stdlib.h>
#include <iostream>
#include "logger.hpp"

// #define LOG_CORE_DUMP_CAPTURE
namespace common {

Logger::Logger(int &argsize, char**& program) {
    /* 如果需要调整日志配置，请修改下面代码 */
    google::ParseCommandLineFlags(&argsize, &program, false);
    google::InitGoogleLogging(program[0]);

    FLAGS_colorlogtostderr = true;          // 设置输出到屏幕的日志显示对应颜色
    FLAGS_logbufsecs = 0;                   // 缓冲日志立即输出
    FLAGS_max_log_size = 100;               // 最大日志大小为 100MB
    FLAGS_stop_logging_if_full_disk = true; // 当磁盘被写满时，停止日志输出
    FLAGS_alsologtostderr = true;           // 输出到屏幕的日志也输出到文件

    google::SetStderrLogging(google::INFO); // 设置级别高于INFO 的日志同一时候输出到屏幕

#ifdef LOG_CORE_DUMP_CAPTURE
    google::InstallFailureSignalHandler();  // 捕捉 core dumped
#endif
    /* 如果需要调整文件位置，请修改下面代码 */
#if GCC_VERSION >= 90400
    auto curr_path = std::filesystem::current_path(); // 获取当前路径
    auto log_path = curr_path.parent_path() / "log";  // 日志文件存放路径,支持/重载
    if (!std::filesystem::exists(log_path)) {
        std::filesystem::create_directory(log_path);
        AINFO << "Create Log Folder: " << log_path.string();
    }
#else
    auto curr_path = boost::filesystem::current_path(); // 获取当前路径
    auto log_path = curr_path.parent_path() / "log";    // 日志文件存放路径,支持/重载
    if (!boost::filesystem::exists(log_path)){
        boost::filesystem::create_directory(log_path); //文件夹不存在则创建一个
        AINFO << "Create Log Folder: " << log_path.string();
    }
#endif

    std::string module_path = log_path.string() + "/" + MODULE_NAME;

    /* 判断是否需要ROS节点作为模块名称 */
#ifdef ROS
    module_path = LOGDIR + ros::this_node::getName();
#endif
    google::SetLogDestination(google::INFO, (module_path + "_INFO_").c_str());
    google::SetLogDestination(google::WARNING, (module_path + "_WARNING_").c_str());
    google::SetLogDestination(google::ERROR, (module_path + "_ERROR_").c_str());
    google::SetLogDestination(google::FATAL, (module_path + "_FATAL_").c_str());
}

Logger::~Logger()
{
    google::ShutdownGoogleLogging();
    std::cout << "[LOGGER WARNING] Logger IS FINISHED!" << std::endl;
}

}  // namespace common
