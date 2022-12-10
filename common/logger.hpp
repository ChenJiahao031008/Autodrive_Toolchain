// LOG LINE PREFIX FORMAT
//
// Log lines have this form:
//
//     Lmmdd hh:mm:ss.uuuuuu threadid file:line] msg...
//
// where the fields are defined as follows:
//
//   L                A single character, representing the log level
//                    (eg 'I' for INFO)
//   mm               The month (zero padded; ie May is '05')
//   dd               The day (zero padded)
//   hh:mm:ss.uuuuuu  Time in hours, minutes and fractional seconds
//   threadid         The space-padded thread ID as returned by GetTID()
//                    (this matches the PID on Linux)
//   file             The file name
//   line             The line number
//   msg              The user-supplied message
//

#pragma once
#include <glog/logging.h>
#include <glog/raw_logging.h>
#include <stdlib.h>
#include <cstdarg>
#include <string>
// #include "config_flags.hpp"

#define MODULE_NAME "MAIN_MODULE"

#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL_)
#if GCC_VERSION >= 90400
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

/* 当存在ROS环境时，可以用ROS模块的节点名称代替默认模块名称 */
// #define ROS
#ifdef ROS
#include <ros/ros.h>
#define MODULE_NAME ros::this_node::getName().c_str()
#endif

/*  调整glog默认配置  */
#define DEFAULT_CONFIG
#ifdef DEFAULT_CONFIG
namespace common
{
class Logger
{
public:
    Logger(int &argsize, char **&program);
    ~Logger();
}; // namespace Logger
} // namespace common
#endif

#define LEFT_BRACKET "["
#define RIGHT_BRACKET "]"


#define ADEBUG_MODULE(module) \
    VLOG(4) << LEFT_BRACKET << module << RIGHT_BRACKET << "[DEBUG] "
#define ADEBUG ADEBUG_MODULE(MODULE_NAME)
#define AINFO ALOG_MODULE(MODULE_NAME, INFO)
#define AWARN ALOG_MODULE(MODULE_NAME, WARN)
#define AERROR ALOG_MODULE(MODULE_NAME, ERROR)
#define AFATAL ALOG_MODULE(MODULE_NAME, FATAL)

#ifndef ALOG_MODULE_STREAM
#define ALOG_MODULE_STREAM(log_severity) ALOG_MODULE_STREAM_##log_severity
#endif

#ifndef ALOG_MODULE
#define ALOG_MODULE(module, log_severity) \
    ALOG_MODULE_STREAM(log_severity)      \
    (module)
#endif

#define ALOG_MODULE_STREAM_INFO(module)                           \
    google::LogMessage(__FILE__, __LINE__, google::INFO).stream() \
        << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_WARN(module)                              \
    google::LogMessage(__FILE__, __LINE__, google::WARNING).stream() \
        << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_ERROR(module)                           \
    google::LogMessage(__FILE__, __LINE__, google::ERROR).stream() \
        << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_FATAL(module)                           \
    google::LogMessage(__FILE__, __LINE__, google::FATAL).stream() \
        << LEFT_BRACKET << module << RIGHT_BRACKET

#define AINFO_IF(cond) ALOG_IF(INFO, cond, MODULE_NAME)
#define AWARN_IF(cond) ALOG_IF(WARN, cond, MODULE_NAME)
#define AERROR_IF(cond) ALOG_IF(ERROR, cond, MODULE_NAME)
#define AFATAL_IF(cond) ALOG_IF(FATAL, cond, MODULE_NAME)
#define ALOG_IF(severity, cond, module) \
    !(cond) ? (void)0                   \
            : google::LogMessageVoidify() & ALOG_MODULE(module, severity)

#define ACHECK(cond) CHECK(cond) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#define AINFO_EVERY(freq) \
    LOG_EVERY_N(INFO, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AWARN_EVERY(freq) \
    LOG_EVERY_N(WARNING, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AERROR_EVERY(freq) \
    LOG_EVERY_N(ERROR, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#if !defined(RETURN_IF_NULL)
#define RETURN_IF_NULL(ptr)              \
    if (ptr == nullptr)                  \
    {                                    \
        AWARN << #ptr << " is nullptr."; \
        return;                          \
    }
#endif

#if !defined(RETURN_VAL_IF_NULL)
#define RETURN_VAL_IF_NULL(ptr, val)     \
    if (ptr == nullptr)                  \
    {                                    \
        AWARN << #ptr << " is nullptr."; \
        return val;                      \
    }
#endif

#if !defined(RETURN_IF)
#define RETURN_IF(condition)               \
    if (condition)                         \
    {                                      \
        AWARN << #condition << " is met."; \
        return;                            \
    }
#endif

#if !defined(RETURN_VAL_IF)
#define RETURN_VAL_IF(condition, val)      \
    if (condition)                         \
    {                                      \
        AWARN << #condition << " is met."; \
        return val;                        \
    }
#endif

#if !defined(_RETURN_VAL_IF_NULL2__)
#define _RETURN_VAL_IF_NULL2__
#define RETURN_VAL_IF_NULL2(ptr, val) \
    if (ptr == nullptr)               \
    {                                 \
        return (val);                 \
    }
#endif

#if !defined(_RETURN_VAL_IF2__)
#define _RETURN_VAL_IF2__
#define RETURN_VAL_IF2(condition, val) \
    if (condition)                     \
    {                                  \
        return (val);                  \
    }
#endif

#if !defined(_RETURN_IF2__)
#define _RETURN_IF2__
#define RETURN_IF2(condition) \
    if (condition)            \
    {                         \
        return;               \
    }
#endif
