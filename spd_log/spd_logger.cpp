#include "spd_logger.h"

spdlog::level::level_enum SpdLogger::global_level = spdlog::level::trace;

spdlog::level::level_enum SpdLogger::getGlobalLevel() {
    return global_level;
}

SpdLogger::SpdLogger() { init(); }

/**
 * @brief 创建sink
 * 
 * @param log_file_name 文件名
 * @return std::vector<spdlog::sink_ptr> 
 */
std::vector<spdlog::sink_ptr>
SpdLogger::createSinks(const std::string &log_file_name) {
    std::vector<spdlog::sink_ptr> sinks;

    auto sink1 = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    sink1->set_level(SpdLogger::getGlobalLevel());
    sinks.push_back(sink1);

    auto sink2 = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        log_file_name, 1024 * 1024 * 100, 2);
    sinks.push_back(sink2);
    return sinks;
}

/**
 * @brief 创建logger
 * 
 * @param logger_name logger名
 */
void SpdLogger::createLogger(const std::string &logger_name) {
    auto now     = std::time(nullptr);
    auto nowtime = std::localtime(&now);
    char buffer[9];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", nowtime);
    std::string time_str(buffer);

    std::string log_file_name = "../log/" + logger_name + "_" + time_str + ".log";
    auto sinks                = SpdLogger::createSinks(log_file_name);

    auto logger = std::make_shared<spdlog::logger>(logger_name, begin(sinks), end(sinks));
    logger->set_level(spdlog::level::trace);
    spdlog::register_logger(logger);
}

/**
 * @brief 获取已注册的logger
 * 
 * @param logger_name  logger名
 * @return std::shared_ptr<spdlog::logger> 
 */
std::shared_ptr<spdlog::logger>
SpdLogger::getLogger(const std::string &logger_name) {
    auto logger = spdlog::get(logger_name);
    if(!logger){//looger指向为空
      createLogger(logger_name);
      logger = spdlog::get(logger_name);
    }
    return logger;
}

/**
 * @brief 初始化
 * 
 */
void SpdLogger::init() {
    auto level = spdlog::level::trace;
    if (std::getenv("STAGE") != NULL)
    {
        std::string stage = std::getenv("STAGE");
        if (stage == "dev")
            level = spdlog::level::debug;
    }
    SpdLogger::global_level = level;

    spdlog::flush_every(std::chrono::seconds(1));
    spdlog::flush_on(spdlog::level::trace);

    SpdLogger::createLogger("master");
}
