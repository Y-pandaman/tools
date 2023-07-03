#ifndef SPD_LOGGER_H
#define SPD_LOGGER_H

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/spdlog.h"
#include <stdlib.h>
#include <vector>

class SpdLogger {
public:
  SpdLogger();
  static void init();
  static spdlog::level::level_enum getGlobalLevel();
  static std::vector<spdlog::sink_ptr>
  createSinks(const std::string &log_file_name);
  static void createLogger(const std::string &logger_name);
  static std::shared_ptr<spdlog::logger>
  getLogger(const std::string &logger_name);

private:
    static spdlog::level::level_enum global_level;
};

#endif