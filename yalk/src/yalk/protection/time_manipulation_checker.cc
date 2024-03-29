#include "yalk/protection/time_manipulation_checker.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>

#include "yalk/base/errno.h"
#include "yalk/utils/filesystem.h"
#include "yalk/utils/log.h"

namespace yalk {

TimeManipulationChecker::~TimeManipulationChecker() { Stop(); }

bool TimeManipulationChecker::Init(const json &config) {
  // 0. check input
  // 0.1. chekc input config
  json tm_checker_config;
  if (!config.contains("tm_checker") || !config["tm_checker"].is_object()) {
    tm_checker_config = {{"product_name", "yalk"},
                         {"expiration_time", 0},
                         {"max_time_diff", 3600},
                         {"max_time_changes", 5},
                         {"time_check_interval", 600}};
    ADEBUG_F("Use default config for time manipulation checker: %s",
             tm_checker_config.dump().c_str());
  } else {
    tm_checker_config = config["tm_checker"];
    ADEBUG_F("Use given config for time manipulation checker : %s",
             tm_checker_config.dump().c_str());
  }
  // 0.2. check config content
  if (!tm_checker_config.contains("product_name") ||
      !tm_checker_config["product_name"].is_string() ||
      !tm_checker_config.contains("expiration_time") ||
      !tm_checker_config["expiration_time"].is_number_integer() ||
      !tm_checker_config.contains("max_time_diff") ||
      !tm_checker_config["max_time_diff"].is_number_integer() ||
      !tm_checker_config.contains("max_time_changes") ||
      !tm_checker_config["max_time_changes"].is_number_integer() ||
      !tm_checker_config.contains("time_check_interval") ||
      !tm_checker_config["time_check_interval"].is_number_integer()) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid input config for time manipulation checker: " +
                         tm_checker_config.dump());
    return false;
  }

  // 1. initialize variables
  // 1.1. init parameters
  TimeManipulationCheckerParams params;
  params.max_time_changes = tm_checker_config["max_time_changes"];
  params.license_expiration = tm_checker_config["expiration_time"];
  params.reasonable_time_diff = tm_checker_config["max_time_diff"];
  params.time_check_interval = tm_checker_config["time_check_interval"];
  // 1.2. init timestamp files
  std::string app_data = GetAppDataDirectory(tm_checker_config["product_name"]);
  params.timestamp_file = app_data + "timestamp.dat";
  params.time_change_history_file = app_data + "time_change_history.dat";

  return Init(params);
}

bool TimeManipulationChecker::Init(
    const TimeManipulationCheckerParams &params) {
  params_ = params;

  // create timestamp file if not exist
  if (params_.timestamp_file.empty() ||
      params_.time_change_history_file.empty()) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid timestamp file path: " + params_.timestamp_file);
    return false;
  }
  TouchFile(params_.timestamp_file);
  TouchFile(params_.time_change_history_file);

  // 2. start time manipulation checker
  Start();

  SetLastErrorCode(ErrorCode::kOK);
  return true;
}

bool TimeManipulationChecker::Check() const { return !time_changed_; }

void TimeManipulationChecker::Start() {
  if (!time_check_running_) {
    // Start a background thread for periodic time checks
    time_check_running_ = true;
    time_check_thread = std::make_shared<std::thread>(
        &TimeManipulationChecker::DetectTimeManipulation, this);
  }
}

void TimeManipulationChecker::Stop() {
  // Clean up and exit
  time_check_running_ = false;
  // Notify the time manipulation detection thread to exit
  time_check_cv.notify_one();
  // Wait for the time manipulation detection thread to finish
  if (time_check_thread && time_check_thread->joinable()) {
    time_check_thread->join();
  }
}

void TimeManipulationChecker::Reset() {
  time_changed_ = false;
  mock_timestamp_ = 0;
  mock_timestamp_set_time_ = 0;
}

void TimeManipulationChecker::SetMockTimestamp(time_t timestamp) {
  mock_timestamp_ = 0;
  mock_timestamp_set_time_ = GetCurrentTimestamp();
  mock_timestamp_ = timestamp;
}

time_t TimeManipulationChecker::GetCurrentTimestamp() const {
  if (mock_timestamp_ > 0 && mock_timestamp_set_time_ > 0) {
    return mock_timestamp_ + (std::time(nullptr) - mock_timestamp_set_time_);
  }
  return std::time(nullptr);
}

time_t TimeManipulationChecker::ReadStoredTimestamp() {
  std::ifstream timestamp_file(params_.timestamp_file);
  if (!timestamp_file.is_open()) {
    throw std::runtime_error("Could not open the timestamp file");
  }

  std::string timestamp_str;
  std::getline(timestamp_file, timestamp_str);
  if (timestamp_str.empty()) {
    // throw std::runtime_error("The timestamp file is empty");
    auto current_timestamp = GetCurrentTimestamp();
    StoreTimestamp(current_timestamp);
    return current_timestamp;
  }
  return std::stol(timestamp_str);
}

void TimeManipulationChecker::StoreTimestamp(time_t timestamp) {
  std::ofstream timestamp_file(params_.timestamp_file);
  if (!timestamp_file.is_open()) {
    throw std::runtime_error("Could not open the timestamp file");
  }

  timestamp_file << timestamp;
}

std::vector<time_t> TimeManipulationChecker::ReadTimeChangeHistory() {
  std::vector<time_t> time_change_history;

  std::ifstream history_file(params_.time_change_history_file);
  if (!history_file.is_open()) {
    throw std::runtime_error("Could not open the time change history file");
  }

  std::string line;
  while (std::getline(history_file, line)) {
    if (line.empty()) {
      continue;
    }
    time_change_history.push_back(std::stol(line));
  }

  return time_change_history;
}

void TimeManipulationChecker::StoreTimeChangeHistory(
    const std::vector<time_t> &time_change_history) {
  std::ofstream history_file(params_.time_change_history_file);
  if (!history_file.is_open()) {
    throw std::runtime_error("Could not open the time change history file");
  }

  // Write the history of time changes
  for (const auto &timestamp : time_change_history) {
    history_file << timestamp << '\n';
  }
}

bool TimeManipulationChecker::IsTimeDifferenceValid(time_t stored_timestamp,
                                                    time_t current_time) {
  return std::abs(difftime(stored_timestamp, current_time)) <=
         params_.reasonable_time_diff;
}

// Validate time changes
bool TimeManipulationChecker::ValidateTimeChange(time_t stored_timestamp,
                                                 time_t current_time) {
  if (!IsTimeDifferenceValid(stored_timestamp, current_time)) {
    return false;
  }

  // Read the history of time changes
  std::vector<time_t> time_change_history = ReadTimeChangeHistory();

  // Observe the frequency of time changes
  if (time_change_history.size() >= params_.max_time_changes) {
    // Too many time changes, consider suspicious
    return false;
  }

  // Check the history of time changes
  for (const auto &time_change : time_change_history) {
    if (params_.license_expiration > 0 &&
        std::abs(difftime(time_change, params_.license_expiration)) <=
            params_.reasonable_time_diff) {
      // Time change correlates with the license expiration, consider suspicious
      return false;
    }
  }

  // If the time change is valid, store it in the history
  time_change_history.push_back(current_time);
  StoreTimeChangeHistory(time_change_history);

  return true;
}

void TimeManipulationChecker::DetectTimeManipulation() {
  while (time_check_running_) {
    std::unique_lock<std::mutex> lock(time_check_mutex);

    try {
      time_t stored_timestamp = ReadStoredTimestamp();
      time_t current_time = GetCurrentTimestamp();

      // AINFO_F("Validate Time Change: %ld -> %ld", stored_timestamp,
      //         current_time);

      if (current_time < stored_timestamp) {
        // Potential time manipulation detected
        if (!ValidateTimeChange(stored_timestamp, current_time)) {
          // Invalid time change, take appropriate action
          AWARN_F("Invalid time change detected. Exiting the application.");
          time_changed_ = true;
        } else {
          // Valid time change, update the stored timestamp
          StoreTimestamp(current_time);
          time_changed_ = false;
        }
      } else {
        // Valid time change, update the stored timestamp
        StoreTimestamp(current_time);
        time_changed_ = false;
      }
    } catch (const std::exception &e) {
      AERROR_F("Failed to detect time manipulation: %s", e.what());
    }

    // Periodic time check, e.g., every 10 minutes
    time_check_cv.wait_for(lock,
                           std::chrono::seconds(params_.time_check_interval));
  }
}

}  // namespace yalk
