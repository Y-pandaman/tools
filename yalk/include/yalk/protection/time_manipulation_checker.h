#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "yalk/base/common.h"

namespace yalk {

struct TimeManipulationCheckerParams {
  /// \brief The path of file to store the last timestamp.
  std::string timestamp_file;

  /// \brief The path of file to store the time change history.
  std::string time_change_history_file;

  /// \brief Maximum allowed time changes.
  size_t max_time_changes = 5;

  /// \brief The license expiration time in seconds since epoch.
  time_t license_expiration = 1735689600;

  /// \brief The reasonable range of time difference in seconds.
  int reasonable_time_diff = 3600;

  /// \brief The interval to check the timestamp in seconds.
  size_t time_check_interval = 600;
};

class TimeManipulationChecker {
 public:
  /// \brief Default constructor.
  TimeManipulationChecker() = default;
  /// \brief Default destructor.
  ~TimeManipulationChecker();

 public:
  /// \brief Initialize the time manipulation checker.
  /// \param config The configuration from command line or config file.
  /// \return True if the time manipulation checker is initialized successfully,
  /// false otherwise.
  bool Init(const json &config);

  /// \brief Initialize the time manipulation checker.
  /// \param params The parameters for the time manipulation checker.
  /// \return True if the time manipulation checker is initialized successfully,
  /// false otherwise.
  bool Init(const TimeManipulationCheckerParams &params);

  /// \brief Start the time manipulation checker.
  void Start();

  /// \brief Stop the time manipulation checker.
  void Stop();

  /// \brief Check if the system time is manipulated.
  /// \return True if the system time is manipulated, false otherwise.
  bool Check() const;

  /// \brief Reset the time manipulation checker.
  void Reset();

 public:
  /// \brief Set the mock timestamp.
  /// \param timestamp The mock timestamp.
  void SetMockTimestamp(time_t timestamp);

 private:
  /// \brief Get current timestamp from system or from the mock timestamp.
  time_t GetCurrentTimestamp() const;

  /// \brief Read the stored timestamp from the application data directory.
  /// \return The stored timestamp.
  time_t ReadStoredTimestamp();

  /// \brief Store the timestamp in the application data directory.
  /// \param timestamp The timestamp to store.
  void StoreTimestamp(time_t timestamp);

  /// \brief Read the time change history from the application data directory.
  /// \n The time change history is a list of timestamps when the system time is
  // changed
  /// \return The time change history.
  std::vector<time_t> ReadTimeChangeHistory();

  /// \brief Store the time change history in the application data directory.
  /// \n The time change history is a list of timestamps when the system time is
  // changed
  /// \param time_change_history The time change history to store.
  void StoreTimeChangeHistory(const std::vector<time_t> &time_change_history);

  /// \brief Check if the time difference is within a reasonable range.
  /// \param stored_timestamp The stored timestamp.
  /// \param current_time The current time.
  /// \return True if the time difference is within a reasonable range, false
  /// otherwise.
  bool IsTimeDifferenceValid(time_t stored_timestamp, time_t current_time);

  // Validate time changes
  /// \brief Validate the time change between the stored timestamp and the
  /// current time.
  /// \n If the time difference is within a reasonable range, the stored
  /// timestamp will be updated to the current time.
  /// \n If the time difference is not within a reasonable range, the stored
  /// timestamp will not be updated.
  /// \n If the time difference is not within a reasonable range and the number
  /// of time changes exceeds the maximum allowed time changes, the system time
  /// is considered to be manipulated.
  /// \param stored_timestamp The stored timestamp.
  /// \param current_time The current time.
  /// \return True if the time change is valid, false otherwise.
  bool ValidateTimeChange(time_t stored_timestamp, time_t current_time);

  /// \brief Detect time manipulation.
  void DetectTimeManipulation();

 private:
  /// \brief The parameters for the time manipulation checker.
  TimeManipulationCheckerParams params_;

  /// \brief The mutex to protect the timestamp check.
  std::mutex time_check_mutex;

  /// \brief The flag to indicate if the timestamp check is running.
  bool time_check_running_ = false;

  /// \brief The thread to check the timestamp.
  std::shared_ptr<std::thread> time_check_thread = nullptr;

  /// \brief The condition variable to notify the timestamp check.
  std::condition_variable time_check_cv;

  /// \brief The flag to indicate if the system time has been changed.
  bool time_changed_ = false;

  /// \brief Mock timestamp for testing.
  time_t mock_timestamp_ = 0;

  /// \brief The system time when the mock timestamp is set.
  time_t mock_timestamp_set_time_ = 0;
};

}  // namespace yalk
