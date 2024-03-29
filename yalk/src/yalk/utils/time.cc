#include "yalk/utils/time.h"

#include <chrono>

namespace yalk {

int64_t GetCurrentUnixTimestamp() {
  return std::chrono::duration_cast<std::chrono::seconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace yalk
