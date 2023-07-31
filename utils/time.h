#pragma once

#include <cstdint>

namespace dvis_cpp {
namespace util {

/// \brief Get current unix timestamp in seconds.
/// \return The current unix timestamp in seconds.
int64_t GetCurrentUnixTimestampInSeconds();

/// \brief Get current unix timestamp in milliseconds.
/// \return The current unix timestamp in milliseconds.
uint64_t GetCurrentUnixTimestampInMilliseconds();

}  // namespace util
}  // namespace dvis_cpp
