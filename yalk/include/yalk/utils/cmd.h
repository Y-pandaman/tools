#pragma once

#include <array>
#include <memory>
#include <string>

namespace yalk {

/// \brief Helper function to retrieve the output of a command
/// \param cmd The command to execute
/// \return The output of the command
std::string exec(const char* cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

}  // namespace yalk
