#include "yalk/hwid/disk_common.h"

#include <fstream>
#include <regex>
#include <sstream>
#include <vector>

#include "yalk/utils/log.h"

namespace yalk {

std::string GetRootFilesystemDevice() {
  std::ifstream mounts_file("/proc/mounts");
  std::string line;

  while (std::getline(mounts_file, line)) {
    std::istringstream line_stream(line);
    std::vector<std::string> tokens;

    for (std::string token; std::getline(line_stream, token, ' ');) {
      tokens.push_back(token);
    }

    if (tokens.size() >= 2 && tokens[1] == "/") {
      mounts_file.close();
      return tokens[0];
    }
  }

  mounts_file.close();
  throw std::runtime_error("Root filesystem device not found");
}

std::string GetDiskDevice(const std::string &device_path) {
  // Resolve the device path to its canonical path
  // (e.g. /dev/sda1 -> /dev/sda1, /dev/disk/by-uuid/1234 -> /dev/sda1)
  char *real_path = realpath(device_path.c_str(), nullptr);
  std::string canonical_path;
  if (real_path == nullptr) {
    // throw std::runtime_error("Failed to resolve the device path");
    AERROR_F("Failed to resolve the device path: %s", device_path.c_str());
    canonical_path = device_path;
  } else {
    canonical_path = real_path;
  }
  free(real_path);

  std::string disk_device = canonical_path;
  if (canonical_path.empty()) {
    // throw std::runtime_error("Failed to resolve the device path");
    AERROR_F("Failed to resolve the device path: %s", device_path.c_str());
    canonical_path = device_path;
  }

  // Extract the disk device from the canonical path
  // (e.g. /dev/sda1 -> /dev/sda, /dev/sda -> /dev/sda)
  std::regex device_regex(
      "/dev/([sh]d[a-z]+|[xv]vda|mmcblk[0-9]+|nvme[0-9]+n[0-9]+)(p[0-9]+)?");
  std::smatch match;
  if (std::regex_match(canonical_path, match, device_regex)) {
    disk_device = match[1].str();
    disk_device = "/dev/" + disk_device;
  } else {
    // throw std::runtime_error("Unable to determine the disk device");
    AERROR_F("Unable to determine the disk device: %s", canonical_path.c_str());
    return canonical_path;
  }

  return disk_device;
}

std::string ReadSysfsAttribute(const std::string &path) {
  std::ifstream file(path);
  std::string value;

  if (file.is_open()) {
    std::getline(file, value);
    file.close();
  }

  return value;
}

}  // namespace yalk
