#include "yalk/hwid/cpu_info.h"

#include "yalk/utils/filesystem.h"

namespace yalk {

std::string CPUInfoIdentifier::GetIdentifier() const {
  try {
    // 1. get the cpu info
    std::string cpu_info = GetCPUInfo();
    if (cpu_info.empty()) {
      AERROR_F("Failed to get the cpu info.");
    }

    // 2. get the motherboard info
    std::string motherboard_info = GetMotherboardInfo();
    if (motherboard_info.empty()) {
      AERROR_F("Failed to get the motherboard info.");
    }

    // 3. get the HWID
    std::string hwid = cpu_info + ":" + motherboard_info;
    return hwid;
  } catch (const std::exception& e) {
    AERROR_F("Failed to get the HWID from the cpu info: %s", e.what());
    return "";
  }
}

std::string CPUInfoIdentifier::GetCPUInfo() const {
  // 1. read the file
  std::string cpuinfo;
  if (!ReadDataFromFile("/proc/cpuinfo", cpuinfo)) {
    AERROR_F("Failed to read the file: %s", "/proc/cpuinfo");
    return "";
  }

  // 2. get the cpu model name
  std::istringstream cpuinfo_stream(cpuinfo);
  std::string line;
  std::string cpu_model_name;
  while (std::getline(cpuinfo_stream, line)) {
    if (line.find("model name") != std::string::npos) {
      // get the cpu model name
      cpu_model_name = line.substr(line.find(':') + 1);
      // remove the leading spaces
      cpu_model_name.erase(0, cpu_model_name.find_first_not_of(' '));
      break;
    }
  }

  return cpu_model_name;
}

std::string CPUInfoIdentifier::GetMotherboardInfo() const {
  // 1. get the motherboard name
  std::string motherboard_name;
  if (!ReadDataFromFile("/sys/devices/virtual/dmi/id/board_name",
                        motherboard_name)) {
    AERROR_F("Failed to read the file: %s",
             "/sys/devices/virtual/dmi/id/board_name");
  }

  // 2. remove the trailing spaces
  motherboard_name.erase(motherboard_name.find_last_not_of(" \n\r\t") + 1);

  return motherboard_name;
}

YALK_REGISTER_HWID(CPUInfoIdentifier);

}  // namespace yalk
