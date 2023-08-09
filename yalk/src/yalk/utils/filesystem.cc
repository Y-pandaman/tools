#include "yalk/utils/filesystem.h"

#include <fstream>

#include "yalk/utils/log.h"

namespace yalk {

bool ReadDataFromFile(const std::string& file, std::string& data) {
  // 1. read the file as a string
  std::ifstream ifs(file);
  if (!ifs.is_open()) {
    AERROR_F("Failed to open the file: %s", file.c_str());
    return false;
  }

  data = std::string((std::istreambuf_iterator<char>(ifs)),
                     std::istreambuf_iterator<char>());

  // 2. clean up
  ifs.close();

  return true;
}

bool WriteDataToFile(const std::string& file, const std::string& data) {
  // 1. open the file
  std::ofstream ofs(file);
  if (!ofs.is_open()) {
    AERROR_F("Failed to open the file: %s", file.c_str());
    return false;
  }

  // 2. write the data to the file
  ofs << data;

  // 3. clean up
  ofs.close();

  return true;
}

std::string GetAppDataDirectory(const std::string& app_name) {
  std::string data_dir;

#if defined(_WIN32)
  char* appdata = std::getenv("APPDATA");
  if (appdata) {
    data_dir = std::string(appdata) + "\\" + app_name + "\\";
  } else {
    throw std::runtime_error("APPDATA environment variable not found");
  }
#elif defined(__APPLE__) || defined(__unix__)
  char* home = std::getenv("HOME");
  if (home) {
    data_dir = std::string(home) + "/.local/share/" + app_name + "/";
  } else {
    throw std::runtime_error("HOME environment variable not found");
  }
#else
  throw std::runtime_error("Unsupported platform");
#endif

  // Create the directory if it doesn't exist
  std::string mkdir_command =
#if defined(_WIN32)
      "mkdir \"" + data_dir + "\" 2>nul";
#else
      "mkdir -p \"" + data_dir + "\"";
#endif
  int result = std::system(mkdir_command.c_str());
  if (result != 0) {
    throw std::runtime_error("Failed to create the data directory");
  }

  return data_dir;
}

void TouchFile(const std::string& filename) {
  std::ofstream file(filename);
  file.close();
}

}  // namespace yalk
