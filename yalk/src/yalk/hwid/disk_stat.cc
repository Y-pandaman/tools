#include "yalk/hwid/disk_stat.h"

#include <sys/stat.h>
#include <sys/sysmacros.h>

#include "yalk/hwid/disk_common.h"
#include "yalk/utils/filesystem.h"

namespace yalk {

std::string DiskStatIdentifier::GetIdentifier() const {
  try {
    std::string root_fs_device = GetRootFilesystemDevice();
    std::string disk_device = GetDiskDevice(root_fs_device);

    std::string major_minor = GetDiskMajorMinor(disk_device);
    std::string size = GetDiskSize(disk_device);

    return major_minor + ":" + size;
  } catch (std::exception &e) {
    return "";
  }
};

std::string DiskStatIdentifier::GetDiskSize(
    const std::string &device_path) const {
  // Get the name of the device.
  // e.g. /dev/sda -> sda
  std::string device_name =
      device_path.substr(device_path.find_last_of('/') + 1);

  // Read the size of the device.
  std::string sysfs_path = "/sys/block/" + device_name;
  std::string size_path = sysfs_path + "/size";
  std::string size;
  if (!ReadDataFromFile(size_path, size)) {
    AERROR_F("Failed to read size from %s", size_path.c_str());
    return "";
  }

  // Remove the trailing newline.
  size.erase(size.find_last_not_of(" \n\r\t") + 1);

  return size;
}

std::string DiskStatIdentifier::GetDiskMajorMinor(
    const std::string &device_path) const {
  // Read the stat of the device.
  struct stat statbuf;
  if (stat(device_path.c_str(), &statbuf) != 0) {
    // throw std::runtime_error("Failed to stat device: " + device_path);
    AERROR_F("Failed to stat device: %s", device_path.c_str());
    return ":";
  }

  // Extract the major and minor number from the stat.
  int major_number = major(statbuf.st_rdev);
  int minor_number = minor(statbuf.st_rdev);

  return std::to_string(major_number) + ":" + std::to_string(minor_number);
}

YALK_REGISTER_HWID(DiskStatIdentifier);

}  // namespace yalk
