#pragma once

#include <string>

namespace yalk {

/// \brief Get the device path of the root filesystem.
/// \return The device path (e.g. /dev/sda1).
std::string GetRootFilesystemDevice();

/// \brief Get the device path of the disk where the given device path is
/// located.
/// \param device_path The device path (e.g. /dev/sda1).
/// \return The device path of the disk (e.g. /dev/sda).
std::string GetDiskDevice(const std::string &device_path);

/// \brief Read the content of a sysfs attribute.
/// \param path The path of the sysfs attribute.
/// \return The content of the sysfs attribute.
std::string ReadSysfsAttribute(const std::string &path);

}  // namespace yalk
