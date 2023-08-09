#pragma once

#include "yalk/hwid/base.h"

namespace yalk {

/// \brief Hardware identifier based on the Stat of the disk where the
/// root filesystem is located.
class DiskStatIdentifier : public HardwareIdentifier {
 public:
  /// \brief Default constructor.
  DiskStatIdentifier() = default;
  /// \brief Default destructor.
  ~DiskStatIdentifier() = default;

 public:
  /// \brief Get the HWID from the Stat of the disk where the root filesystem is
  /// located.
  /// \return The HWID.
  std::string GetIdentifier() const override;

 private:
  /// \brief Get the size of the disk where the given device path is located.
  std::string GetDiskSize(const std::string &device_path) const;

  /// \brief Get the major and minor number of the disk where the given device
  /// path is located.
  std::string GetDiskMajorMinor(const std::string &device_path) const;
};

}  // namespace yalk
