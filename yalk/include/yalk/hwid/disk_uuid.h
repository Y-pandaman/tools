#pragma once

#include "yalk/hwid/base.h"

namespace yalk {

/// \brief Hardware identifier based on the UUID of the disk where the
/// root filesystem is located.
class DiskUUIDIdentifier : public HardwareIdentifier {
 public:
  /// \brief Default constructor.
  DiskUUIDIdentifier() = default;
  /// \brief Default destructor.
  ~DiskUUIDIdentifier() = default;

 public:
  /// \brief Get the HWID from the UUID of the disk where the root directory is
  /// located.
  /// \return The HWID.
  std::string GetIdentifier() const override;

 private:
#ifdef BUILD_WITH_BLKID
  /// \brief Get the UUID of the disk where the given mount point is located by
  /// blkid.
  /// \param mount_point The mount point.
  /// \return The UUID of the disk partition.
  std::string GetUUIDByMountPoint(const std::string &mount_point) const;
#endif

  /// \brief Get the UUID of the disk where the given disk device path is
  /// located.
  /// \param device_path The disk device path (e.g. /dev/sda).
  /// \return The UUID of the disk.
  std::string GetUUIDByDiskDevice(const std::string &device_path) const;
};

}  // namespace yalk
