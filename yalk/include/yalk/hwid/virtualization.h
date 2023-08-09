#pragma once

#include "yalk/hwid/base.h"

namespace yalk {

struct VirtualizationInfo {
  std::string cpuid;
  std::string product_name;
  std::string vendor;
};

enum class VirtualizationType {
  kPhysicalMachine = 0,
  kVMWare,
  kHyperV,
  kVirtualBox,
  kLXC,
  kParallels,
  kRedHat,
  kXen,
  kQEMU,
  kKVM,
  kDocker,
  kPodMan,
  kOpenBSD,
  kOther,
};

/// \brief Hardware identifier based on the UUID of the disk where the
/// root directory is located.
class VirtualizationIdentifier : public HardwareIdentifier {
 public:
  /// \brief Default constructor.
  VirtualizationIdentifier() = default;
  /// \brief Default destructor.
  ~VirtualizationIdentifier() = default;

 public:
  /// \brief Get the HWID from the virtualization status
  /// \return The HWID.
  std::string GetIdentifier() const override;

 private:
  VirtualizationInfo CollectInfo() const;
  std::string CollectCPUID() const;

  bool IsInDocker(const VirtualizationInfo& info) const;
  bool IsInVirtualMachine(const VirtualizationInfo& info) const;
};

}  // namespace yalk
