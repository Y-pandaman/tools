#pragma once

#include "yalk/hwid/base.h"

namespace yalk {

/// \brief Hardware identifier based on the MAC address of the first
/// non-loopback interface.
class MacAddressIdentifier : public HardwareIdentifier {
 public:
  /// \brief Default constructor.
  MacAddressIdentifier() = default;
  /// \brief Default destructor.
  ~MacAddressIdentifier() = default;

 public:
  /// \brief Get the HWID from the MAC address of the first non-loopback
  /// interface.
  /// \return The HWID.
  std::string GetIdentifier() const override;
};
}  // namespace yalk
