#pragma once

#include "yalk/hwid/base.h"

namespace yalk {

/// \brief Hardware identifier based on the cpu info.
class CPUInfoIdentifier : public HardwareIdentifier {
 public:
  /// \brief Default constructor.
  CPUInfoIdentifier() = default;
  /// \brief Default destructor.
  ~CPUInfoIdentifier() = default;

 public:
  /// \brief Get the HWID from the cpu info.
  /// \return The HWID.
  std::string GetIdentifier() const override;

 private:
  /// \brief Get the cpu info.
  /// \return The cpu info.
  std::string GetCPUInfo() const;

  /// \brief Get the motherboard info.
  /// \return The motherboard info.
  std::string GetMotherboardInfo() const;
};

}  // namespace yalk
