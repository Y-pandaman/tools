#pragma once

#include <string>

#include "yalk/base/common.h"
#include "yalk/hwid/base.h"

namespace yalk {

class HardwareInfoCollector {
 public:
  /// \brief Default constructor.
  HardwareInfoCollector() = default;
  /// \brief Default destructor.
  ~HardwareInfoCollector() = default;

 public:
  /// \brief Initialize the collector
  /// \param config The configuration from command line or config file.
  /// \param public_key The public key used to encrypt the collected
  /// hardware identifiers.
  bool Init(const json& config, const std::string& public_key);

  /// \brief Collect hardware identifiers.
  /// \return The hardware information.
  HardwareInfo CollectHWInfo() const;

  /// \brief Generate the machine fingerprint and encrypt it.
  /// \return The encrypted machine fingerprint.
  std::string GenerateMachineFingerprint() const;

  /// \brief save the machine fingerprint to a file.
  /// \param file_path The file path to save the machine fingerprint.
  /// \return True if the machine fingerprint is saved successfully, false
  /// otherwise.
  bool SaveMachineFingerprint(const std::string& file_path) const;

 private:
  /// \brief mapping between hardware identifier and its name
  std::map<std::string, std::unique_ptr<HardwareIdentifier>> hwid_map_;

  /// \brief version of the hardware information
  int version_ = 1;

  /// \brief The public key used to encrypt the collected hwids.
  std::string public_key_;
};

}  // namespace yalk
