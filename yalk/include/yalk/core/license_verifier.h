#pragma once

#include "yalk/core/base.h"
#include "yalk/core/hwid_collecter.h"
#include "yalk/license/base.h"

namespace yalk {

class LicenseVerifier : public LicenseOperation {
 public:
  /// \brief Default constructor.
  LicenseVerifier() = default;
  /// \brief Default destructor.
  ~LicenseVerifier() = default;

 public:
  /// \brief Initialize the license verifier.
  /// \param config The configuration from command line or config file.
  /// \param public_key The public key used to verify and decrypt the license
  /// data.
  /// \return True if the license verifier is initialized successfully, false
  /// otherwise.
  bool Init(const json& config, const std::string& public_key);

 public:
  /// \brief Verify the license.
  /// \param license The license string.
  /// \return True if the license is verified successfully, false otherwise.
  bool VerifyLicenseFromString(const std::string& license) const;

  /// \brief Verify the license from a file.
  /// \param file_path The file path to read the license.
  /// \return True if the license is verified successfully, false otherwise.
  bool VerifyLicenseFromFile(const std::string& file_path) const;

  /// \brief Verify the license from a given environment variable.
  /// \param env_var_name The environment variable name.
  /// \return True if the license is verified successfully, false otherwise.
  bool VerifyLicenseFromEnvVar(const std::string& env_var_name) const;

  /// \brief Re-generate the machine fingerprint
  void RegenerateMachineFingerprint();

 private:
  /// \brief The license validator.
  std::unique_ptr<LicenseValidator> validator_;

  /// \brief The hardware info collector.
  std::unique_ptr<HardwareInfoCollector> hw_info_collector_;

  /// \brief The public key used to verify and decrypt the license data.
  std::string public_key_;
};

}  // namespace yalk
