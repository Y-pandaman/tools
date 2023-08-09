#pragma once

#include <memory>

#include "yalk/base/common.h"
#include "yalk/base/errno.h"
#include "yalk/core/base.h"
#include "yalk/license/base.h"

namespace yalk {

class LicenseIssuer : public LicenseOperation {
 public:
  /// \brief Default constructor.
  LicenseIssuer() = default;
  /// \brief Default destructor.
  ~LicenseIssuer() = default;

 public:
  /// \brief Initialize the license issuer.
  /// \param config The configuration from command line or config file.
  /// \param private_key The private key used to sign and encrypt the license
  /// data.
  bool Init(const json& config, const std::string& private_key);

  /// \brief Issue a license.
  /// \return The license string.
  std::string IssueLicense() const;

  /// \brief Issue a license and save it to a file.
  /// \param file_path The file path to save the license.
  /// \return True if the license is saved successfully, false otherwise.
  bool IssueLicenseAndSaveToFile(const std::string& file_path) const;

 public:
  /// \brief Read the machine fingerprint from a file.
  /// \param file_path The file path to read the machine fingerprint.
  /// \return True if the machine fingerprint is read successfully, false
  /// otherwise.
  bool ReadMachineFingerprintFromFile(const std::string& file_path);

  /// \brief Read the machine fingerprint from a string.
  /// \param machine_fingerprint The machine fingerprint string.
  /// \return True if the machine fingerprint is read successfully, false
  /// otherwise.
  bool ReadMachineFingerprintFromString(const std::string& machine_fingerprint);

 private:
  /// \brief The license generator
  std::unique_ptr<LicenseGenerator> license_generator_ = nullptr;

  /// \brief The private key used to sign and encrypt the license data.
  std::string private_key_ = "";
};

}  // namespace yalk
