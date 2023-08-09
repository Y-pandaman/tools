#pragma once

#include <cstdint>
#include <string>
#include <utility>

#include "yalk/base/common.h"
#include "yalk/utils/registerer.h"

namespace yalk {
/// \brief Base class for all license signers.
class LicenseGenerator {
 public:
  /// \brief Default constructor.
  LicenseGenerator() = default;

 public:
  /// \brief generate a license based on the product info, user info, HWID and
  /// extra info.
  /// \param product_info The product info.
  /// \param user_info The user info.
  /// \param hw_info The Hardware info.
  /// \param limitation_info The limitation info.
  /// \param extra_info The extra info.
  /// \return The license string.
  virtual std::string GenerateLicense(const ProductInfo& product_info,
                                      const UserInfo& user_info,
                                      const HardwareInfo& hw_info,
                                      const LimitationInfo& limitation_info,
                                      const json& extra_info = {}) const = 0;

  /// \brief generate a license based on the given info in JSON format.
  /// \param all_info The complete info in JSON format.
  /// \return The license string.
  virtual std::string GenerateLicense(const json& all_info) const = 0;

 public:
  /// \brief Set the private key used to sign the license.
  /// \param private_key The private key.
  void SetPrivateKey(std::string private_key) {
    private_key_ = std::move(private_key);
  }

 protected:
  /// \brief The private key used to sign the license.
  std::string private_key_;
};

YALK_REGISTER_REGISTERER(LicenseGenerator);
#define YALK_REGISTER_LICENSE_GENERATOR(name) \
  YALK_REGISTER_CLASS(LicenseGenerator, name)

/// \brief Base class for all license validators.
class LicenseValidator {
 public:
  /// \brief Default constructor.
  LicenseValidator() = default;

 public:
  /// \brief Validate the given license.
  /// \param license The license string.
  /// \param request_product_info The product info in the request.
  /// \param request_user_info The user info in the request.
  /// \param request_hw_info The hardware info in the request.
  /// \param request_limitation_info The limitation info in the request.
  /// \param request_extra_info The extra info in the request.
  /// \return True if the license is valid, false otherwise.
  virtual bool ValidateLicense(const std::string& license,
                               const ProductInfo& request_product_info,
                               const UserInfo& request_user_info,
                               const HardwareInfo& request_hw_info,
                               const LimitationInfo& request_limitation_info,
                               const json& request_extra_info = {}) const = 0;

  /// \brief Validate the given license.
  /// \param license The license string.
  /// \param request_info The complete info in JSON format.
  /// \return True if the license is valid, false otherwise.
  virtual bool ValidateLicense(const std::string& license,
                               const json& request_info) const = 0;

 public:
  /// \brief Set the public key used to validate the license.
  /// \param public_key The public key.
  void SetPublicKey(std::string public_key) {
    public_key_ = std::move(public_key);
  }

 protected:
  /// \brief The public key used to validate the license.
  std::string public_key_;
};

YALK_REGISTER_REGISTERER(LicenseValidator);
#define YALK_REGISTER_LICENSE_VALIDATOR(name) \
  YALK_REGISTER_CLASS(LicenseValidator, name)

}  // namespace yalk
