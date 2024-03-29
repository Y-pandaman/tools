#pragma once

#include "yalk/license/base.h"

namespace yalk {

class LocalLicenseGenerator : public LicenseGenerator {
 public:
  /// \brief Default constructor.
  LocalLicenseGenerator() = default;
  /// \brief Default destructor.
  ~LocalLicenseGenerator() = default;

 public:
  /// \brief generate a license based on the product info, user info, HWID and
  /// extra info.
  /// \param product_info The product info.
  /// \param user_info The user info.
  /// \param hw_info The Hardware info.
  /// \param limitation_info The limitation info.
  /// \param extra_info The extra info.
  /// \return The license string.
  std::string GenerateLicense(const ProductInfo& product_info,
                              const UserInfo& user_info,
                              const HardwareInfo& hw_info,
                              const LimitationInfo& limitation_info,
                              const json& extra_info = {}) const override;

  /// \brief generate a license based on the given info in JSON format.
  /// \param all_info The complete info in JSON format.
  /// \return The license string.
  std::string GenerateLicense(const json& all_info) const override;
};

}  // namespace yalk
