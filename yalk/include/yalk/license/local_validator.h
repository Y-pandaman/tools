#pragma once

#include "yalk/license/base.h"

namespace yalk {

class LocalLicenseValidator : public LicenseValidator {
 public:
  /// \brief Default constructor.
  LocalLicenseValidator() = default;
  /// \brief Default destructor.
  ~LocalLicenseValidator() = default;

  /// \brief Validate the given license.
  /// \param license The license string.
  /// \param request_product_info The product info in the request.
  /// \param request_user_info The user info in the request.
  /// \param request_hw_info The hardware info in the request.
  /// \param request_limitation_info The limitation info in the request.
  /// \param request_extra_info The extra info in the request.
  /// \return True if the license is valid, false otherwise.
  bool ValidateLicense(const std::string& license,
                       const ProductInfo& request_product_info,
                       const UserInfo& request_user_info,
                       const HardwareInfo& request_hw_info,
                       const LimitationInfo& request_limitation_info,
                       const json& request_extra_info = {}) const override;

  /// \brief Validate the given license.
  /// \param license The license string.
  /// \param request_info The complete info in JSON format.
  /// \return True if the license is valid, false otherwise.
  bool ValidateLicense(const std::string& license,
                       const json& request_info) const override;
};

}  // namespace yalk
