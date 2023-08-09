#pragma once

#include "yalk/base/common.h"
#include "yalk/base/errno.h"

namespace yalk {

class LicenseOperation {
 public:
  /// \brief Default constructor.
  LicenseOperation() = default;
  /// \brief Default destructor.
  ~LicenseOperation() = default;

 public:
  /// \brief Initialize the license operation.
  /// \param config The configuration from command line or config file.
  /// \return True if the license operation is initialized successfully, false
  /// otherwise.
  virtual bool Init(const json& config);

 public:
  /// \brief Set the product info.
  /// \param product_info The product info.
  void SetProductInfo(const ProductInfo& product_info);

  /// \brief Set the user info.
  /// \param user_info The user info.
  void SetUserInfo(const UserInfo& user_info);

  /// \brief Set the hardware info.
  /// \param hw_info The hardware info.
  void SetHardwareInfo(const HardwareInfo& hw_info);

  /// \brief Set the limitation info.
  /// \param limitation_info The limitation info.
  void SetLimitationInfo(const LimitationInfo& limitation_info);

  /// \brief Set the extra info.
  /// \param extra_info The extra info.
  void SetExtraInfo(const json& extra_info);

 public:
  /// \brief Get the product info.
  /// \return The product info.
  const ProductInfo& GetProductInfo() const;

  /// \brief Get the user info.
  /// \return The user info.
  const UserInfo& GetUserInfo() const;

  /// \brief Get the hardware info.
  /// \return The hardware info.
  const HardwareInfo& GetHardwareInfo() const;

  /// \brief Get the limitation info.
  /// \return The limitation info.
  const LimitationInfo& GetLimitationInfo() const;

  /// \brief Get the extra info.
  /// \return The extra info.
  const json& GetExtraInfo() const;

 protected:
  /// \brief The product info.
  ProductInfo product_info_ = {"", ""};

  /// \brief The user info.
  UserInfo user_info_ = {"", ""};

  /// \brief The hardware info.
  HardwareInfo hw_info_ = {0, {}};

  /// \brief The limitation info.
  LimitationInfo limitation_info_ = {0, 0, 0, {}, LicenseType::kEvaluation,
                                     0, 0};

  /// \brief The extra info.
  json extra_info_ = {};
};

}  // namespace yalk
