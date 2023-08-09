#include "yalk/core/base.h"

#include "yalk/base/errno.h"
#include "yalk/utils/log.h"
#include "yalk/utils/parser.h"

namespace yalk {

bool LicenseOperation::Init(const json& config) {
  // 0. check the config
  if (!config.contains("common") || !config["common"].is_object()) {
    ADEBUG_F("Use default config for license operation");
  } else if (!ParseLicenseInfoFromJSON(config["common"], product_info_,
                                       user_info_, hw_info_, limitation_info_,
                                       extra_info_)) {
    SetLastErrorCode(
        ErrorCode::kInvalidArgument,
        "Failed to parse license info from JSON config for license operation");
    return false;
  }

  SetLastErrorCode(ErrorCode::kOK);
  return true;
}

void LicenseOperation::SetProductInfo(const ProductInfo& product_info) {
  product_info_ = product_info;
}

void LicenseOperation::SetUserInfo(const UserInfo& user_info) {
  user_info_ = user_info;
}

void LicenseOperation::SetHardwareInfo(const HardwareInfo& hw_info) {
  hw_info_ = hw_info;
}

void LicenseOperation::SetLimitationInfo(
    const LimitationInfo& limitation_info) {
  limitation_info_ = limitation_info;
}

void LicenseOperation::SetExtraInfo(const json& extra_info) {
  extra_info_ = extra_info;
}

const ProductInfo& LicenseOperation::GetProductInfo() const {
  return product_info_;
}

const UserInfo& LicenseOperation::GetUserInfo() const { return user_info_; }

const HardwareInfo& LicenseOperation::GetHardwareInfo() const {
  return hw_info_;
}

const LimitationInfo& LicenseOperation::GetLimitationInfo() const {
  return limitation_info_;
}

const json& LicenseOperation::GetExtraInfo() const { return extra_info_; }

}  // namespace yalk
