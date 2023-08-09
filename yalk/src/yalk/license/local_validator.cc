#include "yalk/license/local_validator.h"

#include "yalk/base/common.h"
#include "yalk/base/errno.h"
#include "yalk/utils/crypto.h"
#include "yalk/utils/log.h"
#include "yalk/utils/parser.h"
#include "yalk/utils/time.h"

namespace yalk {

bool LocalLicenseValidator::ValidateLicense(
    const std::string& license, const ProductInfo& request_product_info,
    const UserInfo& request_user_info, const HardwareInfo& request_hw_info,
    const LimitationInfo& request_limitation_info,
    const json& request_extra_info) const {
  // construct a json object
  json all_info;
  MergeLicenseInfo(request_product_info, request_user_info, request_hw_info,
                   request_limitation_info, request_extra_info, all_info);

  // fallback to the json version
  return ValidateLicense(license, all_info);
}

bool LocalLicenseValidator::ValidateLicense(const std::string& license,
                                            const json& request_info) const {
  // 0. check inputs
  // 0.1. check the validity of the input
  if (!request_info.contains("product_name") ||
      !request_info.contains("product_version") ||
      !request_info.contains("type") ||
      !request_info.contains("issue_number")) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid license info: " + request_info.dump());
    return false;
  }

  // 1. parse the license
  // 1.1. parse license data from string
  LicenseData license_data;
  if (!ParseLicenseInfoFromString(license, license_data)) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Failed to parse license data.");
    return false;
  }
  // 1.2. decrypt license info
  std::string all_info_str;
  try {
    all_info_str = DecryptData(license_data.encrypted, public_key_, true);
    // check if the decryption result is valid
    if (all_info_str.empty()) {
      SetLastErrorCode(ErrorCode::kInvalidKey,
                       "Failed to decrypt license info.");
      return false;
    }
  } catch (const std::exception& e) {
    SetLastErrorCode(
        ErrorCode::kInvalidKey,
        "Failed to decrypt license info: " + std::string(e.what()));
    return false;
  }

  // TODO check the integrity of the parsed license info and the decrypted
  // license info
  try {
    license_data.info = json::parse(all_info_str);
  } catch (const std::exception& e) {
    SetLastErrorCode(
        ErrorCode::kLicenseInvalid,
        "Failed to parse decrypted license info: " + std::string(e.what()));
    return false;
  }

  // 2. check the license
  // 2.1. check the signature
  if (!VerifyData(license_data.info.dump(), public_key_,
                  license_data.signature)) {
    SetLastErrorCode(ErrorCode::kLicenseInvalid,
                     "Failed to verify license signature.");
    return false;
  }

  // 2.2. check the product info
  if (license_data.info["product_name"] != request_info["product_name"] ||
      license_data.info["product_version"] != request_info["product_version"]) {
    SetLastErrorCode(
        ErrorCode::kLicenseProductMismatch,
        "Failed to verify product info (requested: " +
            request_info["product_name"].get<std::string>() + ", " +
            request_info["product_version"].get<std::string>() + "; actual: " +
            license_data.info["product_name"].get<std::string>() + ", " +
            license_data.info["product_version"].get<std::string>() + ").");
    return false;
  }

  // 2.3. check the user info
  if (license_data.info.contains("user_name") &&
      license_data.info["user_name"] != request_info["user_name"]) {
    SetLastErrorCode(
        ErrorCode::kLicenseUserMismatch,
        "Failed to verify user name (requested: " +
            request_info["user_name"].get<std::string>() + "; actual: " +
            license_data.info["user_name"].get<std::string>() + ").");
    return false;
  }
  if (license_data.info.contains("user_email") &&
      license_data.info["user_email"] != request_info["user_email"]) {
    SetLastErrorCode(
        ErrorCode::kLicenseUserMismatch,
        "Failed to verify user email (requested: " +
            request_info["user_email"].get<std::string>() + "; actual: " +
            license_data.info["user_email"].get<std::string>() + ").");
    return false;
  }

  // 2.4. check the hardware info
  // TODO use more strict hardware info check
  if ((license_data.info.contains("hw_version") &&
       license_data.info["hw_version"] != request_info["hw_version"]) ||
      (license_data.info.contains("hw_info") &&
       license_data.info["hw_info"] != request_info["hw_info"])) {
    AERROR_F("Failed to verify hardware info");
    SetLastErrorCode(ErrorCode::kLicenseHardwareMismatch,
                     "Failed to verify hardware info.");
    return false;
  }

  // 2.5. check the limitation info
  // 2.5.1. check the expiration time
  if (license_data.info.contains("expiration_time") &&
      license_data.info["expiration_time"] > 0) {
    // 2.5.1.1. check if the expiration time is earlier than the requested
    // expiration time
    if (license_data.info["expiration_time"] <
        request_info["expiration_time"]) {
      SetLastErrorCode(
          ErrorCode::kLicenseExpired,
          "Failed to verify expiration time (requested: " +
              std::to_string(request_info["expiration_time"].get<int>()) +
              "; actual: " +
              std::to_string(license_data.info["expiration_time"].get<int>()) +
              ").");
      return false;
    }
    // 2.5.1.2. check if the expiration time is earlier than the current time
    // (if the current time is available)
    auto current_time = GetCurrentUnixTimestamp();
    if (current_time > 0 &&
        license_data.info["expiration_time"] < GetCurrentUnixTimestamp()) {
      SetLastErrorCode(
          ErrorCode::kLicenseExpired,
          "Failed to verify expiration time (current: " +
              std::to_string(current_time) + "; expiration: " +
              std::to_string(license_data.info["expiration_time"].get<int>()) +
              ").");
      return false;
    }
  }
  // 2.5.2. check the max uses
  if (license_data.info.contains("max_uses") &&
      license_data.info["max_uses"] < request_info["max_uses"] &&
      license_data.info["max_uses"] > 0) {
    SetLastErrorCode(
        ErrorCode::kLicenseExceedUses,
        "Failed to verify max uses (requested: " +
            std::to_string(request_info["max_uses"].get<int>()) + "; actual: " +
            std::to_string(license_data.info["max_uses"].get<int>()) + ").");
    return false;
  }
  // 2.5.3. check the max qps
  if (license_data.info.contains("max_qps") &&
      license_data.info["max_qps"] < request_info["max_qps"] &&
      license_data.info["max_qps"] > 0) {
    SetLastErrorCode(
        ErrorCode::kLicenseExceedQPS,
        "Failed to verify max qps (requested: " +
            std::to_string(request_info["max_qps"].get<int>()) + "; actual: " +
            std::to_string(license_data.info["max_qps"].get<int>()) + ").");
    return false;
  }
  // 2.5.4. check the features
  if (license_data.info.contains("features") &&
      license_data.info["features"].size() > 0 &&
      license_data.info["features"] != request_info["features"]) {
    SetLastErrorCode(ErrorCode::kLicenseFeatureMismatch,
                     "Failed to verify features (requested: " +
                         request_info["features"].dump() + "; actual: " +
                         license_data.info["features"].dump() + ").");
    return false;
  }
  // 2.5.5. check the type
  if (license_data.info["type"] != request_info["type"]) {
    SetLastErrorCode(
        ErrorCode::kLicenseTypeMismatch,
        "Failed to verify type (requested: " +
            ToString(static_cast<LicenseType>(request_info["type"])) +
            "; actual: " +
            ToString(static_cast<LicenseType>(license_data.info["type"])) +
            ").");
    return false;
  }
  // 2.5.6. check the issue number
  if (license_data.info["issue_number"] != request_info["issue_number"]) {
    SetLastErrorCode(
        ErrorCode::kLicenseInvalid,
        "Failed to verify issue number (requested: " +
            std::to_string(request_info["issue_number"].get<int>()) +
            "; actual: " +
            std::to_string(license_data.info["issue_number"].get<int>()) +
            ").");
    return false;
  }

  SetLastErrorCode(ErrorCode::kOK);
  return true;
}

YALK_REGISTER_LICENSE_VALIDATOR(LocalLicenseValidator)

}  // namespace yalk
