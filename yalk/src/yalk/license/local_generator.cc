#include "yalk/license/local_generator.h"

#include <iostream>

#include "yalk/base/common.h"
#include "yalk/base/errno.h"
#include "yalk/license/base.h"
#include "yalk/utils/base64.h"
#include "yalk/utils/crypto.h"
#include "yalk/utils/parser.h"

namespace yalk {

std::string LocalLicenseGenerator::GenerateLicense(
    const ProductInfo& product_info, const UserInfo& user_info,
    const HardwareInfo& hw_info, const LimitationInfo& limitation_info,
    const json& extra_info) const {
  // construct a json object json all_info;
  json all_info;
  MergeLicenseInfo(product_info, user_info, hw_info, limitation_info,
                   extra_info, all_info);

  // fallback to the json version
  return GenerateLicense(all_info);
}

std::string LocalLicenseGenerator::GenerateLicense(const json& all_info) const {
  // 0. check inputs
  // 0.1. check the validity of the input
  if (!all_info.contains("product_name") ||
      !all_info.contains("product_version") ||
      !all_info.contains("user_name") || !all_info.contains("user_email") ||
      !all_info.contains("hw_version") || !all_info.contains("hw_info") ||
      !all_info.contains("expiration_time") || !all_info.contains("max_uses") ||
      !all_info.contains("max_qps") || !all_info.contains("features") ||
      !all_info.contains("type") || !all_info.contains("issue_time") ||
      !all_info.contains("issue_number")) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid license info: " + all_info.dump());
    return "";
  }

  // 0.2. check key
  if (private_key_.empty()) {
    SetLastErrorCode(ErrorCode::kInvalidArgument, "Invalid private key.");
    return "";
  }

  // 1. generate the license
  LicenseData license_data;
  license_data.info = all_info;
  std::string license = all_info.dump();
  // std::cout << "license: " << license << std::endl;
  // 1.1. encrypt the license
  try {
    license_data.encrypted = EncryptData(license, private_key_, false);
  } catch (const std::exception& e) {
    SetLastErrorCode(ErrorCode::kFail, e.what());
    return "";
  }
  // std::cout << "encrypted: " << license_data.encrypted << std::endl;
  // 1.2. sign the license
  try {
    license_data.signature = SignData(license, private_key_);
  } catch (const std::exception& e) {
    SetLastErrorCode(ErrorCode::kFail, e.what());
    return "";
  }

  // 2. pretty print the license in the following format:
  std::string license_data_str;
  try {
    DumpLicenseInfoToString(license_data, license_data_str);
  } catch (const std::exception& e) {
    SetLastErrorCode(ErrorCode::kFail, e.what());
    return "";
  }

  SetLastErrorCode(ErrorCode::kOK);
  return license_data_str;
}

YALK_REGISTER_LICENSE_GENERATOR(LocalLicenseGenerator);

}  // namespace yalk
