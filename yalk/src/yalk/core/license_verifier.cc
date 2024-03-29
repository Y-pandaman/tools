#include "yalk/core/license_verifier.h"

#include <cstdlib>
#include <memory>

#include "yalk/base/errno.h"
#include "yalk/utils/log.h"
#include "yalk/utils/parser.h"

namespace yalk {

bool LicenseVerifier::Init(const json& config, const std::string& public_key) {
  // 0. check input
  // 0.1. check common config
  if (!LicenseOperation::Init(config)) {
    return false;
  }

  // 0.2. check verifier config
  json verifier_config;
  if (!config.contains("verifier") || !config["verifier"].is_object()) {
    verifier_config = {{"validator", "LocalLicenseValidator"}};
    ADEBUG_F("Use default config for license verifier: %s",
             verifier_config.dump().c_str());
  } else {
    verifier_config = config["verifier"];
    ADEBUG_F("Use given config for license verifier : %s",
             verifier_config.dump().c_str());
  }

  // 1. initialize license validator
  if (!verifier_config.contains("validator") ||
      !verifier_config["validator"].is_string()) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid input config for license validator: " +
                         verifier_config.dump());
    return false;
  }
  auto validator = LicenseValidatorRegisterer::GetInstanceByName(
      verifier_config["validator"]);
  if (!validator) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Failed to initialize license validator: " +
                         verifier_config["validator"].dump());
    return false;
  }
  validator_.reset(validator);

  // 2. initialize hardware info collector
  // 2.1. init collector
  hw_info_collector_ = std::make_unique<HardwareInfoCollector>();
  if (!hw_info_collector_->Init(config, public_key)) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Failed to initialize hardware info collector");
    return false;
  }
  // 2.2. collect current hardware info
  hw_info_ = hw_info_collector_->CollectHWInfo();

  // 3. initialize public key
  public_key_ = public_key;
  validator_->SetPublicKey(public_key);

  SetLastErrorCode(ErrorCode::kOK);
  return true;
}

bool LicenseVerifier::VerifyLicenseFromString(
    const std::string& license) const {
  return validator_->ValidateLicense(license, product_info_, user_info_,
                                     hw_info_, limitation_info_, extra_info_);
}

bool LicenseVerifier::VerifyLicenseFromFile(
    const std::string& file_path) const {
  // 1. read license from file
  std::string license;
  if (!ReadDataFromFile(file_path, license)) {
    SetLastErrorCode(ErrorCode::kIOError,
                     "Failed to read license from file: " + file_path);
    return false;
  }

  // 2. verify license
  return VerifyLicenseFromString(license);
}

bool LicenseVerifier::VerifyLicenseFromEnvVar(
    const std::string& env_var_name) const {
  // 1. get license file path from env var
  // 1.1. check env var name
  if (env_var_name.empty()) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid environment variable name");
    return false;
  }
  // 1.2. get env var value
  char* env_var_value = std::getenv(env_var_name.c_str());
  if (env_var_value == nullptr) {
    SetLastErrorCode(
        ErrorCode::kInvalidArgument,
        "Failed to get the value of environment variable: " + env_var_name);
    return false;
  }
  std::string file_path = env_var_value;
  // 1.3. clean up
  std::free(env_var_value);

  // 2. verify license
  return VerifyLicenseFromFile(file_path);
}

void LicenseVerifier::RegenerateMachineFingerprint() {
  // 1. collect current hardware info
  hw_info_ = hw_info_collector_->CollectHWInfo();
}

}  // namespace yalk
