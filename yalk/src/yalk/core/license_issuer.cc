#include "yalk/core/license_issuer.h"

#include "yalk/base/errno.h"
#include "yalk/utils/crypto.h"
#include "yalk/utils/mock.h"
#include "yalk/utils/parser.h"

namespace yalk {

bool LicenseIssuer::Init(const json& config, const std::string& private_key) {
  // 0. check input
  // 0.1. check common config
  if (!LicenseOperation::Init(config)) {
    return false;
  }

  // 0.2. check issuer config
  json issuer_config;
  if (!config.contains("issuer") || !config["issuer"].is_object()) {
    ADEBUG_F("Use default config for license issuer");
    issuer_config = {{"generator", "LocalLicenseGenerator"}};
  } else {
    issuer_config = config["issuer"];
  }

  // 1. initialize license generator
  if (!issuer_config.contains("generator") ||
      !issuer_config["generator"].is_string()) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid input config for license generator");
    return false;
  }
  auto generator =
      LicenseGeneratorRegisterer::GetInstanceByName(issuer_config["generator"]);
  if (!generator) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Failed to initialize license generator: " +
                         issuer_config["generator"].get<std::string>());
    return false;
  }
  license_generator_.reset(generator);

  // 2. initialize private key
  private_key_ = private_key;
  generator->SetPrivateKey(private_key);

  SetLastErrorCode(ErrorCode::kOK);
  return true;
}

std::string LicenseIssuer::IssueLicense() const {
  // 0. check inputs
  // 0.1. check hw info
  if (hw_info_.version == 0 || hw_info_.info.empty()) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid hardware information to issue license");
    return "";
  }
  // 0.2. check private key
  if (private_key_.empty()) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid private key to issue license");
    return "";
  }

  // 1. generate license
  return license_generator_->GenerateLicense(
      product_info_, user_info_, hw_info_, limitation_info_, extra_info_);
}

bool LicenseIssuer::IssueLicenseAndSaveToFile(
    const std::string& file_path) const {
  // 1. generate license
  std::string license = IssueLicense();
  if (license.empty()) {
    return false;
  }

  // 2. save license to file
  if (!WriteDataToFile(file_path, license)) {
    SetLastErrorCode(ErrorCode::kIOError,
                     "Failed to save license to file: " + file_path);
    return false;
  }

  SetLastErrorCode(ErrorCode::kOK);
  return true;
}

bool LicenseIssuer::ReadMachineFingerprintFromFile(
    const std::string& file_path) {
  // 1. read machine fingerprint from file
  std::string machine_fingerprint;
  if (!ReadDataFromFile(file_path, machine_fingerprint)) {
    SetLastErrorCode(
        ErrorCode::kIOError,
        "Failed to read machine fingerprint from file: " + file_path);
    return false;
  }

  // 2. fallback to read machine fingerprint from string
  return ReadMachineFingerprintFromString(machine_fingerprint);
}

bool LicenseIssuer::ReadMachineFingerprintFromString(
    const std::string& machine_fingerprint) {
  // 2. decrypt machine fingerprint
  // 2.1. decrypt machine fingerprint with private key
  std::string decrypted_machine_fingerprint =
      yalk::DecryptData(machine_fingerprint, private_key_, false);
  // 2.2. convert machine fingerprint string to json
  json decrypted_machine_fingerprint_json;
  try {
    decrypted_machine_fingerprint_json =
        json::parse(decrypted_machine_fingerprint);
  } catch (const std::exception& e) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid machine fingerprint to parse");
    return false;
  }

  // 3. validate hardware information
  if (!decrypted_machine_fingerprint_json.contains("hw_version") ||
      !decrypted_machine_fingerprint_json["hw_version"].is_number()) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid machine fingerprint: no hw version");
    return false;
  }
  if (!decrypted_machine_fingerprint_json.contains("hw_info") ||
      !decrypted_machine_fingerprint_json["hw_info"].is_object()) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Invalid machine fingerprint: no hw info");
    return false;
  }

  // 4. set hardware information
  hw_info_.version =
      decrypted_machine_fingerprint_json["hw_version"].get<int>();
  hw_info_.info = decrypted_machine_fingerprint_json["hw_info"];

  SetLastErrorCode(ErrorCode::kOK);
  return true;
}

}  // namespace yalk
