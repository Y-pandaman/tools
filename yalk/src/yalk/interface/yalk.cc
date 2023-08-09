#include "yalk/interface/yalk.h"

#include <sstream>

#include "yalk/base/common.h"
#include "yalk/base/errno.h"
#include "yalk/core/hwid_collecter.h"
#include "yalk/core/license_issuer.h"
#include "yalk/core/license_verifier.h"
#include "yalk/utils/base64.h"
#include "yalk/utils/crypto.h"
#include "yalk/utils/log.h"
#include "yalk/utils/mock.h"
#include "yalk/utils/parser.h"

YALK_EVENT_TYPE yalk_keygen(const char* config, const char* output_dir_path) {
  // 0. check inputs
  if (!output_dir_path || strlen(output_dir_path) == 0) {
    AERROR_F("Invalid output dir path");
    return YALK_INVALID_ARGUMENT;
  }

  // 1. generate key pair
  std::string public_key_path =
      std::string(output_dir_path) + "/public_key.pem";
  std::string private_key_path =
      std::string(output_dir_path) + "/private_key.pem";
  yalk::GenerateKeyPair(public_key_path, private_key_path);

  // 2. print key paths
  AINFO_F("Public key saved to %s", public_key_path.c_str());
  AINFO_F("Private key saved to %s", private_key_path.c_str());

  // 3. print base64 encoded keys
  std::string public_key_base64 =
      yalk::Base64Encode(yalk::LoadPublicKey(public_key_path));
  AINFO_F("Base64 encoded public key:\n%s", public_key_base64.c_str());
  std::string private_key_base64 =
      yalk::Base64Encode(yalk::LoadPrivateKey(private_key_path));
  AINFO_F("Base64 encoded private key:\n%s", private_key_base64.c_str());

  return YALK_OK;
}

YALK_EVENT_TYPE yalk_collect(const char* config, const char* output_file_path,
                             const char* public_key_file_path,
                             const char* public_key_base64) {
  // 0. check inputs
  if ((!public_key_file_path || strlen(public_key_file_path) == 0) &&
      (!public_key_base64 || strlen(public_key_base64) == 0)) {
    AERROR_F("Invalid public key file path");
    return YALK_INVALID_ARGUMENT;
  }

  if (public_key_file_path) {
    // 1. read public key and encode in base64
    std::string public_key_base64_str;
    try {
      public_key_base64_str =
          yalk::Base64Encode(yalk::LoadPublicKey(public_key_file_path));
    } catch (const std::exception& e) {
      AERROR_F("Failed to read public key from %s", public_key_file_path);
      return YALK_IO_ERROR;
    }

    // 2. fallback to yalk_collect_from_string
    return yalk_collect_from_string(config, output_file_path,
                                    public_key_base64_str.c_str());
  } else {
    // 1. fallback to yalk_collect_from_string
    return yalk_collect_from_string(config, output_file_path,
                                    public_key_base64);
  }
}

YALK_EVENT_TYPE yalk_collect_from_string(const char* config,
                                         const char* output_file_path,
                                         const char* public_key_base64) {
  // 0. check inputs
  if (!public_key_base64 || strlen(public_key_base64) == 0) {
    AERROR_F("Invalid public key");
    return YALK_INVALID_ARGUMENT;
  }

  // 1. decode public key
  std::string public_key_str;
  try {
    public_key_str = yalk::Base64Decode(public_key_base64);
  } catch (const std::exception& e) {
    AERROR_F("Failed to decode public key");
    return YALK_IO_ERROR;
  }

  // 2. initialize hardware info collector
  yalk::HardwareInfoCollector collecter;
  if (!collecter.Init({{"hwid", kHWIDAlgorithms[1]}}, public_key_str)) {
    AERROR_F(
        "Failed to initialize hardware info collector: errno=[%d](%s), "
        "msg=[%s]",
        static_cast<int>(yalk::GetLastErrorCode()),
        yalk::ToString(yalk::GetLastErrorCode()).c_str(),
        yalk::GetLastErrorMessage().c_str());
    return YALK_FAILED;
  }

  // 3. collect hardware info
  if (output_file_path && strlen(output_file_path) > 0) {
    if (!collecter.SaveMachineFingerprint(output_file_path)) {
      AERROR_F("Failed to save HWID to %s: errno=[%d](%s), msg=[%s]",
               output_file_path, static_cast<int>(yalk::GetLastErrorCode()),
               yalk::ToString(yalk::GetLastErrorCode()).c_str(),
               yalk::GetLastErrorMessage().c_str());
      return YALK_FAILED;
    }
    AINFO_F("HWID saved to %s", output_file_path);
  } else {
    std::string hwid = collecter.GenerateMachineFingerprint();
    if (hwid.empty()) {
      AERROR_F("Failed to generate HWID: errno=[%d](%s), msg=[%s]",
               static_cast<int>(yalk::GetLastErrorCode()),
               yalk::ToString(yalk::GetLastErrorCode()).c_str(),
               yalk::GetLastErrorMessage().c_str());
      return YALK_FAILED;
    }
    AINFO_F("HWID (encoded):\n%s", yalk::Base64Encode(hwid).c_str());
  }

  return YALK_OK;
}

YALK_EVENT_TYPE yalk_issue(const char* config, const char* output_file_path,
                           const char* machine_fingerprint_file_path,
                           const char* machine_fingerprint_base64,
                           const char* private_key_file_path,
                           const char* private_key_base64) {
  // 0. check inputs
  if ((!machine_fingerprint_file_path ||
       strlen(machine_fingerprint_file_path) == 0) &&
      (!machine_fingerprint_base64 ||
       strlen(machine_fingerprint_base64) == 0)) {
    AERROR_F("Invalid machine fingerprint input");
    return YALK_INVALID_ARGUMENT;
  } else if ((!private_key_file_path || strlen(private_key_file_path) == 0) &&
             (!private_key_base64 || strlen(private_key_base64) == 0)) {
    AERROR_F("Invalid private key input");
    return YALK_INVALID_ARGUMENT;
  } else if (!output_file_path || strlen(output_file_path) == 0) {
    AERROR_F("Invalid output file path");
    return YALK_INVALID_ARGUMENT;
  }

  // 1. read private key and machine fingerprint
  // 1.1. read private key
  std::string private_key_base64_str;
  if (private_key_file_path) {
    try {
      private_key_base64_str =
          yalk::Base64Encode(yalk::LoadPrivateKey(private_key_file_path));
    } catch (const std::exception& e) {
      AERROR_F("Failed to read private key from %s", private_key_file_path);
      return YALK_IO_ERROR;
    }
  } else {
    private_key_base64_str = private_key_base64;
  }

  // 1.2. read machine fingerprint
  std::string machine_fingerprint_base64_str;
  if (machine_fingerprint_file_path) {
    // read machine fingerprint from file
    if (!yalk::ReadDataFromFile(machine_fingerprint_file_path,
                                machine_fingerprint_base64_str)) {
      AERROR_F("Failed to read machine fingerprint from %s",
               machine_fingerprint_file_path);
      return YALK_IO_ERROR;
    }
    // encode machine fingerprint
    try {
      machine_fingerprint_base64_str =
          yalk::Base64Encode(machine_fingerprint_base64_str);
    } catch (const std::exception& e) {
      AERROR_F("Failed to encode machine fingerprint");
      return YALK_IO_ERROR;
    }
  } else {
    machine_fingerprint_base64_str = machine_fingerprint_base64;
  }

  // 2. fallback to yalk_issue_from_string
  return yalk_issue_from_string(config, output_file_path,
                                machine_fingerprint_base64_str.c_str(),
                                private_key_base64_str.c_str());
}

YALK_EVENT_TYPE yalk_issue_from_string(const char* config,
                                       const char* output_file_path,
                                       const char* machine_fingerprint_base64,
                                       const char* private_key_base64) {
  // 0. check inputs
  if (!machine_fingerprint_base64 || strlen(machine_fingerprint_base64) == 0) {
    AERROR_F("Invalid machine fingerprint");
    return YALK_INVALID_ARGUMENT;
  } else if (!private_key_base64 || strlen(private_key_base64) == 0) {
    AERROR_F("Invalid private key");
    return YALK_INVALID_ARGUMENT;
  }

  // 1. decode private key
  std::string private_key_str;
  std::string machine_fingerprint_str;
  try {
    private_key_str = yalk::Base64Decode(private_key_base64);
    machine_fingerprint_str = yalk::Base64Decode(machine_fingerprint_base64);
  } catch (const std::exception& e) {
    AERROR_F("Failed to decode private key or machine fingerprint");
    return YALK_INVALID_ARGUMENT;
  }

  // 2. init issuer
  // 2.1. init config
  json config_json;
  if (!config || strlen(config) == 0) {
    config_json = {{"issuer", {{"generator", "LocalLicenseGenerator"}}}};
  } else {
    try {
      config_json = json::parse(config);
    } catch (const std::exception& e) {
      AERROR_F("Failed to parse config: %s", e.what());
      return YALK_INVALID_ARGUMENT;
    }
  }
  // 2.2. init issuer
  yalk::LicenseIssuer issuer;
  AINFO_F("Config:\n%s", config_json.dump().c_str());
  if (!issuer.Init(config_json, private_key_str)) {
    AERROR_F("Failed to initialize license issuer");
    return YALK_INVALID_ARGUMENT;
  }
  // 2.3. read machine fingerprint
  if (!issuer.ReadMachineFingerprintFromString(machine_fingerprint_str)) {
    AERROR_F("Failed to read machine fingerprint from string");
    return YALK_INVALID_ARGUMENT;
  }

  // 3. issue license
  if (output_file_path && strlen(output_file_path) > 0) {
    issuer.IssueLicenseAndSaveToFile(output_file_path);
    AINFO_F("Issued license saved to %s", output_file_path);
  } else {
    std::string license = issuer.IssueLicense();
    AINFO_F("Issued license:\n%s", license.c_str());
  }

  return YALK_OK;
}

YALK_EVENT_TYPE yalk_verify(const char* config, const char* license_env_name,
                            const char* license_file_path,
                            const char* license_base64,
                            const char* public_key_env_name,
                            const char* public_key_file_path,
                            const char* public_key_base64, bool* is_valid) {
  // 0. check inputs
  if ((!license_env_name || strlen(license_env_name) == 0) &&
      (!license_file_path || strlen(license_file_path) == 0) &&
      (!license_base64 || strlen(license_base64) == 0)) {
    AERROR_F("Invalid license input");
    return YALK_INVALID_ARGUMENT;
  } else if ((!public_key_env_name || strlen(public_key_env_name) == 0) &&
             (!public_key_file_path || strlen(public_key_file_path) == 0) &&
             (!public_key_base64 || strlen(public_key_base64) == 0)) {
    AERROR_F("Invalid public key input");
    return YALK_INVALID_ARGUMENT;
  } else if (!is_valid) {
    AERROR_F("Invalid output pointer");
    return YALK_INVALID_ARGUMENT;
  }
  *is_valid = false;

  // 1. read license and public key
  // 1.1. read pubilc key and encode it
  std::string public_key_base64_str;
  // 1.1.1. read public key file path from env
  std::string public_key_file_path_str;
  if (public_key_env_name) {
    const char* public_key_env = std::getenv(public_key_env_name);
    if (!public_key_env) {
      AERROR_F("Failed to read public key file path from env %s",
               public_key_env_name);
      return YALK_IO_ERROR;
    }
    public_key_file_path_str = public_key_env;
    public_key_file_path = public_key_file_path_str.c_str();
  }
  // 1.1.2. read public key from file
  if (public_key_file_path) {
    try {
      public_key_base64_str =
          yalk::Base64Encode(yalk::LoadPublicKey(public_key_file_path));
    } catch (const std::exception& e) {
      AERROR_F("Failed to read public key from %s", public_key_file_path);
      return YALK_IO_ERROR;
    }
  } else {
    // 1.1.3. read public key from string
    public_key_base64_str = public_key_base64;
  }
  // 1.2. read license and encode it
  // 1.2.1. read license file path from env
  std::string license_file_path_str;
  if (license_env_name) {
    // read license from env
    const char* license_env = std::getenv(license_env_name);
    if (!license_env) {
      AERROR_F("Failed to read license file path from env %s",
               license_env_name);
      return YALK_IO_ERROR;
    }
    license_file_path_str = license_env;
    license_file_path = license_file_path_str.c_str();
  }
  // 1.2.2. read license from file
  std::string license_base64_str;
  if (license_file_path) {
    // read license from file
    if (!yalk::ReadDataFromFile(license_file_path, license_base64_str)) {
      AERROR_F("Failed to read license from %s", license_file_path);
      return YALK_IO_ERROR;
    }
    try {
      license_base64_str = yalk::Base64Encode(license_base64_str);
    } catch (const std::exception& e) {
      AERROR_F("Failed to encode license");
      return YALK_IO_ERROR;
    }
  } else {
    // 1.2.3. read license from string
    license_base64 = license_file_path;
  }

  // 2. fallback to yalk_verify_from_string
  return yalk_verify_from_string(config, license_base64_str.c_str(),
                                 public_key_base64_str.c_str(), is_valid);
}

YALK_EVENT_TYPE yalk_verify_from_string(const char* config,
                                        const char* license_base64,
                                        const char* public_key_base64,
                                        bool* is_valid) {
  // 0. check inputs
  if (!license_base64 || strlen(license_base64) == 0) {
    AERROR_F("Invalid license");
    return YALK_INVALID_ARGUMENT;
  } else if (!public_key_base64 || strlen(public_key_base64) == 0) {
    AERROR_F("Invalid public key");
    return YALK_INVALID_ARGUMENT;
  } else if (!is_valid) {
    AERROR_F("Invalid output pointer");
    return YALK_INVALID_ARGUMENT;
  }
  *is_valid = false;

  // 1. decode public key
  std::string public_key_str;
  std::string license_str;
  try {
    public_key_str = yalk::Base64Decode(public_key_base64);
    license_str = yalk::Base64Decode(license_base64);
  } catch (const std::exception& e) {
    AERROR_F("Failed to decode license or public key");
    return YALK_INVALID_ARGUMENT;
  }

  // 2. init verifier
  // 2.1. init config
  json config_json;
  if (!config || strlen(config) == 0) {
    config_json = {{"verifier", {{"validator", "LocalLicenseValidator"}}},
                   {"hwid", kHWIDAlgorithms[1]}};
  } else {
    try {
      config_json = json::parse(config);
    } catch (const std::exception& e) {
      AERROR_F("Failed to parse config: %s", e.what());
      return YALK_FAILED;
    }
  }
  // if (config_json.contains("common")) {
  //   AINFO_F("Requsted license info:\n%s",
  //           config_json["common"].dump(2).c_str());
  // }

  // 2.2. init issuer
  yalk::LicenseVerifier verifier;
  if (!verifier.Init(config_json, public_key_str)) {
    AERROR_F("Failed to init verifier: errno=[%d](%s), msg=[%s]",
             static_cast<int>(yalk::GetLastErrorCode()),
             yalk::ToString(yalk::GetLastErrorCode()).c_str(),
             yalk::GetLastErrorMessage().c_str());
  }

  // 3. verify license
  try {
    *is_valid = verifier.VerifyLicenseFromString(license_str);
  } catch (const std::exception& e) {
    AERROR_F("Failed to verify license: %s", e.what());
    return YALK_FAILED;
  }

  // check error number
  auto error_code = yalk::GetLastErrorCode();
  if (yalk::ErrorCode::kOK != error_code) {
    switch (static_cast<int>(error_code)) {
      case static_cast<int>(yalk::ErrorCode::kFail):
        return YALK_FAILED;
      case static_cast<int>(yalk::ErrorCode::kIOError):
        return YALK_IO_ERROR;
      case static_cast<int>(yalk::ErrorCode::kInvalidArgument):
        return YALK_INVALID_ARGUMENT;
      case static_cast<int>(yalk::ErrorCode::kInvalidKey):
        return YALK_INVALID_KEY;
      case static_cast<int>(yalk::ErrorCode::kLicenseExpired):
        return LICENSE_EXPIRED;
      case static_cast<int>(yalk::ErrorCode::kLicenseNotActivated):
        return LICENSE_NOT_ACTIVATED;
      case static_cast<int>(yalk::ErrorCode::kLicenseProductMismatch):
        return LICENSE_PRODUCT_MISMATCH;
      case static_cast<int>(yalk::ErrorCode::kLicenseUserMismatch):
        return LICENSE_USER_MISMATCH;
      case static_cast<int>(yalk::ErrorCode::kLicenseHardwareMismatch):
        return LICENSE_HARDWARE_MISMATCH;
      case static_cast<int>(yalk::ErrorCode::kLicenseFeatureMismatch):
        return LICENSE_FEATURE_MISMATCH;
      case static_cast<int>(yalk::ErrorCode::kLicenseTypeMismatch):
        return LICENSE_TYPE_MISMATCH;
      case static_cast<int>(yalk::ErrorCode::kLicenseExceedUses):
        return LICENSE_EXCEED_USES;
      case static_cast<int>(yalk::ErrorCode::kLicenseExceedQPS):
        return LICENSE_EXCEED_QPS;
      default:
        return LICENSE_INVALID;
    }
  }

  return YALK_OK;
}
