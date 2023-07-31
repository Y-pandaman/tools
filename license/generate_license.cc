#include <absl/flags/flag.h>
#include <absl/flags/parse.h>

#include "dvis_cpp/encryption/crypto/base64.h"
#include "dvis_cpp/util/json.h"
#include "dvis_cpp/util/log.h"
#include "dvis_cpp/util/yalk_helper.h"

#ifdef ENABLE_YALK
#include "yalk.h"
#endif

ABSL_FLAG(std::string, public_key, "", "Path to public key file.");
ABSL_FLAG(std::string, private_key, "", "Path to private key file.");
ABSL_FLAG(std::string, machine_fingerprint, "",
          "Path to machine fingerprint file.");
ABSL_FLAG(std::string, license, "", "Path to output license file.");
ABSL_FLAG(std::string, mac_address, "", "MAC address of the device.");

int main(int argc, char** argv) {
  // parse args
  absl::ParseCommandLine(argc, argv);
  std::string public_key_file = absl::GetFlag(FLAGS_public_key);
  std::string private_key_file = absl::GetFlag(FLAGS_private_key);
  std::string machine_fingerprint_file =
      absl::GetFlag(FLAGS_machine_fingerprint);
  std::string license_file = absl::GetFlag(FLAGS_license);
  std::string mac_address = absl::GetFlag(FLAGS_mac_address);

#ifdef ENABLE_YALK
  // generate key pair if not provided
  YALK_EVENT_TYPE ret = YALK_OK;
  if (public_key_file.empty() || private_key_file.empty()) {
    AINFO_F("Generating key pair since not provided.");
    ret = yalk_keygen(nullptr, "/tmp");
    if (YALK_OK != ret) {
      AERROR_F("Failed to generate key pair: {}", ret);
      return 1;
    }
    public_key_file = "/tmp/public_key.pem";
    private_key_file = "/tmp/private_key.pem";
  }

  // generate machine data file if not provided
  if (machine_fingerprint_file.empty()) {
    AINFO_F("Generating machine fingerprint since not provided.");
    // pre-define mac address
    nlohmann::json yalk_config_json =
        nlohmann::json::parse(dvis_cpp::util::yalk_config);
    if (!mac_address.empty()) {
      yalk_config_json["hwid_values"]["mac"] = mac_address;
    }
    // generate machine fingerprint
    ret = yalk_collect(yalk_config_json.dump().c_str(), "/tmp/machine.dat",
                       public_key_file.c_str(), nullptr);
    if (YALK_OK != ret) {
      AERROR_F("Failed to generate machine fingerprint: {}", ret);
      return 1;
    }
    machine_fingerprint_file = "/tmp/machine.dat";
  }

  // issue license
  if (license_file.empty()) {
    license_file = "/tmp/license.dat";
  }
  ret = yalk_issue(dvis_cpp::util::yalk_config, license_file.c_str(),
                   machine_fingerprint_file.c_str(), nullptr,
                   private_key_file.c_str(), nullptr);
  if (YALK_OK != ret) {
    AERROR_F("Failed to issue license: {}", ret);
    return 1;
  }

  // valid license
  bool is_valid = false;
  ret = yalk_verify(dvis_cpp::util::yalk_config, nullptr, license_file.c_str(),
                    nullptr, nullptr, public_key_file.c_str(), nullptr,
                    &is_valid);
  if (YALK_OK != ret) {
    AERROR_F("Failed to verify license: {}", ret);
    return 1;
  }
  if (!is_valid) {
    AERROR_F("License is not valid.");
    return 1;
  }
  AINFO_F("License is valid.");
#endif

  return 0;
}
