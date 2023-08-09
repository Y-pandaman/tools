#include "yalk/core/hwid_collecter.h"

#include "yalk/base/common.h"
#include "yalk/base/errno.h"
#include "yalk/hwid/base.h"
#include "yalk/utils/crypto.h"
#include "yalk/utils/parser.h"

namespace yalk {

bool HardwareInfoCollector::Init(const json &config,
                                 const std::string &public_key) {
  // 0. check input
  json hwid_config;
  if (!config.contains("hwid") || !config["hwid"].is_object()) {
    hwid_config = kHWIDAlgorithms[version_];
    ADEBUG_F("Use default config for hardware collecor with version %d: %s",
             version_, hwid_config.dump().c_str());
  } else {
    hwid_config = config["hwid"];
    ADEBUG_F("Use given config for hardware collecor with version %d: %s",
             version_, hwid_config.dump().c_str());
  }

  // 1. initialize hardware identifiers
  hwid_map_.clear();
  for (auto &hwid : hwid_config.items()) {
    auto instance =
        HardwareIdentifierRegisterer::GetInstanceByName(hwid.value());
    if (instance) {
      hwid_map_[hwid.key()].reset(instance);
    } else {
      AWARN_F("Failed to initialize hardware identifier %s: %s",
              hwid.key().c_str(), hwid.value().get<std::string>().c_str());
    }
  }
  // check if any hardware identifier is initialized
  if (hwid_map_.empty()) {
    SetLastErrorCode(ErrorCode::kInvalidArgument,
                     "Failed to initialize any hardware identifier");
    return false;
  }

  // 2. initialize public key
  public_key_ = public_key;

  SetLastErrorCode(ErrorCode::kOK);
  return true;
}

HardwareInfo HardwareInfoCollector::CollectHWInfo() const {
  // 0. initialize hardware information
  HardwareInfo hw_info;
  hw_info.version = version_;

  // 1. collect hardware information item by item
  for (const auto &hwid : hwid_map_) {
    auto identifier = hwid.second->GetIdentifier();
    if (identifier.empty()) {
      SetLastErrorCode(ErrorCode::kFail,
                       "Failed to collect hardware identifier " + hwid.first);
      // continue;
    }
    hw_info.info[hwid.first] = hwid.second->GetIdentifier();
  }

  // 2. check if any hardware identifier is collected
  bool is_all_fields_empty = true;
  for (const auto &hwid : hw_info.info.items()) {
    if (!hwid.value().empty()) {
      is_all_fields_empty = false;
      break;
    }
  }
  if (hw_info.info.empty() || is_all_fields_empty) {
    SetLastErrorCode(ErrorCode::kFail,
                     "Failed to collect any hardware identifier");
  } else {
    SetLastErrorCode(ErrorCode::kOK);
  }

  return hw_info;
}

std::string HardwareInfoCollector::GenerateMachineFingerprint() const {
  // 1. collect hardware information
  HardwareInfo hw_info = CollectHWInfo();
  if (!IsOK()) {
    return "";
  }

  // 2. generate machine fingerprint
  json all_info;
  all_info["hw_version"] = hw_info.version;
  all_info["hw_info"] = hw_info.info;

  // 3. encrypt the machine fingerprint
  std::string encrypted = EncryptData(all_info.dump(), public_key_, true);
  if (encrypted.empty()) {
    SetLastErrorCode(ErrorCode::kFail, "Failed to encrypt machine fingerprint");
    return "";
  }

  SetLastErrorCode(ErrorCode::kOK);
  return encrypted;
}

bool HardwareInfoCollector::SaveMachineFingerprint(
    const std::string &file_path) const {
  // 1. generate machine fingerprint
  auto machine_fingerprint = GenerateMachineFingerprint();
  if (machine_fingerprint.empty()) {
    return false;
  }

  // 2. save the machine fingerprint to a file
  if (!WriteDataToFile(file_path, machine_fingerprint)) {
    SetLastErrorCode(ErrorCode::kFail,
                     "Failed to save machine fingerprint to file " + file_path);
    return false;
  }

  SetLastErrorCode(ErrorCode::kOK);
  return true;
}

}  // namespace yalk
