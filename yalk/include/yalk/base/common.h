#pragma once

#include <string>

#include "yalk/utils/json.h"

using json = nlohmann::json;

enum class LicenseType {
  /// \brief The license is for evaluation.
  kEvaluation = 0,
  /// \brief The license is for individual use.
  kIndividual = 1,
  /// \brief The license is for commercial use.
  kCommercial = 2,
};

static std::string ToString(LicenseType type) {
  switch (type) {
    case LicenseType::kEvaluation:
      return "evaluation";
    case LicenseType::kIndividual:
      return "individual";
    case LicenseType::kCommercial:
      return "commercial";
    default:
      return "unknown";
  }
}

static LicenseType FromString(const std::string& type) {
  if (type == "evaluation") {
    return LicenseType::kEvaluation;
  } else if (type == "individual") {
    return LicenseType::kIndividual;
  } else if (type == "commercial") {
    return LicenseType::kCommercial;
  } else {
    return LicenseType::kEvaluation;
  }
}

/// \brief The product info.
struct ProductInfo {
  /// \brief The product name.
  std::string name;
  /// \brief The product version.
  std::string version;
};

/// \brief The user info.
struct UserInfo {
  /// \brief The user name.
  std::string name;
  /// \brief The user email.
  std::string email;
};

/// \brief The hardware info.
struct HardwareInfo {
  /// \brief The version of the hardware identification algorithm.
  int64_t version;
  /// \brief The collection of hardware identifiers.
  json info;
};

/// \brief The license limitation info.
struct LimitationInfo {
  /// \brief The license expiration time (in unix timestamp seconds).
  int64_t expiration_time;
  /// \brief The maximum number of uses.
  int64_t max_uses;
  /// \brief The maximum number of queries per second.
  int64_t max_qps;
  /// \brief The allowed product features.
  json features;
  /// \brief The license type
  LicenseType type;
  /// \brief The license issue time (in unix timestamp seconds).
  int64_t issue_time;
  /// \brief The license issue number.
  int64_t issue_number;
};

/// \brief License descriptor for data I/O.
struct LicenseData {
  /// \brief license info
  json info;
  /// \brief license signature
  std::string signature;
  /// \brief key-encrypted license info
  std::string encrypted;
};

/// \brief The LUT for the hwid version and the corresponding hwid algorithms.
static std::map<int64_t, json> kHWIDAlgorithms = {
    {1,
     {{"mac", "MacAddressIdentifier"},
      {"disk_uuid", "DiskUUIDIdentifier"},
      {"disk_stat", "DiskStatIdentifier"},
      {"cpu_info", "CPUInfoIdentifier"},
      {"virtualization", "VirtualizationIdentifier"}}},
};
