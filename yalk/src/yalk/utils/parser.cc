#include "yalk/utils/parser.h"

#include <fstream>
#include <iostream>
#include <regex>

#include "yalk/base/common.h"
#include "yalk/utils/base64.h"
#include "yalk/utils/log.h"

namespace yalk {

bool MergeLicenseInfo(const ProductInfo& product_info,
                      const UserInfo& user_info, const HardwareInfo& hw_info,
                      const LimitationInfo& limitation_info,
                      const json& extra_info, json& all_info) {
  all_info["product_name"] = product_info.name;
  all_info["product_version"] = product_info.version;
  all_info["user_name"] = user_info.name;
  all_info["user_email"] = user_info.email;
  all_info["hw_version"] = hw_info.version;
  all_info["hw_info"] = hw_info.info;
  all_info["expiration_time"] = limitation_info.expiration_time;
  all_info["max_uses"] = limitation_info.max_uses;
  all_info["max_qps"] = limitation_info.max_qps;
  all_info["features"] = limitation_info.features;
  all_info["type"] = static_cast<int>(limitation_info.type);
  all_info["issue_time"] = limitation_info.issue_time;
  all_info["issue_number"] = limitation_info.issue_number;
  all_info["extra_info"] = extra_info;

  return true;
}

bool ParseLicenseInfoFromJSON(const json& all_info, ProductInfo& product_info,
                              UserInfo& user_info, HardwareInfo& hw_info,
                              LimitationInfo& limitation_info,
                              json& extra_info) {
  // parse product info
  if (all_info.contains("product_name") &&
      all_info.contains("product_version")) {
    product_info.name = all_info["product_name"];
    product_info.version = all_info["product_version"];
  } else {
    AWARN_F("Failed to parse product name from the json.");
    return false;
  }

  // parse user info
  if (all_info.contains("user_name")) {
    user_info.name = all_info["user_name"];
  }
  if (all_info.contains("user_email")) {
    user_info.email = all_info["user_email"];
  }

  // parse hardware info
  if (all_info.contains("hw_version") && all_info.contains("hw_info")) {
    hw_info.version = all_info["hw_version"];
    hw_info.info = all_info["hw_info"];
  }

  // parse limitation info
  if (all_info.contains("expiration_time")) {
    limitation_info.expiration_time = all_info["expiration_time"];
  }
  if (all_info.contains("max_uses")) {
    limitation_info.max_uses = all_info["max_uses"];
  }
  if (all_info.contains("max_qps")) {
    limitation_info.max_qps = all_info["max_qps"];
  }
  if (all_info.contains("features")) {
    limitation_info.features = all_info["features"];
  }
  if (all_info.contains("type")) {
    limitation_info.type =
        static_cast<LicenseType>(all_info["type"].get<int>());
  }
  if (all_info.contains("issue_time")) {
    limitation_info.issue_time = all_info["issue_time"];
  }
  if (all_info.contains("issue_number")) {
    limitation_info.issue_number = all_info["issue_number"];
  }

  // parse extra info
  if (all_info.contains("extra_info")) {
    extra_info = all_info["extra_info"];
  } else {
    extra_info = {};
  }

  return true;
}

bool ParseLicenseInfoFromFile(const std::string& license_file,
                              LicenseData& license_data) {
  // 1. read the license file as a string
  std::string license_data_str;
  if (!ReadDataFromFile(license_file, license_data_str)) {
    AERROR_F("Failed to read the license file: %s", license_file.c_str());
    return false;
  }

  // 2. parse the license data from string
  if (!ParseLicenseInfoFromString(license_data_str, license_data)) {
    AERROR_F("Failed to parse the license data from string.");
    return false;
  }

  return true;
}

bool ParseLicenseInfoFromString(const std::string& license_data_str,
                                LicenseData& license_data) {
  // 0. clear the license data
  license_data = {};

  // 1. parse the license info
  std::regex info_regex(
      R"((\w+)\s+([\d\.\w-]+)\s+(\w+)\s+license activation key, number\s+(-?\d+),\s+type\s+(-?\d+)\.)");

  auto iter = std::sregex_iterator(license_data_str.begin(),
                                   license_data_str.end(), info_regex);
  if (iter == std::sregex_iterator()) {
    AERROR_F("Cannot find license info.");
    return false;
  }
  license_data.info["product_name"] = iter->str(1);
  license_data.info["product_version"] = iter->str(2);
  license_data.info["type"] = FromString(iter->str(3));
  try {
    license_data.info["issue_number"] = std::stoi(iter->str(4));
    license_data.info["type"] =
        static_cast<LicenseType>(std::stoi(iter->str(5)));
  } catch (const std::exception& e) {
    AERROR_F("Failed to parse the license info: %s", e.what());
    return false;
  }

  // 2. parse the user info
  // check only user name
  std::regex user_name_regex(R"(Issued to\s(.+)\.\n\n)");
  iter = std::sregex_iterator(license_data_str.begin(), license_data_str.end(),
                              user_name_regex);
  if (iter != std::sregex_iterator()) {
    license_data.info["user_name"] = iter->str(1);
    license_data.info["user_email"] = "";
  }
  // check user name and email
  std::regex user_regex(R"(Issued to\s(.+)\s\((.+)\)\.\n\n)");
  iter = std::sregex_iterator(license_data_str.begin(), license_data_str.end(),
                              user_regex);
  if (iter != std::sregex_iterator()) {
    license_data.info["user_name"] = iter->str(1);
    if (iter->str(2) != "") {
      license_data.info["user_email"] = iter->str(2);
    }
  }
  // check if the user info is valid
  if (license_data.info["user_name"] == "") {
    AERROR_F("Cannot find user info.");
    return false;
  }

  // 3. parse the limitation info
  // 3.1. parse normal limitation info
  std::regex license_info_regex(
      R"((Expiration:\s+([\d/]+)\.|Max uses:\s+(\d+)\.|Max qps:\s+(\d+)\.|Features:\s+(.+)\.)\n?)");
  iter = std::sregex_iterator(license_data_str.begin(), license_data_str.end(),
                              license_info_regex);
  while (iter != std::sregex_iterator()) {
    std::string info = iter->str(0);
    if (info.find("Expiration:") != std::string::npos) {
      try {
        license_data.info["expiration_time"] = stoi(iter->str(2));
      } catch (const std::exception& e) {
        AERROR_F("Failed to parse the expiration time: %s", e.what());
        return false;
      }
    } else if (info.find("Max uses:") != std::string::npos) {
      try {
        license_data.info["max_uses"] = stoi(iter->str(3));
      } catch (const std::exception& e) {
        AERROR_F("Failed to parse the max uses: %s", e.what());
        return false;
      }
    } else if (info.find("Max qps:") != std::string::npos) {
      try {
        license_data.info["max_qps"] = stoi(iter->str(4));
      } catch (const std::exception& e) {
        AERROR_F("Failed to parse the max qps: %s", e.what());
        return false;
      }
    } else if (info.find("Features:") != std::string::npos) {
      std::string features = iter->str(5);
      std::regex feature_regex(R"(([^,\s]+))");
      auto feature_iter =
          std::sregex_iterator(features.begin(), features.end(), feature_regex);
      std::vector<std::string> feature_list;
      while (feature_iter != std::sregex_iterator()) {
        feature_list.push_back(feature_iter->str(1));
        ++feature_iter;
      }
      license_data.info["features"] = feature_list;
    }
    ++iter;
  }
  // 3.2. parse special limitation info
  if (!license_data.info.contains("expiration_time") &&
      license_data_str.find("Expiration: never.\n") != std::string::npos) {
    license_data.info["expiration_time"] = -1;
  }
  if (!license_data.info.contains("max_uses") &&
      license_data_str.find("Max uses: unlimited.\n") != std::string::npos) {
    license_data.info["max_uses"] = -1;
  }
  if (!license_data.info.contains("max_qps") &&
      license_data_str.find("Max qps: unlimited.\n") != std::string::npos) {
    license_data.info["max_qps"] = -1;
  }
  if (!license_data.info.contains("features") &&
      license_data_str.find("Features: all.\n") != std::string::npos) {
    license_data.info["features"] = std::vector<std::string>{};
  }

  // 4. parse the license content
  // 4.1. find the start of license content
  std::regex license_start_regex(
      R"(Do not modify this file. Its entire content, including the plain text section, is used by the activation manager.\n\n)");
  iter = std::sregex_iterator(license_data_str.begin(), license_data_str.end(),
                              license_start_regex);
  if (iter == std::sregex_iterator()) {
    AERROR_F("Cannot find the start of license content");
    return false;
  }
  ++iter;

  // 4.2. parse the signature
  std::regex signature_regex(R"(([0-9A-Fa-f]{64})\n)");
  iter = std::sregex_iterator(license_data_str.begin(), license_data_str.end(),
                              signature_regex);
  if (iter == std::sregex_iterator()) {
    AERROR_F("Cannot find the signature of license content");
    return false;
  }
  std::stringstream signature_ss;
  auto signature_end_pos = 0;
  while (iter != std::sregex_iterator()) {
    signature_ss << iter->str(1);
    signature_end_pos = iter->position() + iter->length();
    ++iter;
  }
  license_data.signature = signature_ss.str();

  // 4.3. parse the encrypted content (from the end of the signature to the end)
  std::regex license_content_regex(R"(([0-9A-Za-z+/=]{1,76})(?:\n|))");
  iter = std::sregex_iterator(license_data_str.begin() + signature_end_pos,
                              license_data_str.end(), license_content_regex);
  if (iter == std::sregex_iterator()) {
    AERROR_F("Cannot find the encrypted of license content");
    return false;
  }
  std::stringstream encoded_ss;
  while (iter != std::sregex_iterator()) {
    encoded_ss << iter->str(1);
    ++iter;
  }
  license_data.encrypted = Base64Decode(encoded_ss.str());
  // AINFO_F("Encoded license content: %s", encoded_ss.str().c_str());

  return true;
}

bool DumpLicenseInfoToFile(const LicenseData& license_data,
                           const std::string& license_file) {
  // 1. dump the license data to string
  std::string license_data_str;
  if (!DumpLicenseInfoToString(license_data, license_data_str)) {
    AERROR_F("Cannot dump license data to string.");
    return false;
  }

  // 2. dump the license data to file
  if (!WriteDataToFile(license_file, license_data_str)) {
    AERROR_F("Cannot dump license data to file.");
    return false;
  }

  return true;
}

bool DumpLicenseInfoToString(const LicenseData& license_data,
                             std::string& license_data_str) {
  // 2. pretty print the license in the following format:
  std::stringstream ss;

  // 2.1. human readable info
  if (!license_data.info.contains("product_name") ||
      !license_data.info.contains("product_version") ||
      !license_data.info.contains("type") ||
      !license_data.info.contains("issue_number") ||
      license_data.info["product_name"] == "" ||
      license_data.info["product_version"] == "") {
    throw std::runtime_error("Cannot find product info.");
  }
  ss << license_data.info["product_name"].get<std::string>() << " "
     << license_data.info["product_version"].get<std::string>() << " "
     << ToString(LicenseType(license_data.info["type"]))
     << " license activation key, number " << license_data.info["issue_number"]
     << ", type " << static_cast<int>(license_data.info["type"]) << ".\n\n";

  // 2.2. human readable info
  // 2.2.1. user info
  if (license_data.info.contains("user_email") &&
      license_data.info.contains("user_name") &&
      license_data.info["user_email"] != "" &&
      license_data.info["user_name"] != "") {
    ss << "Issued to " << license_data.info["user_name"].get<std::string>()
       << " (" << license_data.info["user_email"].get<std::string>()
       << ").\n\n";
  } else if (license_data.info.contains("user_name") &&
             license_data.info["user_name"] != "") {
    ss << "Issued to " << license_data.info["user_name"].get<std::string>()
       << ".\n\n";
  } else {
    ss << "Issued to anonymous user.\n\n";
  }

  // 2.2.2. license info
  // expiration time
  if (license_data.info.contains("expiration_time") &&
      license_data.info["expiration_time"] > 0) {
    ss << "Expiration: " << license_data.info["expiration_time"] << ".\n";
  } else {
    ss << "Expiration: never.\n";
  }
  // max uses
  if (license_data.info.contains("max_uses") &&
      license_data.info["max_uses"] > 0) {
    ss << "Max uses: " << license_data.info["max_uses"] << ".\n";
  } else {
    ss << "Max uses: unlimited.\n";
  }
  // max qps
  if (license_data.info.contains("max_qps") &&
      license_data.info["max_qps"] > 0) {
    ss << "Max qps: " << license_data.info["max_qps"] << ".\n";
  } else {
    ss << "Max qps: unlimited.\n";
  }
  // features
  if (license_data.info.contains("features") &&
      !license_data.info["features"].empty()) {
    // ss << "Features: " << license_data.info["features"] << ".\n";
    // print features in a more readable way
    ss << "Features: ";
    for (auto& feature : license_data.info["features"]) {
      ss << feature.get<std::string>();
      if (feature != license_data.info["features"].back()) {
        ss << ", ";
      }
    }
    ss << ".\n";
  }
  ss << "\n";

  // 2.3. license string
  // 2.3.1. license header (64 characters per line)
  ss << "Do not modify this file. Its entire content, including the plain text "
        "section, is used by the activation manager.\n\n";

  // 2.3.2. license signature
  // print the signature in the following format:
  // <signature> (64 characters per line)
  for (size_t i = 0; i < license_data.signature.size(); i += 64) {
    ss << license_data.signature.substr(i, 64) << "\n";
  }
  ss << "\n";

  // 2.3.3. license content
  // print the license in the following format:
  // <license> (base64 encoded, 64 characters per line)
  std::string encoded = Base64Encode(license_data.encrypted);
  for (size_t i = 0; i < encoded.size(); i += 64) {
    ss << encoded.substr(i, 64) << "\n";
  }

  // 3. return the string
  license_data_str = ss.str();

  return true;
}

}  // namespace yalk
