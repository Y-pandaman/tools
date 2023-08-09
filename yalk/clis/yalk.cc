#include "yalk/interface/yalk.h"

#include "yalk/base/common.h"
#include "yalk/utils/args.h"
#include "yalk/utils/crypto.h"
#include "yalk/utils/log.h"

void KeygenCommand(args::Subparser& parser) {
  // 0. parse args
  args::Positional<std::string> output(parser, "OUTPUT_DIR", "output dir");
  parser.Parse();

  // 1. generate key pair
  if (YALK_OK != yalk_keygen(nullptr, args::get(output).c_str())) {
    AERROR_F("Failed to generate key pair");
  }
}

void CollectCommand(args::Subparser& parser) {
  // 0. parse args
  args::ValueFlag<std::string> output(parser, "OUTPUT", "output path", {'o'});
  args::Positional<std::string> key(parser, "KEY", "public key path");
  parser.Parse();
  if (!key) {
    AERROR_F("Public key is required");
    return;
  }

  // 1. collect hardware info
  if (YALK_OK != yalk_collect(nullptr, args::get(output).c_str(),
                              args::get(key).c_str(), nullptr)) {
    AERROR_F("Failed to collect hardware info");
  }
}

void IssueCommand(args::Subparser& parser) {
  // 0. parse args
  std::unordered_map<std::string, LicenseType> map{
      {"default", LicenseType::kEvaluation},
      {"evaluation", LicenseType::kEvaluation},
      {"individual", LicenseType::kIndividual},
      {"commercial", LicenseType::kCommercial}};

  // product info
  args::ValueFlag<std::string> product_name(
      parser, "PRODUCT_NAME", "product name", {"product-name", "pn"}, "YALK");
  args::ValueFlag<std::string> product_version(
      parser, "PRODUCT_VERSION", "product version", {"product-version", "pv"},
      "1.0.0");
  args::ValueFlag<int> issue_number(parser, "ISSUE_NUMBER", "issue number",
                                    {"issue-number", "in"}, 0);
  args::MapFlag<std::string, LicenseType> license_type(
      parser, "LICENSE_TYPE", "license type", {"license-type", "lt"}, map,
      LicenseType::kEvaluation);

  // user info
  args::ValueFlag<std::string> user_name(parser, "USER_NAME", "user name",
                                         {"user-name", "un"});
  args::ValueFlag<std::string> user_email(parser, "USER_EMAIL", "user email",
                                          {"user-email", "ue"});

  // limitation info
  args::ValueFlag<int> expiration_time(parser, "EXPIRATION_TIME",
                                       "expiration time (seconds)",
                                       {"expiration-time", "et"});
  args::ValueFlag<int> max_uses(parser, "MAX_USES", "max use times",
                                {"max-uses", "mu"});
  args::ValueFlag<int> max_qps(parser, "MAX_QPS", "max query per second",
                               {"max-qps", "mq"});
  args::ValueFlag<std::string> features(parser, "FEATURES", "features",
                                        {"features", "f"});

  // output license
  args::ValueFlag<std::string> output(parser, "OUTPUT", "output path", {'o'});

  // private key
  args::Positional<std::string> private_key(parser, "PRIVATE_KEY",
                                            "path to private key file");

  // machine fingerprint
  args::Positional<std::string> machine_fingerprint(
      parser, "MACHINE_FINGERPRINT", "path to machine fingerprint file");

  parser.Parse();

  // 1. construct config
  json config{{"common", json::object()}};
  json& common_info = config["common"];
  common_info["product_name"] = args::get(product_name);
  common_info["product_version"] = args::get(product_version);
  common_info["issue_number"] = args::get(issue_number);
  common_info["type"] = static_cast<int>(args::get(license_type));
  if (user_name) {
    common_info["user_name"] = args::get(user_name);
  }
  if (user_email) {
    common_info["user_email"] = args::get(user_email);
  }
  if (expiration_time) {
    common_info["expiration_time"] = args::get(expiration_time);
  }
  if (max_uses) {
    common_info["max_uses"] = args::get(max_uses);
  }
  if (max_qps) {
    common_info["max_qps"] = args::get(max_qps);
  }
  if (features) {
    // divied features by comma
    common_info["features"] = json::array();
    std::stringstream ss(args::get(features));
    std::string feature;
    while (std::getline(ss, feature, ',')) {
      common_info["features"].push_back(feature);
    }
  }

  // 2. issue license
  if (YALK_OK != yalk_issue(config.dump().c_str(), args::get(output).c_str(),
                            args::get(machine_fingerprint).c_str(), nullptr,
                            args::get(private_key).c_str(), nullptr)) {
    AERROR_F("Failed to issue license");
  }
}

void VerifyCommand(args::Subparser& parser) {
  // 0. parse args
  std::unordered_map<std::string, LicenseType> map{
      {"default", LicenseType::kEvaluation},
      {"evaluation", LicenseType::kEvaluation},
      {"individual", LicenseType::kIndividual},
      {"commercial", LicenseType::kCommercial}};

  // product info
  args::ValueFlag<std::string> product_name(
      parser, "PRODUCT_NAME", "product name", {"product-name", "pn"}, "YALK");
  args::ValueFlag<std::string> product_version(
      parser, "PRODUCT_VERSION", "product version", {"product-version", "pv"},
      "1.0.0");
  args::ValueFlag<int> issue_number(parser, "ISSUE_NUMBER", "issue number",
                                    {"issue-number", "in"}, 0);
  args::MapFlag<std::string, LicenseType> license_type(
      parser, "LICENSE_TYPE", "license type", {"license-type", "lt"}, map,
      LicenseType::kEvaluation);

  // user info
  args::ValueFlag<std::string> user_name(parser, "USER_NAME", "user name",
                                         {"user-name", "un"});
  args::ValueFlag<std::string> user_email(parser, "USER_EMAIL", "user email",
                                          {"user-email", "ue"});

  // limitation info
  args::ValueFlag<int> expiration_time(parser, "EXPIRATION_TIME",
                                       "expiration time (seconds)",
                                       {"expiration-time", "et"});
  args::ValueFlag<int> max_uses(parser, "MAX_USES", "max use times",
                                {"max-uses", "mu"});
  args::ValueFlag<int> max_qps(parser, "MAX_QPS", "max query per second",
                               {"max-qps", "mq"});
  args::ValueFlag<std::string> features(parser, "FEATURES", "features",
                                        {"features", "f"});

  // output license
  args::ValueFlag<std::string> output(parser, "OUTPUT", "output path", {'o'});

  // private key
  args::Positional<std::string> public_key(parser, "PUBLIC_KEY",
                                           "path to public key file");

  // license file
  args::Positional<std::string> license_file(parser, "LICENSE_FILE",
                                             "path to license file");

  parser.Parse();

  // 1. read pubilc key
  if (!public_key) {
    AERROR_F("Public key is required");
    return;
  }
  std::string public_key_str = yalk::LoadPublicKey(args::get(public_key));

  // 2. construct config
  json config{{"common", json::object()}};
  json& common_info = config["common"];
  common_info["product_name"] = args::get(product_name);
  common_info["product_version"] = args::get(product_version);
  common_info["issue_number"] = args::get(issue_number);
  common_info["type"] = static_cast<int>(args::get(license_type));
  if (user_name) {
    common_info["user_name"] = args::get(user_name);
  }
  if (user_email) {
    common_info["user_email"] = args::get(user_email);
  }
  if (expiration_time) {
    common_info["expiration_time"] = args::get(expiration_time);
  }
  if (max_uses) {
    common_info["max_uses"] = args::get(max_uses);
  }
  if (max_qps) {
    common_info["max_qps"] = args::get(max_qps);
  }
  if (features) {
    // divied features by comma
    common_info["features"] = json::array();
    std::stringstream ss(args::get(features));
    std::string feature;
    while (std::getline(ss, feature, ',')) {
      common_info["features"].push_back(feature);
    }
  }

  // 3. verify license
  if (!license_file) {
    AERROR_F("License file is required");
  }
  bool is_valid = false;
  if (YALK_OK != yalk_verify(config.dump().c_str(), nullptr,
                             args::get(license_file).c_str(), nullptr, nullptr,
                             args::get(public_key).c_str(), nullptr,
                             &is_valid)) {
    AERROR_F("Failed to verify license");
  }

  // 4. print result
  if (is_valid) {
    AINFO_F("License is valid");
  } else {
    AERROR_F("License is invalid");
  }
}

int main(int argc, char* argv[]) {
  // loguru::init(argc, argv);

  args::ArgumentParser parser("Command line interface for YALK.");
  args::Group commands(parser, "commands");
  args::Command keygen(commands, "keygen", "Generate public/private key pair.",
                       &KeygenCommand);
  args::Command commit(commands, "collect", "Collect machine fingerprint.",
                       &CollectCommand);
  args::Command issue(commands, "issue", "Issue a license.", &IssueCommand);
  args::Command verify(commands, "verify", "Verify a license.", &VerifyCommand);

  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help) {
    std::cout << parser;
  } catch (args::Error& e) {
    std::cerr << e.what() << std::endl << parser;
    return 1;
  }

  return 0;
}
