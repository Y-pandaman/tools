#pragma once

#include <string>

#include "yalk/base/common.h"

namespace yalk {

/// \brief Generate a mock RSA key pair.
/// \return The RSA key pair with predefined values. The first element is the
/// public key and the second element is the private key.
std::pair<std::string, std::string> MockRSAKeyPair();

/// \brief Generate a mock product info.
/// \return The product info with predefined values.
ProductInfo MockProductInfo();

/// \brief Generate a mock user info.
/// \return The user info with predefined values.
UserInfo MockUserInfo();

/// \brief Generate a mock hardware info.
/// \return The hardware info with predefined values.
HardwareInfo MockHardwareInfo();

/// \brief Generate a mock limitation info.
/// \return The limitation info with predefined values.
LimitationInfo MockLimitationInfo();

/// \brief Generate a mock complete info in JSON format.
/// \return The complete info in JSON format with predefined values.
json MockAllInfo();

/// \brief Generate a mock license data
/// \return The license data with predefined values.
LicenseData MockLicenseData();

/// \brief Generate a random product info.
/// \return The product info with random values.
ProductInfo RandomProductInfo();

/// \brief Generate a random user info.
/// \return The user info with random values.
UserInfo RandomUserInfo();

/// \brief Generate a random hardware info.
/// \return The hardware info with random values.
HardwareInfo RandomHardwareInfo();

/// \brief Generate a random limitation info.
/// \return The limitation info with random values.
LimitationInfo RandomLimitationInfo();

/// \brief Generate a random complete info in JSON format.
/// \return The complete info in JSON format with random values.
json RandomAllInfo();

}  // namespace yalk
