#pragma once

#include <string>

#include "yalk/base/common.h"
#include "yalk/utils/filesystem.h"
#include "yalk/utils/json.h"

namespace yalk {

/// \brief Merge the given license data into a complete info in JSON format.
/// \param product_info The product info.
/// \param user_info The user info.
/// \param hw_info The Hardware info.
/// \param limitation_info The limitation info.
/// \param extra_info The extra info.
/// \param all_info The complete info in JSON format.
bool MergeLicenseInfo(const ProductInfo& product_info,
                      const UserInfo& user_info, const HardwareInfo& hw_info,
                      const LimitationInfo& limitation_info,
                      const json& extra_info, json& all_info);

/// \brief Parse license info from a complete info in JSON format.
/// \param all_info The complete info in JSON format.
/// \param product_info The product info.
/// \param user_info The user info.
/// \param hw_info The Hardware info.
/// \param limitation_info The limitation info.
/// \param extra_info The extra info.
bool ParseLicenseInfoFromJSON(const json& all_info, ProductInfo& product_info,
                              UserInfo& user_info, HardwareInfo& hw_info,
                              LimitationInfo& limitation_info,
                              json& extra_info);

/// \brief Parse license data from the given license file.
/// \param license_file The license file.
/// \param license_data The license data.
/// \return True if the operation is successful, false otherwise.
bool ParseLicenseInfoFromFile(const std::string& license_file,
                              LicenseData& license_data);

/// \brief Parse license data from the given license string.
/// \param license_data_str The license data in string format.
/// \param license_data The license data.
/// \return True if the operation is successful, false otherwise.
bool ParseLicenseInfoFromString(const std::string& license_data_str,
                                LicenseData& license_data);

/// \brief Dump the license data as a license file with the given path.
/// \param license_data The license data.
/// \param license_file The license file.
/// \return True if the operation is successful, false otherwise.
bool DumpLicenseInfoToFile(const LicenseData& license_data,
                           const std::string& license_file);

/// \brief Dump the license data as a license string.
/// \param license_data The license data.
/// \param license_data_str The license data in string format.
/// \return True if the operation is successful, false otherwise.
bool DumpLicenseInfoToString(const LicenseData& license_data,
                             std::string& license_data_str);

}  // namespace yalk
