#pragma once

#include <string>

namespace yalk {

enum class ErrorCode {
  // common error codes
  /// \brief Generic code indicating that no error has occurred.
  kOK = 0,
  /// \brief Generic code indicating that an error has occurred.
  kFail = 1,
  /// \brief I/O error.
  kIOError = 2,
  /// \brief Invalid argument.
  kInvalidArgument = 3,
  /// \brief Invalid key.
  kInvalidKey = 4,

  // license-related error codes
  /// \brief Generic code indicating that the license is invalid.
  kLicenseInvalid = 0x100,
  /// \brief License is expired.
  kLicenseExpired = 0x101,
  /// \brief License is not activated.
  kLicenseNotActivated = 0x102,
  /// \brief License is not issued to the current product.
  kLicenseProductMismatch = 0x103,
  /// \brief License is not issued to the current user.
  kLicenseUserMismatch = 0x104,
  /// \brief License is not issued to the current hardware.
  kLicenseHardwareMismatch = 0x105,
  /// \brief License is not issued to the current feature.
  kLicenseFeatureMismatch = 0x106,
  /// \brief License is not issued to the requested license type.
  kLicenseTypeMismatch = 0x107,
  /// \brief License is not issued to the requested use times.
  kLicenseExceedUses = 0x108,
  /// \brief License is not issued to the requested use QPS.
  kLicenseExceedQPS = 0x109,
};

static ErrorCode last_error_code = ErrorCode::kOK;
static std::string last_error_message = "";

/// \brief Set the last error code.
/// \param error_code The error code.
/// \param error_message The error message.
void SetLastErrorCode(ErrorCode error_code,
                      const std::string &error_message = "");

/// \brief Get the last error code.
/// \return The last error code.
ErrorCode GetLastErrorCode();

/// \brief Get the last error message.
/// \return The last error message.
std::string GetLastErrorMessage();

/// \brief Check if the last error code is OK.
/// \return True if the last error code is OK.
bool IsOK();

/// \brief Convert the error code to string.
/// \param error_code The error code.
/// \return The string representation of the error code.
std::string ToString(ErrorCode error_code);

}  // namespace yalk
