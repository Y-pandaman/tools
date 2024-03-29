#include "yalk/base/errno.h"

namespace yalk {

void SetLastErrorCode(ErrorCode error_code, const std::string &error_message) {
  last_error_code = error_code;
  last_error_message = error_message;
}

ErrorCode GetLastErrorCode() { return last_error_code; }

std::string GetLastErrorMessage() { return last_error_message; }

bool IsOK() { return last_error_code == ErrorCode::kOK; }

std::string ToString(ErrorCode error_code) {
  switch (error_code) {
    case ErrorCode::kOK:
      return "OK";
    case ErrorCode::kFail:
      return "Fail";
    case ErrorCode::kIOError:
      return "IOError";
    case ErrorCode::kInvalidArgument:
      return "InvalidArgument";
    case ErrorCode::kInvalidKey:
      return "InvalidKey";
    case ErrorCode::kLicenseInvalid:
      return "LicenseInvalid";
    case ErrorCode::kLicenseExpired:
      return "LicenseExpired";
    case ErrorCode::kLicenseNotActivated:
      return "LicenseNotActivated";
    case ErrorCode::kLicenseProductMismatch:
      return "LicenseProductMismatch";
    case ErrorCode::kLicenseUserMismatch:
      return "LicenseUserMismatch";
    case ErrorCode::kLicenseHardwareMismatch:
      return "LicenseHardwareMismatch";
    case ErrorCode::kLicenseFeatureMismatch:
      return "LicenseFeatureMismatch";
    case ErrorCode::kLicenseTypeMismatch:
      return "LicenseTypeMismatch";
    case ErrorCode::kLicenseExceedUses:
      return "LicenseExceedUses";
    case ErrorCode::kLicenseExceedQPS:
      return "LicenseExceedQPS";
    default:
      return "Unknown";
  }
}

}  // namespace yalk
