#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The event type of the license operation.
 * @note These values are should be consistent with the internal enum class
 * ErrorCode.
 */
typedef enum {
  // The license is valid or the operation is successful.
  YALK_OK = 0,
  // The license is invalid or the operation is failed.
  YALK_FAILED = 1,
  // The input or output file is not found or invalid.
  YALK_IO_ERROR = 2,
  // The input argument is invalid.
  YALK_INVALID_ARGUMENT = 3,
  // The input key is invalid.
  YALK_INVALID_KEY = 4,
  // The license is not found or invalid.
  LICENSE_INVALID = 0x100,
  // The license is expired.
  LICENSE_EXPIRED = 0x101,
  // The license is not activated.
  LICENSE_NOT_ACTIVATED = 0x102,
  // The license is not issued to the current product.
  LICENSE_PRODUCT_MISMATCH = 0x103,
  // The license is not issued to the current user.
  LICENSE_USER_MISMATCH = 0x104,
  // The license is not issued to the current hardware.
  LICENSE_HARDWARE_MISMATCH = 0x105,
  // The license is not issued to the current feature.
  LICENSE_FEATURE_MISMATCH = 0x106,
  // The license is not issued to the requested license type.
  LICENSE_TYPE_MISMATCH = 0x107,
  // The license is not issued to the requested use times.
  LICENSE_EXCEED_USES = 0x108,
  // The license is not issued to the requested use QPS.
  LICENSE_EXCEED_QPS = 0x109,
} YALK_EVENT_TYPE;

/**
 * @brief Generate the key pair.
 *
 * @param config The configuration from command line or config file in JSON
 * string.
 * @param output_dir_path The path to the output directory.
 * @return True if the key pair is generated successfully, false otherwise.
 */
YALK_EVENT_TYPE yalk_keygen(const char* config, const char* output_dir_path);

/**
 * @brief Collect the machine fingerprint.
 * @param config The configuration from command line or config file in JSON
 * string.
 * @param output_file_path The path to the output file (machine fingerprint).
 * @param public_key_file_path The path to the public key file (PEM format).
 * @param public_key_base64 The public key string (Base64 format).
 * @n one of the two parameters public_key_file_path and public_key_base64
 * should be set.
 * @return True if the machine fingerprint is collected successfully, false
 * otherwise.
 */
YALK_EVENT_TYPE yalk_collect(const char* config, const char* output_file_path,
                             const char* public_key_file_path,
                             const char* public_key_base64);

/**
 * @brief Collect the machine fingerprint from string.
 *
 * @param config The configuration from command line or config file in JSON
 * @param output_file_path The path to the output file (machine fingerprint).
 * @param public_key_base64 The public key string (Base64 format)
 * @return true if the machine fingerprint is collected successfully, false
 * otherwise.
 */
YALK_EVENT_TYPE yalk_collect_from_string(const char* config,
                                         const char* output_file_path,
                                         const char* public_key_base64);

/**
 * @brief Issue a license.
 *
 * @param config The configuration from command line or config file in JSON
 * string.
 * @param output_file_path The path to the output file (license).
 * @param machine_fingerprint_file_path The path to the machine fingerprint
 * file.
 * @param machine_fingerprint_base64 The machine fingerprint string (Base64
 * format).
 * @param private_key_file_path The path to the private key file (PEM format).
 * @param private_key_base64 The private key string (Base64 format).
 * @return true if the license is issued successfully, false otherwise.
 * @n one of the two parameters machine_fingerprint_file_path and
 * machine_fingerprint_base64 should be set.
 * @n one of the two parameters private_key_file_path and private_key_base64
 * should be set.
 */
YALK_EVENT_TYPE yalk_issue(const char* config, const char* output_file_path,
                           const char* machine_fingerprint_file_path,
                           const char* machine_fingerprint_base64,
                           const char* private_key_file_path,
                           const char* private_key_base64);

/**
 * @brief Issue a license from string.
 *
 * @param config The configuration from command line or config file in JSON.
 * @param output_file_path The path to the output file (license).
 * @param machine_fingerprint_base64 The machine fingerprint string (Base64
 * format).
 * @param private_key_base64 The private key string (Base64 format).
 * @return true if the license is issued successfully, false otherwise.
 */
YALK_EVENT_TYPE yalk_issue_from_string(const char* config,
                                       const char* output_file_path,
                                       const char* machine_fingerprint_base64,
                                       const char* private_key_base64);

/**
 * @brief Verify a license.
 *
 * @param config The configuration from command line or config file in JSON
 * string.
 * @param license_env_name The environment variable name of the license file.
 * @param license_file_path The path to the license file.
 * @param license_base64 The license string (Base64 format).
 * @param public_key_env_name The environment variable name of the public key
 * file.
 * @param public_key_file_path The path to the public key file (PEM format).
 * @param public_key_base64 The public key string (Base64 format).
 * @param is_valid The pointer to the variable to store the result of the
 * verification.
 * @return true if the license is verified successfully, false otherwise.
 * @n one of the three parameters license_env_name, license_file_path and
 * license_base64 should be set.
 * @n one of the three parameters public_key_env_name, public_key_file_path
 * and public_key_base64 should be set.
 */
YALK_EVENT_TYPE yalk_verify(const char* config, const char* license_env_name,
                            const char* license_file_path,
                            const char* license_base64,
                            const char* public_key_env_name,
                            const char* public_key_file_path,
                            const char* public_key_base64, bool* is_valid);

/**
 * @brief Verify a license from string.
 *
 * @param config The configuration from command line or config file in JSON.
 * @param license_base64 The license string (Base64 format).
 * @param public_key_base64 The public key string (Base64 format).
 * @param is_valid The pointer to the variable to store the result of the
 * verification.
 * @return true if the license is verified successfully, false otherwise.
 */
YALK_EVENT_TYPE yalk_verify_from_string(const char* config,
                                        const char* license_base64,
                                        const char* public_key_base64,
                                        bool* is_valid);

#ifdef __cplusplus
}
#endif
