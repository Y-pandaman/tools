#pragma once

#include <openssl/evp.h>

#include <string>

namespace yalk {

/// \brief Load the private key from the given file.
/// \param file_path The path to the private key file.
/// \return The private key string.
std::string LoadPrivateKey(const std::string& file_path);

/// \brief Load the public key from the given file.
/// \param file_path The path to the public key file.
/// \return The public key string.
std::string LoadPublicKey(const std::string& file_path);

/// \brief Generate a pair of public and private keys and save them to the
/// given files.
/// \param public_key_file_path The path to the public key file.
/// \param private_key_file_path The path to the private key file.
void GenerateKeyPair(const std::string& public_key_file_path,
                     const std::string& private_key_file_path);

/// \brief Convert the given string to a RSA key.
/// \param key The key string.
/// \param is_public_key Whether the key is a public key.
/// \return The RSA key.
RSA* StringToRSAKey(const std::string& key, bool is_public_key);

/// \brief Convert the given RSA key to a string.
/// \param rsa_key The RSA key.
/// \param is_public_key Whether the key is a public key.
/// \return The key string.
std::string RSAKeyToString(RSA* rsa_key, bool is_public_key);

/// \brief Encrypt the given data with the given key.
/// \param data The data to be encrypted.
/// \param key The key to encrypt the data.
/// \param is_public_key Whether the key is a public key.
/// \return The encrypted data.
std::string EncryptData(const std::string& data, const std::string& key,
                        bool is_public_key);

/// \brief Decrypt the given data with the given key.
/// \param data The data to be decrypted.
/// \param key The key to decrypt the data.
/// \param is_public_key Whether the key is a public key.
/// \return The decrypted data.
std::string DecryptData(const std::string& data, const std::string& key,
                        bool is_public_key);

/// \brief Sign the given data with the given private key.
/// \param data The data to be signed.
/// \param private_key The private key to sign the data.
/// \return The signature.
std::string SignData(const std::string& data, const std::string& private_key);

/// \brief Verify the given data with the given public key and signature.
/// \param data The data to be verified.
/// \param public_key The public key to verify the data.
/// \param signature The signature to verify the data.
/// \return Whether the data is verified.
bool VerifyData(const std::string& data, const std::string& public_key,
                const std::string& signature);

}  // namespace yalk
