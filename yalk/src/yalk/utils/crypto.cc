#include "yalk/utils/crypto.h"

#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/rsa.h>

#include <cstring>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace yalk {

std::string LoadPrivateKey(const std::string& file_path) {
  // 0. checking
  // 0.1. check if the file path is empty
  if (file_path.empty()) {
    throw std::runtime_error("LoadPrivateKey: File path is empty.");
  }
  // 0.2. check if the file exists
  if (!std::ifstream(file_path).good()) {
    throw std::runtime_error("LoadPrivateKey: File does not exist.");
  }

  // 1. read the private key from the file
  FILE* privkey_file = std::fopen(file_path.c_str(), "r");
  RSA* rsa_key =
      PEM_read_RSAPrivateKey(privkey_file, nullptr, nullptr, nullptr);
  fclose(privkey_file);

  // 2. convert the private key to a string
  std::string private_key;
  private_key.resize(i2d_RSAPrivateKey(rsa_key, nullptr));
  auto* private_key_ptr = reinterpret_cast<unsigned char*>(&private_key[0]);
  i2d_RSAPrivateKey(rsa_key, &private_key_ptr);

  return private_key;
}

std::string LoadPublicKey(const std::string& file_path) {
  // 0. checking
  // 0.1. check if the file path is empty
  if (file_path.empty()) {
    throw std::runtime_error("LoadPublicKey: File path is empty.");
  }
  // 0.2. check if the file exists
  if (!std::ifstream(file_path).good()) {
    throw std::runtime_error("LoadPublicKey: File does not exist.");
  }

  // 1. read the public key from the file
  FILE* pubkey_file = std::fopen(file_path.c_str(), "r");
  RSA* rsa_key = PEM_read_RSAPublicKey(pubkey_file, nullptr, nullptr, nullptr);
  fclose(pubkey_file);

  // 2. convert the public key to a string
  std::string public_key;
  public_key.resize(i2d_RSAPublicKey(rsa_key, nullptr));
  auto* public_key_ptr = reinterpret_cast<unsigned char*>(&public_key[0]);
  i2d_RSAPublicKey(rsa_key, &public_key_ptr);

  return public_key;
}

void GenerateKeyPair(const std::string& public_key_file_path,
                     const std::string& private_key_file_path) {
  // 0. checking
  // 0.1. check if the file path is empty
  if (public_key_file_path.empty()) {
    throw std::runtime_error("GenerateKeyPair: Public key file path is empty.");
  }
  if (private_key_file_path.empty()) {
    throw std::runtime_error(
        "GenerateKeyPair: Private key file path is empty.");
  }

  // 1. generate a new RSA key pair
  // 1.1. Generate new RSA key
  RSA* rsa = RSA_new();
  BIGNUM* bn = BN_new();
  BN_set_word(bn, RSA_F4);
  RSA_generate_key_ex(rsa, 2048, bn, nullptr);

  // 1.2. Save private key
  FILE* private_key_file = fopen(private_key_file_path.c_str(), "wb");
  PEM_write_RSAPrivateKey(private_key_file, rsa, nullptr, nullptr, 0, nullptr,
                          nullptr);
  fclose(private_key_file);

  // 1.3. Save public key
  FILE* public_key_file = fopen(public_key_file_path.c_str(), "wb");
  PEM_write_RSAPublicKey(public_key_file, rsa);
  fclose(public_key_file);

  // 1.4. Cleanup
  BN_free(bn);
  RSA_free(rsa);
}

RSA* StringToRSAKey(const std::string& key, bool is_public_key) {
  // 0. checking
  // 0.1. check if the key is empty
  if (key.empty()) {
    throw std::runtime_error("StringToRSAKey: Key is empty.");
  }

  // 1. convert the key to a RSA key
  if (is_public_key) {
    const auto* public_key_ptr =
        reinterpret_cast<const unsigned char*>(&key[0]);
    return d2i_RSAPublicKey(nullptr, &public_key_ptr, key.size());
  } else {
    const auto* private_key_ptr =
        reinterpret_cast<const unsigned char*>(&key[0]);
    return d2i_RSAPrivateKey(nullptr, &private_key_ptr, key.size());
  }
}

std::string RSAKeyToString(RSA* rsa_key, bool is_public_key) {
  // 0. checking
  // 0.1. check if the key is empty
  if (rsa_key == nullptr) {
    throw std::runtime_error("RSAKeyToString: Key is empty.");
  }

  // 1. convert the key to a string
  std::string key;
  if (is_public_key) {
    key.resize(i2d_RSAPublicKey(rsa_key, nullptr));
    auto* public_key_ptr = reinterpret_cast<unsigned char*>(&key[0]);
    i2d_RSAPublicKey(rsa_key, &public_key_ptr);
  } else {
    key.resize(i2d_RSAPrivateKey(rsa_key, nullptr));
    auto* private_key_ptr = reinterpret_cast<unsigned char*>(&key[0]);
    i2d_RSAPrivateKey(rsa_key, &private_key_ptr);
  }

  return key;
}

std::string EncryptData(const std::string& data, const std::string& key,
                        bool is_public_key) {
  // 0. checking
  // 0.1. check if the data is empty
  if (data.empty()) {
    throw std::runtime_error("EncryptData: Data is empty.");
  }

  // 1. convert the key to a RSA key
  RSA* rsa_key = StringToRSAKey(key, is_public_key);

  // 2. encrypt the data
  // 2.1. Determine key length
  int rsa_key_size = RSA_size(rsa_key);
  int max_data_size = rsa_key_size - RSA_PKCS1_PADDING_SIZE;
  int num_blocks = (data.length() + max_data_size - 1) / max_data_size;

  // 2.2. Allocate memory for the encrypted data
  auto* output_buffer = new unsigned char[num_blocks * rsa_key_size];
  int output_length = 0;

  // 2.3. Encrypt the data
  // Encrypt each block of data
  for (int i = 0; i < num_blocks; i++) {
    const std::string block_data =
        data.substr(i * max_data_size, max_data_size);

    // Encrypt the block of data
    int encrypted_length = 0;
    if (is_public_key) {
      encrypted_length = RSA_public_encrypt(
          static_cast<int>(block_data.length()),
          reinterpret_cast<const unsigned char*>(block_data.data()),
          output_buffer + (i * rsa_key_size), rsa_key, RSA_PKCS1_PADDING);
    } else {
      encrypted_length = RSA_private_encrypt(
          static_cast<int>(block_data.length()),
          reinterpret_cast<const unsigned char*>(block_data.data()),
          output_buffer + (i * rsa_key_size), rsa_key, RSA_PKCS1_PADDING);
    }

    if (encrypted_length == -1) {
      // throw std::runtime_error("EncryptData: RSA_public_encrypt failed.");
      return "";
    }
    output_length += encrypted_length;
  }

  // 2.4. Resize the encrypted data to its actual length
  std::string encrypted_data(reinterpret_cast<char*>(output_buffer),
                             output_length);
  delete[] output_buffer;

  // 3. cleanup
  RSA_free(rsa_key);

  // 4. return the encrypted data
  return encrypted_data;
}

std::string DecryptData(const std::string& data, const std::string& key,
                        bool is_public_key) {
  // 0. checking
  // 0.1. check if the data is empty
  if (data.empty()) {
    throw std::runtime_error("DecryptData: Data is empty.");
  }

  // 1. convert the key to a RSA key
  RSA* rsa_key = StringToRSAKey(key, is_public_key);

  // 2. decrypt the data
  // 2.1. Determine key length
  int rsa_key_size = RSA_size(rsa_key);
  int max_data_size = rsa_key_size;
  int num_blocks = (data.length() + max_data_size - 1) / max_data_size;

  // 2.2. Allocate memory for the decrypted data
  unsigned char* output_buffer = new unsigned char[num_blocks * max_data_size];

  // 2.3. Decrypt the data
  int output_length = 0;
  for (int i = 0; i < num_blocks; i++) {
    unsigned char* temp = new unsigned char[max_data_size];
    const unsigned char* block_data = reinterpret_cast<const unsigned char*>(
        data.data() + (i * max_data_size));

    // Decrypt the block of data
    int decrypted_length = 0;
    if (is_public_key) {
      decrypted_length = RSA_public_decrypt(rsa_key_size, block_data, temp,
                                            rsa_key, RSA_PKCS1_PADDING);
    } else {
      decrypted_length = RSA_private_decrypt(rsa_key_size, block_data, temp,
                                             rsa_key, RSA_PKCS1_PADDING);
    }
    if (decrypted_length == -1) {
      // throw std::runtime_error("DecryptData: RSA_public_decrypt failed.");
      return "";
    }
    memcpy(output_buffer + output_length, temp, decrypted_length);
    output_length += decrypted_length;
  }

  // 2.4. Resize the decrypted data to its actual length
  std::string decrypted_data(reinterpret_cast<char*>(output_buffer),
                             output_length);
  delete[] output_buffer;

  // 3. cleanup
  RSA_free(rsa_key);

  // 4. return the decrypted data
  return decrypted_data;
}

std::string SignData(const std::string& data, const std::string& private_key) {
  // 0. checking
  // 0.1. check if the data is empty
  if (data.empty()) {
    throw std::runtime_error("SignData: Data is empty.");
  }
  // 0.2. check if the key is empty
  if (private_key.empty()) {
    throw std::runtime_error("SignData: Private key is empty.");
  }

  // 1. convert the key to a RSA key
  RSA* rsa_key = StringToRSAKey(private_key, false);
  if (rsa_key == nullptr) {
    throw std::runtime_error("SignData: Failed to convert key to RSA key");
  }

  // 2. sign the data
  // Create SHA256 digest context
  EVP_MD_CTX* ctx = EVP_MD_CTX_new();
  if (!ctx) {
    RSA_free(rsa_key);
    throw std::runtime_error("SignData: Failed to create digest context");
  }
  if (EVP_DigestInit_ex(ctx, EVP_sha256(), NULL) != 1) {
    EVP_MD_CTX_free(ctx);
    RSA_free(rsa_key);
    throw std::runtime_error("SignData: Failed to initialize digest context");
  }

  // Update digest context with input data
  if (EVP_DigestUpdate(ctx, data.c_str(), data.length()) != 1) {
    EVP_MD_CTX_free(ctx);
    RSA_free(rsa_key);
    throw std::runtime_error("SignData: Failed to update digest context");
  }

  // Finalize digest context
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_len = 0;
  if (EVP_DigestFinal_ex(ctx, digest, &digest_len) != 1) {
    EVP_MD_CTX_free(ctx);
    RSA_free(rsa_key);
    throw std::runtime_error("SignData: Failed to finalize digest context");
  }

  // Allocate buffer for signature
  unsigned int siglen = RSA_size(rsa_key);
  unsigned char* sigbuf = new unsigned char[siglen];
  memset(sigbuf, 0, siglen);

  // Generate signature using RSA private key
  int ret = RSA_sign(NID_sha256, (const unsigned char*)digest, digest_len,
                     sigbuf, &siglen, rsa_key);
  if (ret != 1) {
    delete[] sigbuf;
    EVP_MD_CTX_free(ctx);
    RSA_free(rsa_key);
    throw std::runtime_error("SignData: Failed to generate RSA signature");
  }

  // Convert signature to hex string
  std::ostringstream ss;
  for (unsigned int i = 0; i < siglen; i++) {
    ss << std::hex << std::setfill('0') << std::setw(2)
       << (unsigned int)sigbuf[i];
  }

  // Free resources
  delete[] sigbuf;
  EVP_MD_CTX_free(ctx);
  RSA_free(rsa_key);

  return ss.str();
}

bool VerifyData(const std::string& data, const std::string& public_key,
                const std::string& signature) {
  // 0. checking
  // 0.1. check if the data is empty
  if (data.empty()) {
    throw std::runtime_error("VerifyData: Data is empty.");
  }

  // 0.2. check if the signature is empty
  if (signature.empty()) {
    throw std::runtime_error("VerifyData: Signature is empty.");
  }

  // 0.3. check if the signature is valid
  if (signature.length() % 2 != 0) {
    throw std::runtime_error("VerifyData: Signature is invalid.");
  }

  // 0.4.check if public key is empty
  if (public_key.empty()) {
    throw std::runtime_error("VerifyData: Public key is empty.");
  }

  // 1. convert the key to a RSA key
  RSA* rsa_key = StringToRSAKey(public_key, true);
  if (!rsa_key) {
    throw std::runtime_error("VerifyData: Failed to convert key to RSA key");
  }

  // 2. verify the data
  // Create SHA256 digest context
  EVP_MD_CTX* ctx = EVP_MD_CTX_new();
  if (!ctx) {
    RSA_free(rsa_key);
    throw std::runtime_error("VerifyData: Failed to create digest context");
  }
  if (EVP_DigestInit_ex(ctx, EVP_sha256(), NULL) != 1) {
    EVP_MD_CTX_free(ctx);
    RSA_free(rsa_key);
    throw std::runtime_error("VerifyData: Failed to initialize digest context");
  }

  // Update digest context with input data
  if (EVP_DigestUpdate(ctx, data.c_str(), data.length()) != 1) {
    EVP_MD_CTX_free(ctx);
    RSA_free(rsa_key);
    throw std::runtime_error("VerifyData: Failed to update digest context");
  }

  // Finalize digest context
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_len = 0;
  if (EVP_DigestFinal_ex(ctx, digest, &digest_len) != 1) {
    EVP_MD_CTX_free(ctx);
    RSA_free(rsa_key);
    throw std::runtime_error("VerifyData: Failed to finalize digest context");
  }

  // Convert signature from hex string to byte array
  unsigned int siglen = signature.length() / 2;
  unsigned char* sigbuf = new unsigned char[siglen];
  for (unsigned int i = 0; i < siglen; i++) {
    std::string hex_byte = signature.substr(i * 2, 2);
    sigbuf[i] = (unsigned char)strtol(hex_byte.c_str(), NULL, 16);
  }

  // Verify signature using RSA public key
  int ret = RSA_verify(NID_sha256, (const unsigned char*)digest, digest_len,
                       sigbuf, siglen, rsa_key);
  bool verified = (ret == 1);

  // Free resources
  delete[] sigbuf;
  EVP_MD_CTX_free(ctx);
  RSA_free(rsa_key);

  return verified;
}

}  // namespace yalk
