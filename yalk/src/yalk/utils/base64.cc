#include "yalk/utils/base64.h"

#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <openssl/evp.h>

#include <iostream>
#include <sstream>

namespace yalk {

std::string Base64Encode(const std::string &input) {
  // create a BIO filter chain consisting of a base64 encoder and a memory
  // buffer
  BIO *bio, *b64;
  BUF_MEM *buffer_ptr;

  b64 = BIO_new(BIO_f_base64());
  bio = BIO_new(BIO_s_mem());
  bio = BIO_push(b64, bio);

  // configure the base64 encoder to not add newlines
  BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);

  // write the data to the encoder
  BIO_write(bio, input.c_str(), input.length());
  BIO_flush(bio);

  // get the pointer to the encoded data
  BIO_get_mem_ptr(bio, &buffer_ptr);

  // copy the encoded data to a std::string
  std::string output(buffer_ptr->length, 0);
  std::copy(buffer_ptr->data, buffer_ptr->data + buffer_ptr->length,
            output.begin());

  // cleanup
  BIO_free_all(bio);

  return output;
}

std::string Base64Decode(const std::string &input) {
  // create a BIO filter chain consisting of a base64 decoder and a memory
  // buffer
  BIO *bio, *b64;

  int decode_length = input.length();
  char *decode_buffer = new char[decode_length];

  b64 = BIO_new(BIO_f_base64());
  bio = BIO_new_mem_buf(input.c_str(), input.length());
  bio = BIO_push(b64, bio);

  // configure the base64 decoder to not expect newlines
  BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);

  // decode the data
  decode_length = BIO_read(bio, decode_buffer, input.length());
  std::string output(decode_buffer, decode_length);

  // cleanup
  delete[] decode_buffer;
  BIO_free_all(bio);

  return output;
}

}  // namespace yalk
