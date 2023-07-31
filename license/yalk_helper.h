#pragma once

#include "dvis_cpp/util/log.h"
#include "dvis_cpp/util/obfuscate.h"

#ifdef ENABLE_YALK
#include "yalk.h"
#endif

namespace dvis_cpp {
namespace util {

static const char* yalk_config = AY_OBFUSCATE(
    "{\"common\":{\"issue_number\":0,\"product_name\":\"DVIS\",\"product_"
    "version\":\"1.0.0\",\"type\":0},\"hwid\":{\"mac\":"
    "\"MacAddressIdentifier\"}}");

static const char* public_key_base64 = AY_OBFUSCATE(
    "MIIBCgKCAQEA0cbCnXlD8OPo/EWw4xg0cbu6mAFMKqBWxJo3i06LzNKigK9rl/"
    "i1+"
    "VhPYfpXJBwCCWw92nAh7xTEFQ7Q2k5Plg3cPGZQ6jVFJFOLLyEGdO5X6qf2jt5PSbtNezq5c9F"
    "EeAnUOKK9JEclIbIcscEYgPM3x+kYL/VM+LuY11d1077kDDfCH5miHNF/"
    "pvkziP3hTF4NiCJhocoV2mvf/"
    "hh16IJgbo1XvZvps4I1KswPPi20VM14WiNmO9bxcbYS8P5u6Y1bj96QucPUTkqUebTsG1ueUZO"
    "7ZreVrtNXObsZtVctnAWFBLz/1BoC0YmEHmmqInajI+aLSSvS9LPY2xfMWQIDAQAB");

static const char* model_encryption_key =
    AY_OBFUSCATE("lyQZGuiS/zCqH13f31BbvR1lbnXcuonZlkTXbVadqGo=");

inline bool ValidateLicense(const std::string& license_file) {
#ifndef ENABLE_YALK
  return true;
#else

  // verify the license at application startup
  bool is_valid = false;
  YALK_EVENT_TYPE ret =
      yalk_verify(yalk_config, nullptr, license_file.c_str(), nullptr, nullptr,
                  nullptr, public_key_base64, &is_valid);

  // print the result
  if (is_valid && ret == YALK_OK) {
    AINFO_F("The license is valid.");
    return true;
  } else {
    AERROR_F("The license is invalid (error code: {}).", ret);
    return false;
  }
#endif
}
}  // namespace util
}  // namespace dvis_cpp
