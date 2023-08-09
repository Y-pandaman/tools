#pragma once

#include <string>

namespace yalk {

/// \brief Encode the input string to base64.
/// \param input The input string.
/// \return The base64 encoded string.
std::string Base64Encode(const std::string &input);

/// \brief Decode the input string from base64.
/// \param input The input string.
/// \return The base64 decoded string.
std::string Base64Decode(const std::string &input);

}  // namespace yalk
