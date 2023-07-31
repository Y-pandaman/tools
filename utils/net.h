#pragma once

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>

#include <memory>
#include <string>

namespace dvis_cpp {
namespace util {

/// \brief Get the local IPv4 address of the machine.
/// \return The local IPv4 address of the machine.
std::string GetLocalIPv4Address();

}  // namespace util
}  // namespace dvis_cpp
