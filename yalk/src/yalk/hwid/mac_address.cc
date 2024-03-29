#include "yalk/hwid/mac_address.h"

#include <ifaddrs.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <netpacket/packet.h>

#include <array>
#include <iostream>
#include <memory>

#include "yalk/utils/log.h"

namespace yalk {

std::string MacAddressIdentifier::GetIdentifier() const {
  // get list of network interfaces
  struct ifaddrs *ifaddr, *ifa;
  if (getifaddrs(&ifaddr) == -1) {
    AERROR_F("Error getting network interfaces");
    return "";
  }

  // use unique_ptr to manage memory for network interfaces
  std::unique_ptr<struct ifaddrs, decltype(&freeifaddrs)> ifaddr_ptr(
      ifaddr, &freeifaddrs);

  // iterate over network interfaces
  std::string macAddress;
  for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    // ignore loopback interface
    if (ifa->ifa_flags & IFF_LOOPBACK) {
      continue;
    }
    // ignore interfaces without MAC address (e.g. virtual interfaces)
    if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_PACKET) {
      continue;
    }

    // get MAC address of first non-loopback interface
    auto *s = reinterpret_cast<struct sockaddr_ll *>(ifa->ifa_addr);
    std::array<unsigned char, ETH_ALEN> mac;
    std::copy(s->sll_addr, s->sll_addr + ETH_ALEN, mac.begin());

    // convert MAC address to string
    for (size_t i = 0; i < mac.size(); i++) {
      char buffer[3];
      snprintf(buffer, sizeof(buffer), "%02x", mac[i]);
      macAddress += buffer;
      if (i < mac.size() - 1) {
        macAddress += ":";
      }
    }

    break;
  }

  return macAddress;
}

YALK_REGISTER_HWID(MacAddressIdentifier);

}  // namespace yalk
