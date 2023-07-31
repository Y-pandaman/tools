#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
std::string GetLocalIPv4Address() {
  // get list of network interfaces
  struct ifaddrs *ifaddr, *ifa;
  if (getifaddrs(&ifaddr) == -1) {
    perror("Error getting network interfaces");
    return "";
  }

  // use unique_ptr to manage memory for network interfaces
  std::unique_ptr<struct ifaddrs, decltype(&freeifaddrs)> ifaddr_ptr(
      ifaddr, &freeifaddrs);

  // iterate over network interfaces
  std::string ipAddress;
  for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    // ignore loopback interface
    if (ifa->ifa_flags & IFF_LOOPBACK) {
      continue;
    }
    // ignore interfaces without IPv4 address
    if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }

    // get IPv4 address of first non-loopback interface
    struct sockaddr_in *sin =
        reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
    char addrBuf[INET_ADDRSTRLEN];
    const char *addrStr =
        inet_ntop(AF_INET, &sin->sin_addr, addrBuf, INET_ADDRSTRLEN);

    // check if conversion to string was successful
    if (addrStr == nullptr) {
      perror("Error converting IPv4 address to string");
      continue;
    }

    ipAddress = addrStr;
    break;
  }

  return ipAddress;
}
