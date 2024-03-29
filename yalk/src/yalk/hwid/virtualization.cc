#include "yalk/hwid/virtualization.h"

#if defined(__x86_64__) || defined(__i386__)
#include <cpuid.h>
#endif

#include <cstring>
#include <fstream>
#include <iostream>

#include "yalk/utils/log.h"
#include "yalk/utils/parser.h"

namespace yalk {

std::string VirtualizationIdentifier::GetIdentifier() const {
  auto info = CollectInfo();

  if (IsInDocker(info)) {
    return "docker";
  } else if (IsInVirtualMachine(info)) {
    return "vm";
  } else {
    return "physical";
  }
}

std::string VirtualizationIdentifier::CollectCPUID() const {
#if defined(__arm__) || defined(__aarch64__)
  // ignore arm platform
  return "";
#else
  // this method only works on x86 and x86_64 platform
  int cpuInfo[4] = {};

  //
  // Upon execution, code should check bit 31 of register ECX
  // (the “hypervisor present bit”). If this bit is set, a hypervisor is
  // present. In a non-virtualized environment, the bit will be clear.
  //

#if defined(_MSC_VER)
  __cpuid(cpuInfo, 1);
#else
  asm volatile("cpuid"
               : "=a"(cpuInfo[0]), "=b"(cpuInfo[1]), "=c"(cpuInfo[2]),
                 "=d"(cpuInfo[3])
               : "a"(1), "c"(0));
#endif

  if (!(cpuInfo[2] & (1 << 31))) {
    return "physical";
  }

  //
  // A hypervisor is running on the machine. Query the vendor id.
  //
  const int queryVendorIdMagic = 0x40000000;
#if defined(_MSC_VER)
  __cpuid(cpuInfo, queryVendorIdMagic);
#else
  asm volatile("cpuid"
               : "=a"(cpuInfo[0]), "=b"(cpuInfo[1]), "=c"(cpuInfo[2]),
                 "=d"(cpuInfo[3])
               : "a"(queryVendorIdMagic), "c"(0));
#endif

  const int vendorIdLength = 13;
  using VendorIdStr = char[vendorIdLength];

  VendorIdStr hyperVendorId = {};

  memcpy(hyperVendorId + 0, &cpuInfo[1], 4);
  memcpy(hyperVendorId + 4, &cpuInfo[2], 4);
  memcpy(hyperVendorId + 8, &cpuInfo[3], 4);
  hyperVendorId[12] = '\0';

  return hyperVendorId;
#endif
}

VirtualizationInfo VirtualizationIdentifier::CollectInfo() const {
  // 0. init
  VirtualizationInfo info;

  // 1. get cpuid
  info.cpuid = CollectCPUID();

  // 2. get product name
  if (!ReadDataFromFile("/sys/devices/virtual/dmi/id/product_name",
                        info.product_name)) {
    AWARN_F("Failed to read product name");
  }

  // 3. get vendor
  if (!ReadDataFromFile("/sys/devices/virtual/dmi/id/sys_vendor",
                        info.vendor)) {
    AWARN_F("Failed to read vendor");
  }

  return info;
}

bool VirtualizationIdentifier::IsInDocker(
    const VirtualizationInfo& info) const {
  std::ifstream cgroup_file("/proc/1/cgroup");
  if (cgroup_file.is_open()) {
    std::string line;
    while (std::getline(cgroup_file, line)) {
      if (line.find("docker") != std::string::npos ||
          line.find("kubepods") != std::string::npos) {
        return true;
      }
    }
  }
  return false;
}

bool VirtualizationIdentifier::IsInVirtualMachine(
    const VirtualizationInfo& info) const {
  // 1. check cpuid
  if (info.cpuid == "KVMKVMKVM\0\0\0") {
    // KVM
    return true;
  } else if (info.cpuid == "Microsoft Hv") {
    // Microsoft Hyper-V or Windows Virtual PC
    return true;
  } else if (info.cpuid == "VMwareVMware") {
    // VMware
    return true;
  } else if (info.cpuid == "XenVMMXenVMM") {
    // Xen
    return true;
  } else if (info.cpuid == "prl hyperv  ") {
    // Parallels
    return true;
  } else if (info.cpuid == "VBoxVBoxVBox") {
    // VirtualBox
    return true;
  } else if (info.cpuid == "bhyve bhyve") {
    // bhyve
    return true;
  } else if (info.cpuid == "TCGTCGTCGTCG") {
    // QEMU
    return true;
  } else if (info.cpuid == "LKVMLKVMLKVM") {
    // LKVM
    return true;
  } else if (info.cpuid == "OpenBSDVMM58") {
    // VMM
    return true;
  }

  // 2. check product name
  if (info.product_name.find("VMware") != std::string::npos) {
    return true;
  } else if (info.product_name.find("VirtualBox") != std::string::npos) {
    return true;
  } else if (info.product_name.find("KVM") != std::string::npos) {
    return true;
  } else if (info.product_name.find("Microsoft Hv") != std::string::npos) {
    return true;
  } else if (info.product_name == "HVM domU") {
    return true;
  } else if (info.product_name == "bhyve bhyve" ||
             info.product_name.find("bhyve") != std::string::npos) {
    return true;
  } else if (info.product_name == "QEMU Virtual Machine" ||
             info.product_name.find("QEMU") != std::string::npos) {
    return true;
  } else if (info.product_name == "OpenBSD VMM" ||
             info.product_name.find("OpenBSD") != std::string::npos) {
    return true;
  }

  // 3. check vendor
  if (info.vendor == "Microsoft Corporation" ||
      info.vendor.find("Microsoft") != std::string::npos) {
    return true;
  } else if (info.vendor == "VMware, Inc." ||
             info.vendor.find("VMware") != std::string::npos) {
    return true;
  } else if (info.vendor == "innotek GmbH" ||
             info.vendor.find("VirtualBox") != std::string::npos) {
    return true;
  } else if (info.vendor == "QEMU" ||
             info.vendor.find("QEMU") != std::string::npos) {
    return true;
  } else if (info.vendor == "KVM" ||
             info.vendor.find("KVM") != std::string::npos) {
    return true;
  } else if (info.vendor == "Parallels Software International Inc." ||
             info.vendor == "Parallels International GmbH." ||
             info.vendor.find("Parallels") != std::string::npos) {
    return true;
  } else if (info.vendor == "Xen" ||
             info.vendor.find("Xen") != std::string::npos) {
    return true;
  } else if (info.vendor == "OpenBSD" ||
             info.vendor.find("OpenBSD") != std::string::npos) {
    return true;
  }

  return false;
}

YALK_REGISTER_HWID(VirtualizationIdentifier);

}  // namespace yalk
