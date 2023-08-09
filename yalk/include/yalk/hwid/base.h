#pragma once

#include <string>

#include "yalk/utils/registerer.h"

namespace yalk {

class HardwareIdentifier {
 public:
  HardwareIdentifier() = default;
  ~HardwareIdentifier() = default;

 public:
  virtual std::string GetIdentifier() const = 0;
};

YALK_REGISTER_REGISTERER(HardwareIdentifier);
#define YALK_REGISTER_HWID(name) YALK_REGISTER_CLASS(HardwareIdentifier, name)

}  // namespace yalk
