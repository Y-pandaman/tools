#include <iostream>

#include "yalk.h"
#include "yalk_helper.h"

int main(int argc, char *argv[]) {
  // verify the license at application startup
  bool is_valid = false;
  YALK_EVENT_TYPE ret =
      yalk_verify(yalk_config, nullptr, "/tmp/yalk.lic", nullptr, nullptr,
                  nullptr, public_key_base64, &is_valid);

  // print the result
  if (is_valid && ret == YALK_OK) {
    std::cout << "The license is valid." << std::endl;
    return 0;
  } else {
    std::cout << "The license is invalid (error code: " << ret << ")."
              << std::endl;
    return ret;
  }
}
