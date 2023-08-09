#include "spd_logger.h"

int main(int argc, char const *argv[]) {
  SpdLogger testlog;
  auto logger = testlog.getLogger("master");
  while (1) {
    logger->trace("test success");
    logger->debug("test success");
    logger->info("test success");
    logger->warn("test success");
    logger->error("test success");
    logger->critical("test success");
  }

  return 0;
}
