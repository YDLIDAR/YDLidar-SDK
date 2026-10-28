#if defined(__APPLE__)

#include <glob.h>
#include <set>
#include <string>
#include <vector>

#include "core/serial/serial.h"

namespace ydlidar {
namespace core {
namespace serial {

std::vector<PortInfo> list_ports() {
  std::vector<PortInfo> results;
  std::set<std::string> uniq;

  const char *patterns[] = {
    "/dev/cu.*",
    "/dev/tty.*",
    "/dev/cu.usb*",
    "/dev/tty.usb*",
    "/dev/cu.wchusb*",
    "/dev/cu.SLAB*",
  };

  for (const char *pattern : patterns) {
    glob_t g;
    if (::glob(pattern, 0, nullptr, &g) == 0) {
      for (size_t i = 0; i < g.gl_pathc; ++i) {
        uniq.insert(std::string(g.gl_pathv[i]));
      }
    }
    globfree(&g);
  }

  for (const auto &dev : uniq) {
    PortInfo info;
    info.port = dev;
    info.description = "macOS serial device";
    info.hardware_id = "n/a";
    info.device_id = "";
    results.push_back(info);
  }

  return results;
}

} // namespace serial
} // namespace core
} // namespace ydlidar

#endif
