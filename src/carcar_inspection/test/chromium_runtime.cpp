// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
// 仅用于APP-001隔离域227测试，复用本机已有snap库；不改系统或用户浏览器。
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>

int main(int argc, char ** argv) {
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  if (!domain || std::string(domain) != "227") {
    std::cerr << "仅允许离线测试域227\n";
    return 2;
  }
  std::vector<std::string> args{
    "bwrap", "--die-with-parent", "--unshare-pid",
    "--ro-bind", "/snap/core24/current", "/", "--dev", "/dev", "--proc", "/proc",
    "--ro-bind", "/snap/chromium/current", "/opt",
    "--ro-bind", "/snap/gnome-46-2404/current", "/mnt",
    "--ro-bind", "/snap/mesa-2404/current", "/media",
    "--ro-bind", "/usr/share/fonts", "/usr/share/fonts",
    "--bind", "/tmp", "/tmp", "--chdir", "/tmp",
    "--setenv", "LD_LIBRARY_PATH",
    "/usr/lib/aarch64-linux-gnu:/opt/usr/lib/aarch64-linux-gnu:/mnt/usr/lib/aarch64-linux-gnu:"
    "/mnt/usr/lib/aarch64-linux-gnu/pulseaudio:/media/usr/lib/aarch64-linux-gnu:/opt/usr/lib/chromium-browser",
    "--", "/opt/usr/lib/chromium-browser/chrome"};
  for (int i = 1; i < argc; ++i) args.emplace_back(argv[i]);
  std::vector<char *> raw;
  for (auto & arg : args) raw.push_back(arg.data());
  raw.push_back(nullptr);
  execvp(raw[0], raw.data());
  perror("inspection browser test runtime");
  return 127;
}
