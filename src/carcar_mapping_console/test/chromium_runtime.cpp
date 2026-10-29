// 仅用于隔离域219的真实浏览器测试；只读挂载已有snap运行库，不改系统/用户浏览器。
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
int main(int argc,char ** argv) {
  const char * domain=std::getenv("ROS_DOMAIN_ID");
  if(!domain||std::string(domain)!="219"){std::cerr<<"仅允许离线域219\n";return 2;}
  std::vector<std::string> args{"bwrap","--die-with-parent","--unshare-pid",
    "--ro-bind","/snap/core24/current","/","--dev","/dev","--proc","/proc",
    "--ro-bind","/snap/chromium/current","/opt","--ro-bind","/snap/gnome-46-2404/current","/mnt",
    "--ro-bind","/snap/mesa-2404/current","/media","--ro-bind","/usr/share/fonts","/usr/share/fonts","--bind","/tmp","/tmp","--chdir","/tmp",
    "--setenv","HOME","/tmp","--setenv","LD_LIBRARY_PATH",
    "/usr/lib/aarch64-linux-gnu:/opt/usr/lib/aarch64-linux-gnu:/mnt/usr/lib/aarch64-linux-gnu:/mnt/usr/lib/aarch64-linux-gnu/pulseaudio:/media/usr/lib/aarch64-linux-gnu:/opt/usr/lib/chromium-browser",
    "--","/opt/usr/lib/chromium-browser/chrome"};
  for(int i=1;i<argc;++i)args.emplace_back(argv[i]);
  std::vector<char*> raw;for(auto & arg:args)raw.push_back(arg.data());raw.push_back(nullptr);
  execvp(raw[0],raw.data());perror("browser test runtime");return 127;
}
