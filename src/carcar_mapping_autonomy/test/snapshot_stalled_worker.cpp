#include <chrono>
#include <thread>
// 测试专用，不安装；模拟磁盘或栅格工作进程迟迟不返回。
int main(){std::this_thread::sleep_for(std::chrono::seconds(30));return 0;}
