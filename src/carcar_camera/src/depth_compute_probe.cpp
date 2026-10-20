#include <iostream>
#include <dlfcn.h>
#include <librealsense2/rs.hpp>

// 只实例化SDK处理块及查询CUDA驱动；不打开相机/串口、不修改系统。
int main()
{
  try {
    rs2::log_to_console(RS2_LOG_SEVERITY_INFO);
    rs2::pointcloud pointcloud;
    std::cout << "pointcloud_filter_name=" << pointcloud.get_info(RS2_CAMERA_INFO_NAME) << '\n'
              << "note=SDK 2.58.4 CUDA/NEON hybrid keeps NEON name; inspect SDK log and CUDA trace\n";
    void * driver = dlopen("libcuda.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!driver) {std::cout << "cuda_driver=unavailable\n"; return 0;}
    auto init = reinterpret_cast<int (*)(unsigned)>(dlsym(driver, "cuInit"));
    auto count = reinterpret_cast<int (*)(int *)>(dlsym(driver, "cuDeviceGetCount"));
    if (!init || !count) {std::cout << "cuda_driver=missing_symbols\n"; dlclose(driver); return 0;}
    int devices = 0;
    const int init_code = init(0);
    std::cout << "cuInit=" << init_code << '\n';
    if (init_code == 0) {
      const int count_code = count(&devices);
      std::cout << "cuDeviceGetCount=" << count_code << "\ncuda_devices=" << devices << '\n';
    }
    dlclose(driver);
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
