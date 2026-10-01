#include <chrono>
#include <csignal>
#include <memory>
#include <thread>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

// 复用 Nav2 1.1.20 全部地图实现，使用为独立生命周期提供的 NodeOptions 构造函数。
class StaticDepthCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  StaticDepthCostmap()
  : nav2_costmap_2d::Costmap2DROS(
      rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=/costmap"}))
  {}

  ~StaticDepthCostmap() override
  {
    // 在基类 plugin_loader_ 卸载插件前释放回调组及执行器。
    // 部分回调的析构代码由插件动态库提供，不能留到插件卸载之后执行。
    executor_thread_.reset();
    executor_.reset();
    callback_group_.reset();
  }
};

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int)
{
  stop_requested = 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  // 保持 ROS context 有效，先停止执行器和地图线程、销毁节点，再关闭 context。
  // 避免信号线程关闭 context 后，Costmap 内部 CallbackGroup 才析构。
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  // Humble 的默认回调组可能仍持有由 layers 动态库创建的实体。
  // 让插件库活到整个 ROS 节点/context 清理结束，而不是随 Costmap 成员提前卸载。
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> layer_lifetime(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  layer_lifetime.loadLibraryForClass("nav2_costmap_2d::VoxelLayer");
  auto node = std::make_shared<StaticDepthCostmap>();
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    while (rclcpp::ok() && !stop_requested) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor.remove_node(node->get_node_base_interface());
  }
  node->on_rcl_preshutdown();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
