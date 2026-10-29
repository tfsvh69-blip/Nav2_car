#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <slam_toolbox/slam_toolbox_async.hpp>
#include "carcar_mapping_autonomy/startup_accumulation.hpp"
// 桥接接纳与闭环事件。静止起步只临时放行 Karto 的时间门槛，不改匹配、损失函数或移动门槛。
class MappingSlam: public slam_toolbox::AsynchronousSlamToolbox {
  class Listener:public karto::MapperLoopClosureListener {
  public:
    explicit Listener(MappingSlam & owner):owner_(owner){}
    void BeginLoopClosure(const std::string&)override{owner_.loop_active_=true;++owner_.loop_generation_;owner_.publish();}
    void EndLoopClosure(const std::string&)override{owner_.loop_active_=false;++owner_.loop_generation_;owner_.publish();}
  private:MappingSlam & owner_;
  };
public:
  MappingSlam():AsynchronousSlamToolbox(rclcpp::NodeOptions()),listener_(*this){}
  void configure()override{
    AsynchronousSlamToolbox::configure();
    startup_extra_limit_=std::clamp(static_cast<int>(declare_parameter("startup_stationary_scans",3)),0,8);
    startup_window_sec_=std::clamp(declare_parameter("startup_stationary_window",2.0),0.5,5.0);
    // 静止时SLAM可能只发布首个位姿。新组合保留最后接纳位姿，防止订阅者晚启动死等。
    // 位姿年龄仍不作健康真值；扫描、独立TF、轮式与接纳诊断分别监督。
    pose_pub_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose",rclcpp::QoS(1).reliable().transient_local());
    pub_=create_publisher<std_msgs::msg::String>("/mapping/slam/status",rclcpp::QoS(1).reliable().transient_local());
    attach();
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{publish();});
  }
  ~MappingSlam()override{detach();}
protected:
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)override{
    const auto before=sequence_.load();
    AsynchronousSlamToolbox::laserCallback(scan);
    const bool parent_accepted=sequence_.load()!=before;
    if(startup_.closed){
      // Humble外层在0.8*d²时更新“尝试”基准，Karto按最后接纳的d²判断。
      // 短途停车会落在两门槛之间。按最后接纳的里程计重试，Karto全部原门槛继续生效。
      if(!parent_accepted&&have_accepted_odom_&&!loop_active_.load()&&
        processor_type_==slam_toolbox::PROCESS&&!isPaused(slam_toolbox::NEW_MEASUREMENTS)&&
        pose_helper_&&smapper_&&smapper_->getMapper()){
        const double stamp=rclcpp::Time(scan->header.stamp).seconds();
        if(stamp-accepted_odom_at_>=minimum_time_interval_.seconds()&&stamp-last_retry_at_>=minimum_time_interval_.seconds()){
          karto::Pose2 odometry;
          if(pose_helper_->getOdomPose(odometry,scan->header.stamp)){
            auto*mapper=smapper_->getMapper();
            const double travel=mapper->getParamMinimumTravelDistance();
            const double heading=mapper->getParamMinimumTravelHeading();
            if(accepted_odom_.SquaredDistance(odometry)>=travel*travel||
              std::abs(std::remainder(odometry.GetHeading()-accepted_odom_.GetHeading(),2*M_PI))>=heading){
              last_retry_at_=stamp;
              if(auto*laser=getLaser(scan)){++admission_retries_;addScan(laser,scan,odometry);}
            }
          }
        }
      }
      return;
    }
    karto::Pose2 pose;
    if(!pose_helper_||!pose_helper_->getOdomPose(pose,scan->header.stamp))return;
    const double travel=smapper_&&smapper_->getMapper()?smapper_->getMapper()->getParamMinimumTravelDistance():.10;
    const auto decision=carcar_mapping_autonomy::consider_stationary_startup(
      startup_,parent_accepted,isPaused(slam_toolbox::NEW_MEASUREMENTS),pose.GetX(),pose.GetY(),
      rclcpp::Time(scan->header.stamp).seconds(),travel,minimum_time_interval_.seconds(),
      startup_extra_limit_,startup_window_sec_);
    startup_extras_.store(startup_.extras);startup_closed_.store(startup_.closed);
    if(decision==carcar_mapping_autonomy::StartupDecision::accept_extra&&processor_type_==slam_toolbox::PROCESS&&smapper_&&smapper_->getMapper()){
      // Karto 默认 MinimumTimeInterval 为 3600 秒，静止扫描即使绕过外层节流也会被丢掉。
      // 只在这一次补帧期间改为 0，返回前恢复，正常移动门槛保持不变。
      auto*mapper=smapper_->getMapper();
      const double saved_interval=mapper->getParamMinimumTimeInterval();
      mapper->setParamMinimumTimeInterval(0);
      struct Restore{karto::Mapper*mapper;double saved;~Restore(){mapper->setParamMinimumTimeInterval(saved);}} restore{mapper,saved_interval};
      if(auto*laser=getLaser(scan))addScan(laser,scan,pose);
    }
  }
  karto::LocalizedRangeScan * addScan(karto::LaserRangeFinder*laser,const sensor_msgs::msg::LaserScan::ConstSharedPtr&scan,karto::Pose2&pose)override{
    const auto input_odometry=pose;
    ++attempts_;auto*result=AsynchronousSlamToolbox::addScan(laser,scan,pose);
    accepted_=result!=nullptr;
    if(result){++sequence_;accepted_stamp_=rclcpp::Time(scan->header.stamp).nanoseconds();
      accepted_odom_=input_odometry;accepted_odom_at_=rclcpp::Time(scan->header.stamp).seconds();have_accepted_odom_=true;}
    publish();return result;
  }
  bool deserializePoseGraphCallback(const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> res)override{
    detach();bool ok=false;
    try{ok=AsynchronousSlamToolbox::deserializePoseGraphCallback(header,req,res);attach();}
    catch(...){attach();throw;}
    ++loop_generation_;accepted_=false;startup_={};startup_extras_.store(0);startup_closed_.store(false);have_accepted_odom_=false;return ok;
  }
  bool serializePoseGraphCallback(const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response> res)override{
    // Karto会序列化监听器列表：临时摘除本适配器，保持原2.6.10工程可恢复。
    detach();bool ok=false;
    try{ok=AsynchronousSlamToolbox::serializePoseGraphCallback(header,req,res);attach();}
    catch(...){attach();throw;}
    return ok;
  }
private:
  void attach(){if(smapper_&&smapper_->getMapper()){mapper_=smapper_->getMapper();mapper_->AddListener(&listener_);}}
  void detach(){if(mapper_){mapper_->RemoveListener(&listener_);mapper_=nullptr;}}
  void publish(){if(!pub_)return;nlohmann::json j{{"sequence",sequence_.load()},{"attempts",attempts_.load()},
    {"accepted",accepted_.load()},{"accepted_stamp_ns",accepted_stamp_.load()},
    {"loop_generation",loop_generation_.load()},{"loop_active",loop_active_.load()},
    {"startup_extras",startup_extras_.load()},{"startup_closed",startup_closed_.load()},
    {"admission_retries",admission_retries_.load()},
    {"note","接纳与协方差不是独立真值；静止起步只有限补帧，之后仍按移动门槛节流"}};
    std_msgs::msg::String m;m.data=j.dump();pub_->publish(m);}
  Listener listener_;karto::Mapper*mapper_=nullptr;
  carcar_mapping_autonomy::StartupAccumulation startup_;
  int startup_extra_limit_=3;double startup_window_sec_=2;
  std::atomic<int>startup_extras_{0};std::atomic<bool>startup_closed_{false};
  std::atomic<uint64_t>sequence_{0},attempts_{0},loop_generation_{0};
  std::atomic<int64_t>accepted_stamp_{0};std::atomic<bool>accepted_{false},loop_active_{false};
  karto::Pose2 accepted_odom_;bool have_accepted_odom_=false;
  double accepted_odom_at_=0,last_retry_at_=-1e9;
  std::atomic<uint64_t>admission_retries_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{auto n=std::make_shared<MappingSlam>();n->configure();n->loadPoseGraphByParams();rclcpp::spin(n);rclcpp::shutdown();n.reset();return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}}
