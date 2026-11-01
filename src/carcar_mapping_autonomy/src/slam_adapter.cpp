#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <slam_toolbox/slam_toolbox_async.hpp>
#include "carcar_mapping_autonomy/startup_accumulation.hpp"
#include "carcar_mapping_autonomy/online_snapshot.hpp"
#include "carcar_mapping_autonomy/loop_guard.hpp"
// 桥接接纳与闭环事件。静止起步只临时放行 Karto 的时间门槛，不改匹配、损失函数或移动门槛。
class MappingSlam: public slam_toolbox::AsynchronousSlamToolbox {
  class Listener:public karto::MapperLoopClosureListener {
  public:
    explicit Listener(MappingSlam & owner):owner_(owner){}
    void BeginLoopClosure(const std::string&)override{
      owner_.loop_active_=true;++owner_.loop_generation_;owner_.publish();
      owner_.loop_guard_.begin(*owner_.mapper_);
    }
    void EndLoopClosure(const std::string&)override{
      if(!owner_.loop_guard_.finish(*owner_.mapper_,*owner_.solver_)){
        owner_.admission_enabled_=false;owner_.permit_once_=false;
        RCLCPP_ERROR(owner_.get_logger(),"回环校正未通过几何复核，已撤销本次约束与位姿更新；冻结入图等待停车验证：%s",owner_.loop_guard_.state().dump().c_str());
      }
      owner_.loop_active_=false;++owner_.loop_generation_;owner_.publish();
    }
  private:MappingSlam & owner_;
  };
public:
  MappingSlam():AsynchronousSlamToolbox(rclcpp::NodeOptions()),listener_(*this){}
  void configure()override{
    AsynchronousSlamToolbox::configure();
    startup_extra_limit_=std::clamp(static_cast<int>(declare_parameter("startup_stationary_scans",3)),0,12);
    startup_window_sec_=std::clamp(declare_parameter("startup_stationary_window",2.0),0.5,5.0);
    // 静止时SLAM可能只发布首个位姿。新组合保留最后接纳位姿，防止订阅者晚启动死等。
    // 位姿年龄仍不作健康真值；扫描、独立TF、轮式与接纳诊断分别监督。
    pose_pub_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose",rclcpp::QoS(1).reliable().transient_local());
    pub_=create_publisher<std_msgs::msg::String>("/mapping/slam/status",rclcpp::QoS(1).reliable().transient_local());
    admission_service_=create_service<std_srvs::srv::SetBool>("/mapping/slam/set_admission",
      [this](std::shared_ptr<std_srvs::srv::SetBool::Request> q,std::shared_ptr<std_srvs::srv::SetBool::Response> r){
        admission_enabled_=q->data;permit_once_=false;r->success=true;
        r->message=q->data?"扫描写入已开放":"扫描写入已冻结；原始观测继续保留";publish();});
    once_service_=create_service<std_srvs::srv::Trigger>("/mapping/slam/admit_once",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){
        r->success=!admission_enabled_.load()&&!permit_once_.load();
        if(r->success)permit_once_=true;
        r->message=r->success?"仅允许下一帧新鲜扫描进入恢复验证":"单帧验证要求已冻结且无在途许可";});
    recovery_sub_=create_subscription<std_msgs::msg::String>("/mapping/recovery/state",rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String&m){try{const auto j=nlohmann::json::parse(m.data);
        // 发布事件直接闭门，避免网关的在途扫描在取消导航期间污染图。
        recovery_gate_=j.value("gate_write",false);
      }catch(...){}});
    // 网关冻结时不再转发入图扫描。独立只读观察只维护有依据的TF时间，绝不消耗单帧入图许可。
    observation_sub_=create_subscription<sensor_msgs::msg::LaserScan>(
      declare_parameter("observation_scan_topic",std::string("/scan/filtered")),rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan){
        if(admission_enabled_.load()&&!recovery_gate_.load())return;
        const double age=(now()-rclcpp::Time(scan->header.stamp)).seconds();
        if(!std::isfinite(age)||age<-.1||age>.5||scan->ranges.empty()||scan->ranges.size()>100000||
          scan->header.frame_id.empty()||!std::isfinite(scan->angle_min)||!std::isfinite(scan->angle_increment)||scan->angle_increment<=0)return;
        karto::Pose2 odometry;
        if(!pose_helper_->getOdomPose(odometry,scan->header.stamp))return;
        if(!tf_->canTransform(base_frame_,scan->header.frame_id,rclcpp::Time(scan->header.stamp),rclcpp::Duration::from_seconds(0)))return;
        // 与上游TF发布线程共用锁。新观测仅表示运输链鲜活，定位可信仍由恢复事务验证。
        boost::mutex::scoped_lock lock(map_to_odom_mutex_);
        if(rclcpp::Time(scan->header.stamp)>rclcpp::Time(scan_header.stamp))scan_header=scan->header;
        observation_stamp_=rclcpp::Time(scan->header.stamp).nanoseconds();
      });
    attach();
    if(declare_parameter("online_snapshots",false)){
      const auto root=declare_parameter("snapshot_root",std::string("/home/jetson/luhao/my_nav_carcar/log/mapping_recovery"));
      const auto worker=declare_parameter("snapshot_worker",ament_index_cpp::get_package_prefix("carcar_mapping_autonomy")+"/lib/carcar_mapping_autonomy/mapping_snapshot_worker");
      snapshots_=std::make_unique<carcar_mapping_autonomy::OnlineSnapshot>(*this,root,worker,
        [this](auto&prepared,const auto&request){return capture(prepared,request);},[this](const auto&metadata){
          return metadata.at("loop_generation").template get<uint64_t>()==loop_generation_.load()&&
            metadata.at("restore_generation").template get<uint64_t>()==restore_generation_.load()&&!loop_active_.load();});
    }
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{publish();});
  }
  ~MappingSlam()override{detach();}
protected:
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)override{
    const double age=(now()-rclcpp::Time(scan->header.stamp)).seconds();
    scan_callback_age_ms_=age*1000;scan_callback_max_age_ms_=std::max(scan_callback_max_age_ms_,scan_callback_age_ms_);
    if(!std::isfinite(age)||age<-.1||age>.5){++scan_stale_rejections_;return;}
    const bool validation_frame=!admission_enabled_.load()||recovery_gate_.load();
    if(validation_frame&&!permit_once_.exchange(false))return;
    const auto before=sequence_.load();
    AsynchronousSlamToolbox::laserCallback(scan);
    const bool parent_accepted=sequence_.load()!=before;
    if(validation_frame){
      // 显式单帧验证不能消耗许可后又被正常建图的位移／时间节流静默丢弃。
      // 首帧仍由基类完成恢复初始化；后续仅补这一帧，匹配和外部多帧几何校验保持有效。
      if(!parent_accepted&&processor_type_==slam_toolbox::PROCESS&&
         !isPaused(slam_toolbox::NEW_MEASUREMENTS)&&pose_helper_&&smapper_&&smapper_->getMapper()){
        karto::Pose2 odometry;
        if(pose_helper_->getOdomPose(odometry,scan->header.stamp)){
          auto*mapper=smapper_->getMapper();const double interval=mapper->getParamMinimumTimeInterval();
          mapper->setParamMinimumTimeInterval(0);
          struct Restore{karto::Mapper*mapper;double interval;~Restore(){mapper->setParamMinimumTimeInterval(interval);}} restore{mapper,interval};
          if(auto*laser=getLaser(scan))addScan(laser,scan,odometry);
        }
      }
      return;
    }
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
    if(result){++sequence_;accepted_stamp_=rclcpp::Time(scan->header.stamp).nanoseconds();last_accepted_id_=result->GetUniqueId();
      accepted_odom_=input_odometry;accepted_odom_at_=rclcpp::Time(scan->header.stamp).seconds();have_accepted_odom_=true;}
    publish();return result;
  }
  bool deserializePoseGraphCallback(const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
    std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> res)override{
    restore_error_.clear();
    for(const auto*suffix:{".posegraph",".data"}){
      std::error_code error;const auto size=std::filesystem::file_size(req->filename+suffix,error);
      if(error||size==0){restore_error_="恢复文件缺损，未替换生产图："+std::string(suffix);publish();return false;}
    }
    detach();bool ok=false;
    try{ok=AsynchronousSlamToolbox::deserializePoseGraphCallback(header,req,res);attach();}
    catch(...){attach();throw;}
    if(ok){++restore_generation_;loop_guard_.reset_graph();}
    ++loop_generation_;last_accepted_id_=-1;accepted_=false;startup_={};startup_extras_.store(0);startup_closed_.store(false);have_accepted_odom_=false;publish();return ok;
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
  std::optional<carcar_mapping_autonomy::CapturedSnapshot> capture(carcar_mapping_autonomy::GraphSnapshot&prepared,const nlohmann::json&request){
    // 不等待正在优化或发布地图的锁；下一次定时回调再试，不阻塞扫描回调。
    boost::unique_lock<boost::mutex> lock(smapper_mutex_,boost::try_to_lock);if(!lock.owns_lock())return {};
    const auto began=std::chrono::steady_clock::now();
    if(!smapper_||!dataset_||sequence_<3||loop_active_||!admission_enabled_||recovery_gate_)
      throw std::runtime_error("生产SLAM尚未处于可捕获的可信状态");
    auto*mapper=smapper_->getMapper();const auto scans=mapper->GetAllProcessedScans();karto::LocalizedRangeScan*matched=nullptr;
    for(auto*scan:scans)if(scan&&scan->GetUniqueId()==last_accepted_id_)matched=scan;
    if(!matched)throw std::runtime_error("没有与生产位姿图对应的接纳扫描");
    auto metadata=request;metadata["sequence"]=sequence_.load();metadata["scan_id"]=last_accepted_id_;
    metadata["capture_stamp_ns"]=now().nanoseconds();metadata["scan_stamp_ns"]=accepted_stamp_.load();
    metadata["capture_steady_seconds"]=std::chrono::duration<double>(began.time_since_epoch()).count();
    metadata["loop_generation"]=loop_generation_.load();metadata["restore_generation"]=restore_generation_.load();
    metadata["resolution"]=resolution_;metadata["min_pass_through"]=mapper->getParamMinPassThrough();metadata["occupancy_threshold"]=mapper->getParamOccupancyThreshold();
    metadata["mapping_range_m"]=matched->GetLaserRangeFinder()->GetRangeThreshold();
    const auto pose=matched->GetCorrectedPose();metadata["scan_pose"]={pose.GetX(),pose.GetY(),pose.GetHeading()};
    detach();carcar_mapping_autonomy::GraphSnapshot archives;
    try{archives=carcar_mapping_autonomy::capture_graph(*mapper,*dataset_,std::move(prepared));attach();}
    catch(...){attach();throw;}
    metadata["capture_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    lock.unlock();return carcar_mapping_autonomy::CapturedSnapshot{std::move(archives),std::move(metadata)};
  }
  void attach(){if(smapper_&&smapper_->getMapper()){mapper_=smapper_->getMapper();mapper_->AddListener(&listener_);}}
  void detach(){if(mapper_){mapper_->RemoveListener(&listener_);mapper_=nullptr;}}
  void publish(){if(!pub_)return;nlohmann::json j{{"sequence",sequence_.load()},{"attempts",attempts_.load()},
    {"accepted",accepted_.load()},{"accepted_stamp_ns",accepted_stamp_.load()},
    {"loop_generation",loop_generation_.load()},{"loop_active",loop_active_.load()},
    {"loop_guard",loop_guard_.state()},
    {"startup_extras",startup_extras_.load()},{"startup_closed",startup_closed_.load()},
    {"startup_extra_limit",startup_extra_limit_},{"startup_window_seconds",startup_window_sec_},
    {"admission_retries",admission_retries_.load()},
    {"scan_callback_age_ms",scan_callback_age_ms_},{"scan_callback_max_age_ms",scan_callback_max_age_ms_},{"scan_stale_rejections",scan_stale_rejections_},
    {"admission_enabled",admission_enabled_.load()&&!recovery_gate_.load()},{"restore_generation",restore_generation_.load()},{"restore_error",restore_error_},
    {"frozen_observation_stamp_ns",observation_stamp_.load()},
    {"localization_requires_verification",!admission_enabled_.load()||recovery_gate_.load()},
    {"note","接纳与协方差不是独立真值；静止起步只有限补帧，之后仍按移动门槛节流"}};
    std_msgs::msg::String m;m.data=j.dump();pub_->publish(m);}
  Listener listener_;karto::Mapper*mapper_=nullptr;
  carcar_mapping_autonomy::LoopGuard loop_guard_;
  std::unique_ptr<carcar_mapping_autonomy::OnlineSnapshot> snapshots_;int last_accepted_id_=-1;
  double scan_callback_age_ms_=0,scan_callback_max_age_ms_=0;uint64_t scan_stale_rejections_=0;
  carcar_mapping_autonomy::StartupAccumulation startup_;
  int startup_extra_limit_=3;double startup_window_sec_=2;
  std::atomic<int>startup_extras_{0};std::atomic<bool>startup_closed_{false};
  std::atomic<uint64_t>sequence_{0},attempts_{0},loop_generation_{0};
  std::atomic<int64_t>accepted_stamp_{0};std::atomic<bool>accepted_{false},loop_active_{false};
  std::atomic<int64_t>observation_stamp_{0};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr observation_sub_;
  karto::Pose2 accepted_odom_;bool have_accepted_odom_=false;
  double accepted_odom_at_=0,last_retry_at_=-1e9;
  std::atomic<uint64_t>admission_retries_{0};
  std::atomic<bool>admission_enabled_{true},permit_once_{false},recovery_gate_{false};std::atomic<uint64_t>restore_generation_{0};
  std::string restore_error_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr admission_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr once_service_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{auto n=std::make_shared<MappingSlam>();n->configure();n->loadPoseGraphByParams();rclcpp::spin(n);rclcpp::shutdown();n.reset();return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}}
