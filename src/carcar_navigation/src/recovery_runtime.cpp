#include "carcar_navigation/recovery_runtime.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_set>
#include <utility>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>

namespace carcar_navigation {
namespace {
template<class UUID> std::string uuid_text(const UUID & uuid) {
  std::ostringstream s;
  for (auto b : uuid) {s << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);}
  return s.str();
}
double parameter(rclcpp::Node::SharedPtr node, const std::string & key, double fallback) {
  if (node->has_parameter(key)) {return node->get_parameter(key).as_double();}
  return fallback;
}
}  // namespace

SensorCache::SensorCache(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group,
  const std::string & scan, const std::string & costmap, const std::string & footprint)
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(costmap,
    rclcpp::QoS(1).reliable().transient_local(), [this](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      data_.costmap = msg; data_.costmap_received = Steady::now(); ++data_.costmap_count;
    }, options);
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(scan,
    rclcpp::SensorDataQoS().keep_last(1), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      data_.scan = msg; data_.scan_received = Steady::now(); ++data_.scan_count;
    }, options);
  // Nav2 published_footprint 是 volatile 发布者，不要求 transient_local。
  footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(footprint,
    rclcpp::QoS(1).reliable().durability_volatile(), [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      data_.footprint = msg; data_.footprint_received = Steady::now(); ++data_.footprint_count;
    }, options);
}
SensorSnapshot SensorCache::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_); return data_;
}
bool fresh_stamp(const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & now,
  TimePoint received, double age)
{
  if (received == TimePoint{} || !std::isfinite(age) || age <= 0) {return false;}
  const double elapsed = (now - rclcpp::Time(stamp, now.get_clock_type())).seconds();
  return elapsed >= 0 && elapsed <= age && seconds(received) <= age;
}
HeadingReference heading_reference(const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & goal, double lookahead)
{
  HeadingReference ref;
  if (path.poses.empty() || path.header.frame_id != pose.header.frame_id) {return ref;}
  const double x=pose.pose.position.x, y=pose.pose.position.y;
  double best=std::numeric_limits<double>::infinity(), projection=0;
  size_t index=0;
  // 最近线段投影，避免采样到机器人身后的旧路径起点。
  for (size_t i=0; i+1<path.poses.size(); ++i) {
    const auto & a=path.poses[i].pose.position;
    const auto & b=path.poses[i+1].pose.position;
    double dx=b.x-a.x, dy=b.y-a.y, len2=dx*dx+dy*dy;
    double t=len2>1e-12 ? std::clamp(((x-a.x)*dx+(y-a.y)*dy)/len2,0.0,1.0) : 0;
    double d=std::hypot(x-a.x-t*dx,y-a.y-t*dy);
    if (d<best) {best=d; projection=t; index=i;}
  }
  auto target=path.poses.back().pose.position;
  double remaining=lookahead;
  for (size_t i=index; i+1<path.poses.size(); ++i) {
    const auto & a=path.poses[i].pose.position;
    const auto & b=path.poses[i+1].pose.position;
    double t=i==index ? projection : 0.0;
    double len=std::hypot(b.x-a.x,b.y-a.y);
    if (len*(1-t)>=remaining && len>1e-9) {
      t += remaining/len;
      target.x=a.x+t*(b.x-a.x); target.y=a.y+t*(b.y-a.y); break;
    }
    remaining -= len*(1-t);
  }
  ref.valid=true; ref.segment=index;
  if (std::hypot(path.poses.back().pose.position.x-x,path.poses.back().pose.position.y-y)<0.10 &&
    goal.header.frame_id==pose.header.frame_id) {
    ref.yaw=tf2::getYaw(goal.pose.orientation);
  } else {
    ref.yaw=std::atan2(target.y-y,target.x-x);
  }
  ref.error=normalize(ref.yaw-tf2::getYaw(pose.pose.orientation));
  ref.valid=std::isfinite(ref.yaw) && std::isfinite(ref.error);
  return ref;
}
bool scan_points_in_base(const sensor_msgs::msg::LaserScan & scan, tf2_ros::Buffer & tf,
  const std::string & base, std::vector<geometry_msgs::msg::Point> & points)
{
  if (scan.header.frame_id.empty() || scan.ranges.empty() ||
    !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) ||
    scan.angle_increment==0 || !(scan.range_max>scan.range_min)) {return false;}
  try {
    auto transform=tf.lookupTransform(base,scan.header.frame_id,
      tf2_ros::fromRclcpp(rclcpp::Time(scan.header.stamp)));
    bool usable=false;
    for (size_t i=0;i<scan.ranges.size();++i) {
      double r=scan.ranges[i];
      if (std::isinf(r) && r>0) {usable=true;}
      if (!std::isfinite(r) || r<scan.range_min || r>scan.range_max) {continue;}
      usable=true;
      double a=scan.angle_min+i*scan.angle_increment;
      geometry_msgs::msg::PointStamped in,out;
      in.point.x=r*std::cos(a); in.point.y=r*std::sin(a);
      tf2::doTransform(in,out,transform); points.push_back(out.point);
    }
    return usable;
  } catch (const tf2::TransformException &) {return false;}
}
SafetyResult swept_clear(const SensorSnapshot & d, tf2_ros::Buffer & tf,
  const rclcpp::Time & now, const std::string & base, double dx, double dy, double age)
{
  if (!d.scan || !d.costmap || !d.footprint || d.footprint->polygon.points.size()<3) {
    return {false,"DATA_MISSING","等待扫描、原始代价地图及真实包络"};
  }
  if (!fresh_stamp(d.scan->header.stamp,now,d.scan_received,age) ||
    !fresh_stamp(d.costmap->header.stamp,now,d.costmap_received,age) ||
    !fresh_stamp(d.footprint->header.stamp,now,d.footprint_received,age)) {
    return {false,"DATA_EXPIRED","消息时间戳或本地接收时间过期"};
  }
  const auto & md=d.costmap->metadata;
  if (!std::isfinite(md.resolution) || md.resolution<=0 || md.size_x==0 || md.size_y==0 ||
    md.size_x>10000 || md.size_y>10000 ||
    size_t(md.size_x)*md.size_y!=d.costmap->data.size() ||
    !std::isfinite(md.origin.position.x) || !std::isfinite(md.origin.position.y) ||
    !std::isfinite(dx) || !std::isfinite(dy)) {
    return {false,"DATA_MISSING","代价地图元数据或扫掠参数无效"};
  }
  try {
    auto foot_tf=tf.lookupTransform(base,d.footprint->header.frame_id,
      tf2_ros::fromRclcpp(rclcpp::Time(d.footprint->header.stamp)));
    nav2_costmap_2d::Footprint footprint;
    double half_l=0,half_w=0;
    for (const auto & p : d.footprint->polygon.points) {
      geometry_msgs::msg::PointStamped in,out; in.point.x=p.x; in.point.y=p.y;
      tf2::doTransform(in,out,foot_tf);
      if (!std::isfinite(out.point.x) || !std::isfinite(out.point.y)) {
        return {false,"DATA_MISSING","包络包含非有限坐标"};
      }
      half_l=std::max(half_l,std::abs(out.point.x));
      half_w=std::max(half_w,std::abs(out.point.y));
      footprint.push_back(out.point);
    }
    if (half_l<0.01 || half_w<0.01 || half_l>2 || half_w>2) {
      return {false,"DATA_MISSING","包络尺寸无效"};
    }
    // Nav2 已经发布 padded footprint；增加半格余量避免离散步长漏掉格边障碍。
    nav2_costmap_2d::padFootprint(footprint,md.resolution*0.5);
    auto pose=tf.lookupTransform(d.costmap->header.frame_id,base,tf2::TimePointZero);
    if (pose.header.stamp.sec!=0 &&
      (now-rclcpp::Time(pose.header.stamp,now.get_clock_type())).seconds()>age) {
      return {false,"TF_UNAVAILABLE","车体 TF 已过期"};
    }
    // Costmap2D 为轴对齐格网，先把世界坐标转换到地图原点的局部坐标。
    double origin_yaw=tf2::getYaw(md.origin.orientation);
    if (!std::isfinite(origin_yaw)) {origin_yaw=0;}
    nav2_costmap_2d::Costmap2D map(md.size_x,md.size_y,md.resolution,0,0,0);
    std::copy(d.costmap->data.begin(),d.costmap->data.end(),map.getCharMap());
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(&map);
    const double yaw=tf2::getYaw(pose.transform.rotation);
    const int steps=std::max(1,int(std::ceil(std::hypot(dx,dy)/(md.resolution*0.5))));
    // 倒车时忽略当前包络已压住的前方格子，只拦新进入的后方体积。
    const bool escape_rear=dx<0 && std::abs(dx)>=std::abs(dy);
    std::unordered_set<uint64_t> start_cells;
    for (int i=0;i<=steps;++i) {
      double t=double(i)/steps;
      double wx=pose.transform.translation.x+std::cos(yaw)*dx*t-std::sin(yaw)*dy*t;
      double wy=pose.transform.translation.y+std::sin(yaw)*dx*t+std::cos(yaw)*dy*t;
      wx-=md.origin.position.x; wy-=md.origin.position.y;
      double mx=std::cos(origin_yaw)*wx+std::sin(origin_yaw)*wy;
      double my=-std::sin(origin_yaw)*wx+std::cos(origin_yaw)*wy;
      nav2_costmap_2d::Footprint transformed;
      nav2_costmap_2d::transformFootprint(mx,my,yaw-origin_yaw,footprint,transformed);
      std::vector<nav2_costmap_2d::MapLocation> polygon,cells;
      for (const auto & p : transformed) {
        nav2_costmap_2d::MapLocation loc;
        if (!map.worldToMap(p.x,p.y,loc.x,loc.y)) {
          return {false,"COSTMAP_OUT_OF_BOUNDS","完整包络扫掠越出局部地图"};
        }
        polygon.push_back(loc);
      }
      double outline=checker.footprintCost(transformed);
      map.convexFillCells(polygon,cells);
      if (i==0) {
        for (const auto & cell : cells) {
          start_cells.insert((uint64_t(cell.x)<<32)|cell.y);
        }
        if (escape_rear) {continue;}
      }
      for (const auto & cell : cells) {
        if (escape_rear && start_cells.count((uint64_t(cell.x)<<32)|cell.y)) {continue;}
        auto cost=map.getCost(cell.x,cell.y);
        if (cost==255) {return {false,"COSTMAP_UNKNOWN","扫掠区包含未知格"};}
        if (cost==254) {return {false,"COSTMAP_OBSTACLE","扫掠区包含致命障碍"};}
      }
      if (!escape_rear && outline>=254) {return {false,"COSTMAP_OBSTACLE","包络轮廓碰撞"};}
    }
    std::vector<geometry_msgs::msg::Point> points;
    if (!scan_points_in_base(*d.scan,tf,base,points)) {
      return {false,"TF_UNAVAILABLE","扫描格式或扫描时刻安装 TF 不可用"};
    }
    for (const auto & p : points) {
      const bool in_y=p.y>=std::min(0.0,dy)-half_w-0.02 && p.y<=std::max(0.0,dy)+half_w+0.02;
      if (escape_rear) {
        if (in_y && p.x< -half_l+md.resolution && p.x>=dx-half_l-0.02) {
          return {false,"SCAN_OBSTACLE","扫描点进入包络扫掠区"};
        }
      } else if (in_y && p.x>=std::min(0.0,dx)-half_l-0.02 && p.x<=std::max(0.0,dx)+half_l+0.02) {
        return {false,"SCAN_OBSTACLE","扫描点进入包络扫掠区"};
      }
    }
    return {true,"NONE","数据新鲜、TF 有效、完整包络扫掠通过"};
  } catch (const tf2::TransformException & e) {return {false,"TF_UNAVAILABLE",e.what()};}
}

std::shared_ptr<RecoveryRuntime> RecoveryRuntime::get(const BT::NodeConfiguration & config)
{
  std::shared_ptr<RecoveryRuntime> runtime;
  if (config.blackboard->get("carcar_recovery_runtime",runtime)) {return runtime;}
  auto node=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  // 单目标/多目标是两个不同的 blackboard 节点；回调组不能跨节点复用。
  const std::string key=node->get_fully_qualified_name()+
    std::to_string(reinterpret_cast<uintptr_t>(node->get_node_base_interface().get()));
  static std::mutex registry_mutex;
  static std::map<std::string,std::weak_ptr<RecoveryRuntime>> registry;
  std::lock_guard<std::mutex> lock(registry_mutex);
  runtime=registry[key].lock();
  if (!runtime) {runtime=std::make_shared<RecoveryRuntime>(node); registry[key]=runtime;}
  config.blackboard->set("carcar_recovery_runtime",runtime);
  return runtime;
}
RecoveryRuntime::RecoveryRuntime(rclcpp::Node::SharedPtr n) : node(n),tf(n->get_clock())
{
  node->get_parameter_or("robot_base_frame",base_frame,base_frame);
  node->get_parameter_or("global_frame",global_frame,global_frame);
  response_timeout=parameter(node,"recovery.response_timeout",1);
  cancel_timeout=parameter(node,"recovery.cancel_timeout",1);
  settle_timeout=parameter(node,"recovery.settle_timeout",2);
  still_duration=parameter(node,"recovery.still_duration",1);
  data_age=parameter(node,"recovery.max_data_age",0.5);
  recovery_timeout=parameter(node,"recovery.total_timeout",90);
  for (double v : {response_timeout,cancel_timeout,settle_timeout,still_duration,data_age,recovery_timeout}) {
    if (!std::isfinite(v) || v<=0) {throw std::invalid_argument("恢复时限必须为正有限数");}
  }
  group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
  executor_=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(group,node->get_node_base_interface());
  listener_=std::make_unique<tf2_ros::TransformListener>(tf);
  rclcpp::SubscriptionOptions options; options.callback_group=group;
  health_sub_=node->create_subscription<std_msgs::msg::Bool>("/localization_monitor/ready",
    rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      ready_.received=true; ready_.value=msg->data; ready_.received_at=Steady::now();
    },options);
  recovery_allowed_sub_=node->create_subscription<std_msgs::msg::Bool>("/localization_monitor/recovery_allowed",
    rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      recovery_allowed_.received=true; recovery_allowed_.value=msg->data;
      recovery_allowed_.received_at=Steady::now();
    },options);
  odom_sub_=node->create_subscription<nav_msgs::msg::Odometry>("/wheel/odometry",
    rclcpp::QoS(1).best_effort(),[this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      bool gap=odom_received_==TimePoint{} || seconds(odom_received_)>data_age;
      odom_=msg; odom_received_=Steady::now();
      const auto & v=msg->twist.twist;
      bool stopped=std::hypot(v.linear.x,v.linear.y)<0.01 && std::abs(v.angular.z)<0.02;
      if (!stopped || gap) {still_since_=TimePoint{};}
      if (stopped && still_since_==TimePoint{}) {still_since_=Steady::now();}
    },options);
  pose_feedback_=node->create_subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>(
    "/navigate_to_pose/_action/feedback",rclcpp::QoS(1).best_effort(),[this](nav2_msgs::action::NavigateToPose_FeedbackMessage::ConstSharedPtr msg) {
      observe_uuid(uuid_text(msg->goal_id.uuid),false);
    },options);
  through_feedback_=node->create_subscription<nav2_msgs::action::NavigateThroughPoses_FeedbackMessage>(
    "/navigate_through_poses/_action/feedback",rclcpp::QoS(1).best_effort(),[this](nav2_msgs::action::NavigateThroughPoses_FeedbackMessage::ConstSharedPtr msg) {
      observe_uuid(uuid_text(msg->goal_id.uuid),true);
    },options);
  permit_pub_=node->create_publisher<std_msgs::msg::UInt64>("/navigation/motion_permit",rclcpp::QoS(1));
  diag_pub_=node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/recovery/status",rclcpp::QoS(10));
  sensors();
  diag_timer_=node->create_wall_timer(std::chrono::seconds(1),[this] {
    publish_recovery_status();
  },group);
  thread_=std::make_unique<nav2_util::NodeThread>(executor_);
}
RecoveryRuntime::~RecoveryRuntime() {
  permit(false);
  if (motion) {motion->cancel();}
  for (auto & session : retiring) {session->cancel();}
  thread_.reset(); // 回调彻底停止后，才允许成员缓存与订阅析构。
  motion.reset(); retiring.clear();
}
std::shared_ptr<SensorCache> RecoveryRuntime::sensors(const std::string & scan,
  const std::string & costmap,const std::string & footprint)
{
  const std::string key=scan+"|"+costmap+"|"+footprint;
  auto it=caches_.find(key);
  if (it==caches_.end()) {
    it=caches_.emplace(key,std::make_shared<SensorCache>(node,group,scan,costmap,footprint)).first;
  }
  return it->second;
}
bool RecoveryRuntime::healthy() const {return localization_ready().healthy();}
HealthSample RecoveryRuntime::localization_ready() const {
  std::lock_guard<std::mutex> lock(mutex_); return ready_;
}
HealthSample RecoveryRuntime::recovery_allowed() const {
  std::lock_guard<std::mutex> lock(mutex_); return recovery_allowed_;
}
void RecoveryRuntime::note_tick() {
  std::lock_guard<std::mutex> lock(mutex_); last_bt_tick_=Steady::now();
}
bool RecoveryRuntime::odom_fresh() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return odom_ && fresh_stamp(odom_->header.stamp,node->now(),odom_received_,data_age);
}
bool RecoveryRuntime::still(double duration) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return odom_ && fresh_stamp(odom_->header.stamp,node->now(),odom_received_,data_age) &&
    still_since_!=TimePoint{} && seconds(still_since_)>=duration;
}
void RecoveryRuntime::restart_stillness() {
  std::lock_guard<std::mutex> lock(mutex_); still_since_=TimePoint{};
}
nav_msgs::msg::Odometry::ConstSharedPtr RecoveryRuntime::odom() const {
  std::lock_guard<std::mutex> lock(mutex_); return odom_;
}
void RecoveryRuntime::observe_uuid(const std::string & id,bool through) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto & current=through ? through_uuid_ : pose_uuid_;
  if (std::find(retired_ids_.begin(),retired_ids_.end(),id)!=retired_ids_.end()) {return;}
  if (id!=current) {
    if (!current.empty()) {retired_ids_.push_back(current);}
    current=id;
  }
}
std::string RecoveryRuntime::goal_uuid(bool through) const {
  std::lock_guard<std::mutex> lock(mutex_); return through ? through_uuid_ : pose_uuid_;
}
bool RecoveryRuntime::pose(geometry_msgs::msg::PoseStamped & result,const std::string & frame) {
  try {
    auto t=tf.lookupTransform(frame.empty()?global_frame:frame,base_frame,tf2::TimePointZero);
    if (t.header.stamp.sec!=0 && (node->now()-rclcpp::Time(t.header.stamp)).seconds()>data_age) {return false;}
    result.header=t.header; result.pose.orientation=t.transform.rotation;
    result.pose.position.x=t.transform.translation.x; result.pose.position.y=t.transform.translation.y;
    return true;
  } catch (const tf2::TransformException &) {return false;}
}
SafetyResult RecoveryRuntime::inputs_ready() {
  if (!healthy()) {return {false,"LOCALIZATION_UNHEALTHY","定位门控未就绪或过期"};}
  if (!odom_fresh()) {return {false,"ODOM_EXPIRED","轮式里程计缺失或过期"};}
  auto d=sensors()->snapshot();
  if (!d.scan || !d.costmap || !d.footprint) {return {false,"DATA_MISSING","恢复输入尚未齐备"};}
  if (!fresh_stamp(d.scan->header.stamp,node->now(),d.scan_received,data_age) ||
    !fresh_stamp(d.costmap->header.stamp,node->now(),d.costmap_received,data_age) ||
    !fresh_stamp(d.footprint->header.stamp,node->now(),d.footprint_received,data_age)) {
    return {false,"DATA_EXPIRED","恢复输入已过期"};
  }
  geometry_msgs::msg::PoseStamped p;
  if (!pose(p)) {return {false,"TF_UNAVAILABLE","导航车体 TF 不可用"};}
  std::vector<geometry_msgs::msg::Point> points;
  if (!scan_points_in_base(*d.scan,tf,base_frame,points) ||
    d.footprint->polygon.points.size()<3 || d.costmap->metadata.resolution<=0 ||
    size_t(d.costmap->metadata.size_x)*d.costmap->metadata.size_y!=d.costmap->data.size() ||
    d.costmap->data.empty()) {
    return {false,"DATA_INVALID","扫描 TF、包络或地图格式无效"};
  }
  try {
    tf.lookupTransform(base_frame,d.footprint->header.frame_id,
      tf2_ros::fromRclcpp(rclcpp::Time(d.footprint->header.stamp)));
    tf.lookupTransform(d.costmap->header.frame_id,base_frame,tf2::TimePointZero);
  } catch (const tf2::TransformException &) {return {false,"TF_UNAVAILABLE","包络或地图 TF 不可用"};}
  return {true,"NONE","定位、传感器与里程计就绪"};
}
void RecoveryRuntime::permit(bool allowed) {
  std_msgs::msg::UInt64 msg; msg.data=allowed && !fault ? action_sequence : 0;
  if (permit_pub_) {permit_pub_->publish(msg);}
}
void RecoveryRuntime::diagnostic(const std::string & phase,const std::string & reason) {
  DiagSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase+reason==last_diag_key_ && last_diag_!=TimePoint{} && seconds(last_diag_)<1) {return;}
    last_diag_=Steady::now(); last_diag_key_=phase+reason;
    snap.phase=phase; snap.reason=reason; snap.fault=fault; snap.epoch=epoch;
    snap.navigation_uuid=navigation_uuid; snap.previous_navigation_uuid=previous_navigation_uuid;
    snap.backup_used=backup_used;
    snap.recovery_elapsed=recovery_active?seconds(recovery_started):0;
    snap.recovery_deadline=recovery_timeout; snap.last_bt_tick=last_bt_tick_;
    snap.motion=motion;
    diag_snapshot_=snap;
  }
  emit_recovery_status(std::move(snap));
}
void RecoveryRuntime::publish_recovery_status() {
  DiagSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_bt_tick_==TimePoint{} || diag_snapshot_.phase.empty()) {return;}
    snap=diag_snapshot_;
    snap.last_bt_tick=last_bt_tick_;
  }
  emit_recovery_status(std::move(snap));
}
void RecoveryRuntime::emit_recovery_status(DiagSnapshot snap) {
  diagnostic_msgs::msg::DiagnosticStatus msg;
  msg.name="RecoverySupervisor"; msg.hardware_id="carcar_nav_bt_nodes";
  msg.level=snap.fault?2:0; msg.message=snap.phase;
  auto add=[&](std::string key,std::string value) {
    diagnostic_msgs::msg::KeyValue kv; kv.key=key; kv.value=value; msg.values.push_back(kv);
  };
  add("phase",snap.phase); add("reason_code",snap.reason); add("epoch",std::to_string(snap.epoch));
  add("navigation_uuid",snap.navigation_uuid);add("previous_navigation_uuid",snap.previous_navigation_uuid);
  add("backup_reserved",std::to_string(snap.backup_used));
  add("recovery_elapsed",std::to_string(snap.recovery_elapsed));
  add("recovery_deadline_s",std::to_string(snap.recovery_deadline));
  add("bt_tick_age_s",snap.last_bt_tick==TimePoint{}?"0":std::to_string(seconds(snap.last_bt_tick)));
  auto d=sensors()->snapshot();
  add("scan_count",std::to_string(d.scan_count)); add("costmap_count",std::to_string(d.costmap_count));
  add("footprint_count",std::to_string(d.footprint_count));
  if (snap.motion) {
    auto s=snap.motion->state(); add("action_uuid",s.uuid); add("action_elapsed",std::to_string(s.elapsed));
    add("action_revision",std::to_string(s.revision));
    add("response_remaining_s",std::to_string(s.response_remaining));
    add("result_remaining_s",std::to_string(s.result_remaining));
    add("cancel_remaining_s",std::to_string(s.cancel_remaining));
    add("cancel_state",s.cancel_requested?(s.cancel_ack?"ACKNOWLEDGED":"REQUESTED"):"NONE");
  }
  diag_pub_->publish(msg);
}
void RecoveryRuntime::fail(const std::string & reason) {
  fault=true; fault_reason=reason; permit(false); diagnostic("FAILED",reason);
}
void RecoveryRuntime::begin_recovery() {
  if (!recovery_active) {recovery_active=true; recovery_started=Steady::now(); forward_distance=0;}
}
void RecoveryRuntime::forward_progress(double distance) {
  if (!std::isfinite(distance)) {return;}
  forward_distance=std::max(0.0,forward_distance+distance);
  if (forward_distance>=0.20) {
    backup_used=0; spin_used=false; escape_stage=0; observe_started=TimePoint{};
    forward_distance=0; recovery_active=false;
  }
}
bool RecoveryRuntime::reserve_backup(double distance) {
  if (!std::isfinite(distance) || distance<=0 || distance>0.100001 || backup_used+distance>0.300001) {return false;}
  backup_used+=distance; forward_distance=0; return true;
}
void RecoveryRuntime::release_backup(double distance) {backup_used=std::max(0.0,backup_used-distance);}
}  // namespace carcar_navigation
