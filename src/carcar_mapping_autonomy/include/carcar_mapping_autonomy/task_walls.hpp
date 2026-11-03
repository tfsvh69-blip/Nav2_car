#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include "carcar_mapping_autonomy/map_geometry.hpp"
#include "carcar_mapping_autonomy/map_fingerprint.hpp"

namespace carcar_mapping_autonomy {
struct TaskWall {std::string id;std::vector<Pose> points;};
using TaskWallSet=std::vector<TaskWall>;

inline bool task_wall_id_valid(const std::string&id){
  return !id.empty()&&id.size()<=128&&id.find_first_not_of(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")==std::string::npos;
}
inline TaskWallSet parse_task_walls(const Json&value){
  if(!value.is_array()||value.size()>32)throw std::invalid_argument("人工墙须为不超过32条的列表");
  TaskWallSet walls;std::set<std::string>ids;size_t vertices=0;
  for(const auto&entry:value){
    if(!entry.is_object()||!entry.contains("id")||!entry["id"].is_string()||
       !task_wall_id_valid(entry["id"].get<std::string>()))throw std::invalid_argument("人工墙ID无效");
    TaskWall wall;wall.id=entry["id"].get<std::string>();
    if(!ids.insert(wall.id).second)throw std::invalid_argument("人工墙ID重复");
    if(!entry.contains("points")||!entry["points"].is_array()||entry["points"].size()<2||
       entry["points"].size()>64)throw std::invalid_argument("每条人工墙需要2至64个顶点");
    vertices+=entry["points"].size();if(vertices>256)throw std::invalid_argument("人工墙顶点总数超过256");
    for(const auto&point:entry["points"]){
      if(!point.is_array()||point.size()!=2||!point[0].is_number()||!point[1].is_number())
        throw std::invalid_argument("人工墙顶点须为map坐标中的[x米,y米]");
      const Pose p{point[0].get<double>(),point[1].get<double>(),0};
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||std::abs(p.x)>1e6||std::abs(p.y)>1e6)
        throw std::invalid_argument("人工墙顶点不是有界有限坐标");
      if(!wall.points.empty()&&std::hypot(p.x-wall.points.back().x,p.y-wall.points.back().y)<1e-9)
        throw std::invalid_argument("人工墙相邻顶点不能重合");
      wall.points.push_back(p);
    }
    walls.push_back(std::move(wall));
  }
  return walls;
}
inline Json task_walls_json(const TaskWallSet&walls){
  Json result=Json::array();
  for(const auto&wall:walls){Json points=Json::array();for(const auto&p:wall.points)points.push_back({p.x,p.y});
    result.push_back({{"id",wall.id},{"points",std::move(points)}});}
  return result;
}
namespace task_wall_detail {
inline double point_segment_distance(const Pose&p,const Pose&a,const Pose&b){
  const double dx=b.x-a.x,dy=b.y-a.y,length=dx*dx+dy*dy;
  const double t=length>0?std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/length,0.,1.):0;
  return std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
}
inline double cross(const Pose&a,const Pose&b,const Pose&c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
inline bool intersects(const Pose&a,const Pose&b,const Pose&c,const Pose&d){
  const double ab_c=cross(a,b,c),ab_d=cross(a,b,d),cd_a=cross(c,d,a),cd_b=cross(c,d,b);
  // 端点和共线接触由距离补验，避免只用朝向乘积把分离的共线段判成交叉。
  return ((ab_c>0&&ab_d<0)||(ab_c<0&&ab_d>0))&&((cd_a>0&&cd_b<0)||(cd_a<0&&cd_b>0));
}
inline Pose grid_point(const nav_msgs::msg::OccupancyGrid&map,const Pose&p){
  const double angle=grid_yaw(map),c=std::cos(angle),s=std::sin(angle);
  const double x=p.x-map.info.origin.position.x,y=p.y-map.info.origin.position.y;
  return {(c*x+s*y)/map.info.resolution,(-s*x+c*y)/map.info.resolution,0};
}
inline bool in_grid(const nav_msgs::msg::OccupancyGrid&map,const Pose&p){
  return p.x>=-1e-9&&p.y>=-1e-9&&p.x<=map.info.width+1e-9&&p.y<=map.info.height+1e-9;
}
inline void validate_map(const nav_msgs::msg::OccupancyGrid&map){
  if(!grid_valid(map)||map.data.size()>4000000||map.info.width>65536||map.info.height>65536||
     !std::isfinite(map.info.origin.position.z))throw std::invalid_argument("人工墙等待有效且不超过400万格的二维原图");
  if(std::any_of(map.data.begin(),map.data.end(),[](int8_t value){return value< -1||value>100;}))
    throw std::invalid_argument("原图包含无效占用值");
}
// 超覆盖DDA包含线穿过的每个闭格；角点触及的两侧格和沿格线的两侧格都保留。
inline void raster_segment(nav_msgs::msg::OccupancyGrid&mask,Pose a,Pose b,size_t&work){
  const auto w=static_cast<int>(mask.info.width),h=static_cast<int>(mask.info.height);
  a.x=std::clamp(a.x,0.,double(w));a.y=std::clamp(a.y,0.,double(h));
  b.x=std::clamp(b.x,0.,double(w));b.y=std::clamp(b.y,0.,double(h));
  auto mark=[&](int x,int y){if(++work>1000000)throw std::invalid_argument("人工墙栅格化超过有界工作量");
    if(x>=0&&y>=0&&x<w&&y<h)mask.data[static_cast<size_t>(y)*w+x]=100;};
  auto integer=[](double v){return std::abs(v-std::round(v))<=1e-10;};
  auto endpoint=[&](const Pose&p){const int x=std::floor(p.x),y=std::floor(p.y);mark(x,y);
    if(integer(p.x))mark(x-1,y);
    if(integer(p.y))mark(x,y-1);
    if(integer(p.x)&&integer(p.y))mark(x-1,y-1);
  };
  endpoint(a);endpoint(b);
  const double dx=b.x-a.x,dy=b.y-a.y;
  const int sx=(dx>0)-(dx<0),sy=(dy>0)-(dy<0);
  int x=std::floor(a.x),y=std::floor(a.y);
  const bool along_x=sx==0&&integer(a.x),along_y=sy==0&&integer(a.y);
  auto covered=[&]{mark(x,y);if(along_x)mark(x-1,y);if(along_y)mark(x,y-1);if(along_x&&along_y)mark(x-1,y-1);};
  const double infinity=std::numeric_limits<double>::infinity();
  double tx=sx?((sx>0?x+1.:x)-a.x)/dx:infinity;
  double ty=sy?((sy>0?y+1.:y)-a.y)/dy:infinity;
  const double step_x=sx?1/std::abs(dx):infinity,step_y=sy?1/std::abs(dy):infinity;
  covered();
  while(std::min(tx,ty)<=1+1e-12){
    if(std::abs(tx-ty)<=1e-12){mark(x+sx,y);mark(x,y+sy);x+=sx;y+=sy;tx+=step_x;ty+=step_y;}
    else if(tx<ty){x+=sx;tx+=step_x;}else{y+=sy;ty+=step_y;}
    covered();
  }
}
}  // namespace task_wall_detail

// 只审查人工禁行边界，不代替原图、原始扫描、制动预测或定位资格检查。
inline bool walls_segment_safe(const TaskWallSet&walls,const Pose&start,const Pose&end,double radius=.22){
  if(!std::isfinite(start.x)||!std::isfinite(start.y)||!std::isfinite(end.x)||!std::isfinite(end.y)||
     !std::isfinite(radius)||radius<0||std::abs(start.x)>1e6||std::abs(start.y)>1e6||
     std::abs(end.x)>1e6||std::abs(end.y)>1e6)return false;
  if(walls.size()>32)return false;
  size_t vertices=0;
  for(const auto&wall:walls){
    vertices+=wall.points.size();if(wall.points.size()<2||wall.points.size()>64||vertices>256)return false;
    for(const auto&point:wall.points)if(!std::isfinite(point.x)||!std::isfinite(point.y)||
      std::abs(point.x)>1e6||std::abs(point.y)>1e6)return false;
    for(size_t i=1;i<wall.points.size();++i){
    const auto&a=wall.points[i-1];const auto&b=wall.points[i];
    if(task_wall_detail::intersects(start,end,a,b))return false;
    const double distance=std::min({task_wall_detail::point_segment_distance(start,a,b),
      task_wall_detail::point_segment_distance(end,a,b),task_wall_detail::point_segment_distance(a,start,end),
      task_wall_detail::point_segment_distance(b,start,end)});
    if(distance<=radius+1e-9)return false;
    }
  }
  return true;
}
inline bool walls_segment_safe(const Json&walls,const Pose&start,const Pose&end,double radius=.22){
  try{return walls_segment_safe(parse_task_walls(walls),start,end,radius);}catch(const std::exception&){return false;}
}
// 手动方向也须覆盖当前实测速度、0.30秒响应延迟、加减速和最终制动；允许双向命令。
inline bool walls_motion_safe(const Json&walls,const Pose&start,const geometry_msgs::msg::Twist&requested,
    const geometry_msgs::msg::Twist&measured){
  if(!std::isfinite(start.yaw)||!std::isfinite(requested.linear.x)||!std::isfinite(requested.angular.z)||
     !std::isfinite(measured.linear.x)||!std::isfinite(measured.linear.y)||!std::isfinite(measured.angular.z)||
     std::abs(requested.linear.x)>.18||std::abs(requested.angular.z)>.36||
     std::abs(measured.linear.x)>.25||std::abs(measured.angular.z)>.6||std::abs(measured.linear.y)>.02)return false;
  try{
    const auto parsed=parse_task_walls(walls);auto pose=start;double v=measured.linear.x,w=measured.angular.z;
    const auto began=std::chrono::steady_clock::now();constexpr double dt=.05;
    for(int step=0;step<50;++step){
      if(step%10==0&&std::chrono::steady_clock::now()-began>std::chrono::milliseconds(8))return false;
      const double t=step*dt;
      if(t+1e-9>=.30){
        v+=std::clamp((t<1.?requested.linear.x:0)-v,-.20*dt,.20*dt);
        w+=std::clamp((t<1.?requested.angular.z:0)-w,-.80*dt,.80*dt);
      }
      auto next=pose;const double angle=w*dt;
      if(std::abs(w)<1e-9){next.x+=v*dt*std::cos(pose.yaw);next.y+=v*dt*std::sin(pose.yaw);}
      else{next.x+=v/w*(std::sin(pose.yaw+angle)-std::sin(pose.yaw));next.y+=v/w*(std::cos(pose.yaw)-std::cos(pose.yaw+angle));}
      next.yaw=std::remainder(pose.yaw+angle,2*M_PI);
      const double sagitta=std::abs(w)<1e-9?0:std::abs(v/w)*(1-std::cos(std::abs(angle)/2));
      if(!walls_segment_safe(parsed,pose,next,.22+sagitta))return false;
      pose=next;if(t>=1.&&std::abs(v)+std::abs(w)<1e-9)return true;
    }
  }catch(const std::exception&){}
  return false;
}

struct TaskWallMaps {nav_msgs::msg::OccupancyGrid navigation,mask;};
inline TaskWallMaps rasterize_task_walls(const nav_msgs::msg::OccupancyGrid&map,const TaskWallSet&walls){
  task_wall_detail::validate_map(map);TaskWallMaps result{map,map};result.mask.data.assign(map.data.size(),0);
  // 类型化入口也核验边界，避免调用方绕过JSON入口后带入非有限数值。
  if(walls.size()>32)throw std::invalid_argument("人工墙条数超过32");
  size_t vertices=0;
  size_t work=0;
  for(const auto&wall:walls){
    vertices+=wall.points.size();
    if(wall.points.size()<2||wall.points.size()>64||vertices>256)throw std::invalid_argument("人工墙顶点数量无效");
    for(const auto&p:wall.points)if(!task_wall_detail::in_grid(map,task_wall_detail::grid_point(map,p)))
      throw std::invalid_argument("人工墙顶点超出当前原图范围，请复核墙位置");
    for(size_t i=1;i<wall.points.size();++i)task_wall_detail::raster_segment(result.mask,
      task_wall_detail::grid_point(map,wall.points[i-1]),task_wall_detail::grid_point(map,wall.points[i]),work);
  }
  for(size_t i=0;i<result.mask.data.size();++i)if(result.mask.data[i])result.navigation.data[i]=100;
  return result;
}

// 仅保存本进程所属task；不读取工程文件，不序列化到地图资产。
class TaskScene {
public:
  explicit TaskScene(std::string task_id):task_id_(std::move(task_id)){
    if(!task_wall_id_valid(task_id_))throw std::invalid_argument("场景节点需要有效task_id");
  }
  void generation(uint64_t value){
    if(have_generation_&&value<generation_)return;
    if(have_generation_&&value==generation_)return;
    generation_=value;have_generation_=true;
    if(!walls_.empty()){review_=true;review_map_after_=map_sequence_;phase_="REVIEW_REQUIRED";
      failure_="导航坐标代次已变化，请等待新图并停稳复核人工墙后确认";}
    else rebuild();
  }
  void map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr value){
    try{if(!value)throw std::invalid_argument("原图为空");task_wall_detail::validate_map(*value);}
    catch(const std::exception&e){map_valid_=false;phase_="FAILED";failure_=e.what();return;}
    map_=std::move(value);map_valid_=true;++map_sequence_;source_fingerprint_=map_content_fingerprint(*map_);
    if(review_){phase_="REVIEW_REQUIRED";return;}
    rebuild();
  }
  void stale(){map_valid_=false;phase_="FAILED";failure_="原始地图超过10秒未更新，保持停车";}
  bool request(const Json&input){
    std::string id;bool accepted=false;std::string error;
    try{
      if(!input.is_object()||!input.contains("request_id")||!input["request_id"].is_string()||
         !task_wall_id_valid(input["request_id"].get<std::string>()))throw std::invalid_argument("人工墙request_id无效");
      id=input["request_id"].get<std::string>();
      if(input.value("task_id",std::string())!=task_id_)throw std::invalid_argument("人工墙请求属于其他task，未应用");
      if(!have_generation_||unsigned_value(input,"navigation_generation")!=generation_)
        throw std::invalid_argument("人工墙请求的导航坐标代次已过期，未应用");
      const auto canonical=input.dump();
      for(const auto&record:requests_)if(record.id==id){
        if(record.payload!=canonical)throw std::invalid_argument("同一request_id不能修改人工墙请求");
        last_request_=id;last_result_=record.accepted?"accepted":"rejected";request_error_=record.error;
        return record.accepted;
      }
      const auto action=input.value("action",std::string());
      if(action=="replace"){
        if(unsigned_value(input,"expected_revision")!=revision_)throw std::invalid_argument("人工墙revision冲突，未覆盖已有墙");
        auto next=parse_task_walls(input.at("walls"));
        if(!map_||!map_valid_)throw std::invalid_argument("等待有效原图，未修改人工墙");
        auto rendered=rasterize_task_walls(*map_,next);
        commit(std::move(rendered),&next);
      }else if(action=="confirm"){
        if(unsigned_value(input,"revision")!=revision_)throw std::invalid_argument("人工墙确认revision已过期");
        if(!review_)throw std::invalid_argument("当前人工墙不需要重新确认");
        if(!map_||!map_valid_)throw std::invalid_argument("等待有效原图，未确认人工墙");
        if(map_sequence_<=review_map_after_)throw std::invalid_argument("等待导航代次变化后的原图，未确认旧图上的人工墙");
        auto rendered=rasterize_task_walls(*map_,walls_);commit(std::move(rendered));
      }else throw std::invalid_argument("人工墙action仅支持replace或confirm");
      accepted=true;
    }catch(const std::exception&e){error=e.what();}
    last_request_=id;last_result_=accepted?"accepted":"rejected";request_error_=error;
    if(!id.empty()&&input.is_object()){
      requests_.push_back({id,input.dump(),error,accepted});if(requests_.size()>32)requests_.pop_front();
    }
    return accepted;
  }
  void invalid_request(const std::string&error){last_request_.clear();last_result_="rejected";request_error_=error;}
  Json state()const{
    const auto walls=task_walls_json(walls_);
    Json result{{"task_id",task_id_},{"revision",revision_},{"wall_revision",revision_},
      {"navigation_generation",generation_},{"phase",phase_},{"needs_confirmation",review_},
      {"walls",walls},{"items",walls},{"request_id",last_request_},{"request_result",last_result_},
      {"error",request_error_.empty()?failure_:request_error_},{"request_error",request_error_},
      {"reason",failure_},{"map_ready",ready()},{"generation_received",have_generation_},
      {"applied_navigation_generation",applied_generation_},{"applied_revision",applied_revision_},
      {"output_sequence",output_sequence_},{"source_map_sequence",map_sequence_},{"temporary",true}};
    if(ready())result["navigation_fingerprint"]=std::to_string(navigation_fingerprint_);
    if(map_){result["source_fingerprint"]=std::to_string(source_fingerprint_);
      result["source_map_stamp_ns"]=int64_t(map_->header.stamp.sec)*1000000000+map_->header.stamp.nanosec;
      result["source_geometry"]={{"frame_id",map_->header.frame_id},{"width",map_->info.width},
        {"height",map_->info.height},{"resolution",map_->info.resolution},
        {"origin",{map_->info.origin.position.x,map_->info.origin.position.y,grid_yaw(*map_)}}};}
    return result;
  }
  bool ready()const{return phase_=="READY"&&map_valid_&&have_generation_;}
  uint64_t output_sequence()const{return output_sequence_;}
  const std::shared_ptr<const TaskWallMaps>&output()const{return output_;}
private:
  struct Request {std::string id,payload,error;bool accepted;};
  static uint64_t unsigned_value(const Json&input,const char*key){
    if(!input.contains(key)||!input[key].is_number_integer()||
       (!input[key].is_number_unsigned()&&input[key].get<int64_t>()<0))
      throw std::invalid_argument(std::string("人工墙字段须为非负整数：")+key);
    return input[key].get<uint64_t>();
  }
  void commit(TaskWallMaps rendered,TaskWallSet*next=nullptr){
    // 全部可能分配内存的准备完成后再提交墙、revision和输出，失败不能出现半次修改。
    const auto fingerprint=map_content_fingerprint(rendered.navigation);
    auto output=std::make_shared<const TaskWallMaps>(std::move(rendered));
    if(next){walls_.swap(*next);++revision_;}
    output_=std::move(output);navigation_fingerprint_=fingerprint;++output_sequence_;review_=false;
    applied_generation_=generation_;applied_revision_=revision_;phase_="READY";failure_.clear();
  }
  void rebuild(){
    if(!map_||!map_valid_||!have_generation_){phase_="FAILED";failure_="等待原始地图和导航坐标代次";return;}
    try{commit(rasterize_task_walls(*map_,walls_));}
    catch(const std::exception&e){review_=!walls_.empty();phase_=review_?"REVIEW_REQUIRED":"FAILED";failure_=e.what();}
  }
  std::string task_id_,phase_="FAILED",failure_="等待原始地图和导航坐标代次",last_request_,last_result_,request_error_;
  uint64_t generation_=0,revision_=0,applied_generation_=0,applied_revision_=0,output_sequence_=0;
  uint64_t source_fingerprint_=0,navigation_fingerprint_=0;
  uint64_t map_sequence_=0,review_map_after_=0;
  bool have_generation_=false,map_valid_=false,review_=false;
  TaskWallSet walls_;std::deque<Request>requests_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::shared_ptr<const TaskWallMaps>output_;
};
}  // namespace carcar_mapping_autonomy
