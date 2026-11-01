#pragma once
#include <map>
#include <string>
#include <nlohmann/json.hpp>

namespace carcar_mapping_console {
// 任务会话可以共享；手动持有者只有一个。公开设备标识与操纵令牌分离。
class OperatorSessions {
public:
  bool connect(const std::string&id,const std::string&public_id){
    if(id.empty()||clients_.count(id))return false;
    clients_.emplace(id,Client{public_id,"设备"+std::to_string(++next_label_)});
    if(owner_.empty()&&clients_.size()==1){owner_=id;++generation_;return true;}
    return false;
  }
  bool connected(const std::string&id)const{return !id.empty()&&clients_.count(id);}
  bool owns(const std::string&id)const{return connected(id)&&owner_==id;}
  bool disconnect(const std::string&id){
    clients_.erase(id);if(owner_!=id)return false;owner_.clear();++generation_;return true;
  }
  bool claim(const std::string&id){
    if(!connected(id))return false;
    if(owner_!=id){owner_=id;++generation_;}return true;
  }
  void label(const std::string&id,const std::string&label){
    if(connected(id)&&!label.empty()&&label.size()<=48)clients_.at(id).label=label;
  }
  const std::string&owner()const{return owner_;}
  std::size_t size()const{return clients_.size();}
  nlohmann::json status(const std::string&id={})const{
    nlohmann::json result{{"protocol",1},{"shared_tasks",true},{"connected_clients",clients_.size()},
      {"generation",generation_},{"owner_id",""},{"owner_label",""}};
    if(clients_.count(owner_)){result["owner_id"]=clients_.at(owner_).public_id;result["owner_label"]=clients_.at(owner_).label;}
    if(connected(id)){result["client_id"]=clients_.at(id).public_id;result["is_owner"]=owns(id);}
    return result;
  }
private:
  struct Client{std::string public_id,label;};
  std::map<std::string,Client>clients_;std::string owner_;std::uint64_t generation_=0,next_label_=0;
};
}
