#pragma once
#include <functional>
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
namespace carcar_mapping_console {
struct HttpRequest {std::string method, target, body;};
struct HttpResponse {
  unsigned status = 200;
  std::string type = "application/json; charset=utf-8";
  std::vector<unsigned char> body;
  std::map<std::string, std::string> headers;
};
class HttpServer {
public:
  using Handler = std::function<HttpResponse(const HttpRequest &)>;
  // 第二参数为空表示断开；会话 ID 仅由服务端产生。
  using Control = std::function<void(const std::string &, const std::string &)>;
  HttpServer(const std::string & address, unsigned short port, Handler, Control);
  ~HttpServer();
  void broadcast(const std::string & state);
  void set_inspect_upstream(const std::string & host, unsigned short port);
  nlohmann::json diagnostics() const;
  // 等待HTTP工作退出并清理回调；调用方状态须保持存活至本方法返回。
  void stop();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace carcar_mapping_console
