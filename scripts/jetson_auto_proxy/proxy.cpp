// 项目自有代码；TLS/HTTP健康检查复用系统libcurl，不解密转发的业务TLS。
#include <curl/curl.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <csignal>
#include <fstream>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace proxy {
using Clock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;
std::atomic<bool> stopping{false};
static_assert(std::atomic<bool>::is_always_lock_free, "Signal stop flag must be lock-free");
struct Fd {
  int n = -1;
  explicit Fd(int value = -1) : n(value) {}
  ~Fd() { if (n >= 0) close(n); }
  Fd(const Fd&) = delete;
  Fd& operator=(const Fd&) = delete;
  Fd(Fd&& other) noexcept : n(other.n) { other.n = -1; }
  explicit operator bool() const { return n >= 0; }
};
void log(const std::string& value) {
  static std::mutex mutex;
  std::lock_guard<std::mutex> guard(mutex);
  std::cerr << value << std::endl;
}
std::string ip_text(uint32_t host_order) {
  in_addr address{htonl(host_order)};
  char value[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &address, value, sizeof(value));
  return value;
}
struct Upstream {
  std::string name, host;
  int port;
  std::string key() const { return host + ":" + std::to_string(port); }
  std::string url() const { return "http://" + key(); }
};
struct Config {
  int port = 17892, probe_timeout = 4000, interval = 15000;
  int cooldown = 15000, connect_timeout = 2000, max_clients = 48;
  bool gateways = true;
  std::string probe_url = "https://api.openai.com/v1/models", ca_file;
  std::vector<Upstream> fixed;
};
Config read_config(const std::string& path) {
  Config c;
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot read config: " + path);
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') continue;
    auto pos = line.find('=');
    if (pos == std::string::npos) throw std::runtime_error("Invalid config line");
    auto key = line.substr(0, pos), value = line.substr(pos + 1);
    if (key == "probe_url") c.probe_url = value;
    else if (key == "ca_file") c.ca_file = value;
    else if (key == "upstream") {
      Upstream p; std::istringstream row(value);
      if (!(row >> p.name >> p.host >> p.port)) throw std::runtime_error("Invalid upstream");
      in_addr address{};
      if (inet_pton(AF_INET, p.host.c_str(), &address) != 1 || p.port < 1 || p.port > 65535)
        throw std::runtime_error("Upstream requires numeric IPv4 and valid port");
      c.fixed.push_back(p);
    } else {
      size_t used = 0; int n = std::stoi(value, &used);
      if (used != value.size()) throw std::runtime_error("Invalid numeric value");
      if (key == "listen_port") c.port = n;
      else if (key == "auto_gateways") c.gateways = n != 0;
      else if (key == "probe_timeout_ms") c.probe_timeout = n;
      else if (key == "probe_interval_ms") c.interval = n;
      else if (key == "cooldown_ms") c.cooldown = n;
      else if (key == "connect_timeout_ms") c.connect_timeout = n;
      else if (key == "max_clients") c.max_clients = n;
      else throw std::runtime_error("Unknown config key: " + key);
    }
  }
  if (c.port < 1 || c.port > 65535 || c.probe_timeout < 100 || c.probe_timeout > 30000 ||
      c.interval < 500 || c.interval > 300000 || c.cooldown < 500 || c.cooldown > 300000 ||
      c.connect_timeout < 100 || c.connect_timeout > 10000 || c.max_clients < 1 || c.max_clients > 128 ||
      c.fixed.size() > 16 || c.probe_url.rfind("https://", 0) != 0)
    throw std::runtime_error("Config out of range (HTTPS probe required)");
  return c;
}
std::vector<std::pair<unsigned, std::string>> gateways() {
  std::ifstream file("/proc/net/route");
  std::vector<std::pair<unsigned, std::string>> result;
  std::string line; std::getline(file, line);
  while (std::getline(file, line)) {
    std::istringstream row(line);
    std::string iface, destination, gateway, flags, ref, use;
    unsigned metric;
    if (!(row >> iface >> destination >> gateway >> flags >> ref >> use >> metric)) continue;
    try {
      if (destination != "00000000" || !(std::stoul(flags, nullptr, 16) & 2)) continue;
      // /proc/net/route 使用小端十六进制字节序。
      uint32_t raw = static_cast<uint32_t>(std::stoul(gateway, nullptr, 16));
      uint32_t host = ((raw & 255) << 24) | ((raw & 65280) << 8) |
                      ((raw >> 8) & 65280) | ((raw >> 24) & 255);
      if (host) result.emplace_back(metric, ip_text(host));
    } catch (const std::exception&) { continue; }
  }
  std::sort(result.begin(), result.end());
  return result;
}
std::vector<Upstream> candidates(const Config& c) {
  std::vector<Upstream> result;
  if (c.gateways) for (const auto& g : gateways())
    result.push_back({"Tablet_gateway", g.second, 6880});
  result.insert(result.end(), c.fixed.begin(), c.fixed.end());
  std::set<std::string> seen;
  result.erase(std::remove_if(result.begin(), result.end(), [&](const Upstream& p) {
    return !seen.insert(p.key()).second;
  }), result.end());
  if (result.size() > 24) result.resize(24);
  return result;
}
bool private_ip(uint32_t n) {
  return (n >> 24) == 10 || (n >> 20) == 0xac1 || (n >> 16) == 0xc0a8;
}
std::string no_proxy() {
  std::set<std::string> entries = {"localhost", "127.0.0.1", "::1", "xiaozhu", "xiaozhu.local",
    "192.168.5.0/24", "192.168.5.1", "192.168.5.11", "192.168.5.17", "192.168.5.36"};
  ifaddrs* addresses = nullptr;
  if (getifaddrs(&addresses) == 0) {
    for (auto* it = addresses; it; it = it->ifa_next) {
      if (!it->ifa_addr || !it->ifa_netmask || it->ifa_addr->sa_family != AF_INET ||
          !(it->ifa_flags & IFF_UP) || (it->ifa_flags & IFF_LOOPBACK)) continue;
      auto n = ntohl(reinterpret_cast<sockaddr_in*>(it->ifa_addr)->sin_addr.s_addr);
      auto mask = ntohl(reinterpret_cast<sockaddr_in*>(it->ifa_netmask)->sin_addr.s_addr);
      entries.insert(ip_text(n));
      if (private_ip(n)) {
        uint32_t count = ~mask;
        if (count <= 255) for (uint32_t i = 0; i <= count; ++i)
          entries.insert(ip_text((n & mask) + i));
      }
    }
    freeifaddrs(addresses);
  }
  for (const auto& g : gateways()) entries.insert(g.second);
  std::string result;
  for (const auto& entry : entries) { if (!result.empty()) result += ','; result += entry; }
  return result;
}
struct Health {
  bool ok = false;
  long http = 0;
  CURLcode code = CURLE_FAILED_INIT;
};
Health probe(const Upstream& p, const Config& c) {
  CURL* curl = curl_easy_init();
  if (!curl) return {};
  auto url = p.url();
  curl_easy_setopt(curl, CURLOPT_URL, c.probe_url.c_str());
  curl_easy_setopt(curl, CURLOPT_PROXY, url.c_str());
  curl_easy_setopt(curl, CURLOPT_NOPROXY, ""); // 不受外层环境绕过或递归代理影响。
  curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(c.probe_timeout));
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, static_cast<long>(c.probe_timeout));
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);
  if (!c.ca_file.empty()) curl_easy_setopt(curl, CURLOPT_CAINFO, c.ca_file.c_str());
  Health result;
  result.code = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &result.http);
  result.ok = result.code == CURLE_OK && (result.http == 200 || result.http == 401);
  curl_easy_cleanup(curl);
  return result;
}
struct State {
  Health health;
  Clock::time_point checked{}, blocked{};
  unsigned generation = 0;
};
class Pool {
  Config config_;
  mutable std::mutex mutex_;
  std::vector<Upstream> order_;
  std::map<std::string, State> state_;
 public:
  explicit Pool(Config c) : config_(std::move(c)) {}
  void refresh() {
    auto order = candidates(config_);
    std::vector<std::future<void>> jobs;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      order_ = order;
      for (auto it = state_.begin(); it != state_.end();) {
        bool exists = std::any_of(order.begin(), order.end(), [&](const Upstream& p) { return p.key() == it->first; });
        if (!exists) it = state_.erase(it); else ++it;
      }
    }
    for (const auto& p : order) {
      unsigned generation;
      {
        std::lock_guard<std::mutex> guard(mutex_);
        auto& s = state_[p.key()];
        if (Clock::now() < s.blocked) continue;
        generation = s.generation;
      }
      jobs.push_back(std::async(std::launch::async, [this, p, generation] {
        Health h = probe(p, config_);
        std::lock_guard<std::mutex> guard(mutex_);
        auto& s = state_[p.key()];
        // 旧探测不得覆盖探测期间发生的真实连接失败。
        if (s.generation != generation) return;
        if (s.checked == Clock::time_point{} || s.health.ok != h.ok)
          log("[health] " + p.name + " " + p.key() + (h.ok ? " OK" : " DOWN") +
              " curl=" + std::to_string(h.code) + " http=" + std::to_string(h.http));
        s.health = h; s.checked = Clock::now();
      }));
    }
    for (auto& job : jobs) job.get();
  }
  std::vector<Upstream> ready() const {
    std::lock_guard<std::mutex> guard(mutex_);
    std::vector<Upstream> result;
    auto now = Clock::now();
    for (const auto& p : order_) {
      auto it = state_.find(p.key());
      if (it != state_.end() && it->second.health.ok && now >= it->second.blocked &&
          now - it->second.checked < Ms(config_.interval + config_.probe_timeout + 5000)) result.push_back(p);
    }
    return result;
  }
  void fail(const Upstream& p) {
    std::lock_guard<std::mutex> guard(mutex_);
    auto& s = state_[p.key()];
    s.health.ok = false; s.blocked = Clock::now() + Ms(config_.cooldown); ++s.generation;
    log("[cooldown] " + p.name + " " + p.key());
  }
  std::string status() const {
    std::lock_guard<std::mutex> guard(mutex_);
    std::ostringstream out;
    out << "Local proxy: http://127.0.0.1:" << config_.port << "\n";
    bool selected = false;
    auto now = Clock::now();
    for (const auto& p : order_) {
      auto it = state_.find(p.key());
      if (it == state_.end()) { out << p.name << ' ' << p.key() << " CHECKING\n"; continue; }
      const auto& s = it->second;
      bool fresh = s.checked != Clock::time_point{} && now - s.checked < Ms(config_.interval + config_.probe_timeout + 5000);
      bool usable = fresh && s.health.ok && now >= s.blocked;
      out << p.name << ' ' << p.key() << (usable ? " OK" : " DOWN")
          << " curl=" << s.health.code << " http=" << s.health.http
          << " age_ms=" << (s.checked == Clock::time_point{} ? -1 : std::chrono::duration_cast<Ms>(now-s.checked).count());
      if (now < s.blocked) out << " COOLDOWN";
      if (usable && !selected) { out << " SELECTED"; selected = true; }
      out << '\n';
    }
    if (!selected) out << "NO_HEALTHY_UPSTREAM\n";
    return out.str();
  }
};
void nonblocking(int fd) { fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK); }
bool wait_fd(int fd, short events, Clock::time_point deadline) {
  while (!stopping && Clock::now() < deadline) {
    auto remaining = std::chrono::duration_cast<Ms>(deadline - Clock::now()).count();
    pollfd p{fd, events, 0};
    int n = poll(&p, 1, static_cast<int>(std::min<long long>(remaining + 1, 250)));
    if (n > 0) return (p.revents & events) != 0;
    if (n < 0 && errno != EINTR) return false;
  }
  return false;
}
Fd connect_to(const std::string& host, int port, int timeout) {
  Fd fd(socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0));
  if (!fd) return Fd{};
  nonblocking(fd.n);
  sockaddr_in address{}; address.sin_family = AF_INET; address.sin_port = htons(port);
  if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) return Fd{};
  if (connect(fd.n, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    if (errno != EINPROGRESS || !wait_fd(fd.n, POLLOUT, Clock::now() + Ms(timeout))) return Fd{};
    int error = 0; socklen_t size = sizeof(error);
    if (getsockopt(fd.n, SOL_SOCKET, SO_ERROR, &error, &size) || error) return Fd{};
  }
  int yes = 1, idle = 60, interval = 20, count = 3;
  setsockopt(fd.n, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
  setsockopt(fd.n, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  setsockopt(fd.n, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
  setsockopt(fd.n, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
  return fd;
}
bool send_all(int fd, const std::string& data, int timeout = 2000) {
  size_t offset = 0; auto deadline = Clock::now() + Ms(timeout);
  while (offset < data.size()) {
    if (!wait_fd(fd, POLLOUT, deadline)) return false;
    ssize_t n = send(fd, data.data() + offset, data.size() - offset, MSG_NOSIGNAL);
    if (n > 0) offset += static_cast<size_t>(n);
    else if (n == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) return false;
  }
  return true;
}
bool read_headers(int fd, std::string& data, int timeout) {
  auto deadline = Clock::now() + Ms(timeout);
  while (data.find("\r\n\r\n") == std::string::npos) {
    if (data.size() >= 65536 || !wait_fd(fd, POLLIN, deadline)) return false;
    char buffer[8192]; ssize_t n = recv(fd, buffer, std::min(sizeof(buffer), 65536-data.size()), 0);
    if (n > 0) data.append(buffer, static_cast<size_t>(n));
    else if (n == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) return false;
  }
  return true;
}
void reply(int client, int code, const std::string& reason, const std::string& body = "") {
  send_all(client, "HTTP/1.1 " + std::to_string(code) + " " + reason +
    "\r\nContent-Type: text/plain; charset=utf-8\r\nConnection: close\r\nContent-Length: " +
    std::to_string(body.size()) + "\r\n\r\n" + body);
}
// 双向有界缓冲，处理背压和TCP半关闭；没有应用层空闲断开计时。
void relay(int left, int right) {
  int fds[2]{left, right};
  bool eof[2]{false, false}, shut[2]{false, false};
  std::string pending[2]; // pending[i] 来自i，待写入1-i。
  while (!stopping) {
    for (int i = 0; i < 2; ++i) if (eof[i] && pending[i].empty() && !shut[i]) {
      shutdown(fds[1-i], SHUT_WR); shut[i] = true;
    }
    if (shut[0] && shut[1]) return;
    pollfd p[2]{{left, 0, 0}, {right, 0, 0}};
    for (int i = 0; i < 2; ++i) {
      if (!eof[i] && pending[i].size() < 65536) p[i].events |= POLLIN;
      if (!pending[1-i].empty()) p[i].events |= POLLOUT;
      if (!p[i].events) p[i].fd = -1;
    }
    int count = poll(p, 2, 500);
    if (count < 0) { if (errno == EINTR) continue; return; }
    for (int i = 0; i < 2; ++i) {
      if (p[i].revents & (POLLERR | POLLNVAL)) return;
      if (p[i].revents & POLLOUT) {
        auto& buffer = pending[1-i];
        ssize_t n = send(fds[i], buffer.data(), buffer.size(), MSG_NOSIGNAL);
        if (n > 0) buffer.erase(0, static_cast<size_t>(n));
        else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) return;
      }
      if ((p[i].revents & (POLLIN | POLLHUP)) && !eof[i] && pending[i].size() < 65536) {
        char buffer[16384];
        ssize_t n = recv(fds[i], buffer, std::min(sizeof(buffer), 65536-pending[i].size()), 0);
        if (n > 0) pending[i].append(buffer, static_cast<size_t>(n));
        else if (n == 0) eof[i] = true;
        else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return;
      }
    }
  }
}
void handle(int client, Pool& pool, const Config& config) {
  std::string data;
  if (!read_headers(client, data, 5000)) { reply(client, 400, "Bad Request"); return; }
  std::istringstream first(data.substr(0, data.find("\r\n")));
  std::string method, target, version;
  first >> method >> target >> version;
  if (method == "GET" && target == "/__jetson_proxy_status") {
    reply(client, 200, "OK", pool.status()); return;
  }
  bool tunnel = method == "CONNECT";
  if (version.rfind("HTTP/1.", 0) != 0 || (!tunnel && target.rfind("http://", 0) != 0)) {
    reply(client, 400, "Bad Request"); return;
  }
  auto ready = pool.ready();
  if (ready.empty()) { reply(client, 503, "No Healthy Upstream"); return; }
  auto end = data.find("\r\n\r\n") + 4;
  for (const auto& upstream : ready) {
    Fd remote = connect_to(upstream.host, upstream.port, config.connect_timeout);
    if (!remote) { pool.fail(upstream); continue; }
    if (!tunnel) {
      // 业务HTTP字节一旦送出就不重放；仅TCP尚未连接时可换候选。
      if (send_all(remote.n, data)) relay(client, remote.n);
      return;
    }
    std::string response;
    if (!send_all(remote.n, data.substr(0, end)) || !read_headers(remote.n, response, config.connect_timeout)) {
      pool.fail(upstream); continue;
    }
    std::istringstream status(response); std::string protocol; int code = 0;
    status >> protocol >> code;
    if (protocol.rfind("HTTP/1.", 0) != 0 || code != 200) {
      // CONNECT尚未建立，未发送业务TLS/应用数据，可尝试下一候选。
      if (code == 407 || code >= 500 || code == 0) pool.fail(upstream);
      continue;
    }
    if (!send_all(client, response)) return;
    if (data.size() > end && !send_all(remote.n, data.substr(end))) return;
    relay(client, remote.n);
    return;
  }
  reply(client, 502, "Upstream Connection Failed");
}
Fd listener(int port) {
  Fd fd(socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0));
  if (!fd) throw std::runtime_error("socket failed");
  int yes = 1; setsockopt(fd.n, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in address{}; address.sin_family = AF_INET; address.sin_port = htons(port);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(fd.n, reinterpret_cast<sockaddr*>(&address), sizeof(address)) || listen(fd.n, 128))
    throw std::runtime_error("Cannot bind local listener");
  nonblocking(fd.n); return fd;
}
int serve(const Config& config) {
  Fd server = listener(config.port);
  Pool pool(config);
  struct Worker { std::thread thread; std::shared_ptr<std::atomic<bool>> done; };
  std::vector<Worker> workers;
  workers.reserve(static_cast<size_t>(config.max_clients));
  std::thread checker([&] {
    while (!stopping) {
      try { pool.refresh(); } catch (const std::exception& e) { log(std::string("[probe-error] ") + e.what()); }
      for (int elapsed = 0; !stopping && elapsed < config.interval; elapsed += 100)
        std::this_thread::sleep_for(Ms(100));
    }
  });
  log("[start] 127.0.0.1:" + std::to_string(config.port));
  while (!stopping) {
    for (auto it = workers.begin(); it != workers.end();) {
      if (*it->done) { it->thread.join(); it = workers.erase(it); } else ++it;
    }
    pollfd p{server.n, POLLIN, 0};
    if (poll(&p, 1, 250) <= 0) continue;
    int client = accept4(server.n, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
    if (client < 0) continue;
    if (workers.size() >= static_cast<size_t>(config.max_clients)) { close(client); continue; }
    try {
      auto done = std::make_shared<std::atomic<bool>>(false);
      workers.push_back({std::thread([&, client, done] {
        Fd owned(client);
        try { handle(client, pool, config); } catch (const std::exception&) { reply(client, 502, "Proxy Error"); }
        *done = true;
      }), done});
    } catch (const std::exception&) {
      close(client);
      log("[capacity] Cannot start client worker");
    }
  }
  checker.join();
  for (auto& worker : workers) worker.thread.join();
  return 0;
}
int status(const Config& c) {
  Fd fd = connect_to("127.0.0.1", c.port, 1000);
  if (!fd) { std::cerr << "Local proxy NOT LISTENING\n"; return 1; }
  if (!send_all(fd.n, "GET /__jetson_proxy_status HTTP/1.1\r\nHost: localhost\r\n\r\n")) return 1;
  std::string response;
  if (!read_headers(fd.n, response, 2000)) return 1;
  auto deadline = Clock::now() + Ms(2000);
  while (wait_fd(fd.n, POLLIN, deadline)) {
    char buffer[8192]; auto n = recv(fd.n, buffer, sizeof(buffer), 0);
    if (n <= 0) break;
    response.append(buffer, static_cast<size_t>(n));
    if (response.size() > 65536) return 1;
  }
  auto body = response.substr(response.find("\r\n\r\n") + 4);
  std::cout << body;
  return body.find(" SELECTED") == std::string::npos ? 1 : 0;
}
} // namespace proxy
#ifndef JETSON_PROXY_TEST
int main(int argc, char** argv) {
  using namespace proxy;
  signal(SIGPIPE, SIG_IGN);
  signal(SIGTERM, [](int) { stopping.store(true, std::memory_order_relaxed); });
  signal(SIGINT, [](int) { stopping.store(true, std::memory_order_relaxed); });
  std::string home = getenv("HOME") ? getenv("HOME") : "/home/jetson";
  std::string path = home + "/.config/jetson-auto-proxy.conf", mode;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--config" && i+1 < argc) path = argv[++i];
    else if (arg == "--no-proxy" || arg == "--status" || arg == "--probe") mode = arg;
    else { std::cerr << "Usage: jetson-auto-proxy [--config FILE] [--status|--probe|--no-proxy]\n"; return 2; }
  }
  try {
    if (mode == "--no-proxy") { std::cout << no_proxy() << '\n'; return 0; }
    Config c = read_config(path);
    if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) return 2;
    int result;
    if (mode == "--status") result = status(c);
    else if (mode == "--probe") { Pool pool(c); pool.refresh(); std::cout << pool.status(); result = pool.ready().empty() ? 1 : 0; }
    else result = serve(c);
    curl_global_cleanup();
    return result;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
}
#endif
