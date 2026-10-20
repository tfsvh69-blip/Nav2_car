#include "carcar_mapping_console/common.hpp"
#include "carcar_mapping_console/profile_backup.hpp"
#include "carcar_mapping_console/serial_preflight.hpp"
#include "carcar_mapping_console/tablet_wifi_trial.hpp"

#include <openssl/crypto.h>
#include <fcntl.h>
#include <spawn.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <csignal>
#include <ctime>
#include <iostream>
#include <thread>

using namespace carcar_mapping_console;
namespace fs = std::filesystem;
namespace {
constexpr const char * service = "org.freedesktop.NetworkManager";
constexpr const char * manager = "/org/freedesktop/NetworkManager";
constexpr const char * settings = "/org/freedesktop/NetworkManager/Settings";
constexpr const char * settings_interface = "org.freedesktop.NetworkManager.Settings";
constexpr const char * interface = "wlP1p1s0";
const fs::path state_dir = "/var/lib/carcar-tablet-wifi-trial";
const fs::path state_file = state_dir / "state.json";
const fs::path maintenance = "/home/jetson/Desktop/系统配置维护记录.md";
termios password_terminal{};
volatile sig_atomic_t password_hidden = 0;

void password_interrupted(int signal)
{
  if (password_hidden) tcsetattr(STDIN_FILENO, TCSANOW, &password_terminal);
  const char newline = '\n';
  const auto ignored = write(STDERR_FILENO, &newline, 1);
  (void)ignored;
  _exit(128 + signal);
}

std::string date()
{
  const auto time = std::time(nullptr) + 8 * 3600;
  std::tm value{};
  gmtime_r(&time, &value);
  std::ostringstream out;
  out << std::put_time(&value, "%Y-%m-%d %H:%M:%S") << " +08:00";
  return out.str();
}

std::pair<int, std::string> run(std::vector<std::string> args)
{
  int pipes[2];
  if (pipe2(pipes, O_CLOEXEC)) throw std::runtime_error("无法创建命令输出管道");
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, pipes[1], STDOUT_FILENO);
  posix_spawn_file_actions_adddup2(&actions, pipes[1], STDERR_FILENO);
  std::vector<char *> argv;
  for (auto & arg : args) argv.push_back(arg.data());
  argv.push_back(nullptr);
  pid_t pid;
  const int error = posix_spawn(&pid, argv[0], &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  close(pipes[1]);
  if (error) {close(pipes[0]); throw std::runtime_error("无法执行必要检查工具");}
  std::string output;
  char buffer[4096];
  for (;;) {
    const auto count = read(pipes[0], buffer, sizeof(buffer));
    if (count > 0) output.append(buffer, count);
    else if (!count) break;
    else if (errno != EINTR) {close(pipes[0]); throw std::runtime_error("读取命令输出失败");}
  }
  close(pipes[0]);
  int status = 0;
  while (waitpid(pid, &status, 0) < 0)
    if (errno != EINTR) throw std::runtime_error("等待检查工具失败");
  return {WIFEXITED(status) ? WEXITSTATUS(status) : 128, output};
}

void save(json value)
{
  value["updated"] = date();
  const auto temporary = state_file.string() + ".tmp";
  const int fd = open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_NOFOLLOW | O_CLOEXEC, 0600);
  if (fd < 0) throw std::runtime_error("无法写入试验状态");
  const auto data = value.dump(2) + "\n";
  size_t offset = 0;
  while (offset < data.size()) {
    const auto count = write(fd, data.data() + offset, data.size() - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {close(fd); throw std::runtime_error("试验状态写入失败");}
    offset += static_cast<size_t>(count);
  }
  if (fsync(fd)) {close(fd); throw std::runtime_error("试验状态同步失败");}
  close(fd);
  fs::rename(temporary, state_file);
}

json load() {return json::parse(read_file(state_file));}

void audit(const std::string & action, const json & details)
{
  std::ofstream out(maintenance, std::ios::app);
  if (!out) throw std::runtime_error("系统配置维护记录不可写，拒绝继续");
  out << "\n\n## NET-MVP-" << nonce().substr(0, 8) << "：" << action << "\n\n"
      << "日期：" << date() << "。涉及：/etc/NetworkManager/system-connections/（同目录备份）、"
      << state_file << "、NetworkManager运行时连接。\n\n```json\n"
      << details.dump(2) << "\n```\n\n"
      << "只记录软件观测；电脑SSH、平板网页、断上游及锁屏结果仍须现场登记。\n";
  out.flush();
  if (!out) throw std::runtime_error("系统配置维护记录写入失败");
}

class Lock {
  int fd_;
public:
  explicit Lock(bool wait = false)
  {
    fd_ = open((state_dir / "worker.lock").c_str(), O_CREAT | O_RDWR | O_NOFOLLOW | O_CLOEXEC, 0600);
    if (fd_ < 0 || flock(fd_, LOCK_EX | (wait ? 0 : LOCK_NB)))
      throw std::runtime_error("已有试验运行；请用status或restore，不重复切网");
  }
  ~Lock() {close(fd_);}
};

class Network : public TabletTrialBackend {
  GDBusConnection * bus_;
public:
  json state;
  Network()
  {
    GError * error = nullptr;
    bus_ = g_bus_get_sync(G_BUS_TYPE_SYSTEM, nullptr, &error);
    if (!bus_) {
      if (error) g_error_free(error);
      throw std::runtime_error("无法连接系统D-Bus");
    }
  }
  ~Network() override {g_object_unref(bus_);}
  TrialVariant call(const std::string & path, const char * iface,
    const char * method, GVariant * args = nullptr)
  {
    GError * error = nullptr;
    auto * reply = g_dbus_connection_call_sync(bus_, service, path.c_str(), iface,
      method, args, nullptr, G_DBUS_CALL_FLAGS_NONE, 10000, nullptr, &error);
    if (!reply) {
      // 不输出参数/密码；详细返回仅保留方法名与错误域代码。
      const int code = error ? error->code : -1;
      if (error) g_error_free(error);
      throw std::runtime_error(std::string("NetworkManager调用失败: ") + method + " code=" + std::to_string(code));
    }
    return {reply, g_variant_unref};
  }
  TrialVariant property(const std::string & path, const char * iface, const char * key)
  {
    auto reply = call(path, "org.freedesktop.DBus.Properties", "Get", g_variant_new("(ss)", iface, key));
    GVariant * value;
    g_variant_get(reply.get(), "(v)", &value);
    return {value, g_variant_unref};
  }
  std::string text_property(const std::string & path, const char * iface, const char * key)
  {auto value = property(path, iface, key); return g_variant_get_string(value.get(), nullptr);}
  static std::string object(const TrialVariant & reply)
  {const char * value; g_variant_get(reply.get(), "(&o)", &value); return value;}
  std::string device()
  {return object(call(manager, service, "GetDeviceByIpIface", g_variant_new("(s)", interface)));}
  std::string ssid()
  {
    const auto access_point = text_property(device(), "org.freedesktop.NetworkManager.Device.Wireless", "ActiveAccessPoint");
    if (access_point == "/") return "";
    auto bytes = property(access_point, "org.freedesktop.NetworkManager.AccessPoint", "Ssid");
    gsize length = 0;
    const auto * value = static_cast<const char *>(g_variant_get_fixed_array(bytes.get(), &length, 1));
    return length ? std::string(value, length) : std::string{};
  }
  std::string connection(const std::string & uuid)
  {return object(call(settings, settings_interface, "GetConnectionByUuid", g_variant_new("(s)", uuid.c_str())));}
  std::vector<std::string> checkpoints()
  {
    auto values = property(manager, service, "Checkpoints");
    std::vector<std::string> result;
    GVariantIter iter;
    g_variant_iter_init(&iter, values.get());
    const char * path;
    while (g_variant_iter_next(&iter, "&o", &path)) result.emplace_back(path);
    return result;
  }
  json snapshot()
  {
    const auto path = device();
    auto status = property(path, "org.freedesktop.NetworkManager.Device", "State");
    const auto active = text_property(path, "org.freedesktop.NetworkManager.Device", "ActiveConnection");
    const auto ip4 = text_property(path, "org.freedesktop.NetworkManager.Device", "Ip4Config");
    json result{{"interface", interface}, {"device_state", g_variant_get_uint32(status.get())},
      {"ssid", ssid()},
      {"uuid", active == "/" ? "" : text_property(active, "org.freedesktop.NetworkManager.Connection.Active", "Uuid")},
      {"ipv4", json::array()}};
    if (ip4 != "/") {
      auto addresses = property(ip4, "org.freedesktop.NetworkManager.IP4Config", "AddressData");
      GVariantIter iter;
      g_variant_iter_init(&iter, addresses.get());
      while (auto * item = g_variant_iter_next_value(&iter)) {
        TrialVariant entry(item, g_variant_unref);
        const char * address = nullptr;
        guint32 prefix = 0;
        if (g_variant_lookup(item, "address", "&s", &address) && g_variant_lookup(item, "prefix", "u", &prefix))
          result["ipv4"].push_back(std::string(address) + "/" + std::to_string(prefix));
      }
    }
    return result;
  }
  void ready()
  {
    const auto result = run({"/usr/bin/curl", "--max-time", "3", "-fsS", "http://127.0.0.1:8080/api/status"});
    if (result.first) throw std::runtime_error("无法确认网页待命，拒绝切网");
    const auto web = json::parse(result.second);
    if (web.value("phase", "") != "STANDBY" || web.value("task_state", "") != "STANDBY" ||
        web.value("inspection_running", false))
      throw std::runtime_error("必须先结束建图/巡检并确认车辆停稳");
    if (fs::exists("/dev/myserial")) require_unused_serial("/dev/myserial");
  }
  void preflight() override
  {
    ready();
    if (!checkpoints().empty()) throw std::runtime_error("已有网络回滚检查点，拒绝叠加");
    const auto current = snapshot();
    if (current.at("uuid") != state.at("original_uuid") || current.at("device_state") != 100 ||
        current.at("ipv4").empty())
      throw std::runtime_error("当前不在准备时的原网络，拒绝试验");
    connection(state.at("trial_uuid")); // 重启NetworkManager后内存连接消失，必须重新prepare。
  }
  std::string checkpoint(unsigned seconds) override
  {
    const auto path = device();
    GVariantBuilder devices;
    g_variant_builder_init(&devices, G_VARIANT_TYPE("ao"));
    g_variant_builder_add(&devices, "o", path.c_str());
    return object(call(manager, service, "CheckpointCreate", g_variant_new("(@aouu)",
      g_variant_builder_end(&devices), seconds, 0U)));
  }
  void record_armed(const std::string & path) override
  {
    state["checkpoint"] = path;
    state["started_steady"] = steady();
    state["phase"] = "ARMED";
    save(state);
    audit("平板热点180秒试验已建立回滚，尚未请求切换", state);
  }
  void activate_uuid(const std::string & uuid)
  {
    const auto profile = connection(uuid), target = device();
    call(manager, service, "ActivateConnection", g_variant_new("(ooo)", profile.c_str(), target.c_str(), "/"));
  }
  void activate() override {activate_uuid(state.at("trial_uuid"));}
  void rollback(const std::string & path) override
  {
    const auto active = checkpoints();
    if (std::find(active.begin(), active.end(), path) == active.end()) return;
    auto reply = call(manager, service, "CheckpointRollback", g_variant_new("(o)", path.c_str()));
    TrialVariant results(g_variant_get_child_value(reply.get(), 0), g_variant_unref);
    guint32 code = 0;
    const auto target = device();
    if (!g_variant_lookup(results.get(), target.c_str(), "u", &code) || code != 0)
      throw std::runtime_error("NetworkManager未报告原接口回滚成功，保留检查点并停止");
  }
};

std::string read_password()
{
  if (!isatty(STDIN_FILENO)) throw std::runtime_error("密码必须在SSH交互终端本地隐藏输入，不接受管道或命令行");
  termios before{};
  if (tcgetattr(STDIN_FILENO, &before)) throw std::runtime_error("无法读取终端设置");
  password_terminal = before;
  const auto old_int = std::signal(SIGINT, password_interrupted);
  const auto old_term = std::signal(SIGTERM, password_interrupted);
  const auto old_hup = std::signal(SIGHUP, password_interrupted);
  password_hidden = 1;
  auto hidden = before;
  hidden.c_lflag &= ~ECHO;
  if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &hidden)) {
    password_hidden = 0;
    std::signal(SIGINT, old_int); std::signal(SIGTERM, old_term); std::signal(SIGHUP, old_hup);
    throw std::runtime_error("无法隐藏密码输入");
  }
  std::cout << "热点密码（本地隐藏输入，不保存到文件）：" << std::flush;
  std::string password;
  std::getline(std::cin, password);
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &before);
  password_hidden = 0;
  std::signal(SIGINT, old_int); std::signal(SIGTERM, old_term); std::signal(SIGHUP, old_hup);
  std::cout << '\n';
  return password;
}

void prepare(Network & net, const std::string & ssid)
{
  net.ready();
  if (!net.checkpoints().empty()) throw std::runtime_error("存在网络检查点，拒绝准备");
  const auto original = net.snapshot();
  if (original.at("device_state") != 100 || original.at("uuid") == "" || original.at("ipv4").empty())
    throw std::runtime_error("原WiFi尚未连接，不能准备可恢复试验");
  if (original.at("ssid") == ssid)
    throw std::runtime_error("当前已连接目标热点；请先确认并回到原WiFi，不将热点误记为回滚目标");
  // 先校验SSID；实际密码仍仅从终端读取。
  char * generated = g_uuid_string_random();
  const std::string uuid = generated;
  g_free(generated);
  auto password = read_password();
  TrialVariant profile(nullptr, g_variant_unref);
  try {profile.reset(tablet_client_settings(ssid, password, uuid, interface));}
  catch (...) {OPENSSL_cleanse(password.data(), password.size()); throw;}
  OPENSSL_cleanse(password.data(), password.size());
  const fs::path directory = "/etc/NetworkManager/system-connections";
  const auto backup = directory / (".carcar-tablet-" + nonce() + ".backup");
  const auto result = run({"/usr/bin/tar", "--create", "--file", backup.string(),
    "--exclude=" + backup.filename().string(), "--directory", directory.string(), "."});
  if (result.first) throw std::runtime_error("完整网络配置备份失败，未添加试验连接");
  if (chmod(backup.c_str(), 0600)) throw std::runtime_error("备份权限设置失败");
  net.state = {{"schema", 1}, {"phase", "PREPARED"}, {"ssid", ssid}, {"trial_uuid", uuid},
    {"original_uuid", original.at("uuid")}, {"original_snapshot", original},
    {"backup", backup.string()}, {"backup_sha256", sha256(backup)},
    {"timeout_seconds", tablet_trial_seconds}, {"trial_connected", false},
    {"client_configuration", {{"interface", interface}, {"mode", "infrastructure"},
      {"ipv4", "auto/DHCP"}, {"ipv6", "disabled"}, {"autoconnect", false},
      {"storage", "NetworkManager memory only"}, {"key_mgmt", "wpa-psk"}, {"password", "<REDACTED>"}}},
    {"rollback_command", "sudo " + shell_quote(fs::read_symlink("/proc/self/exe").string()) + " restore"}};
  if (fs::exists(state_file)) fs::rename(state_file, state_dir / ("previous-" + nonce() + ".json"));
  audit("备份原配置并准备内存客户端连接；原连接不修改", net.state);
  // 不落盘、不自动连接；重启NetworkManager或机器后该临时配置自动消失。
  auto reply = net.call(settings, settings_interface, "AddConnectionUnsaved",
    g_variant_new("(@a{sa{sv}})", profile.release()));
  net.state["trial_profile_path"] = Network::object(reply);
  save(net.state);
  std::cout << "已准备，未切换网络。密码仅保留在NetworkManager内存。\n" << net.state.dump(2) << '\n';
}

void start_worker()
{
  const auto executable = fs::read_symlink("/proc/self/exe").string();
  const auto logfile = (state_dir / "worker.log").string();
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
  posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, logfile.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0600);
  posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
  posix_spawn_file_actions_addclosefrom_np(&actions, 3);
  posix_spawnattr_t attributes;
  posix_spawnattr_init(&attributes);
  posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETSID);
  std::string operation = "_worker";
  char * args[] = {const_cast<char *>(executable.c_str()), operation.data(), nullptr};
  pid_t pid;
  const auto error = posix_spawn(&pid, executable.c_str(), &actions, &attributes, args, environ);
  posix_spawn_file_actions_destroy(&actions);
  posix_spawnattr_destroy(&attributes);
  if (error) throw std::runtime_error("后台试验无法启动，未切网");
  std::cout << "后台试验已提交，PID=" << pid << "。先建立180秒回滚再切换；结果用status查询。\n"
      << "旧SSH可能断开；请从平板客户端列表找到Jetson新IP后重连。\n";
}

void worker(Network & net)
{
  try {
    begin_tablet_trial(net);
    const double deadline = net.state.at("started_steady").get<double>() + tablet_trial_seconds;
    json previous;
    bool fallback_requested = false;
    while (steady() < deadline + 35) {
      json current;
      try {current = net.snapshot();}
      catch (const std::exception &) {
        // 切换期间ActiveConnection/Ip4Config可能刚好消失；继续观察独立回滚。
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      const bool returned = current.at("uuid") == net.state.at("original_uuid") &&
        current.at("device_state") == 100 && !current.at("ipv4").empty();
      const auto checkpoints = net.checkpoints();
      const auto own = net.state.at("checkpoint").get<std::string>();
      const bool armed = std::find(checkpoints.begin(), checkpoints.end(), own) != checkpoints.end();
      if (returned && (!armed || steady() >= deadline)) {
        net.state["phase"] = "RESTORED";
        net.state["restored_snapshot"] = current;
        save(net.state);
        audit("原WiFi已恢复（软件UUID、接口状态及IPv4确认）", net.state);
        std::cout << date() << " RESTORED " << current.dump() << std::endl;
        return;
      }
      if (!returned && !fallback_requested && (!armed || steady() > deadline + 5)) {
        for (const auto & path : checkpoints)
          if (path != own) throw std::runtime_error("回滚期间出现其他检查点，拒绝覆盖外部操作");
        audit("独立回滚后尚未恢复，额外请求激活原连接", current);
        net.activate_uuid(net.state.at("original_uuid"));
        fallback_requested = true;
      }
      if (current != previous) {
        previous = current;
        net.state["latest_snapshot"] = current;
        if (current.at("uuid") == net.state.at("trial_uuid") && current.at("device_state") == 100 &&
            !current.at("ipv4").empty()) {
          net.state["trial_connected"] = true;
          net.state["tablet_snapshot"] = current;
          net.state["phase"] = "CONNECTED";
          audit("Jetson已取得热点地址；尚不代表客户端互访通过", current);
        }
        save(net.state);
        std::cout << date() << " " << net.state.at("phase") << " " << current.dump() << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    throw std::runtime_error("超时后尚未确认原网络恢复，请保留原路由器供电并检查status");
  } catch (const std::exception & error) {
    net.state["phase"] = "FAILED";
    net.state["error"] = error.what();
    try {save(net.state); audit("试验异常；不销毁独立回滚检查点", net.state);} catch (...) {}
    std::cerr << error.what() << '\n';
    throw;
  }
}

void restore(Network & net)
{
  net.ready();
  const auto own = net.state.value("checkpoint", "");
  const auto checkpoints = net.checkpoints();
  for (const auto & path : checkpoints)
    if (path != own) throw std::runtime_error("存在其他网络检查点，拒绝干预");
  const auto current = net.snapshot();
  const auto active = current.at("uuid").get<std::string>();
  if (!active.empty() && active != net.state.at("original_uuid") && active != net.state.at("trial_uuid"))
    throw std::runtime_error("当前是其他连接，拒绝覆盖");
  audit("主动请求返回原WiFi", {{"before", current}, {"original_uuid", net.state.at("original_uuid")}});
  if (!own.empty() && std::find(checkpoints.begin(), checkpoints.end(), own) != checkpoints.end()) net.rollback(own);
  else if (active != net.state.at("original_uuid")) net.activate_uuid(net.state.at("original_uuid"));
  std::cout << "已请求恢复；用status核对原UUID、IPv4与SSH，不把请求返回当成恢复通过。\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const std::string action = argc > 1 ? argv[1] : "--help";
    if (action == "--help") {
      std::cout << "用法：mapping_tablet_wifi_trial status | prepare --ssid NAME | trial | restore\n"
        << "平板作热点，Jetson仅作客户端；trial固定180秒自动回原网，无永久切换入口。\n"
        << "prepare在本地终端隐藏读取密码；所有变更命令需要sudo。\n";
      return 0;
    }
    if (action != "status" && action != "prepare" && action != "trial" && action != "restore" && action != "_worker")
      throw std::runtime_error("未知操作；请使用--help");
    if ((action == "prepare" && (argc != 4 || std::string(argv[2]) != "--ssid")) ||
        (action != "prepare" && argc != 2))
      throw std::runtime_error("参数无效；密码不能通过命令行传入");
    Network net;
    if (action == "status") {
      json value{{"current", net.snapshot()}, {"checkpoints", net.checkpoints()}};
      if (geteuid() == 0 && fs::exists(state_file)) value["trial"] = load();
      else value["note"] = "sudo运行status可读取完整试验记录；本操作只读";
      std::cout << value.dump(2) << '\n';
      return 0;
    }
    if (geteuid() != 0) throw std::runtime_error("需要sudo；请在SSH终端本地输入系统密码，不要发到聊天里");
    umask(0077);
    fs::create_directories(state_dir);
    struct stat directory{};
    if (lstat(state_dir.c_str(), &directory) || !S_ISDIR(directory.st_mode) ||
        directory.st_uid != 0 || (directory.st_mode & 0077))
      throw std::runtime_error("试验状态目录必须由root持有且权限0700");
    if (action == "restore") {net.state = load(); restore(net); return 0;}
    Lock lock(action == "_worker");
    if (action == "prepare") prepare(net, argv[3]);
    else {
      net.state = load();
      if (net.state.at("phase") != "PREPARED")
        throw std::runtime_error("每次试验须重新prepare；不复用旧倒计时");
      if (action == "trial") {net.preflight(); start_worker();}
      else worker(net);
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "平板网络MVP：" << error.what() << '\n';
    return 1;
  }
}
