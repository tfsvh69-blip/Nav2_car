#include "carcar_mapping_console/common.hpp"
#include "carcar_mapping_console/profile_backup.hpp"
#include "carcar_mapping_console/serial_preflight.hpp"
#include <gio/gio.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <ctime>
using namespace carcar_mapping_console;
namespace fs = std::filesystem;
namespace {
const char * service = "org.freedesktop.NetworkManager";
const char * nm_path = "/org/freedesktop/NetworkManager";
const fs::path state_path = "/home/jetson/.local/state/carcar_mapping/network.json";
GDBusConnection * bus = nullptr;
GVariant * call(const char * path, const char * interface, const char * method, GVariant * args) {
  GError * error = nullptr;
  auto * value = g_dbus_connection_call_sync(bus, service, path, interface, method, args, nullptr,
      G_DBUS_CALL_FLAGS_NONE, 30000, nullptr, &error);
  if (!value) {std::string text = error ? error->message : "D-Bus 调用失败"; if (error) g_error_free(error); throw std::runtime_error(text);}
  return value;
}
std::string object(GVariant * reply) {const char * result; g_variant_get(reply, "(&o)", &result); std::string out = result; g_variant_unref(reply); return out;}
std::pair<int, std::string> run(std::vector<std::string> args) {
  int pipefd[2]; if (pipe(pipefd)) throw std::runtime_error("pipe 失败");
  const auto pid = fork(); if (pid < 0) throw std::runtime_error("fork 失败");
  if (!pid) {
    close(pipefd[0]); dup2(pipefd[1], STDOUT_FILENO); dup2(pipefd[1], STDERR_FILENO); close(pipefd[1]);
    std::vector<char *> argv; for (auto & arg : args) argv.push_back(arg.data()); argv.push_back(nullptr); execvp(argv[0], argv.data()); _exit(127);
  }
  close(pipefd[1]); std::string out; char buffer[1024]; ssize_t count;
  while ((count = read(pipefd[0], buffer, sizeof(buffer))) > 0) out.append(buffer, count);
  close(pipefd[0]); int status; waitpid(pid, &status, 0);
  while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
  return {WIFEXITED(status) ? WEXITSTATUS(status) : 128, out};
}
std::string timestamp() {std::time_t time = std::time(nullptr); std::tm tm{}; localtime_r(&time, &tm); std::ostringstream out; out << std::put_time(&tm, "%Y%m%d_%H%M%S"); return out.str();}
void record(const std::string & action, const json & details) {
  std::ofstream log("/home/jetson/Desktop/系统配置维护记录.md", std::ios::app);
  if (!log) throw std::runtime_error("系统维护记录不可写，拒绝继续");
  log << "\n\n## MAPPING-" << timestamp() << "-" << nonce().substr(0, 8) << "：" << action << "\n\n"
      << "涉及：/etc/NetworkManager/system-connections/，NetworkManager 运行时无线状态。\n\n"
      << "```json\n" << details.dump(2) << "\n```\n\n验证状态：待现场核对；本工具不把命令返回等同平板连接或实机通过。密码不写入日志。\n";
  log.flush(); if (!log) throw std::runtime_error("维护记录写入失败");
}
json read_state() {if (!fs::exists(state_path)) throw std::runtime_error("请先 prepare 创建独立热点"); return json::parse(read_file(state_path));}
void save_state(const json & value) {
  fs::create_directories(state_path.parent_path());
  const auto temporary = state_path.string() + ".tmp";
  {std::ofstream file(temporary); file << value.dump(2); file.flush(); if (!file) throw std::runtime_error("状态保存失败");}
  chmod(temporary.c_str(), 0600); fs::rename(temporary, state_path);
}
std::string active_uuid(const std::string & device) {
  auto result = run({"nmcli", "-g", "GENERAL.CON-UUID", "device", "show", device});
  if (result.first || result.second.empty() || result.second == "--") throw std::runtime_error("当前接口没有可恢复连接");
  return result.second;
}
std::vector<std::string> checkpoints() {
  auto * reply = call(nm_path, "org.freedesktop.DBus.Properties", "Get", g_variant_new("(ss)", service, "Checkpoints"));
  GVariant * boxed; g_variant_get(reply, "(v)", &boxed);
  std::vector<std::string> result; GVariantIter iter; g_variant_iter_init(&iter, boxed); const char * path;
  while (g_variant_iter_next(&iter, "&o", &path)) result.emplace_back(path);
  g_variant_unref(boxed); g_variant_unref(reply); return result;
}
void activate(const std::string & connection, const std::string & device) {
  auto * reply = call(nm_path, service, "ActivateConnection", g_variant_new("(ooo)", connection.c_str(), device.c_str(), "/")); g_variant_unref(reply);
}
std::string connection_path(const std::string & uuid) {return object(call("/org/freedesktop/NetworkManager/Settings", "org.freedesktop.NetworkManager.Settings", "GetConnectionByUuid", g_variant_new("(s)", uuid.c_str())));}
std::string connection_filename(const std::string & path) {
  auto * reply = call(path.c_str(), "org.freedesktop.DBus.Properties", "Get",
    g_variant_new("(ss)", "org.freedesktop.NetworkManager.Settings.Connection", "Filename"));
  GVariant * value; g_variant_get(reply, "(v)", &value);
  const std::string filename = g_variant_get_string(value, nullptr);
  g_variant_unref(value); g_variant_unref(reply); return filename;
}
void no_motor_owner() {
  if (fs::exists("/dev/myserial")) require_unused_serial("/dev/myserial");
}
}
int main(int argc, char ** argv) {
  try {
    if (argc < 2) throw std::runtime_error("用法: mapping_hotspot status|prepare|trial|verify-trial|cancel-trial|activate|restore|remove [--recovery-ready|--tablet-tested]");
    const std::string action = argv[1]; bool recovery = false, tested = false;
    for (int i = 2; i < argc; ++i) {std::string arg = argv[i]; if (arg == "--recovery-ready") recovery = true; else if (arg == "--tablet-tested") tested = true; else throw std::runtime_error("未知参数");}
    if (action == "status") {
      auto result = run({"nmcli", "-f", "GENERAL.DEVICE,GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS", "device", "show", "wlP1p1s0"});
      std::cout << result.second << '\n'; return result.first;
    }
    if (geteuid() != 0) throw std::runtime_error("此操作需要 sudo；不会修改 Polkit。先阅读操作手册风险与恢复步骤。");
    no_motor_owner();
    GError * error = nullptr; bus = g_bus_get_sync(G_BUS_TYPE_SYSTEM, nullptr, &error);
    if (!bus) throw std::runtime_error(error ? error->message : "系统总线不可用");
    const std::string interface = "wlP1p1s0";
    const auto device = object(call(nm_path, service, "GetDeviceByIpIface", g_variant_new("(s)", interface.c_str())));
    if (action == "prepare") {
      if (fs::exists(state_path)) throw std::runtime_error("已有维护状态，拒绝重复创建/覆盖，请先检查原状态");
      const auto existing = run({"nmcli", "-g", "connection.id", "connection", "show", "carcar-mapping"});
      if (!existing.first) throw std::runtime_error("已有同名连接，拒绝覆盖");
      const auto previous = active_uuid(interface);
      if (!checkpoints().empty()) throw std::runtime_error("存在其他网络 checkpoint，拒绝创建");
      auto routes = run({"ip", "-4", "route", "show"});
      if (routes.second.find("10.42.") != std::string::npos) throw std::runtime_error("候选热点网段可能冲突，请先检查路由");
      if (!isatty(STDIN_FILENO)) throw std::runtime_error("密码仅允许本地交互输入，不接受命令行/管道密码");
      std::cout << "输入新热点密码（8～63个ASCII字符，不回显）：" << std::flush;
      termios old_settings; tcgetattr(STDIN_FILENO, &old_settings); auto hidden = old_settings; hidden.c_lflag &= ~ECHO; tcsetattr(STDIN_FILENO, TCSAFLUSH, &hidden);
      std::string password; std::getline(std::cin, password); tcsetattr(STDIN_FILENO, TCSAFLUSH, &old_settings); std::cout << '\n';
      if (password.size() < 8 || password.size() > 63 || !std::all_of(password.begin(), password.end(), [](unsigned char c) {return c >= 32 && c < 127;})) throw std::runtime_error("密码长度或字符不符合 WPA2 要求");
      const fs::path backup = fs::path("/etc/NetworkManager/system-connections") / (".carcar-" + timestamp() + ".backup");
      const auto backup_result = run({"tar", "--create", "--file", backup.string(), "--exclude=.carcar-*", "--directory", "/etc/NetworkManager/system-connections", "."});
      if (backup_result.first) throw std::runtime_error("网络配置备份失败: " + backup_result.second);
      chmod(backup.c_str(), 0600);
      char * generated_uuid = g_uuid_string_random(); std::string uuid = generated_uuid; g_free(generated_uuid);
      json state{{"previous_uuid", previous}, {"hotspot_uuid", uuid}, {"interface", interface}, {"device", device}, {"backup", backup.string()}, {"trial_verified", false}};
      record("创建独立热点配置（尚不启用）", {{"before", "carcar-mapping 不存在；原连接不修改"},
        {"after", {{"ssid", "carcar-mapping"}, {"address", "10.42.0.1/24"}, {"mode", "ap"}, {"band", "bg"}, {"autoconnect", false}, {"security", "WPA2/RSN/CCMP，密码本地输入并脱敏"}}},
        {"backup", backup.string()}, {"previous_uuid", previous}, {"hotspot_uuid", uuid}, {"rollback", "关闭/删除新热点，按 previous_uuid 激活原连接；必要时从 .backup 恢复"}});
      GVariantBuilder all; g_variant_builder_init(&all, G_VARIANT_TYPE("a{sa{sv}}"));
      auto setting = [&](const char * name, std::initializer_list<std::pair<const char *, GVariant *>> fields) {
        GVariantBuilder section; g_variant_builder_init(&section, G_VARIANT_TYPE("a{sv}"));
        for (const auto & field : fields) g_variant_builder_add(&section, "{sv}", field.first, field.second);
        g_variant_builder_add(&all, "{s@a{sv}}", name, g_variant_builder_end(&section));
      };
      setting("connection", {{"id", g_variant_new_string("carcar-mapping")}, {"uuid", g_variant_new_string(uuid.c_str())},
        {"type", g_variant_new_string("802-11-wireless")}, {"interface-name", g_variant_new_string(interface.c_str())}, {"autoconnect", g_variant_new_boolean(false)}});
      const std::string ssid = "carcar-mapping";
      setting("802-11-wireless", {{"ssid", g_variant_new_fixed_array(G_VARIANT_TYPE_BYTE, ssid.data(), ssid.size(), 1)},
        {"mode", g_variant_new_string("ap")}, {"band", g_variant_new_string("bg")}});
      const char * proto[] = {"rsn", nullptr}; const char * cipher[] = {"ccmp", nullptr};
      setting("802-11-wireless-security", {{"key-mgmt", g_variant_new_string("wpa-psk")}, {"psk", g_variant_new_string(password.c_str())},
        {"proto", g_variant_new_strv(proto, -1)}, {"pairwise", g_variant_new_strv(cipher, -1)}, {"group", g_variant_new_strv(cipher, -1)}});
      GVariantBuilder addresses, address; g_variant_builder_init(&addresses, G_VARIANT_TYPE("aa{sv}")); g_variant_builder_init(&address, G_VARIANT_TYPE("a{sv}"));
      g_variant_builder_add(&address, "{sv}", "address", g_variant_new_string("10.42.0.1"));
      g_variant_builder_add(&address, "{sv}", "prefix", g_variant_new_uint32(24));
      g_variant_builder_add_value(&addresses, g_variant_builder_end(&address));
      setting("ipv4", {{"method", g_variant_new_string("shared")}, {"address-data", g_variant_builder_end(&addresses)}});
      setting("ipv6", {{"method", g_variant_new_string("disabled")}});
      save_state(state); // 即使 AddConnection 失败，也保留可审计状态，绝不覆盖原配置。
      state["connection"] = object(call("/org/freedesktop/NetworkManager/Settings", "org.freedesktop.NetworkManager.Settings", "AddConnection", g_variant_new("(@a{sa{sv}})", g_variant_builder_end(&all))));
      std::fill(password.begin(), password.end(), '\0'); save_state(state); std::cout << "热点配置已创建，未切换网络。\n";
      state["file"] = connection_filename(state.at("connection")); save_state(state);
      record("独立热点实际落盘（未启用）", {{"files", state["file"]}, {"before", "该热点文件不存在"},
        {"after", {{"uuid", uuid}, {"ssid", "carcar-mapping"}, {"mode", "ap"}, {"band", "bg"}, {"autoconnect", false},
        {"ipv4", "shared 10.42.0.1/24"}, {"ipv6", "disabled"}, {"security", "wpa-psk/rsn/ccmp；PSK脱敏"}}}, {"backup", backup.string()}});
    } else {
      auto state = read_state();
      if (action == "trial") {
        if (!recovery) throw std::runtime_error("必须先验证有线/本地恢复，再传 --recovery-ready");
        if (active_uuid(interface) != state.at("previous_uuid").get<std::string>() || !checkpoints().empty()) throw std::runtime_error("不在原连接或已有 checkpoint，拒绝试点");
        GVariantBuilder devices; g_variant_builder_init(&devices, G_VARIANT_TYPE("ao")); g_variant_builder_add(&devices, "o", device.c_str());
        state["checkpoint"] = object(call(nm_path, service, "CheckpointCreate", g_variant_new("(@aouu)", g_variant_builder_end(&devices), 180U, 0U)));
        state["trial_verified"] = false; save_state(state);
        record("热点180秒自动回滚试点", {{"before", state["previous_uuid"]}, {"after", state["hotspot_uuid"]}, {"checkpoint", state["checkpoint"]}, {"mechanism", "180秒后 NetworkManager 自动恢复接口；SSH预计中断"}, {"rollback", "等待回滚或本地运行 mapping_hotspot restore"}});
        activate(connection_path(state.at("hotspot_uuid")), device);
      } else if (action == "verify-trial" || action == "cancel-trial") {
        if (action == "verify-trial" && !tested) throw std::runtime_error("须现场确认平板连接成功、自动回滚恢复原WiFi，再传 --tablet-tested");
        if (action == "cancel-trial" && !state.contains("checkpoint")) throw std::runtime_error("没有本工具待结束的试点，不需要取消");
        const auto connection_state = run({"nmcli", "-g", "GENERAL.STATE", "device", "show", interface});
        if (active_uuid(interface) != state.at("previous_uuid").get<std::string>() || connection_state.first || connection_state.second.rfind("100", 0) != 0)
          throw std::runtime_error("须先恢复原WiFi并确认设备状态100，才能结束试点");
        record(action == "verify-trial" ? "准备登记已完成试点" : "取消失败或中止的试点（不登记通过）",
          {{"before", state.value("checkpoint", "已自动结束")}, {"after", "仅销毁本工具checkpoint，原连接保持"}});
        for (const auto & path : checkpoints()) {
          if (path != state.value("checkpoint", "")) throw std::runtime_error("存在非本工具 checkpoint，拒绝修改");
          g_variant_unref(call(nm_path, service, "CheckpointDestroy", g_variant_new("(o)", path.c_str())));
        }
        state["trial_verified"] = action == "verify-trial"; state.erase("checkpoint"); save_state(state);
        record("热点试点结果登记", {{"before", "尚未确认"}, {"after", action == "verify-trial" ? "操作者声明平板试点及原WiFi恢复已完成；移动与停车仍待验证" : "试点已取消，未登记成功；再次试点或撤销配置"}});
      } else if (action == "activate") {
        if (!recovery || !state.value("trial_verified", false) || !checkpoints().empty()) throw std::runtime_error("日常热点须先完成试点并确认无待回滚checkpoint，传 --recovery-ready");
        record("进入日常热点模式", {{"before", "原WiFi"}, {"after", "车载热点；无计时回滚"}, {"rollback", "回覆盖区后用本地/有线/热点SSH运行 mapping_hotspot restore"}});
        activate(connection_path(state.at("hotspot_uuid")), device);
      } else if (action == "remove") {
        if (active_uuid(interface) != state.at("previous_uuid").get<std::string>() || !checkpoints().empty())
          throw std::runtime_error("先恢复原WiFi并结束试点checkpoint，才能删除本次热点");
        const auto connections = run({"nmcli", "-g", "UUID", "connection", "show"});
        if (connections.first) throw std::runtime_error("无法核对热点UUID，拒绝删除");
        std::istringstream ids(connections.second); std::string id; bool found = false;
        while (std::getline(ids, id)) if (id == state.at("hotspot_uuid").get<std::string>()) found = true;
        if (found) {
          const auto connection = connection_path(state.at("hotspot_uuid"));
          const fs::path file = connection_filename(connection);
          if (file.empty() || file.parent_path() != fs::path("/etc/NetworkManager/system-connections"))
            throw std::runtime_error("热点落盘路径不符合预期，拒绝删除");
          state["remove_backup"] = backup_profile(file).string();
          state["file"] = file.string(); save_state(state);
          const std::string rollback = "sudo cp -p -- " + shell_quote(state.at("remove_backup").get<std::string>()) + " " + shell_quote(file.string()) +
            " && sudo chmod 600 -- " + shell_quote(file.string()) + " && sudo nmcli connection load " + shell_quote(file.string());
          record("删除本工具新增热点配置（删除前完整备份）", {{"files", file.string()}, {"before", state["hotspot_uuid"]},
            {"after", "仅删除本次热点；原WiFi不变"}, {"backup", state["remove_backup"]}, {"rollback", rollback}});
          std::cout << "删除前备份：" << state["remove_backup"].get<std::string>() << "\n恢复配置（不激活热点）：\n" << rollback << '\n';
          g_variant_unref(call(connection.c_str(), "org.freedesktop.NetworkManager.Settings.Connection", "Delete", nullptr));
        } else record("归档未创建热点的维护状态", {{"before", state["hotspot_uuid"]}, {"after", "该UUID不存在，未删除系统配置"}});
        fs::rename(state_path, state_path.string() + ".retired_" + timestamp());
      } else if (action == "restore") {
        record("恢复原WiFi", {{"before", "热点或试点连接"}, {"after", state["previous_uuid"]}, {"rollback", "原配置保留；恢复后重新检查DHCP地址"}});
        activate(connection_path(state.at("previous_uuid")), device);
      } else throw std::runtime_error("未知操作");
      std::cout << "网络请求已提交；请现场核对地址、连接与回滚结果。\n";
    }
    if (bus) g_object_unref(bus);
    return 0;
  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; if (bus) g_object_unref(bus); return 1;}
}
