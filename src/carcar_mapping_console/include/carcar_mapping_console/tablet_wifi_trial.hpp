#pragma once

#include <gio/gio.h>
#include <exception>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace carcar_mapping_console {
constexpr unsigned tablet_trial_seconds = 180;
using TrialVariant = std::unique_ptr<GVariant, decltype(&g_variant_unref)>;

// 返回仅用于客户端连接的配置；密码只交给 NetworkManager，不进入 argv 或状态文件。
inline GVariant * tablet_client_settings(
  const std::string & ssid, const std::string & password,
  const std::string & uuid, const std::string & interface)
{
  if (ssid.empty() || ssid.size() > 32 || ssid.find('\0') != std::string::npos ||
      !g_utf8_validate(ssid.data(), ssid.size(), nullptr))
    throw std::runtime_error("热点名称必须为1～32字节的有效UTF-8文本");
  if (password.size() < 8 || password.size() > 63 ||
      password.find_first_of("\r\n\0", 0, 3) != std::string::npos ||
      !g_utf8_validate(password.data(), password.size(), nullptr))
    throw std::runtime_error("本次MVP仅支持8～63字节的WPA2兼容热点密码");
  if (!g_uuid_string_is_valid(uuid.c_str()) || interface.empty())
    throw std::runtime_error("试验UUID或网络接口无效");
  GVariantBuilder all;
  g_variant_builder_init(&all, G_VARIANT_TYPE("a{sa{sv}}"));
  auto section = [&](const char * name,
      std::initializer_list<std::pair<const char *, GVariant *>> fields) {
    GVariantBuilder values;
    g_variant_builder_init(&values, G_VARIANT_TYPE("a{sv}"));
    for (const auto & field : fields)
      g_variant_builder_add(&values, "{sv}", field.first, field.second);
    g_variant_builder_add(&all, "{s@a{sv}}", name, g_variant_builder_end(&values));
  };
  section("connection", {
    {"id", g_variant_new_string("carcar-tablet-wifi-mvp")},
    {"uuid", g_variant_new_string(uuid.c_str())},
    {"type", g_variant_new_string("802-11-wireless")},
    {"interface-name", g_variant_new_string(interface.c_str())},
    {"autoconnect", g_variant_new_boolean(false)}});
  section("802-11-wireless", {
    {"ssid", g_variant_new_fixed_array(G_VARIANT_TYPE_BYTE, ssid.data(), ssid.size(), 1)},
    {"mode", g_variant_new_string("infrastructure")},
    {"security", g_variant_new_string("802-11-wireless-security")}});
  section("802-11-wireless-security", {
    {"key-mgmt", g_variant_new_string("wpa-psk")},
    {"psk", g_variant_new_string(password.c_str())}});
  section("ipv4", {{"method", g_variant_new_string("auto")}});
  section("ipv6", {{"method", g_variant_new_string("disabled")}});
  return g_variant_builder_end(&all);
}

// 真实后端调用系统NetworkManager；测试后端只记录调用，绝不接触真实网络。
class TabletTrialBackend {
public:
  virtual ~TabletTrialBackend() = default;
  virtual void preflight() = 0;
  virtual std::string checkpoint(unsigned seconds) = 0;
  virtual void record_armed(const std::string & checkpoint) = 0;
  virtual void activate() = 0;
  virtual void rollback(const std::string & checkpoint) = 0;
};

inline void begin_tablet_trial(TabletTrialBackend & backend)
{
  backend.preflight();
  const auto checkpoint = backend.checkpoint(tablet_trial_seconds);
  if (checkpoint.empty() || checkpoint == "/")
    throw std::runtime_error("未取得有效回滚检查点，拒绝切网");
  try {
    backend.record_armed(checkpoint);
    backend.activate();
  } catch (...) {
    const auto error = std::current_exception();
    // 即使主动回滚失败，也不销毁检查点；NetworkManager的独立计时仍生效。
    try {backend.rollback(checkpoint);} catch (...) {}
    std::rethrow_exception(error);
  }
}
}  // namespace carcar_mapping_console
