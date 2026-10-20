#include "carcar_mapping_console/tablet_wifi_trial.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>

using namespace carcar_mapping_console;
namespace {
class Backend : public TabletTrialBackend {
public:
  std::vector<std::string> events;
  std::string fail;
  std::string path = "/checkpoint/1";
  unsigned timeout = 0;
  bool rollback_fails = false;
  void note(const std::string & event) {
    events.push_back(event);
    if (fail == event) throw std::runtime_error(event);
  }
  void preflight() override {note("preflight");}
  std::string checkpoint(unsigned seconds) override {
    timeout = seconds; note("checkpoint"); return path;
  }
  void record_armed(const std::string & value) override {
    EXPECT_EQ(value, path); note("record");
  }
  void activate() override {note("activate");}
  void rollback(const std::string & value) override {
    EXPECT_EQ(value, path); note("rollback");
    if (rollback_fails) throw std::runtime_error("rollback unavailable");
  }
};

TEST(TabletWifiTrial, NeverActivatesBeforeIndependent180SecondCheckpointAndDurableRecord) {
  Backend backend;
  begin_tablet_trial(backend);
  EXPECT_EQ(backend.timeout, 180u);
  EXPECT_EQ(backend.events, (std::vector<std::string>{"preflight", "checkpoint", "record", "activate"}));
}
TEST(TabletWifiTrial, RefusesSwitchWhenPreflightOrCheckpointFails) {
  for (const auto & failure : {"preflight", "checkpoint"}) {
    Backend backend; backend.fail = failure;
    EXPECT_THROW(begin_tablet_trial(backend), std::runtime_error);
    EXPECT_EQ(std::find(backend.events.begin(), backend.events.end(), "activate"), backend.events.end());
  }
  for (const auto & path : {"", "/"}) {
    Backend backend; backend.path = path;
    EXPECT_THROW(begin_tablet_trial(backend), std::runtime_error);
    EXPECT_EQ(backend.events, (std::vector<std::string>{"preflight", "checkpoint"}));
  }
}
TEST(TabletWifiTrial, AuditFailurePreventsSwitchAndRequestsRollback) {
  Backend backend; backend.fail = "record";
  EXPECT_THROW(begin_tablet_trial(backend), std::runtime_error);
  EXPECT_EQ(backend.events, (std::vector<std::string>{"preflight", "checkpoint", "record", "rollback"}));
}
TEST(TabletWifiTrial, ActivationFailureRollsBackWithoutMaskingOriginalFailure) {
  Backend backend; backend.fail = "activate"; backend.rollback_fails = true;
  try {begin_tablet_trial(backend); FAIL();}
  catch (const std::runtime_error & error) {EXPECT_STREQ(error.what(), "activate");}
  EXPECT_EQ(backend.events.back(), "rollback");
  EXPECT_EQ(backend.timeout, 180u); // 后端没有取消或延长独立回滚的接口。
}

TrialVariant section(GVariant * profile, const char * name) {
  return {g_variant_lookup_value(profile, name, G_VARIANT_TYPE("a{sv}")), g_variant_unref};
}
TEST(TabletWifiTrial, TargetIsDhcpClientWithoutAutoconnectOrStaticOldNetworkAddress) {
  TrialVariant profile(tablet_client_settings("Hao.iPhone", "dummy-test-passphrase",
    "5bb7992d-98ab-4eed-9f87-1f22041529ad", "wlP1p1s0"), g_variant_unref);
  const auto connection = section(profile.get(), "connection");
  gboolean autoconnect = true;
  ASSERT_TRUE(g_variant_lookup(connection.get(), "autoconnect", "b", &autoconnect));
  EXPECT_FALSE(autoconnect);
  const auto wireless = section(profile.get(), "802-11-wireless");
  const char * mode = nullptr;
  ASSERT_TRUE(g_variant_lookup(wireless.get(), "mode", "&s", &mode));
  EXPECT_STREQ(mode, "infrastructure");
  const auto ipv4 = section(profile.get(), "ipv4");
  const char * method = nullptr;
  ASSERT_TRUE(g_variant_lookup(ipv4.get(), "method", "&s", &method));
  EXPECT_STREQ(method, "auto");
  EXPECT_EQ(g_variant_n_children(ipv4.get()), 1u);
}
TEST(TabletWifiTrial, SsidIsDataAndSupportsChineseWithoutShellInterpolation) {
  const std::string ssid = "平板'$(id)`x`";
  TrialVariant profile(tablet_client_settings(ssid, "dummy-test-passphrase",
    "5bb7992d-98ab-4eed-9f87-1f22041529ad", "wlP1p1s0"), g_variant_unref);
  const auto wireless = section(profile.get(), "802-11-wireless");
  TrialVariant bytes(g_variant_lookup_value(wireless.get(), "ssid", G_VARIANT_TYPE("ay")), g_variant_unref);
  gsize count = 0;
  const auto * data = static_cast<const char *>(g_variant_get_fixed_array(bytes.get(), &count, 1));
  EXPECT_EQ(std::string(data, count), ssid);
}
TEST(TabletWifiTrial, RejectsInvalidCredentialsBeforeProfileCreation) {
  const std::string uuid = "5bb7992d-98ab-4eed-9f87-1f22041529ad";
  for (const auto & ssid : {std::string{}, std::string(33, 'a'), std::string("bad\0ssid", 8), std::string(1, '\xff')})
    EXPECT_THROW(tablet_client_settings(ssid, "dummy-test-passphrase", uuid, "wlan0"), std::runtime_error);
  for (const auto & password : {std::string("short"), std::string(64, 'a'), std::string("bad\0password", 12)})
    EXPECT_THROW(tablet_client_settings("tablet", password, uuid, "wlan0"), std::runtime_error);
}
}  // namespace
