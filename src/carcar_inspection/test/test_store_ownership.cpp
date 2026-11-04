#include <gtest/gtest.h>
#include <unistd.h>
#include "carcar_inspection/store.hpp"

using namespace carcar_inspection;

TEST(StoreOwnership, ConcurrentOpenCannotInterruptActiveRun) {
  char pattern[] = "/tmp/carcar-store-owner-XXXXXX";
  const char * directory = mkdtemp(pattern);
  ASSERT_NE(directory, nullptr);
  const std::filesystem::path root(directory);
  {
    Store owner(root);
    owner.put("runs", {{"id", "active"}, {"status", "RUNNING"}}, 0);
    EXPECT_THROW(Store second(root), std::runtime_error);
    EXPECT_EQ(owner.get("runs", "active")["status"], "RUNNING");
    // 多次失败不能释放原控制台的独占锁。
    EXPECT_THROW(Store third(root), std::runtime_error);
  }
  {
    Store restarted(root);
    EXPECT_EQ(restarted.get("runs", "active")["status"], "INTERRUPTED");
  }
  std::filesystem::remove_all(root);
}

TEST(StoreOwnership, SeparateDataDirectoriesRemainIndependent) {
  char pattern[] = "/tmp/carcar-store-independent-XXXXXX";
  const char * directory = mkdtemp(pattern);
  ASSERT_NE(directory, nullptr);
  const std::filesystem::path root(directory);
  {
    Store first(root / "first");
    Store second(root / "second");
    first.put("settings", {{"id", "value"}, {"value", 1}}, 0);
    EXPECT_TRUE(second.get("settings", "value").is_null());
  }
  std::filesystem::remove_all(root);
}
