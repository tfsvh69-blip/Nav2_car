// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
// 仅监听本机随机端口；不使用ROS、硬件或真实建图服务。
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <sys/socket.h>
#include <sys/time.h>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include "carcar_inspection/mapping_status_monitor.hpp"

namespace {
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;
using carcar_inspection::MappingStatusMonitor;
using namespace std::chrono_literals;

bool wait_until(const std::function<bool()> & condition) {
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return condition();
}

class StatusServer {
public:
  StatusServer()
  : acceptor_(io_, tcp::endpoint(asio::ip::address_v4::loopback(), 0)) {
    acceptor_.non_blocking(true);
    url_ = "http://127.0.0.1:" + std::to_string(acceptor_.local_endpoint().port()) + "/api/status";
    worker_ = std::thread([this] {serve();});
  }

  ~StatusServer() {
    stopping_ = true;
    worker_.join();
  }

  void respond(unsigned status, std::string body, std::chrono::milliseconds delay = 0ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = status;
    body_ = std::move(body);
    delay_ = delay;
  }

  const std::string & url() const {return url_;}
  unsigned started() const {return started_.load();}
  unsigned completed() const {return completed_.load();}

private:
  void serve() {
    while (!stopping_) {
      tcp::socket socket(io_);
      boost::system::error_code error;
      acceptor_.accept(socket, error);
      if (error) {
        std::this_thread::sleep_for(5ms);
        continue;
      }
      timeval limit{};
      limit.tv_usec = 300000;
      ::setsockopt(socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO, &limit, sizeof(limit));
      ::setsockopt(socket.native_handle(), SOL_SOCKET, SO_SNDTIMEO, &limit, sizeof(limit));
      beast::flat_buffer buffer;
      http::request<http::empty_body> request;
      http::read(socket, buffer, request, error);
      if (error) continue;
      unsigned status;
      std::string body;
      std::chrono::milliseconds delay;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        status = status_;
        body = body_;
        delay = delay_;
      }
      ++started_;
      std::this_thread::sleep_for(delay);
      http::response<http::string_body> response{static_cast<http::status>(status), 11};
      response.body() = body;
      response.prepare_payload();
      http::write(socket, response, error);
      ++completed_;
    }
  }

  asio::io_context io_;
  tcp::acceptor acceptor_;
  std::string url_;
  std::mutex mutex_;
  unsigned status_ = 200;
  std::string body_ = R"({"task_state":"STANDBY"})";
  std::chrono::milliseconds delay_{0};
  std::atomic<bool> stopping_{false};
  std::atomic<unsigned> started_{0}, completed_{0};
  std::thread worker_;
};

struct ResponseCase {
  const char * name;
  unsigned status;
  const char * body;
  bool blocked;
};

class MappingStatusResponse : public ::testing::TestWithParam<ResponseCase> {};

TEST_P(MappingStatusResponse, PreservesConsoleDecision) {
  const auto & sample = GetParam();
  StatusServer server;
  server.respond(sample.status, sample.body);
  MappingStatusMonitor monitor(server.url());
  EXPECT_FALSE(monitor.blocks_inspection());
  monitor.start();
  ASSERT_TRUE(wait_until([&] {return server.completed() > 0;}));
  monitor.stop();
  EXPECT_EQ(monitor.blocks_inspection(), sample.blocked);
}

INSTANTIATE_TEST_SUITE_P(ExistingPolicy, MappingStatusResponse, ::testing::Values(
  ResponseCase{"Starting", 200, R"({"task_state":"STARTING"})", true},
  ResponseCase{"Running", 200, R"({"task_state":"RUNNING"})", true},
  ResponseCase{"Stopping", 200, R"({"task_state":"STOPPING"})", true},
  ResponseCase{"Standby", 200, R"({"task_state":"STANDBY"})", false},
  ResponseCase{"Idle", 200, R"({"task_state":"IDLE"})", false},
  ResponseCase{"UnknownState", 200, R"({"task_state":"UNKNOWN"})", false},
  ResponseCase{"MissingState", 200, "{}", false},
  ResponseCase{"HttpError", 503, R"({"task_state":"STANDBY"})", true},
  ResponseCase{"InvalidJson", 200, "invalid", true},
  ResponseCase{"NonObject", 200, "[]", true},
  ResponseCase{"WrongStateType", 200, R"({"task_state":7})", false}),
  [](const ::testing::TestParamInfo<ResponseCase> & info) {return info.param.name;});

TEST(MappingStatusMonitor, EmptyAddressAndRepeatedStopsAreSafe) {
  MappingStatusMonitor monitor("");
  monitor.stop();
  monitor.start();
  monitor.start();
  EXPECT_FALSE(monitor.blocks_inspection());
  monitor.stop();
  monitor.stop();
}

TEST(MappingStatusMonitor, PollsChangesAndStopsWithoutFurtherRequests) {
  StatusServer server;
  server.respond(200, R"({"task_state":"RUNNING"})");
  MappingStatusMonitor monitor(server.url());
  monitor.start();
  monitor.start();
  ASSERT_TRUE(wait_until([&] {return monitor.blocks_inspection();}));
  server.respond(200, R"({"task_state":"STANDBY"})");
  ASSERT_TRUE(wait_until([&] {return !monitor.blocks_inspection();}));
  server.respond(200, R"({"task_state":"STOPPING"})");
  ASSERT_TRUE(wait_until([&] {return monitor.blocks_inspection();}));
  monitor.stop();
  monitor.stop();
  const auto requests = server.started();
  server.respond(200, R"({"task_state":"STANDBY"})");
  std::this_thread::sleep_for(650ms);
  EXPECT_EQ(server.started(), requests);
  EXPECT_TRUE(monitor.blocks_inspection());
  monitor.start();
  ASSERT_TRUE(wait_until([&] {return !monitor.blocks_inspection();}));
}

TEST(MappingStatusMonitor, ConnectionFailureClearsThePreviousBusyCache) {
  auto server = std::make_unique<StatusServer>();
  server->respond(200, R"({"task_state":"RUNNING"})");
  MappingStatusMonitor monitor(server->url());
  monitor.start();
  ASSERT_TRUE(wait_until([&] {return monitor.blocks_inspection();}));
  server.reset();
  ASSERT_TRUE(wait_until([&] {return !monitor.blocks_inspection();}));
}

TEST(MappingStatusMonitor, CachedReadDoesNotWaitForHttpAndDestructorJoinsActiveQuery) {
  StatusServer server;
  auto monitor = std::make_unique<MappingStatusMonitor>(server.url());
  monitor->start();
  ASSERT_TRUE(wait_until([&] {return server.completed() > 0;}));
  const auto requests = server.started();
  server.respond(200, R"({"task_state":"RUNNING"})", 1s);
  ASSERT_TRUE(wait_until([&] {return server.started() > requests;}));
  const auto read_at = std::chrono::steady_clock::now();
  EXPECT_FALSE(monitor->blocks_inspection());
  EXPECT_LT(std::chrono::steady_clock::now() - read_at, 100ms);
  const auto stop_at = std::chrono::steady_clock::now();
  monitor.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - stop_at, 1s);
  const auto stopped_requests = server.started();
  std::this_thread::sleep_for(650ms);
  EXPECT_EQ(server.started(), stopped_requests);
}
}  // namespace
