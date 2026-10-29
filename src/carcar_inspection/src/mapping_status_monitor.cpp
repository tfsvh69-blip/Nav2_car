// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#include "carcar_inspection/mapping_status_monitor.hpp"

#include <chrono>
#include <utility>
#include <sys/socket.h>
#include <sys/time.h>
#include <boost/beast.hpp>
#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>

namespace carcar_inspection {

MappingStatusMonitor::MappingStatusMonitor(std::string status_url)
: status_url_(std::move(status_url)) {}

MappingStatusMonitor::~MappingStatusMonitor() {
  stop();
}

void MappingStatusMonitor::start() {
  if (status_url_.empty() || worker_.joinable()) return;
  stopping_.store(false);
  worker_ = std::thread([this] {watch();});
}

bool MappingStatusMonitor::blocks_inspection() const noexcept {
  return busy_.load();
}

void MappingStatusMonitor::stop() {
  stopping_.store(true);
  if (worker_.joinable()) worker_.join();
}

bool MappingStatusMonitor::query_busy() const {
  try {
    const auto scheme = status_url_.find("://");
    const auto start = scheme == std::string::npos ? 0 : scheme + 3;
    const auto slash = status_url_.find('/', start);
    const auto authority = status_url_.substr(start, slash - start);
    const auto colon = authority.rfind(':');
    const auto host = colon == std::string::npos ? authority : authority.substr(0, colon);
    const auto port = colon == std::string::npos ? 80 : std::stoi(authority.substr(colon + 1));
    const auto target = slash == std::string::npos ? std::string("/") : status_url_.substr(slash);
    boost::asio::io_context io;
    boost::beast::tcp_stream stream(io);
    stream.expires_after(std::chrono::milliseconds(300));
    stream.connect({boost::asio::ip::make_address(host), static_cast<unsigned short>(port)});
    // 同步读写沿用原套接字超时；不将 Beast 的 expires_after 当作同步读的保证。
    struct timeval limit{};
    limit.tv_usec = 300000;
    const auto fd = stream.socket().native_handle();
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &limit, sizeof(limit));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &limit, sizeof(limit));
    boost::beast::http::request<boost::beast::http::empty_body> request{
      boost::beast::http::verb::get, target, 11};
    request.set(boost::beast::http::field::host, authority);
    boost::beast::http::write(stream, request);
    boost::beast::flat_buffer buffer;
    boost::beast::http::response<boost::beast::http::string_body> response;
    boost::beast::http::read(stream, buffer, response);
    if (response.result() != boost::beast::http::status::ok) return true;
    const auto value = nlohmann::json::parse(response.body(), nullptr, false);
    if (!value.is_object()) return true;
    const auto task = value.value("task_state", "");
    return task == "STARTING" || task == "RUNNING" || task == "STOPPING";
  } catch (const std::exception &) {
    return false;
  }
}

void MappingStatusMonitor::watch() {
  while (!stopping_) {
    busy_.store(query_busy());
    for (int i = 0; i < 5 && !stopping_; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

}  // namespace carcar_inspection
