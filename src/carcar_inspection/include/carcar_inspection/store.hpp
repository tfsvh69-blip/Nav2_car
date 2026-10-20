// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <filesystem>
#include <sqlite3.h>
#include "carcar_inspection/domain.hpp"
namespace carcar_inspection {
class Store {
public:
  explicit Store(const std::filesystem::path & directory);
  ~Store();
  Store(const Store &) = delete;
  Store & operator=(const Store &) = delete;
  json list(const std::string & kind) const;
  json get(const std::string & kind, const std::string & id) const;
  json put(const std::string & kind, json value, int expected_revision);
  void erase(const std::string & kind, const std::string & id);
  void event(const std::string & type, const json & data);
  void enqueue_upload(const std::string & id, const std::string & checksum, const std::filesystem::path & file, bool configured);
  json uploads() const;
  json next_upload() const;
  void finish_upload(const std::string & id, const std::string & status);
  std::filesystem::path media_path(const std::string & id) const;
  std::filesystem::path root() const {return root_;}
private:
  void sql(const char * text) const;
  sqlite3 * db_ = nullptr;
  std::filesystem::path root_;
};
}  // namespace carcar_inspection
