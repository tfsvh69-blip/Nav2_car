// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#include "carcar_inspection/upload.hpp"
#include <curl/curl.h>
#include <fstream>
#include <mutex>
#include <vector>
namespace carcar_inspection {
namespace {
size_t collect(char * data, size_t size, size_t count, void * user) {
  auto * body = static_cast<std::string *>(user);
  body->append(data, size * count);
  return size * count;
}
}
UploadReceipt interpret_upload_response(const std::string & id, const std::string & checksum, long http_status,
  const std::string & body) {
  if (http_status != 200) return {"PENDING", "对端未确认"};
  const auto value = json::parse(body, nullptr, false);
  if (!value.is_object() || value.value("id", "") != id || value.value("sha256", "") != checksum)
    return {"PENDING", "确认内容与照片不一致"};
  return {"CONFIRMED", ""};
}
UploadReceipt post_photo(const std::string & url, const std::string & id, const std::string & checksum,
  const std::string & file) {
  if (url.empty()) return {"PENDING_CONFIGURATION", "未配置上传地址"};
  static std::once_flag once; std::call_once(once, curl_global_init, CURL_GLOBAL_DEFAULT);
  std::ifstream input(file, std::ios::binary);
  if (!input) return {"PENDING", "照片文件不可读"};
  const std::vector<char> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  std::string response; long status = 0;
  CURL * curl = curl_easy_init();
  if (!curl) return {"PENDING", "上传客户端初始化失败"};
  struct curl_slist * headers = nullptr;
  const auto id_header = "X-Capture-Id: " + id;
  const auto sum_header = "X-Content-Sha256: " + checksum;
  headers = curl_slist_append(headers, id_header.c_str());
  headers = curl_slist_append(headers, sum_header.c_str());
  headers = curl_slist_append(headers, "Content-Type: image/jpeg");
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, bytes.data());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(bytes.size()));
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, collect);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 2L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
  curl_easy_setopt(curl, CURLOPT_REDIR_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
  const auto rc = curl_easy_perform(curl);
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_slist_free_all(headers); curl_easy_cleanup(curl);
  if (rc != CURLE_OK) return {"PENDING", curl_easy_strerror(rc)};
  return interpret_upload_response(id, checksum, status, response);
}
}  // namespace carcar_inspection
