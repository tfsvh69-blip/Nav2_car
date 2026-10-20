// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <string>
#include "carcar_inspection/domain.hpp"
namespace carcar_inspection {
struct UploadReceipt {std::string status; std::string detail;};
// 只有对端回显同一编号和校验才算确认。超时、拒绝或校验不一致保持待传，不删除文件。
UploadReceipt post_photo(const std::string & url, const std::string & id, const std::string & checksum,
  const std::string & file);
UploadReceipt interpret_upload_response(const std::string & id, const std::string & checksum, long http_status,
  const std::string & body);
}  // namespace carcar_inspection
