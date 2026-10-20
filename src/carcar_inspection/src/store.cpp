// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#include "carcar_inspection/store.hpp"
#include <memory>
namespace carcar_inspection {
namespace {
using Statement = std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)>;
Statement prepare(sqlite3 * db, const char * query) {
  sqlite3_stmt * raw = nullptr;
  require(sqlite3_prepare_v2(db, query, -1, &raw, nullptr) == SQLITE_OK, sqlite3_errmsg(db));
  return Statement(raw, sqlite3_finalize);
}
void bind_text(sqlite3_stmt * stmt, int index, const std::string & text) {
  require(sqlite3_bind_text(stmt, index, text.c_str(), -1, SQLITE_TRANSIENT) == SQLITE_OK, "数据库绑定失败");
}
void kind_valid(const std::string & kind) {
  require(kind == "points" || kind == "tasks" || kind == "runs" || kind == "settings", "记录类型无效");
}
}
Store::Store(const std::filesystem::path & directory) : root_(directory) {
  require(!std::filesystem::is_symlink(root_), "数据目录不能为符号链接");
  std::filesystem::create_directories(root_ / "media");
  try {
    require(sqlite3_open_v2((root_ / "inspection.sqlite3").c_str(), &db_,
      SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX, nullptr) == SQLITE_OK, "数据库打开失败");
    sqlite3_busy_timeout(db_, 1500);
    sql("PRAGMA journal_mode=WAL; PRAGMA synchronous=FULL; PRAGMA foreign_keys=ON;");
    sql("CREATE TABLE IF NOT EXISTS records(kind TEXT NOT NULL,id TEXT NOT NULL,revision INTEGER NOT NULL,body TEXT NOT NULL,PRIMARY KEY(kind,id));"
      "CREATE TABLE IF NOT EXISTS events(sequence INTEGER PRIMARY KEY AUTOINCREMENT,created TEXT DEFAULT CURRENT_TIMESTAMP,type TEXT NOT NULL,body TEXT NOT NULL);"
      "CREATE TABLE IF NOT EXISTS uploads(id TEXT PRIMARY KEY,sha256 TEXT NOT NULL,path TEXT NOT NULL,status TEXT NOT NULL,attempts INTEGER NOT NULL DEFAULT 0);");
    // 被中断的执行不会在重启时重放任何动作。
    for (auto run : list("runs")) if (run.value("status", "") == "RUNNING" || run.value("status", "") == "PAUSED" ||
      run.value("status", "") == "PAUSING" || run.value("status", "") == "CANCELING" || run.value("status", "") == "STOPPING") {
      const int revision = run.at("revision"); run["status"] = "INTERRUPTED";
      run["error"] = "车端程序已重启，旧执行未自动恢复"; put("runs", run, revision);
    }
  } catch (...) {if (db_) sqlite3_close(db_); db_ = nullptr; throw;}
}
Store::~Store() {if (db_) sqlite3_close(db_);}
void Store::sql(const char * text) const {
  char * error = nullptr; const int rc = sqlite3_exec(db_, text, nullptr, nullptr, &error);
  const std::string reason = error ? error : sqlite3_errmsg(db_); sqlite3_free(error);
  require(rc == SQLITE_OK, reason);
}
json Store::list(const std::string & kind) const {
  kind_valid(kind); auto query = prepare(db_, "SELECT body FROM records WHERE kind=? ORDER BY id");
  bind_text(query.get(), 1, kind); json result = json::array(); int rc;
  while ((rc = sqlite3_step(query.get())) == SQLITE_ROW)
    result.push_back(json::parse(reinterpret_cast<const char *>(sqlite3_column_text(query.get(), 0))));
  require(rc == SQLITE_DONE, sqlite3_errmsg(db_)); return result;
}
json Store::get(const std::string & kind, const std::string & id) const {
  kind_valid(kind); valid_id(id); auto query = prepare(db_, "SELECT body FROM records WHERE kind=? AND id=?");
  bind_text(query.get(), 1, kind); bind_text(query.get(), 2, id); const auto rc = sqlite3_step(query.get());
  if (rc == SQLITE_DONE) return nullptr;
  require(rc == SQLITE_ROW, sqlite3_errmsg(db_));
  return json::parse(reinterpret_cast<const char *>(sqlite3_column_text(query.get(), 0)));
}
json Store::put(const std::string & kind, json value, int expected_revision) {
  kind_valid(kind); const std::string id = value.at("id"); valid_id(id);
  if (kind == "points") {value = normalize_point(std::move(value)); validate_point(value);}
  if (kind == "tasks") validate_task(value);
  sql("BEGIN IMMEDIATE");
  try {
    const auto previous = get(kind, id);
    const int revision = previous.is_null() ? 0 : previous.at("revision").get<int>();
    require(revision == expected_revision, "配置已变化，请刷新后再保存");
    value["revision"] = revision + 1;
    auto query = prepare(db_, "INSERT INTO records(kind,id,revision,body) VALUES(?,?,?,?) ON CONFLICT(kind,id) DO UPDATE SET revision=excluded.revision,body=excluded.body");
    bind_text(query.get(), 1, kind); bind_text(query.get(), 2, id);
    sqlite3_bind_int(query.get(), 3, revision + 1); bind_text(query.get(), 4, value.dump());
    require(sqlite3_step(query.get()) == SQLITE_DONE, sqlite3_errmsg(db_));
    sql("COMMIT"); return value;
  } catch (...) {sql("ROLLBACK"); throw;}
}
void Store::erase(const std::string & kind, const std::string & id) {
  kind_valid(kind); valid_id(id);
  auto query = prepare(db_, "DELETE FROM records WHERE kind=? AND id=?");
  bind_text(query.get(), 1, kind); bind_text(query.get(), 2, id);
  require(sqlite3_step(query.get()) == SQLITE_DONE, sqlite3_errmsg(db_));
  require(sqlite3_changes(db_) == 1, "记录不存在");
}
void Store::event(const std::string & type, const json & data) {
  auto query = prepare(db_, "INSERT INTO events(type,body) VALUES(?,?)");
  bind_text(query.get(), 1, type); bind_text(query.get(), 2, data.dump());
  require(sqlite3_step(query.get()) == SQLITE_DONE, sqlite3_errmsg(db_));
}
void Store::enqueue_upload(const std::string & id, const std::string & checksum, const std::filesystem::path & file, bool configured) {
  valid_id(id); require(checksum.size()==64, "照片校验无效"); require(file.filename()==id+".jpg", "照片路径与编号不一致");
  auto query = prepare(db_, "INSERT INTO uploads(id,sha256,path,status,attempts) VALUES(?,?,?,?,0) ON CONFLICT(id) DO NOTHING");
  bind_text(query.get(), 1, id); bind_text(query.get(), 2, checksum); bind_text(query.get(), 3, file.string());
  bind_text(query.get(), 4, configured ? "PENDING" : "PENDING_CONFIGURATION");
  require(sqlite3_step(query.get())==SQLITE_DONE, sqlite3_errmsg(db_));
}
json Store::uploads() const {
  auto query = prepare(db_, "SELECT id,sha256,status,attempts FROM uploads ORDER BY id");
  json result = json::array(); int rc;
  while ((rc = sqlite3_step(query.get())) == SQLITE_ROW) result.push_back({
    {"id", reinterpret_cast<const char *>(sqlite3_column_text(query.get(), 0))},
    {"sha256", reinterpret_cast<const char *>(sqlite3_column_text(query.get(), 1))},
    {"status", reinterpret_cast<const char *>(sqlite3_column_text(query.get(), 2))},
    {"attempts", sqlite3_column_int(query.get(), 3)}});
  require(rc == SQLITE_DONE, sqlite3_errmsg(db_)); return result;
}
json Store::next_upload() const {
  auto query = prepare(db_, "SELECT id,sha256,path,attempts FROM uploads WHERE status='PENDING' ORDER BY id LIMIT 1");
  const auto rc = sqlite3_step(query.get()); if (rc == SQLITE_DONE) return nullptr;
  require(rc == SQLITE_ROW, sqlite3_errmsg(db_));
  return {{"id", reinterpret_cast<const char *>(sqlite3_column_text(query.get(), 0))},
    {"sha256", reinterpret_cast<const char *>(sqlite3_column_text(query.get(), 1))},
    {"path", reinterpret_cast<const char *>(sqlite3_column_text(query.get(), 2))},
    {"attempts", sqlite3_column_int(query.get(), 3)}};
}
void Store::finish_upload(const std::string & id, const std::string & status) {
  valid_id(id); require(status=="PENDING"||status=="CONFIRMED"||status=="PENDING_CONFIGURATION", "上传状态无效");
  auto query = prepare(db_, "UPDATE uploads SET status=?, attempts=attempts+1 WHERE id=?");
  bind_text(query.get(), 1, status); bind_text(query.get(), 2, id);
  require(sqlite3_step(query.get())==SQLITE_DONE, sqlite3_errmsg(db_));
  require(sqlite3_changes(db_)==1, "上传记录不存在");
}
std::filesystem::path Store::media_path(const std::string & id) const {
  valid_id(id); const auto path = root_ / "media" / (id + ".jpg");
  require(!std::filesystem::is_symlink(path), "照片不能为符号链接"); return path;
}
}  // namespace carcar_inspection
