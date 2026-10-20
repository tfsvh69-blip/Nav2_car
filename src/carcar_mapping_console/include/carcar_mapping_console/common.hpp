#pragma once
#include <chrono>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <openssl/evp.h>
#include <nlohmann/json.hpp>
namespace carcar_mapping_console {
using json = nlohmann::json;
inline double steady() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline std::string nonce() {
  std::random_device random;
  std::ostringstream out;
  for (int i = 0; i < 4; ++i) out << std::hex << std::setw(8) << std::setfill('0') << random();
  return out.str();
}
inline std::string read_file(const std::filesystem::path & p) {
  std::ifstream file(p, std::ios::binary);
  if (!file) throw std::runtime_error("无法读取文件: " + p.string());
  return std::string(std::istreambuf_iterator<char>(file), {});
}
inline std::string sha256(const std::filesystem::path & p) {
  std::ifstream file(p, std::ios::binary);
  if (!file) throw std::runtime_error("工程文件缺失");
  auto * context = EVP_MD_CTX_new();
  if (!context) throw std::runtime_error("摘要初始化失败");
  EVP_DigestInit_ex(context, EVP_sha256(), nullptr);
  char buffer[65536];
  while (file) {file.read(buffer, sizeof(buffer)); EVP_DigestUpdate(context, buffer, file.gcount());}
  unsigned char digest[EVP_MAX_MD_SIZE]; unsigned int count = 0;
  EVP_DigestFinal_ex(context, digest, &count); EVP_MD_CTX_free(context);
  std::ostringstream out;
  for (unsigned int i = 0; i < count; ++i) out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]);
  return out.str();
}
inline std::filesystem::path validate_bundle(const std::filesystem::path & root, const std::string & id) {
  if (id.empty() || id.size() > 100 || id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos)
    throw std::runtime_error("工程 ID 无效");
  auto directory = root / id;
  if (std::filesystem::is_symlink(directory)) throw std::runtime_error("不接受工程目录符号链接");
  const auto manifest = json::parse(read_file(directory / "manifest.json"));
  if (!manifest.value("complete", false) || manifest.value("slam_version", "") != "2.6.10" || manifest.value("base_frame", "") != "base_footprint")
    throw std::runtime_error("工程不完整或版本/坐标系不兼容");
  for (const auto & name : {"map.yaml", "map.pgm", "graph.posegraph", "graph.data"}) {
    const auto path = directory / name;
    if (std::filesystem::is_symlink(path) || !std::filesystem::is_regular_file(path) || std::filesystem::file_size(path) == 0 ||
        manifest.at("sha256").at(name).get<std::string>() != sha256(path)) throw std::runtime_error("工程校验失败: " + std::string(name));
  }
  return directory;
}
inline std::string frame_id(int32_t sec, uint32_t nano) {return std::to_string(sec) + ":" + std::to_string(nano);}
inline std::pair<double, double> pixel_to_map(double u, double v, int height, double resolution, double ox, double oy, double yaw) {
  const double x = u * resolution, y = (height - v) * resolution;
  return {ox + std::cos(yaw) * x - std::sin(yaw) * y, oy + std::sin(yaw) * x + std::cos(yaw) * y};
}
}  // namespace carcar_mapping_console
