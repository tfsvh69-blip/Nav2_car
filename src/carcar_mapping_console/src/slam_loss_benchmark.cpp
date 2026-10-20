#include <pluginlib/class_loader.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Cholesky>
#include <karto_sdk/Mapper.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {
using Pose = karto::Pose2;
using Scan = karto::LocalizedRangeScan;
using Vertex = karto::Vertex<Scan>;
using Edge = karto::Edge<Scan>;
using json = nlohmann::json;
namespace fs = std::filesystem;
constexpr double pi = 3.14159265358979323846;
double degrees(double radians) {return radians * 180.0 / pi;}
double wrap(double angle) {return std::remainder(angle, 2.0 * pi);}

struct Graph {
  std::vector<Pose> truth;
  std::vector<std::unique_ptr<Scan>> scans;
  std::vector<std::unique_ptr<Vertex>> vertices;
  std::vector<std::unique_ptr<Edge>> edges;
};

Pose relative(const Pose &from, const Pose &to) {
  const double dx = to.GetX() - from.GetX(), dy = to.GetY() - from.GetY();
  const double c = std::cos(from.GetHeading()), s = std::sin(from.GetHeading());
  return {c * dx + s * dy, -s * dx + c * dy, wrap(to.GetHeading() - from.GetHeading())};
}

void add_edge(Graph &graph, size_t from, size_t to, const Pose &a, const Pose &b) {
  karto::Matrix3 covariance;
  covariance.SetToIdentity();
  covariance(0, 0) = covariance(1, 1) = 0.03 * 0.03;
  covariance(2, 2) = 0.02 * 0.02;
  auto edge = std::make_unique<Edge>(graph.vertices.at(from).get(), graph.vertices.at(to).get());
  edge->SetLabel(new karto::LinkInfo(a, b, covariance));
  graph.edges.push_back(std::move(edge));
}

Graph fixture(const std::string &scene, unsigned seed) {
  Graph graph;
  std::vector<Pose> measurements;
  std::mt19937 random(seed);
  std::normal_distribution<double> noise(0.0, 1.0);
  for (int i = 0; i <= 20; ++i) {
    const double along = std::max(0, i - 10) * 0.5;
    const Pose truth(std::min(i, 10) * 0.5, along, i < 10 ? 0.0 : pi / 2.0);
    graph.truth.push_back(truth);
    // 第二段偏转30度，对应90度拐角被估计为120度；这里是真值已知的合成图。
    const Pose warped(5.0 - along * std::sin(pi / 6.0), along * std::cos(pi / 6.0),
      i < 10 ? 0.0 : 2.0 * pi / 3.0);
    const bool wrong_measurements = scene == "coherent_wrong_turn";
    measurements.push_back(wrong_measurements && i >= 10 ? warped : truth);
    Pose initial = (scene == "clean_far_initial" || wrong_measurements) && i >= 10 ? warped : truth;
    if (i != 0 && !wrong_measurements) {
      initial = {initial.GetX() + 0.02 * noise(random), initial.GetY() + 0.02 * noise(random),
        wrap(initial.GetHeading() + 0.01 * noise(random))};
    }
    auto scan = std::make_unique<Scan>(karto::Name("benchmark"), karto::RangeReadingsVector{});
    scan->SetUniqueId(static_cast<unsigned>(i));
    scan->SetOdometricPose(initial);
    scan->SetCorrectedPose(initial);
    graph.vertices.push_back(std::make_unique<Vertex>(scan.get()));
    graph.scans.push_back(std::move(scan));
  }
  for (size_t i = 1; i < graph.truth.size(); ++i) {
    add_edge(graph, i - 1, i, measurements[i - 1], measurements[i]);
    if (i >= 2) add_edge(graph, i - 2, i, measurements[i - 2], measurements[i]);
  }
  if (scene == "one_bad_link") {
    const auto &correct = graph.truth[18];
    const Pose wrong(correct.GetX() + 1.0, correct.GetY() + 0.7, correct.GetHeading() + pi / 6.0);
    add_edge(graph, 2, 18, graph.truth[2], wrong);
  }
  if (scene == "clean_noisy_constraints") {
    // 正常测量也有噪声：逐条连接独立扰动，避免只测试完全自洽的理想图。
    for (auto &edge : graph.edges) {
      const auto from = static_cast<size_t>(edge->GetSource()->GetObject()->GetUniqueId());
      const auto to = static_cast<size_t>(edge->GetTarget()->GetObject()->GetUniqueId());
      const auto &target = graph.truth[to];
      const Pose measured(target.GetX() + 0.01 * noise(random), target.GetY() + 0.01 * noise(random),
        wrap(target.GetHeading() + 0.005 * noise(random)));
      auto *link = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
      karto::Matrix3 covariance;
      covariance.SetToIdentity();
      covariance(0, 0) = covariance(1, 1) = 0.03 * 0.03;
      covariance(2, 2) = 0.02 * 0.02;
      link->Update(graph.truth[from], measured, covariance);
    }
  }
  return graph;
}

struct Solution {
  std::map<int, Pose> poses;
  double milliseconds = 0;
};

Solution solve(pluginlib::ClassLoader<karto::ScanSolver> &loader, const std::string &loss,
  const std::vector<Vertex *> &vertices, const std::vector<Edge *> &edges)
{
  rclcpp::NodeOptions options;
  options.start_parameter_services(false).start_parameter_event_publisher(false);
  options.parameter_overrides({rclcpp::Parameter("ceres_loss_function", loss),
    rclcpp::Parameter("ceres_linear_solver", "SPARSE_NORMAL_CHOLESKY"),
    rclcpp::Parameter("ceres_preconditioner", "SCHUR_JACOBI"),
    rclcpp::Parameter("ceres_trust_strategy", "LEVENBERG_MARQUARDT")});
  auto node = std::make_shared<rclcpp::Node>("mapping_loss_benchmark", options);
  node->declare_parameter("debug_logging", false);
  auto solver = loader.createSharedInstance("solver_plugins::CeresSolver");
  solver->Configure(node);
  for (auto *vertex : vertices) solver->AddNode(vertex);
  for (auto *edge : edges) solver->AddConstraint(edge);
  const auto before = std::chrono::steady_clock::now();
  solver->Compute();
  Solution result;
  result.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count();
  for (const auto &[id, pose] : solver->GetCorrections()) result.poses.emplace(id, pose);
  if (result.poses.size() != vertices.size()) throw std::runtime_error("优化结果缺少节点");
  for (const auto &[id, pose] : result.poses) {
    (void)id;
    if (!std::isfinite(pose.GetX()) || !std::isfinite(pose.GetY()) || !std::isfinite(pose.GetHeading()))
      throw std::runtime_error("优化结果包含非有限值");
  }
  return result;
}

json accuracy(const Solution &solution, const std::vector<Pose> &truth) {
  double distance_squared = 0, yaw_squared = 0, worst_yaw = 0;
  for (size_t i = 0; i < truth.size(); ++i) {
    const auto &pose = solution.poses.at(static_cast<int>(i));
    const double dx = pose.GetX() - truth[i].GetX(), dy = pose.GetY() - truth[i].GetY();
    const double yaw = degrees(wrap(pose.GetHeading() - truth[i].GetHeading()));
    distance_squared += dx * dx + dy * dy;
    yaw_squared += yaw * yaw;
    worst_yaw = std::max(worst_yaw, std::abs(yaw));
  }
  return {{"position_rmse_m", std::sqrt(distance_squared / truth.size())},
    {"yaw_rmse_deg", std::sqrt(yaw_squared / truth.size())}, {"worst_yaw_error_deg", worst_yaw},
    {"solve_ms", solution.milliseconds}};
}

void write_csv(const fs::path &path, const Solution &solution, const std::vector<Pose> *truth = nullptr) {
  std::ofstream out(path);
  out << "id,x,y,yaw,truth_x,truth_y,truth_yaw\n" << std::setprecision(15);
  for (const auto &[id, pose] : solution.poses) {
    out << id << ',' << pose.GetX() << ',' << pose.GetY() << ',' << pose.GetHeading();
    if (truth) {
      const auto &reference = truth->at(static_cast<size_t>(id));
      out << ',' << reference.GetX() << ',' << reference.GetY() << ',' << reference.GetHeading();
    } else out << ",,,";
    out << '\n';
  }
  if (!out.good()) throw std::runtime_error("CSV输出失败");
}

json graph_consistency(const Solution &solution, const std::vector<Vertex *> &vertices,
  const std::vector<Edge *> &edges)
{
  double shift_squared = 0, yaw_shift_squared = 0, normalized_squared = 0;
  size_t above_three = 0;
  for (auto *vertex : vertices) {
    const auto *scan = vertex->GetObject();
    const auto &before = scan->GetCorrectedPose();
    const auto &after = solution.poses.at(scan->GetUniqueId());
    shift_squared += std::pow(after.GetX() - before.GetX(), 2) + std::pow(after.GetY() - before.GetY(), 2);
    yaw_shift_squared += std::pow(degrees(wrap(after.GetHeading() - before.GetHeading())), 2);
  }
  for (auto *edge : edges) {
    const auto observed = relative(solution.poses.at(edge->GetSource()->GetObject()->GetUniqueId()),
      solution.poses.at(edge->GetTarget()->GetObject()->GetUniqueId()));
    auto *link = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
    if (!link) throw std::runtime_error("图约束类型不支持");
    const auto &expected = link->GetPoseDifference();
    const Eigen::Vector3d residual(observed.GetX() - expected.GetX(), observed.GetY() - expected.GetY(),
      wrap(observed.GetHeading() - expected.GetHeading()));
    const auto covariance = link->GetCovariance();
    Eigen::Matrix3d matrix;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col) matrix(row, col) = covariance(row, col);
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(matrix);
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive())
      throw std::runtime_error("约束协方差不是正定矩阵");
    const double squared = residual.dot(decomposition.solve(residual));
    if (!std::isfinite(squared) || squared < -1e-8) throw std::runtime_error("约束协方差无效");
    normalized_squared += std::max(0.0, squared);
    if (squared > 9.0) ++above_three;
  }
  return {{"pose_shift_rmse_m", std::sqrt(shift_squared / vertices.size())},
    {"yaw_shift_rmse_deg", std::sqrt(yaw_shift_squared / vertices.size())},
    {"standardized_residual_rms", std::sqrt(normalized_squared / edges.size())},
    {"edges_with_normalized_residual_gt3", above_three}, {"solve_ms", solution.milliseconds},
    {"has_physical_ground_truth", false}};
}
}  // namespace

int main(int argc, char **argv) {
  try {
    fs::path output, graph_prefix;
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (i + 1 >= argc) throw std::runtime_error("参数缺少值");
      if (key == "--output") output = argv[++i];
      else if (key == "--graph-prefix") graph_prefix = argv[++i];
      else throw std::runtime_error("未知参数 " + key);
    }
    if (output.empty()) throw std::runtime_error("需要 --output；可选 --graph-prefix 指向可信的本地保存图");
    if (!graph_prefix.empty() && !fs::is_regular_file(graph_prefix.string() + ".posegraph"))
      throw std::runtime_error("位姿图不存在");
    std::ifstream package_stream(fs::path(ament_index_cpp::get_package_share_directory("slam_toolbox")) / "package.xml");
    const std::string package_xml((std::istreambuf_iterator<char>(package_stream)), std::istreambuf_iterator<char>());
    if (package_xml.find("<version>2.6.10</version>") == std::string::npos)
      throw std::runtime_error("比较入口仅验收SLAM Toolbox 2.6.10，拒绝未验证版本");
    fs::create_directories(output);
    rclcpp::init(0, nullptr);
    pluginlib::ClassLoader<karto::ScanSolver> loader("slam_toolbox", "karto::ScanSolver");
    const std::array<std::string, 3> losses{"None", "HuberLoss", "CauchyLoss"};
    const std::array<std::string, 5> scenes{"clean_near_initial", "clean_far_initial", "clean_noisy_constraints", "one_bad_link", "coherent_wrong_turn"};
    const std::array<unsigned, 5> seeds{11, 23, 47, 59, 71};
    json report{{"benchmark_version", 1}, {"slam_toolbox_version", "2.6.10"},
      {"upstream_robust_loss_scale", 0.7}, {"synthetic_has_known_truth", true},
      {"physical_validation", false}, {"trials", json::array()}, {"aggregates", json::object()},
      {"selection_policy", "两种理想无异常场景最坏位置RMSE<=0.03m且最坏航向误差<=1deg；正常带噪约束比None最坏误差增加不超过20%加0.01m/0.2deg；错误连接场景位置及航向最坏误差至少减半，按位置RMSE选择；共同错误观测仅作边界，不参与胜者排名"}};
    for (const auto &loss : losses) {
      for (const auto &scene : scenes) {
        json aggregate{{"worst_position_rmse_m", 0.0}, {"worst_yaw_error_deg", 0.0}};
        for (const auto seed : seeds) {
          auto graph = fixture(scene, seed);
          std::vector<Vertex *> vertices;
          std::vector<Edge *> edges;
          for (const auto &vertex : graph.vertices) vertices.push_back(vertex.get());
          for (const auto &edge : graph.edges) edges.push_back(edge.get());
          const auto solution = solve(loader, loss, vertices, edges);
          auto result = accuracy(solution, graph.truth);
          result.update({{"scene", scene}, {"seed", seed}, {"loss", loss}});
          report["trials"].push_back(result);
          aggregate["worst_position_rmse_m"] = std::max(aggregate["worst_position_rmse_m"].get<double>(), result["position_rmse_m"].get<double>());
          aggregate["worst_yaw_error_deg"] = std::max(aggregate["worst_yaw_error_deg"].get<double>(), result["worst_yaw_error_deg"].get<double>());
          write_csv(output / (scene + "_" + loss + "_" + std::to_string(seed) + ".csv"), solution, &graph.truth);
        }
        report["aggregates"][loss][scene] = aggregate;
      }
    }
    const auto &baseline = report["aggregates"]["None"]["one_bad_link"];
    std::string selected;
    double best = std::numeric_limits<double>::infinity();
    bool fixtures_valid = baseline["worst_yaw_error_deg"].get<double>() > 1.0;
    for (const auto &loss : losses) {
      const auto &aggregate = report["aggregates"][loss];
      bool clean_ok = true;
      for (const auto &scene : {"clean_near_initial", "clean_far_initial"})
        clean_ok = clean_ok && aggregate[scene]["worst_position_rmse_m"].get<double>() <= 0.03 &&
          aggregate[scene]["worst_yaw_error_deg"].get<double>() <= 1.0;
      report["eligibility"][loss]["clean_ok"] = clean_ok;
      fixtures_valid = fixtures_valid && aggregate["coherent_wrong_turn"]["worst_yaw_error_deg"].get<double>() > 20.0;
      if (loss == "None") {fixtures_valid = fixtures_valid && clean_ok; continue;}
      const auto &noisy = aggregate["clean_noisy_constraints"];
      const auto &noisy_baseline = report["aggregates"]["None"]["clean_noisy_constraints"];
      const bool noisy_ok = noisy["worst_position_rmse_m"].get<double>() <=
        noisy_baseline["worst_position_rmse_m"].get<double>() * 1.2 + 0.01 &&
        noisy["worst_yaw_error_deg"].get<double>() <= noisy_baseline["worst_yaw_error_deg"].get<double>() * 1.2 + 0.2;
      report["eligibility"][loss]["noisy_ok"] = noisy_ok;
      const auto &bad = aggregate["one_bad_link"];
      const bool improved = bad["worst_position_rmse_m"].get<double>() <= baseline["worst_position_rmse_m"].get<double>() * 0.5 &&
        bad["worst_yaw_error_deg"].get<double>() <= baseline["worst_yaw_error_deg"].get<double>() * 0.5;
      report["eligibility"][loss]["bad_link_improved"] = improved;
      if (clean_ok && noisy_ok && improved && bad["worst_position_rmse_m"].get<double>() < best) {
        selected = loss; best = bad["worst_position_rmse_m"].get<double>();
      }
    }
    report["selected_synthetic_candidate"] = selected.empty() ? json(nullptr) : json(selected);
    fixtures_valid = fixtures_valid && !selected.empty();
    report["fixtures_valid"] = fixtures_valid;
    if (!graph_prefix.empty()) {
      auto mapper = std::make_unique<karto::Mapper>();
      mapper->LoadFromFile(graph_prefix.string() + ".posegraph");
      auto *graph = mapper->GetGraph();
      if (!graph || graph->GetEdges().empty()) throw std::runtime_error("保存图为空");
      std::vector<Vertex *> vertices;
      for (const auto &[name, sensor_vertices] : graph->GetVertices()) {
        (void)name;
        for (const auto &[id, vertex] : sensor_vertices) {(void)id; vertices.push_back(vertex);}
      }
      std::sort(vertices.begin(), vertices.end(), [](const Vertex *a, const Vertex *b) {
        return a->GetObject()->GetUniqueId() < b->GetObject()->GetUniqueId();
      });
      report["saved_graph"] = {{"nodes", vertices.size()}, {"edges", graph->GetEdges().size()},
        {"note", "保存图的再优化敏感性，无独立真值；残差及变化量不能作为真实地图精度排名"}};
      for (const auto &loss : losses) {
        const auto solution = solve(loader, loss, vertices, graph->GetEdges());
        report["saved_graph"][loss] = graph_consistency(solution, vertices, graph->GetEdges());
        write_csv(output / ("saved_graph_" + loss + ".csv"), solution);
      }
    }
    std::ofstream stream(output / "report.json");
    stream << report.dump(2) << '\n';
    if (!stream.good()) throw std::runtime_error("报告写入失败");
    std::cout << report["aggregates"].dump(2) << '\n'
              << "selected_synthetic_candidate=" << report["selected_synthetic_candidate"] << '\n';
    rclcpp::shutdown();
    return fixtures_valid ? 0 : 1;
  } catch (const karto::Exception &error) {
    std::cerr << error.GetErrorMessage() << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 1;
}
