// 使用本机Nav2实现验证参数链，不连接底盘；定时器只由测试逐帧调用。
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include <nav2_velocity_smoother/velocity_smoother.hpp>

namespace
{
using Twist = geometry_msgs::msg::Twist;
constexpr double kDt = 0.05;

Twist command(double linear, double angular)
{
  Twist value;
  value.linear.x = linear;
  value.angular.z = angular;
  return value;
}

class ManualSmoother : public nav2_velocity_smoother::VelocitySmoother
{
public:
  explicit ManualSmoother(const rclcpp::NodeOptions & options) : VelocitySmoother(options) {}

  void start()
  {
    configure();
    if (get_current_state().label() != "inactive") {
      throw std::runtime_error("smoother configure failed");
    }
    activate();
    if (get_current_state().label() != "active") {
      throw std::runtime_error("smoother activate failed");
    }
    timer_->cancel();
  }

  Twist step(const Twist & value)
  {
    inputCommandCallback(std::make_shared<Twist>(value));
    smootherTimer();
    return last_cmd_;
  }

  Twist timeoutStep()
  {
    last_command_time_ = now() - rclcpp::Duration::from_seconds(0.31);
    smootherTimer();
    return last_cmd_;
  }

  void stop()
  {
    deactivate();
    cleanup();
  }
};

class MappingSmoother : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    const char * domain = std::getenv("ROS_DOMAIN_ID");
    if (!domain || std::string(domain) != "209") {
      throw std::runtime_error("software smoother tests require ROS_DOMAIN_ID=209");
    }
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}

  std::shared_ptr<ManualSmoother> make(const std::vector<rclcpp::Parameter> & overrides = {})
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", NAV_BASE_CONFIG,
      "--params-file", MAPPING_CONFIG,
      "-r", "cmd_vel:=/snapshot/smoother_input",
      "-r", "cmd_vel_smoothed:=/snapshot/smoother_output"});
    options.parameter_overrides(overrides);
    auto smoother = std::make_shared<ManualSmoother>(options);
    smoother->start();
    return smoother;
  }
};

TEST_F(MappingSmoother, MappingConfigurationPreservesCurveWhenStarting)
{
  auto smoother = make();
  // R11返程首帧真实RPP输出；不是按测试期望编造的圆弧。
  const auto target = command(0.05254666, -1.0 / 3.0);
  const auto actual = smoother->step(target);
  EXPECT_GT(actual.linear.x, 0.0);
  EXPECT_NEAR(actual.angular.z, -0.04, 1e-9);
  EXPECT_NEAR(actual.linear.x * target.angular.z,
    actual.angular.z * target.linear.x, 1e-9);
  smoother->stop();
}

TEST_F(MappingSmoother, IndependentAxesReproduceWiderStartingTurn)
{
  auto smoother = make({{"scale_velocities", false}});
  const auto target = command(0.05254666, -1.0 / 3.0);
  const auto actual = smoother->step(target);
  EXPECT_NEAR(actual.linear.x, 0.01, 1e-9);
  EXPECT_NEAR(actual.angular.z, -0.04, 1e-9);
  EXPECT_GT(std::abs(actual.linear.x / actual.angular.z),
    1.5 * std::abs(target.linear.x / target.angular.z));
  smoother->stop();
}

TEST_F(MappingSmoother, RuntimeSpeedEnvelopePreservesAccelerationAndTimeoutLimits)
{
  auto smoother = make();
  Twist previous;
  const auto check = [&previous](const Twist & value) {
      EXPECT_GE(value.linear.x, -1e-9);
      EXPECT_LE(value.linear.x, 0.18 + 1e-9);
      EXPECT_LE(std::abs(value.angular.z), 0.36 + 1e-9);
      EXPECT_LE(std::abs(value.linear.x - previous.linear.x), 0.20 * kDt + 1e-9);
      EXPECT_LE(std::abs(value.angular.z - previous.angular.z), 0.80 * kDt + 1e-9);
      previous = value;
    };
  for (const auto & target : {command(.18, 0), command(0, -.36),
      command(.05, .33), command(.18, .36)})
  {
    for (int tick = 0; tick < 20; ++tick) {check(smoother->step(target));}
  }
  // 0.18m/s以原0.20m/s²减速需0.90s；给一个采样余量，不能沿用0.10的0.55s窗口。
  for (int tick = 0; tick < 19; ++tick) {check(smoother->timeoutStep());}
  EXPECT_DOUBLE_EQ(previous.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(previous.angular.z, 0.0);
  smoother->stop();
}

TEST_F(MappingSmoother, PureRotationRetainsAngularRamp)
{
  auto smoother = make();
  Twist actual;
  for (int tick = 0; tick < 9; ++tick) {
    actual = smoother->step(command(0, .36));
    EXPECT_DOUBLE_EQ(actual.linear.x, 0.0);
    EXPECT_NEAR(actual.angular.z, std::min((tick + 1) * .04, .36), 1e-9);
  }
  smoother->stop();
}
}  // namespace
