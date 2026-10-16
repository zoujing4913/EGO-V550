#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include <MPC.hpp>

class TrackingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { int argc = 0; rclcpp::init(argc, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("MPC.v_max", 0.35), rclcpp::Parameter("MPC.w_max", 0.8),
      rclcpp::Parameter("MPC.position_weight", 20.0), rclcpp::Parameter("MPC.heading_weight", 2.0),
      rclcpp::Parameter("MPC.omega1", 0.1)});
    node = std::make_shared<rclcpp::Node>("tracking_test", options);
    controller.MPC_init(node);
  }
  rclcpp::Node::SharedPtr node;
  MPC_controller controller;
};

TEST_F(TrackingTest, PreservesReferenceSpeedWhenOnStraightTrajectory)
{
  std::vector<Eigen::Vector3d> states;
  std::vector<Eigen::Vector2d> inputs(15, Eigen::Vector2d(0.25, 0));
  for (int i = 0; i < 15; ++i) states.emplace_back(0.25*(i+1)*0.03, 0, 0);
  auto result = controller.MPC_Solve_qp(Eigen::Vector3d::Zero(), states, inputs, 15);
  EXPECT_NEAR(result(0,0), 0.25, 0.005);
  EXPECT_NEAR(result(1,0), 0, 0.001);
}

TEST_F(TrackingTest, TracksCurveWithoutCuttingAcrossIt)
{
  constexpr double dt = 0.03, speed = 0.25, radius = 1.2;
  Eigen::Vector3d actual(0, 0, 0);
  double squared_error = 0;
  for (int step = 0; step < 500; ++step) {
    std::vector<Eigen::Vector3d> states;
    std::vector<Eigen::Vector2d> inputs(15, Eigen::Vector2d(speed, speed/radius));
    for (int i = 0; i < 15; ++i) {
      const double angle = speed/radius*(step+i+1)*dt;
      states.emplace_back(radius*std::sin(angle), radius*(1-std::cos(angle)), angle);
    }
    const auto result = controller.MPC_Solve_qp(actual, states, inputs, 15);
    ASSERT_TRUE(result.allFinite());
    ASSERT_LE(std::abs(result(0,0)), 0.35001);
    ASSERT_LE(std::abs(result(1,0)), 0.80001);
    actual.x() += result(0,0)*std::cos(actual.z())*dt;
    actual.y() += result(0,0)*std::sin(actual.z())*dt;
    actual.z() += result(1,0)*dt;
    squared_error += (actual.head<2>() - states.front().head<2>()).squaredNorm();
  }
  EXPECT_LT(std::sqrt(squared_error/500), 0.04);
}

TEST_F(TrackingTest, InvalidReferenceReturnsStop)
{
  std::vector<Eigen::Vector3d> states(15, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector2d> inputs(15, Eigen::Vector2d(0.25, 0));
  inputs[0].x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(controller.MPC_Solve_qp(Eigen::Vector3d::Zero(), states, inputs, 15).isZero());
}

TEST_F(TrackingTest, RecoversLateralOffset)
{
  constexpr int horizon = 40;
  constexpr double dt = 0.03, speed = 0.25;
  Eigen::Vector3d actual(0, -0.3, 0.2);
  for (int step = 0; step < 500; ++step) {
    std::vector<Eigen::Vector3d> states;
    std::vector<Eigen::Vector2d> inputs(horizon, Eigen::Vector2d(speed, 0));
    for (int i = 0; i < horizon; ++i) states.emplace_back(speed*(step+i+1)*dt, 0, 0);
    auto result = controller.MPC_Solve_qp(actual, states, inputs, horizon);
    actual.x() += result(0,0)*std::cos(actual.z())*dt;
    actual.y() += result(0,0)*std::sin(actual.z())*dt;
    actual.z() += result(1,0)*dt;
  }
  EXPECT_LT(std::abs(actual.y()), 0.03);
  EXPECT_NEAR(actual.x(), speed*500*dt, 0.05);
}
