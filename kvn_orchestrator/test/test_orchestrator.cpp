// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Runs the shipped trees/kvn_orchestrator.xml against a fake robot: Joy, odometry, TF and
// explore_lite status are published in-process and a fake Nav2 action server answers goals. The
// robot lives on its own executor thread because Nav2's BtActionNode cancels goals synchronously
// (it blocks the orchestrator thread until the server answers).

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <fstream>
#include <sstream>
#include <functional>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "kvn_orchestrator/tree_host.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

namespace
{

std::vector<std::string> plugins()
{
  return {
    KVN_PLUGIN_CONDITIONS, KVN_PLUGIN_ACTIONS, KVN_PLUGIN_CONTROL, KVN_PLUGIN_AUTONOMY,
    "nav2_behavior_tree:nav2_navigate_to_pose_action_bt_node",
    "nav2_behavior_tree:nav2_goal_reached_condition_bt_node"};
}

/// What orchestrator.yaml and robot.yaml put on the root blackboard.
kvn_orchestrator::TreeHostOptions default_options()
{
  kvn_orchestrator::TreeHostOptions options;
  options.tree_file = KVN_TREE_FILE;
  options.plugin_libraries = plugins();
  options.blackboard_entries = {
    {"require_robot_status", rclcpp::ParameterValue(true)},
    {"battery_warn_voltage", rclcpp::ParameterValue(13.6)},
    {"battery_critical_voltage", rclcpp::ParameterValue(12.4)},
    {"battery_cells", rclcpp::ParameterValue(4)},
    {"battery_capacity_wh", rclcpp::ParameterValue(36.0)},
    {"battery_cell_voltage_curve", rclcpp::ParameterValue(std::vector<double>{3.0, 4.2})},
    {"battery_wh_per_m", rclcpp::ParameterValue(0.02)},
    {"battery_path_factor", rclcpp::ParameterValue(1.5)},
    {"battery_reserve_wh", rclcpp::ParameterValue(4.0)},
  };
  return options;
}

enum class GoalEnd {kRunning, kSucceed, kAbort};

class OrchestratorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    robot_ = std::make_shared<rclcpp::Node>("fake_robot");
    scan_pub_ = robot_->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    robot_status_pub_ = robot_->create_publisher<std_msgs::msg::String>("/robot_status", 10);
    battery_pub_ = robot_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
    mode_sub_ = robot_->create_subscription<std_msgs::msg::String>(
      "/orchestrator/mode", rclcpp::QoS(10).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = m->data;
      });
    permit_sub_ = robot_->create_subscription<std_msgs::msg::Bool>(
      "/orchestrator/autonomy_enabled", rclcpp::QoS(10).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        permit_ = m->data;
      });
    orch_node_ = std::make_shared<nav2::LifecycleNode>("kvn_orchestrator_test_node");
    orch_node_->configure();
    orch_node_->activate();
    executor_.add_node(orch_node_->get_node_base_interface());

    joy_pub_ = robot_->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
    odom_pub_ = robot_->create_publisher<nav_msgs::msg::Odometry>("/odom_ekf", 10);
    explore_pub_ = robot_->create_publisher<explore_lite_msgs::msg::ExploreStatus>(
      "/explore/status", rclcpp::QoS(10).reliable().transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(robot_);

    status_sub_ = robot_->create_subscription<std_msgs::msg::String>(
      "/orchestrator/status", rclcpp::QoS(10).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        statuses_.push_back(m->data);
      });
    resume_sub_ = robot_->create_subscription<std_msgs::msg::Bool>(
      "/explore/resume", 10, [this](const std_msgs::msg::Bool::SharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        resume_.push_back(m->data);
      });

    // The "hardware": streams Joy/odom/TF at 20 Hz and finishes Nav2 goals on request.
    pump_ = robot_->create_wall_timer(50ms, [this]() {pump();});
    robot_executor_.add_node(robot_);
    robot_thread_ = std::thread([this]() {robot_executor_.spin();});
  }

  void TearDown() override
  {
    host_.reset();
    robot_executor_.cancel();
    robot_thread_.join();
    nav_handle_.reset();
  }

  /// Creates the fake Nav2 NavigateToPose server (on the robot thread's node).
  void start_nav_server()
  {
    nav_server_ = rclcpp_action::create_server<NavigateToPose>(
      robot_, "/navigate_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal> goal) {
        std::lock_guard<std::mutex> lock(mutex_);
        goals_.push_back(goal->pose);
        return reject_goals_ ? rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<NavHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<NavHandle> handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        nav_handle_ = handle;
      });
  }

  void start(double tick_period_s = 0.02)
  {
    if (!delay_server_) {
      start_nav_server();
    }
    auto options = default_options();
    options.tick_period_s = tick_period_s;
    host_ = std::make_unique<kvn_orchestrator::TreeHost>(orch_node_, options);
  }

  /// Robot thread, every 50 ms.
  void pump()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (joy_enabled_) {
      sensor_msgs::msg::Joy joy;
      joy.buttons = buttons_;
      joy.axes = axes_;
      joy_pub_->publish(joy);
    }
    if (scan_enabled_) {
      scan_pub_->publish(sensor_msgs::msg::LaserScan());
    }
    std_msgs::msg::String robot_status;
    robot_status.data = robot_status_;
    robot_status_pub_->publish(robot_status);
    sensor_msgs::msg::BatteryState battery;
    battery.voltage = static_cast<float>(battery_voltage_);
    battery.percentage = std::numeric_limits<float>::quiet_NaN();
    battery_pub_->publish(battery);

    nav_msgs::msg::Odometry odom;
    odom.twist.twist.linear.x = moving_ ? 0.5 : 0.0;
    odom_pub_->publish(odom);

    if (tf_enabled_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = robot_->now();
      tf.header.frame_id = "map";
      tf.child_frame_id = "base_link";
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(tf);
    }

    if (nav_handle_ && nav_handle_->is_canceling()) {
      nav_handle_->canceled(std::make_shared<NavigateToPose::Result>());
      nav_handle_.reset();
      ++cancels_;
    } else if (nav_handle_ && nav_handle_->is_active() && goal_end_ != GoalEnd::kRunning) {
      auto result = std::make_shared<NavigateToPose::Result>();
      if (goal_end_ == GoalEnd::kSucceed) {
        if (!arrived_) {
          // Move first and report success on the next pump, once the new TF has been sent.
          x_ = arrive_x_;
          y_ = arrive_y_;
          arrived_ = true;
          return;
        }
        arrived_ = false;
        nav_handle_->succeed(result);
      } else {
        result->error_code = NavigateToPose::Result::UNKNOWN;
        nav_handle_->abort(result);
      }
      nav_handle_.reset();
      goal_end_ = GoalEnd::kRunning;
    }
  }

  template<typename T>
  T locked(const std::function<T()> & read)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return read();
  }

  // Test-thread accessors to state shared with the robot thread.
  void set_buttons(std::vector<int> b) {locked<int>([&] {buttons_ = std::move(b); return 0;});}
  void set_pose(double x, double y) {locked<int>([&] {x_ = x; y_ = y; return 0;});}
  void set_moving(bool v) {locked<int>([&] {moving_ = v; return 0;});}
  void set_tf(bool v) {locked<int>([&] {tf_enabled_ = v; return 0;});}
  void set_joy(bool v) {locked<int>([&] {joy_enabled_ = v; return 0;});}
  /// axes[1] forward stick, axes[0] steering (left +), axes[4] mode switch.
  void set_axes(std::vector<float> a) {locked<int>([&] {axes_ = std::move(a); return 0;});}
  void set_mode_switch(float v) {locked<int>([&] {axes_.at(4) = v; return 0;});}
  void set_scan(bool v) {locked<int>([&] {scan_enabled_ = v; return 0;});}
  void set_robot_status(const std::string & v) {locked<int>([&] {robot_status_ = v; return 0;});}
  void set_battery(double v) {locked<int>([&] {battery_voltage_ = v; return 0;});}
  std::string mode() {return locked<std::string>([&] {return mode_;});}
  bool permit() {return locked<bool>([&] {return permit_;});}
  void end_goal(GoalEnd end, double arrive_x = 0.0, double arrive_y = 0.0)
  {
    locked<int>([&] {goal_end_ = end; arrive_x_ = arrive_x; arrive_y_ = arrive_y; return 0;});
  }
  bool has_status(const std::string & s)
  {
    return locked<bool>([&] {
        return std::find(statuses_.begin(), statuses_.end(), s) != statuses_.end();
      });
  }
  size_t goal_count() {return locked<size_t>([&] {return goals_.size();});}
  geometry_msgs::msg::PoseStamped goal(size_t i)
  {
    return locked<geometry_msgs::msg::PoseStamped>([&] {return goals_.at(i);});
  }
  bool goal_active() {return locked<bool>([&] {return nav_handle_ != nullptr;});}
  int cancels() {return locked<int>([&] {return cancels_;});}
  std::vector<bool> resume() {return locked<std::vector<bool>>([&] {return resume_;});}
  long resume_true_count()
  {
    const auto r = resume();
    return std::count(r.begin(), r.end(), true);
  }

  bool spin_until(const std::function<bool()> & done, std::chrono::milliseconds timeout = 6s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(10ms);
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    spin_until([] {return false;}, duration);
  }

  void publish_explore(const std::string & status)
  {
    explore_lite_msgs::msg::ExploreStatus msg;
    msg.status = status;
    explore_pub_->publish(msg);
    spin_for(200ms);
  }

  /// Boots the tree and waits until the startup home is taken at (x, y).
  void capture_startup_home(double x, double y)
  {
    set_buttons({0, 0, 0, 0});
    set_pose(x, y);
    start();
    ASSERT_TRUE(spin_until([this] {return has_status("startup_home_captured");}));
  }

  std::shared_ptr<rclcpp::Node> robot_;
  nav2::LifecycleNode::SharedPtr orch_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::executors::SingleThreadedExecutor robot_executor_;
  std::thread robot_thread_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permit_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<explore_lite_msgs::msg::ExploreStatus>::SharedPtr explore_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr nav_server_;
  rclcpp::TimerBase::SharedPtr pump_;
  std::unique_ptr<kvn_orchestrator::TreeHost> host_;

  std::mutex mutex_;  // guards everything below
  std::vector<int> buttons_{0, 0, 0, 0};
  // Mode switch high (autonomous) unless a test says otherwise.
  std::vector<float> axes_{0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  bool scan_enabled_{true};
  std::string robot_status_{"RDY"};
  double battery_voltage_{16.0};
  std::string mode_;
  bool permit_{false};
  bool moving_{false};
  bool tf_enabled_{true};
  bool joy_enabled_{true};
  bool reject_goals_{false};
  bool delay_server_{false};
  double x_{0.0}, y_{0.0};
  double arrive_x_{0.0}, arrive_y_{0.0};
  GoalEnd goal_end_{GoalEnd::kRunning};
  bool arrived_{false};
  std::vector<std::string> statuses_;
  std::vector<bool> resume_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  std::shared_ptr<NavHandle> nav_handle_;
  int cancels_{0};
};

}  // namespace

TEST_F(OrchestratorTest, ReportsStartupAndCapturesHomeOnceStationary)
{
  set_moving(true);
  start();
  spin_for(1500ms);
  EXPECT_TRUE(has_status("starting_up"));
  EXPECT_FALSE(has_status("startup_home_captured")) << "must wait for the rover to be still";

  set_moving(false);
  ASSERT_TRUE(spin_until([this] {return has_status("startup_home_captured");}));
}

TEST_F(OrchestratorTest, WaitsForLocalizationBeforeCapturingHome)
{
  set_tf(false);
  start();
  ASSERT_TRUE(spin_until([this] {return has_status("waiting_for_localization");}));
  EXPECT_FALSE(has_status("startup_home_captured"));

  set_tf(true);
  ASSERT_TRUE(spin_until([this] {return has_status("startup_home_captured");}));
}

TEST_F(OrchestratorTest, StartupHomeIsNotCapturedWhileTeleopIsEnabled)
{
  set_buttons({1, 0, 0, 0});
  start();
  spin_for(2s);
  EXPECT_FALSE(has_status("startup_home_captured"));
}

TEST_F(OrchestratorTest, WaitsForTheNav2ServerThenStarts)
{
  delay_server_ = true;
  start();
  spin_for(1500ms);
  EXPECT_FALSE(host_->ready()) << "no NavigateToPose server yet";
  EXPECT_FALSE(has_status("starting_up"));

  start_nav_server();
  ASSERT_TRUE(spin_until([this] {return host_->ready();}, 10s));
  ASSERT_TRUE(spin_until([this] {return has_status("startup_home_captured");}));
}

TEST_F(OrchestratorTest, ReturnHomeDrivesToHomeOncePerPress)
{
  capture_startup_home(1.0, 2.0);
  publish_explore("exploration_in_progress");
  set_pose(5.0, 5.0);
  end_goal(GoalEnd::kRunning);

  set_buttons({0, 0, 1, 0});
  ASSERT_TRUE(spin_until([this] {return goal_count() == 1u;})) << "no goal after the settle time";
  const auto sent = goal(0);
  EXPECT_EQ(sent.header.frame_id, "map");
  EXPECT_NEAR(sent.pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(sent.pose.position.y, 2.0, 1e-6);
  EXPECT_TRUE(has_status("returning_home"));
  ASSERT_FALSE(resume().empty());
  EXPECT_FALSE(resume().front()) << "exploration must be paused first";

  end_goal(GoalEnd::kSucceed, 1.0, 2.0);  // the rover arrives at home
  ASSERT_TRUE(spin_until([this] {return has_status("home_reached");}));
  spin_for(800ms);
  EXPECT_EQ(goal_count(), 1u) << "holding the switch must not start a second trip";
  EXPECT_EQ(resume_true_count(), 0) << "exploration stays paused after reaching home";

  set_buttons({0, 0, 0, 0});
  spin_for(300ms);
  set_buttons({0, 0, 1, 0});
  ASSERT_TRUE(spin_until([this] {return goal_count() == 2u;}));
}

TEST_F(OrchestratorTest, NavSuccessWithoutArrivalIsNotReportedAsHomeReached)
{
  capture_startup_home(1.0, 2.0);
  set_pose(5.0, 5.0);
  set_buttons({0, 0, 1, 0});
  ASSERT_TRUE(spin_until([this] {return goal_active();}));
  // Nav2 reports "success" for a goal preempted by someone else; the rover is still far away.
  end_goal(GoalEnd::kSucceed, 5.0, 5.0);
  ASSERT_TRUE(spin_until([this] {return has_status("return_cancelled");}));
  EXPECT_FALSE(has_status("home_reached"));
}

TEST_F(OrchestratorTest, FailedGoalIsReportedAndExplorationResumes)
{
  capture_startup_home(1.0, 2.0);
  publish_explore("exploration_in_progress");
  set_buttons({0, 0, 1, 0});
  ASSERT_TRUE(spin_until([this] {return goal_active();}));
  end_goal(GoalEnd::kAbort);
  ASSERT_TRUE(spin_until([this] {return has_status("return_failed");}));
  ASSERT_TRUE(spin_until([this] {return resume_true_count() == 1;}));
}

TEST_F(OrchestratorTest, RejectedGoalIsReported)
{
  reject_goals_ = true;
  capture_startup_home(1.0, 2.0);
  set_buttons({0, 0, 1, 0});
  ASSERT_TRUE(spin_until([this] {return has_status("return_rejected");}));
}

TEST_F(OrchestratorTest, ReleasingTheSwitchCancelsNavigationAndResumesExploration)
{
  capture_startup_home(1.0, 2.0);
  publish_explore("exploration_in_progress");

  set_buttons({0, 0, 1, 0});
  ASSERT_TRUE(spin_until([this] {return goal_active();}));

  set_buttons({0, 0, 0, 0});
  ASSERT_TRUE(spin_until([this] {return cancels() == 1;})) << "Nav2 goal was not canceled";
  ASSERT_TRUE(spin_until([this] {return has_status("return_cancelled");}));
  ASSERT_TRUE(spin_until([this] {return resume().size() >= 2u;}));
  EXPECT_FALSE(resume().front());
  EXPECT_TRUE(resume().back()) << "exploration was running before, so it must resume";
}

TEST_F(OrchestratorTest, CancelBeforeNavigationStartsDoesNotSendAGoal)
{
  capture_startup_home(1.0, 2.0);
  set_buttons({0, 0, 1, 0});
  ASSERT_TRUE(spin_until([this] {return has_status("returning_home");}));
  set_buttons({0, 0, 0, 0});
  ASSERT_TRUE(spin_until([this] {return has_status("return_cancelled");}));
  spin_for(1s);
  EXPECT_EQ(goal_count(), 0u);
  // Exploration was not running, so it must not be resumed.
  EXPECT_EQ(resume_true_count(), 0);
}

TEST_F(OrchestratorTest, EnablingTeleopCancelsNavigation)
{
  capture_startup_home(1.0, 2.0);
  set_buttons({0, 0, 1, 0});
  ASSERT_TRUE(spin_until([this] {return goal_active();}));
  set_buttons({1, 0, 1, 0});
  ASSERT_TRUE(spin_until([this] {return cancels() == 1;}));
}

TEST_F(OrchestratorTest, LosingJoyMessagesCancelsNavigation)
{
  capture_startup_home(1.0, 2.0);
  set_buttons({0, 0, 1, 0});
  ASSERT_TRUE(spin_until([this] {return goal_active();}));
  set_joy(false);
  ASSERT_TRUE(spin_until([this] {return cancels() == 1;}, 4s)) << "stale Joy must stop the rover";
}

TEST_F(OrchestratorTest, TeleopEnabledPreventsReturnHome)
{
  capture_startup_home(1.0, 2.0);
  set_buttons({1, 0, 1, 0});
  spin_for(1500ms);
  EXPECT_EQ(goal_count(), 0u);
  EXPECT_FALSE(has_status("returning_home"));
}

TEST_F(OrchestratorTest, HoldingTheChordReplacesHome)
{
  capture_startup_home(1.0, 2.0);
  set_pose(3.0, 4.0);

  set_buttons({0, 0, 1, 1});
  spin_for(1000ms);
  EXPECT_FALSE(has_status("home_set")) << "the chord must be held for 1.5 s";
  ASSERT_TRUE(spin_until([this] {return has_status("home_set");}));
  EXPECT_EQ(goal_count(), 0u) << "the chord must not start a return trip";

  set_buttons({0, 0, 1, 0});
  ASSERT_TRUE(spin_until([this] {return goal_count() == 1u;}));
  EXPECT_NEAR(goal(0).pose.position.x, 3.0, 1e-6);
  EXPECT_NEAR(goal(0).pose.position.y, 4.0, 1e-6);
}

TEST_F(OrchestratorTest, ChordIsIgnoredWhileMoving)
{
  capture_startup_home(1.0, 2.0);
  set_moving(true);
  set_buttons({0, 0, 1, 1});
  spin_for(2500ms);
  EXPECT_FALSE(has_status("home_set"));
}

TEST_F(OrchestratorTest, ModeSwitchSelectsModeAndAutonomyPermit)
{
  capture_startup_home(0.0, 0.0);
  ASSERT_TRUE(spin_until([this] {return mode() == "autonomous" && permit();}));

  set_mode_switch(0.0f);
  ASSERT_TRUE(spin_until([this] {return mode() == "assisted_teleop" && permit();}));

  set_mode_switch(-1.0f);
  ASSERT_TRUE(spin_until([this] {return mode() == "teleop" && !permit();}));

  // AUX1 always withdraws the permit; in an autonomous mode it is reported as an override.
  set_mode_switch(1.0f);
  set_buttons({1, 0, 0, 0});
  ASSERT_TRUE(spin_until([this] {return mode() == "teleop_override" && !permit();}));
  set_buttons({0, 0, 0, 0});
  ASSERT_TRUE(spin_until([this] {return mode() == "autonomous" && permit();}));
}

TEST_F(OrchestratorTest, TeleopModeBlocksReturnHome)
{
  capture_startup_home(1.0, 2.0);
  set_mode_switch(-1.0f);
  set_buttons({0, 0, 1, 0});
  spin_for(1500ms);
  EXPECT_EQ(goal_count(), 0u);
  EXPECT_FALSE(has_status("returning_home"));
}

TEST_F(OrchestratorTest, InitialisationHoldsAutonomyUntilEveryInputIsHealthy)
{
  set_scan(false);
  start();
  ASSERT_TRUE(spin_until([this] {return has_status("waiting_for_lidar");}));
  EXPECT_EQ(mode(), "initialising");
  EXPECT_FALSE(permit());

  set_scan(true);
  ASSERT_TRUE(spin_until([this] {return mode() == "autonomous" && permit();}));

  // A fault after start-up blocks autonomy as "fault"...
  set_robot_status("FLT:HWMON");
  ASSERT_TRUE(spin_until([this] {return mode() == "fault" && !permit();}));
  EXPECT_TRUE(has_status("robot_fault"));
  // ...but a battery fault must still let the rover drive home.
  set_robot_status("FLT:BATT");
  ASSERT_TRUE(spin_until([this] {return mode() == "autonomous";}));
}

TEST_F(OrchestratorTest, AssistedTeleopSendsAGoalAheadAndCancelsWhenSticksCentre)
{
  capture_startup_home(0.0, 0.0);
  set_axes({0.0f, 0.0f, 0.0f, 0.0f, 0.0f});  // assisted, sticks centred
  ASSERT_TRUE(spin_until([this] {return mode() == "assisted_teleop";}));
  spin_for(500ms);
  EXPECT_EQ(goal_count(), 0u) << "centred sticks must not move the rover";

  set_axes({0.0f, 1.0f, 0.0f, 0.0f, 0.0f});  // full forward
  ASSERT_TRUE(spin_until([this] {return goal_active();}));
  const auto sent = goal(0);
  EXPECT_NEAR(sent.pose.position.x, 1.5, 1e-6) << "lookahead ahead of the rover";
  EXPECT_NEAR(sent.pose.position.y, 0.0, 1e-6);

  set_axes({0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
  ASSERT_TRUE(spin_until([this] {return cancels() == 1;}));
}

TEST_F(OrchestratorTest, LowBatteryDrivesHomeOnlyWhenAutonomyIsAllowed)
{
  capture_startup_home(1.0, 2.0);
  set_pose(5.0, 5.0);
  set_mode_switch(-1.0f);
  set_battery(12.0);  // below the critical voltage
  spin_for(6s);
  EXPECT_EQ(goal_count(), 0u) << "teleop mode: the operator decides";

  set_mode_switch(1.0f);
  ASSERT_TRUE(spin_until([this] {return goal_active();}, 10s)) << "after the 5 s hold";
  EXPECT_NEAR(goal(0).pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(goal(0).pose.position.y, 2.0, 1e-6);
  EXPECT_EQ(mode(), "battery_rth");
  EXPECT_TRUE(has_status("battery_return_critical"));

  end_goal(GoalEnd::kSucceed, 1.0, 2.0);
  ASSERT_TRUE(spin_until([this] {return mode() == "battery_rth_complete" && !permit();}));
  spin_for(800ms);
  EXPECT_EQ(goal_count(), 1u) << "stays home";
}

/// Copies the shipped trees to a temp dir, applies a text edit to the main file and returns its
/// path, to show that behaviour is changed by XML alone.
std::string edited_tree(
  const std::string & name, const std::string & from, const std::string & to)
{
  namespace fs = std::filesystem;
  const fs::path src = fs::path(KVN_TREE_FILE).parent_path();
  const fs::path dst = fs::temp_directory_path() / ("kvn_orchestrator_test_" + name);
  fs::remove_all(dst);
  fs::copy(src, dst, fs::copy_options::recursive);
  std::ifstream in(dst / "kvn_orchestrator.xml");
  std::stringstream text;
  text << in.rdbuf();
  auto xml = text.str();
  const auto pos = xml.find(from);
  EXPECT_NE(pos, std::string::npos) << "'" << from << "' not found in the shipped tree";
  xml.replace(pos, from.size(), to);
  std::ofstream(dst / "kvn_orchestrator.xml") << xml;
  return (dst / "kvn_orchestrator.xml").string();
}

TEST_F(OrchestratorTest, XmlAloneRemapsTheReturnButton)
{
  auto options = default_options();
  options.tree_file = edited_tree("remap", "return_buttons=\"2\"", "return_buttons=\"1\"");
  options.plugin_libraries = plugins();
  options.tick_period_s = 0.02;
  start_nav_server();
  set_pose(1.0, 0.0);
  host_ = std::make_unique<kvn_orchestrator::TreeHost>(orch_node_, options);
  ASSERT_TRUE(spin_until([this] {return has_status("startup_home_captured");}));

  set_buttons({0, 0, 1, 0});
  spin_for(1500ms);
  EXPECT_EQ(goal_count(), 0u) << "AUX3 no longer returns home";
  set_buttons({0, 1, 0, 0});
  ASSERT_TRUE(spin_until([this] {return goal_count() == 1u;}));
}

TEST_F(OrchestratorTest, BadXmlValueFailsAtStartup)
{
  auto options = default_options();
  options.tree_file = edited_tree("bad", "joy_timeout_s=\"1.0\"", "joy_timeout_s=\"-1\"");
  options.plugin_libraries = plugins();
  start_nav_server();  // port values are validated once the tree could be built
  EXPECT_THROW(kvn_orchestrator::TreeHost(orch_node_, options), BT::RuntimeError);
}

TEST_F(OrchestratorTest, UnknownNodeInXmlFailsAtStartup)
{
  auto options = default_options();
  options.tree_file = edited_tree("unknown", "<AlwaysSuccess name=\"Idle\"/>", "<NoSuchNode/>");
  options.plugin_libraries = plugins();
  EXPECT_ANY_THROW(kvn_orchestrator::TreeHost(orch_node_, options));
}

TEST_F(OrchestratorTest, RejectsMissingTreeFile)
{
  auto options = default_options();
  options.tree_file = "/nonexistent/tree.xml";
  options.plugin_libraries = plugins();
  EXPECT_THROW(kvn_orchestrator::TreeHost(orch_node_, options), std::invalid_argument);
}

TEST_F(OrchestratorTest, RejectsUnknownPlugin)
{
  auto options = default_options();
  options.plugin_libraries = {"/nonexistent/libnothing.so"};
  EXPECT_ANY_THROW(kvn_orchestrator::TreeHost(orch_node_, options));
}

TEST_F(OrchestratorTest, ResolvesPluginNames)
{
  using kvn_orchestrator::TreeHost;
  EXPECT_EQ(TreeHost::resolve_plugin("/a/b/libx.so"), "/a/b/libx.so");
  EXPECT_NE(TreeHost::resolve_plugin("rclcpp:rclcpp").find("/lib/librclcpp.so"), std::string::npos);
}

namespace
{

/// Builds a tiny tree around OnAbort with a guard flag the test controls.
struct OnAbortFixture
{
  explicit OnAbortFixture(const std::string & body)
  {
    factory.registerFromPlugin(KVN_PLUGIN_CONTROL);
    factory.registerSimpleCondition(
      "Guard", [this](BT::TreeNode &) {
        return guard ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory.registerSimpleAction(
      "Handler", [this](BT::TreeNode &) {
        ++handler_runs;
        return BT::NodeStatus::SUCCESS;
      });
    tree = factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><ReactiveSequence><Guard/><OnAbort>" +
      body + "<Handler/></OnAbort></ReactiveSequence></BehaviorTree></root>");
  }

  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  bool guard{true};
  int handler_runs{0};
};

}  // namespace

TEST(OnAbort, RunsHandlerWhenHalted)
{
  OnAbortFixture f("<Sleep msec=\"60000\"/>");
  EXPECT_EQ(f.tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(f.handler_runs, 0);
  f.guard = false;
  EXPECT_EQ(f.tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(f.handler_runs, 1);
  EXPECT_EQ(f.tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(f.handler_runs, 1) << "a halted body must trigger the handler only once";
}

TEST(OnAbort, RunsHandlerWhenBodyFails)
{
  OnAbortFixture f("<AlwaysFailure/>");
  EXPECT_EQ(f.tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(f.handler_runs, 1);
}

TEST(OnAbort, SkipsHandlerWhenBodySucceeds)
{
  OnAbortFixture f("<AlwaysSuccess/>");
  EXPECT_EQ(f.tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.handler_runs, 0);
}

TEST(OnAbort, RequiresTwoChildren)
{
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(KVN_PLUGIN_CONTROL);
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><OnAbort><AlwaysSuccess/></OnAbort>"
    "</BehaviorTree></root>");
  EXPECT_ANY_THROW(tree.tickOnce());
}
