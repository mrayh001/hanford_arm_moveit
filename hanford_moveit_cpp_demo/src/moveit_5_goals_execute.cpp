#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <chrono>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

static std::vector<std::map<std::string, double>> make_joint_goals(
  const moveit::core::RobotModelPtr& model,
  const std::string& group_name,
  int n_goals)
{
  std::vector<std::map<std::string, double>> goals;

  const auto* jmg = model->getJointModelGroup(group_name);
  if (!jmg)
    throw std::runtime_error("JointModelGroup not found: " + group_name);

  const auto var_names = jmg->getVariableNames();
  if (var_names.empty())
    throw std::runtime_error("Group has no variables: " + group_name);

  // Use variable bounds (works for continuous + bounded variables).
  std::vector<moveit::core::VariableBounds> vb;
  vb.reserve(var_names.size());
  for (const auto& v : var_names)
    vb.push_back(model->getVariableBounds(v));

  std::vector<double> mids(var_names.size(), 0.0);
  std::vector<double> amps(var_names.size(), 0.5);

  for (size_t i = 0; i < vb.size(); ++i)
  {
    const auto& b = vb[i];
    if (b.position_bounded_)
    {
      const double mn = b.min_position_;
      const double mx = b.max_position_;
      const double mid = 0.5 * (mn + mx);
      double amp = 0.25 * (mx - mn);
      if (amp < 1e-9) amp = 0.5;

      mids[i] = mid;
      amps[i] = amp;
    }
    else
    {
      // Unbounded (continuous joint): center at 0, modest amplitude.
      mids[i] = 0.0;
      amps[i] = 0.5;
    }
  }

  for (int g = 0; g < n_goals; ++g)
  {
    const double phase = g * (M_PI / 2.5);  // spreads 5 goals nicely
    std::map<std::string, double> target;

    for (size_t i = 0; i < var_names.size(); ++i)
    {
      double v = mids[i] + 0.8 * amps[i] * std::sin(phase + (double)i * 0.6);

      // Clamp if bounded
      if (vb[i].position_bounded_)
      {
        if (v < vb[i].min_position_) v = vb[i].min_position_;
        if (v > vb[i].max_position_) v = vb[i].max_position_;
      }

      target[var_names[i]] = v;
    }

    goals.push_back(std::move(target));
  }

  return goals;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("moveit_5_goals_execute");

  // Parameters
  node->declare_parameter<std::string>("planning_group", "hanford_manipulator");
  node->declare_parameter<int>("goals", 5);
  node->declare_parameter<int>("attempts_per_goal", 3);
  node->declare_parameter<double>("pause_s", 0.5);
  node->declare_parameter<double>("planning_time_s", 5.0);
  node->declare_parameter<double>("vel_scale", 0.3);
  node->declare_parameter<double>("acc_scale", 0.3);

  const std::string group_name = node->get_parameter("planning_group").as_string();
  const int n_goals = node->get_parameter("goals").as_int();
  const int attempts_per_goal = node->get_parameter("attempts_per_goal").as_int();
  const double pause_s = node->get_parameter("pause_s").as_double();
  const double planning_time_s = node->get_parameter("planning_time_s").as_double();
  const double vel_scale = node->get_parameter("vel_scale").as_double();
  const double acc_scale = node->get_parameter("acc_scale").as_double();

  // Spin in background (MoveIt needs callbacks)
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  try
  {
    // Load robot model from robot_description (SRDF is loaded by move_group)
    robot_model_loader::RobotModelLoader loader(node, "robot_description");
    moveit::core::RobotModelPtr model = loader.getModel();
    if (!model)
      throw std::runtime_error("Failed to load RobotModel from robot_description.");

    if (!model->hasJointModelGroup(group_name))
    {
      std::string msg = "Planning group not found in RobotModel: " + group_name + "\nAvailable groups:\n";
      for (const auto& g : model->getJointModelGroupNames())
        msg += "  - " + g + "\n";
      throw std::runtime_error(msg);
    }

    RCLCPP_INFO(node->get_logger(), "Using planning group: %s", group_name.c_str());

    // MoveGroupInterface talks to move_group action/services
    moveit::planning_interface::MoveGroupInterface move_group(node, group_name);
    move_group.setPlanningTime(planning_time_s);
    move_group.setMaxVelocityScalingFactor(vel_scale);
    move_group.setMaxAccelerationScalingFactor(acc_scale);

    // Generate goals inside joint limits
    auto goals = make_joint_goals(model, group_name, n_goals);

    for (int i = 0; i < (int)goals.size(); ++i)
    {
      RCLCPP_INFO(node->get_logger(), "=== Goal %d/%d ===", i + 1, (int)goals.size());

      move_group.setStartStateToCurrentState();
      move_group.setJointValueTarget(goals[i]);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool planned = false;

      for (int a = 0; a < attempts_per_goal; ++a)
      {
        auto code = move_group.plan(plan);
        planned = (code == moveit::core::MoveItErrorCode::SUCCESS);
        if (planned) break;
        RCLCPP_WARN(node->get_logger(), "Planning attempt %d failed, retrying...", a + 1);
      }

      if (!planned)
      {
        RCLCPP_ERROR(node->get_logger(), "Planning failed for goal %d. Skipping.", i + 1);
        continue;
      }

      RCLCPP_INFO(node->get_logger(), "Executing goal %d...", i + 1);
      auto exec_code = move_group.execute(plan);
      if (exec_code != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(node->get_logger(), "Execution failed for goal %d.", i + 1);
      }

      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(pause_s)));
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(node->get_logger(), "Exception: %s", e.what());
  }

  exec.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return 0;
}
