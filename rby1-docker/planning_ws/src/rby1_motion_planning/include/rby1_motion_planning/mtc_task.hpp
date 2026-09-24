#pragma once
#include "rby1_motion_planning/common.hpp"
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <atomic>
namespace rby1_motion_planning {
struct MtcResult {
  std::shared_ptr<moveit::task_constructor::Task> task;
  moveit_msgs::msg::DisplayTrajectory display;
  Json metadata;
  void publish();
};
std::shared_ptr<MtcResult> plan_mtc(const rclcpp::Node::SharedPtr& node,
   const planning_scene::PlanningScenePtr& snapshot, const Json& params, const Json& defaults,
   const Clock::time_point& deadline, const std::atomic<bool>& cancelled, double resolution);
}
