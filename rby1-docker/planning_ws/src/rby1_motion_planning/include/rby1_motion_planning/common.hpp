#pragma once
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <stdexcept>
#include <functional>

namespace rby1_motion_planning {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
struct Error : std::runtime_error {
  std::string code;
  Error(std::string c, const std::string& m) : std::runtime_error(m), code(std::move(c)) {}
};
geometry_msgs::msg::Pose parse_pose(const Json& p);
Json pose_json(const Eigen::Isometry3d& pose);
moveit_msgs::msg::PlanningScene scene_from_json(const Json& doc, const std::string& frame);
Json yaml_json(const std::string& path);
moveit_msgs::msg::PlanningScene get_scene(const rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr& client, double timeout);
void apply_scene(const rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr& client, const moveit_msgs::msg::PlanningScene& msg, double timeout);
visualization_msgs::msg::MarkerArray fixture_markers(const Json& doc, const std::string& frame);
Json validate_trajectory(const planning_scene::PlanningSceneConstPtr& scene,
                         const robot_trajectory::RobotTrajectory& trajectory,
                         double resolution, double velocity_scaling, double acceleration_scaling,
                         const moveit::core::RobotState* previous = nullptr,
                         const std::function<void()>& checkpoint = {});
Json state_json(const moveit::core::RobotState& state);
}
