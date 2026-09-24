#include <moveit/controller_manager/controller_manager.h>
#include <pluginlib/class_list_macros.hpp>

namespace rby1_motion_planning {
// MoveItCpp constructs an execution manager even when execution is disabled.
// This plugin intentionally exposes no command handles and creates no action clients.
class PlanningOnlyControllerManager : public moveit_controller_manager::MoveItControllerManager {
public:
  void initialize(const rclcpp::Node::SharedPtr&) override {}
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string&) override {return {};}
  void getControllersList(std::vector<std::string>& names) override {names.clear();}
  void getActiveControllers(std::vector<std::string>& names) override {names.clear();}
  void getControllerJoints(const std::string&,std::vector<std::string>& joints) override {joints.clear();}
  ControllerState getControllerState(const std::string&) override {return {};}
  bool switchControllers(const std::vector<std::string>&,const std::vector<std::string>&) override {return false;}
};
}
PLUGINLIB_EXPORT_CLASS(rby1_motion_planning::PlanningOnlyControllerManager, moveit_controller_manager::MoveItControllerManager)
