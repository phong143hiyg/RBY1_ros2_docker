#include "rby1_motion_planning/common.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <thread>
#include <set>

int main(int argc, char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("scene_loader",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto apply=node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  auto get=node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  auto markers=node->create_publisher<visualization_msgs::msg::MarkerArray>("/rby1_planning/markers",rclcpp::QoS(1).transient_local());
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  std::thread spin([&]{executor.spin();}); int code=0;
  try {
    rby1_motion_planning::Json doc=rby1_motion_planning::yaml_json(node->get_parameter("scene_file").as_string());
    robot_model_loader::RobotModelLoader loader(node,"robot_description"); auto model=loader.getModel();
    if(!model) throw std::runtime_error("Robot model unavailable");
    auto msg=rby1_motion_planning::scene_from_json(doc,model->getModelFrame());
    rby1_motion_planning::apply_scene(apply,msg,30.);
    auto actual=rby1_motion_planning::get_scene(get,5.); std::set<std::string> ids;
    for(auto& o:actual.world.collision_objects) ids.insert(o.id);
    for(auto& o:msg.world.collision_objects) if(!ids.count(o.id)) throw std::runtime_error("Scene readback missing "+o.id);
    markers->publish(rby1_motion_planning::fixture_markers(doc,model->getModelFrame()));
    RCLCPP_INFO(node->get_logger(),"SCENE_APPLIED_VERIFIED frame=%s objects=%zu",model->getModelFrame().c_str(),ids.size());
    // Keep transient-local markers available to late RViz subscribers.
    while(rclcpp::ok()) std::this_thread::sleep_for(std::chrono::milliseconds(200));
  } catch(const std::exception& e) {RCLCPP_ERROR(node->get_logger(),"Scene load failed: %s",e.what()); code=1;}
  executor.cancel(); spin.join(); rclcpp::shutdown(); return code;
}
