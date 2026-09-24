#include "rby1_motion_planning/common.hpp"
#include <yaml-cpp/yaml.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <set>
#include <cmath>

namespace rby1_motion_planning {
geometry_msgs::msg::Pose parse_pose(const Json& p) {
  auto xyz = p.at("position").get<std::vector<double>>();
  auto q = p.value("orientation", Json::array({0.,0.,0.,1.})).get<std::vector<double>>();
  if(xyz.size()!=3 || q.size()!=4) throw Error("INVALID_POSE", "Expected position[3], orientation[x,y,z,w]");
  for(double v: xyz) if(!std::isfinite(v)) throw Error("INVALID_POSE", "Non-finite position");
  double norm=0; for(double v:q) {if(!std::isfinite(v)) throw Error("INVALID_POSE", "Non-finite quaternion"); norm+=v*v;}
  if(std::abs(norm-1.)>.002001) throw Error("INVALID_POSE", "Quaternion norm must be 1 within 0.001");
  geometry_msgs::msg::Pose result;
  result.position.x=xyz[0]; result.position.y=xyz[1]; result.position.z=xyz[2];
  result.orientation.x=q[0]; result.orientation.y=q[1]; result.orientation.z=q[2]; result.orientation.w=q[3]; return result;
}
Json pose_json(const Eigen::Isometry3d& p) {
  Eigen::Quaterniond q(p.rotation()); auto t=p.translation();
  return {{"position",{t.x(),t.y(),t.z()}},{"orientation",{q.x(),q.y(),q.z(),q.w()}}};
}
static Json convert(const YAML::Node& node) {
  if(node.IsMap()) {Json v=Json::object(); for(auto x:node) v[x.first.as<std::string>()]=convert(x.second); return v;}
  if(node.IsSequence()) {Json v=Json::array(); for(auto x:node) v.push_back(convert(x)); return v;}
  if(node.IsNull()) return nullptr;
  auto s=node.as<std::string>();
  if(s=="true") return true;
  if(s=="false") return false;
  try {size_t used=0; double v=std::stod(s,&used); if(used==s.size()) return v;} catch(const std::exception&) {}
  return s;
}
Json yaml_json(const std::string& path) {return convert(YAML::LoadFile(path));}
moveit_msgs::msg::PlanningScene scene_from_json(const Json& doc, const std::string& frame) {
  if(doc.value("frame",frame)!=frame) throw Error("INVALID_FRAME", "Scene frame must equal model planning frame: "+frame);
  moveit_msgs::msg::PlanningScene msg; msg.is_diff=true; msg.robot_state.is_diff=true;
  std::set<std::string> ids;
  for(const auto& o: doc.at("objects")) {
    moveit_msgs::msg::CollisionObject obj; obj.id=o.at("id").get<std::string>();
    if(obj.id.empty() || !ids.insert(obj.id).second) throw Error("INVALID_SCENE", "Object IDs must be unique and nonempty");
    obj.header.frame_id=frame; obj.operation=obj.ADD;
    shape_msgs::msg::SolidPrimitive primitive;
    auto kind=o.at("type").get<std::string>();
    if(kind=="box") primitive.type=primitive.BOX;
    else if(kind=="cylinder") primitive.type=primitive.CYLINDER;
    else throw Error("INVALID_SCENE", "Supported shapes: box, cylinder");
    auto dimensions=o.at("dimensions").get<std::vector<double>>();
    if(dimensions.size()!=(kind=="box"?3u:2u)) throw Error("INVALID_SCENE", "Wrong dimensions count");
    primitive.dimensions.assign(dimensions.begin(),dimensions.end());
    if(primitive.dimensions.size()!=(kind=="box"?3u:2u)) throw Error("INVALID_SCENE", "Wrong dimensions count");
    for(auto d:primitive.dimensions) if(!std::isfinite(d)||d<=0) throw Error("INVALID_SCENE", "Dimensions must be positive finite");
    obj.primitives.push_back(primitive); obj.primitive_poses.push_back(parse_pose(o.at("pose")));
    msg.world.collision_objects.push_back(obj);
  }
  return msg;
}
moveit_msgs::msg::PlanningScene get_scene(const rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr& client, double timeout) {
  if(!client->wait_for_service(std::chrono::duration<double>(timeout))) throw Error("SCENE_TIMEOUT", "get_planning_scene unavailable");
  auto req=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>(); req->components.components=1023;
  auto future=client->async_send_request(req);
  if(future.wait_for(std::chrono::duration<double>(timeout))!=std::future_status::ready) {client->remove_pending_request(future); throw Error("SCENE_TIMEOUT", "Scene snapshot timed out");}
  return future.get()->scene;
}
void apply_scene(const rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr& client, const moveit_msgs::msg::PlanningScene& msg, double timeout) {
  if(!client->wait_for_service(std::chrono::duration<double>(timeout))) throw Error("SCENE_TIMEOUT", "apply_planning_scene unavailable");
  auto req=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>(); req->scene=msg;
  auto future=client->async_send_request(req);
  if(future.wait_for(std::chrono::duration<double>(timeout))!=std::future_status::ready) {client->remove_pending_request(future); throw Error("SCENE_TIMEOUT", "Applying scene timed out");}
  if(!future.get()->success) throw Error("SCENE_REJECTED", "move_group rejected planning scene");
}
visualization_msgs::msg::MarkerArray fixture_markers(const Json& doc, const std::string& frame) {
  visualization_msgs::msg::MarkerArray result;
  visualization_msgs::msg::Marker clear; clear.action=clear.DELETEALL; result.markers.push_back(clear);
  if(!doc.contains("markers")) return result;
  int id=0;
  for(auto it=doc["markers"].begin();it!=doc["markers"].end();++it) {
    visualization_msgs::msg::Marker marker; marker.header.frame_id=frame; marker.ns="task_targets"; marker.id=id++;
    marker.type=marker.TEXT_VIEW_FACING; marker.action=marker.ADD; marker.text=it.key(); marker.pose=parse_pose(it.value());
    marker.scale.z=.035; marker.color.r=.1; marker.color.g=.9; marker.color.b=.9; marker.color.a=1.; result.markers.push_back(marker);
    marker.id=id++; marker.type=marker.ARROW; marker.scale.x=.08; marker.scale.y=.01; marker.scale.z=.015; result.markers.push_back(marker);
  }
  return result;
}
Json state_json(const moveit::core::RobotState& state) {
  Json result=Json::object(); for(const auto& n:state.getVariableNames()) result[n]=state.getVariablePosition(n); return result;
}
}
