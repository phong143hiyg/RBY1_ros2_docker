#include "rby1_motion_planning/mtc_task.hpp"
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <sstream>

namespace rby1_motion_planning {
namespace mtc=moveit::task_constructor;
void MtcResult::publish() {task->introspection().publishSolution(*task->solutions().front());}
static Json scene_metadata(const planning_scene::PlanningSceneConstPtr& scene) {
  Json data={{"world_object_poses",Json::object()},{"attached",Json::array()},{"acm",Json::object()}};
  for(const auto& id:scene->getWorld()->getObjectIds()) data["world_object_poses"][id]=pose_json(scene->getWorld()->getObject(id)->pose_);
  std::vector<const moveit::core::AttachedBody*> bodies; scene->getCurrentState().getAttachedBodies(bodies);
  for(auto* b:bodies) data["attached"].push_back({{"id",b->getName()},{"link",b->getAttachedLinkName()},
      {"pose_in_link",pose_json(b->getPose())},{"touch_links",b->getTouchLinks()}});
  moveit_msgs::msg::AllowedCollisionMatrix matrix; scene->getAllowedCollisionMatrix().getMessage(matrix);
  data["acm"]["names"]=matrix.entry_names; data["acm"]["values"]=Json::array();
  for(auto& row:matrix.entry_values) data["acm"]["values"].push_back(row.enabled);
  return data;
}
static void flatten(const mtc::SolutionBase& s,std::vector<const mtc::SubTrajectory*>& out) {
  if(auto* sequence=dynamic_cast<const mtc::SolutionSequence*>(&s)) for(auto* child:sequence->solutions()) flatten(*child,out);
  else if(auto* wrapped=dynamic_cast<const mtc::WrappedSolution*>(&s)) flatten(*wrapped->wrapped(),out);
  else if(auto* trajectory=dynamic_cast<const mtc::SubTrajectory*>(&s)) out.push_back(trajectory);
  else throw Error("MTC_METADATA","Unsupported solution node type");
}
std::shared_ptr<MtcResult> plan_mtc(const rclcpp::Node::SharedPtr& node,
   const planning_scene::PlanningScenePtr& snapshot,const Json& params,const Json& defaults,
   const Clock::time_point& deadline,const std::atomic<bool>& cancelled,double resolution) {
  auto model=snapshot->getRobotModel(); auto frame=model->getModelFrame();
  const std::string arm="right_arm", hand="gripper_r", tcp="ee_right", object="target_object";
  const std::vector<std::string> touch={"gripper_finger_r1","gripper_finger_r2"};
  auto world_object=snapshot->getWorld()->getObject(object);
  if(!world_object) throw Error("OBJECT_NOT_FOUND",object);
  if(params.value("frame",frame)!=frame) throw Error("INVALID_FRAME","MTC fixture must use model planning frame");
  auto config=defaults; config.update(params);
  double vs=config.value("velocity_scaling",.1),as=config.value("acceleration_scaling",.1);
  auto distance=[&](const char* key) {double d=config.at(key).get<double>(); if(!std::isfinite(d)||d<=0||d>.5) throw Error("INVALID_DISTANCE",key); return d;};
  double approach=distance("approach_distance"),lift=distance("lift_distance"),retreat=distance("retreat_distance");
  Eigen::Isometry3d pick,place_object,tcp_offset=Eigen::Isometry3d::Identity();
  tf2::fromMsg(parse_pose(params.at("pick_tcp")),pick); tf2::fromMsg(parse_pose(params.at("place_object")),place_object);
  if(params.contains("tcp_in_link")) tf2::fromMsg(parse_pose(params["tcp_in_link"]),tcp_offset);
  // W_T_object = W_T_tcp * tcp_T_object. Solver uses link_T_tcp to obtain the desired link transform.
  Eigen::Isometry3d tcp_to_object=pick.inverse()*world_object->pose_;
  Eigen::Isometry3d place_tcp=place_object*tcp_to_object.inverse();
  auto direction=[&](const char* key) {auto v=config.at(key).get<std::vector<double>>(); if(v.size()!=3) throw Error("INVALID_DIRECTION",key);
    Eigen::Vector3d d(v[0],v[1],v[2]); if(!d.allFinite()||std::abs(d.norm()-1.)>1e-6) throw Error("INVALID_DIRECTION","Directions must be unit vectors"); return d;};
  auto approach_dir=direction("approach_direction"),lift_dir=direction("lift_direction"),retreat_dir=direction("retreat_direction");
  auto result=std::make_shared<MtcResult>(); result->task=std::make_shared<mtc::Task>(); auto& task=*result->task;
  task.stages()->setName("RBY1 planning-only pick/place"); task.setRobotModel(model); task.setProperty("group",arm);
  task.setProperty("eef",std::string("rby1_right_tool")); task.setProperty("ik_frame",tcp);
  task.setTimeout(std::max(.001,std::chrono::duration<double>(deadline-Clock::now()).count()));
  // A FixedState captures the current robot and scene together once; all following stages propagate their diffs.
  task.add(std::make_unique<mtc::stages::FixedState>("current state snapshot",snapshot));
  auto pipeline=std::make_shared<mtc::solvers::PipelinePlanner>(node); pipeline->setPlannerId("RRTConnectkConfigDefault");
  pipeline->setMaxVelocityScalingFactor(vs); pipeline->setMaxAccelerationScalingFactor(as);
  auto joint=std::make_shared<mtc::solvers::JointInterpolationPlanner>(); joint->setMaxVelocityScalingFactor(vs); joint->setMaxAccelerationScalingFactor(as);
  auto cart=std::make_shared<mtc::solvers::CartesianPath>(); cart->setStepSize(config.value("cartesian_step",.005));
  cart->setMinFraction(1.); cart->setMaxVelocityScalingFactor(vs); cart->setMaxAccelerationScalingFactor(as);
  auto gripper=[&](const std::string& name,double position) {
    auto stage=std::make_unique<mtc::stages::MoveTo>(name,joint); stage->setGroup(hand);
    stage->restrictDirection(mtc::stages::MoveTo::FORWARD); stage->setGoal(std::map<std::string,double>{{"gripper_finger_r1_joint",position}}); task.add(std::move(stage));
  };
  auto move=[&](const std::string& name,const Eigen::Isometry3d& pose,const mtc::solvers::PlannerInterfacePtr& solver) {
    auto stage=std::make_unique<mtc::stages::MoveTo>(name,solver); stage->setGroup(arm); stage->setIKFrame(tcp_offset,tcp);
    stage->restrictDirection(mtc::stages::MoveTo::FORWARD); geometry_msgs::msg::PoseStamped goal; goal.header.frame_id=frame; goal.pose=tf2::toMsg(pose); stage->setGoal(goal); task.add(std::move(stage));
  };
  auto relative=[&](const std::string& name,const Eigen::Vector3d& d,double length) {
    auto stage=std::make_unique<mtc::stages::MoveRelative>(name,cart); stage->setGroup(arm); stage->setIKFrame(tcp_offset,tcp);
    stage->restrictDirection(mtc::stages::MoveRelative::FORWARD); stage->setMinMaxDistance(length,length);
    geometry_msgs::msg::Vector3Stamped vector; vector.header.frame_id=frame; vector.vector.x=d.x(); vector.vector.y=d.y(); vector.vector.z=d.z();
    stage->setDirection(vector); task.add(std::move(stage));
  };
  std::string support=params.value("support_surface",std::string("table"));
  auto support_contact=[&](const std::string& name,bool allow) {
    if(!snapshot->getWorld()->hasObject(support)) throw Error("OBJECT_NOT_FOUND",support);
    auto stage=std::make_unique<mtc::stages::ModifyPlanningScene>(name); stage->allowCollisions(object,support,allow); task.add(std::move(stage));
  };
  gripper("open gripper",-.04);
  auto pregrasp=pick; pregrasp.translation()-=approach_dir*approach; move("connect pre-grasp",pregrasp,pipeline);
  auto contact=std::make_unique<mtc::stages::ModifyPlanningScene>("allow target finger contact"); contact->allowCollisions(object,touch,true); task.add(std::move(contact));
  relative("Cartesian approach",approach_dir,approach); gripper("close gripper",params.value("gripper_closed",0.));
  support_contact("allow target support contact for lift",true);
  auto attach=std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  attach->setCallback([object,tcp,touch](const planning_scene::PlanningScenePtr& scene,const mtc::PropertyMap&) {
    moveit_msgs::msg::PlanningScene msg; scene->getPlanningSceneMsg(msg);
    auto it=std::find_if(msg.world.collision_objects.begin(),msg.world.collision_objects.end(),[&](auto& o){return o.id==object;});
    if(it==msg.world.collision_objects.end()) throw Error("OBJECT_NOT_FOUND",object);
    moveit_msgs::msg::AttachedCollisionObject attached; attached.object=*it; attached.link_name=tcp; attached.touch_links=touch;
    if(!scene->processAttachedCollisionObjectMsg(attached)) throw Error("ATTACH_FAILED",object);
  }); task.add(std::move(attach));
  relative("lift",lift_dir,lift); support_contact("restore support collision after lift",false);
  auto preplace=place_tcp; preplace.translation()-=approach_dir*lift; move("transfer",preplace,pipeline);
  support_contact("allow target support contact for lower",true);
  move("lower",place_tcp,cart); gripper("open gripper at place",-.04);
  auto detach=std::make_unique<mtc::stages::ModifyPlanningScene>("detach object at place pose"); detach->detachObject(object,tcp);
  detach->setCallback([object,place_object](const planning_scene::PlanningScenePtr& scene,const mtc::PropertyMap&) {
    auto actual=scene->getWorld()->getObject(object);
    if(!actual || (actual->pose_.translation()-place_object.translation()).norm()>.005 ||
       Eigen::Quaterniond(actual->pose_.rotation()).angularDistance(Eigen::Quaterniond(place_object.rotation()))>.02)
      throw Error("PLACE_TRANSFORM_MISMATCH","Detached object must match requested pose without teleporting");
  }); task.add(std::move(detach));
  relative("retreat",retreat_dir,retreat);
  auto restore=std::make_unique<mtc::stages::ModifyPlanningScene>("restore original ACM"); auto acm=snapshot->getAllowedCollisionMatrix();
  restore->setCallback([acm](const planning_scene::PlanningScenePtr& scene,const mtc::PropertyMap&){scene->getAllowedCollisionMatrixNonConst()=acm;}); task.add(std::move(restore));
  task.init(); auto begin=Clock::now(); std::atomic<bool> done{false};
  std::thread watchdog([&]{while(!done) {if(cancelled || Clock::now()>=deadline) {task.preempt();return;} std::this_thread::sleep_for(std::chrono::milliseconds(10));}});
  bool success=false;
  try {success=bool(task.plan(1));} catch(...) {done=true; watchdog.join(); throw;}
  done=true; watchdog.join();
  if(cancelled) throw Error("CANCELLED","MTC worker stopped");
  if(Clock::now()>=deadline) throw Error("TIMEOUT","MTC worker stopped after deadline");
  if(!success || task.solutions().empty()) {std::ostringstream errors; task.explainFailure(errors); throw Error("MTC_PLANNING_FAILED",errors.str());}
  std::vector<const mtc::SubTrajectory*> parts; flatten(*task.solutions().front(),parts);
  Json stages=Json::array(); auto previous=snapshot->getCurrentState();
  result->display.model_id=model->getName(); moveit::core::robotStateToRobotStateMsg(previous,result->display.trajectory_start);
  for(auto* part:parts) {
    auto from=part->start()->scene(),to=part->end()->scene(); std::string name=part->creator()->name();
    Json metadata={{"name",name},{"start",state_json(from->getCurrentState())},{"final",state_json(to->getCurrentState())},
                   {"scene_before",scene_metadata(from)},{"scene_after",scene_metadata(to)}};
    if(part->trajectory() && !part->trajectory()->empty()) {
      auto trajectory=std::make_shared<robot_trajectory::RobotTrajectory>(*part->trajectory());
      trajectory_processing::IterativeParabolicTimeParameterization retime;
      if(!retime.computeTimeStamps(*trajectory,vs*.5,as*.5)) throw Error("RETIMING_FAILED",name);
      metadata["validation"]=validate_trajectory(from,*trajectory,resolution,vs,as,&previous,[&]{
        if(cancelled) throw Error("CANCELLED","MTC validation stopped");
        if(Clock::now()>=deadline) throw Error("TIMEOUT","MTC validation deadline reached");
      }); metadata["event"]=name.find("gripper")!=std::string::npos?Json(name):Json(nullptr);
      if(trajectory->getLastWayPoint().distance(to->getCurrentState())>1e-5) throw Error("DISCONTINUITY",name+": final trajectory/state mismatch");
      moveit_msgs::msg::RobotTrajectory msg; trajectory->getRobotTrajectoryMsg(msg); result->display.trajectory.push_back(msg);
      const_cast<mtc::SubTrajectory*>(part)->setTrajectory(trajectory);
    } else {
      if(previous.distance(from->getCurrentState())>1e-5 || from->getCurrentState().distance(to->getCurrentState())>1e-5) throw Error("DISCONTINUITY",name);
      if(!to->getCurrentState().satisfiesBounds()||to->isStateColliding(to->getCurrentState(),"",false)) throw Error("EVENT_COLLISION",name);
      metadata["event"]=name; metadata["validation"]={{"valid",true},{"event_state_checked",true}};
    }
    previous=to->getCurrentState(); stages.push_back(metadata);
  }
  auto end=task.solutions().front()->end()->scene();
  if(end->getCurrentState().hasAttachedBody(object)) throw Error("DETACH_FAILED",object);
  result->metadata={{"stages",stages},{"planning_time_s",std::chrono::duration<double>(Clock::now()-begin).count()},
                    {"place_object",pose_json(place_object)},{"place_tcp",pose_json(place_tcp)},
                    {"tcp_to_object",pose_json(tcp_to_object)},{"tcp_in_link",pose_json(tcp_offset)},
                    {"final_scene",scene_metadata(end)},{"cartesian_min_fraction",1.}};
  return result;
}
}
