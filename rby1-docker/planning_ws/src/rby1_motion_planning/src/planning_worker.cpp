#include "rby1_motion_planning/common.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <condition_variable>
#include <deque>
#include <thread>
#include <fstream>
#include <optional>
#ifdef RBY1_HAS_MTC
#include "rby1_motion_planning/mtc_task.hpp"
#endif

namespace rby1_motion_planning {
struct StoredPlan {
  moveit_msgs::msg::DisplayTrajectory display;
  Json metadata;
  Json scene_signature;
  moveit::core::RobotState start;
  Clock::time_point expires;
#ifdef RBY1_HAS_MTC
  std::shared_ptr<MtcResult> mtc;
#endif
  StoredPlan(const moveit::core::RobotState& s):start(s) {}
};
class Backend {
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;
  moveit::core::RobotModelPtr model_;
  std::shared_ptr<planning_pipeline::PlanningPipeline> pipeline_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr responses_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr requests_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  std::mutex mutex_, joints_mutex_; std::condition_variable cv_;
  std::deque<Json> queue_; std::thread worker_; bool stop_=false;
  std::string active_; Json statuses_=Json::object(); std::atomic<bool> cancel_{false};
  std::map<std::string,std::pair<double,Clock::time_point>> joint_values_;
  std::map<std::string,std::unique_ptr<StoredPlan>> plans_;
  uint64_t revision_=1; std::string model_id_; Json initial_acm_, caps_; Json task_parameters_;
  moveit_msgs::msg::AllowedCollisionMatrix baseline_acm_;
  double timeout_=5., resolution_=.02, vs_=.1, as_=.1, ttl_=120.;

  void emit(const Json& value) {std_msgs::msg::String msg; msg.data=value.dump(); responses_->publish(msg);}
  Json envelope(const std::string& id,const std::string& type) {return {{"v",1},{"id",id},{"type",type},{"execution_enabled",false}};}
  void fail(const std::string& id,const std::string& code,const std::string& message) {
    auto out=envelope(id,"response"); out["ok"]=false; out["error"]={{"code",code},{"message",message}}; emit(out);
  }
  static Json signature(const moveit_msgs::msg::PlanningScene& msg) {
    Json result={{"objects",Json::array()},{"attached",Json::array()},{"acm_names",msg.allowed_collision_matrix.entry_names},
                 {"acm",Json::array()},{"defaults",msg.allowed_collision_matrix.default_entry_names},
                 {"default_values",msg.allowed_collision_matrix.default_entry_values}};
    auto object=[](const moveit_msgs::msg::CollisionObject& o) {
      Json x={{"id",o.id},{"frame",o.header.frame_id},{"shapes",Json::array()},{"poses",Json::array()}};
      for(auto& p:o.primitives) x["shapes"].push_back({{"type",p.type},{"dimensions",p.dimensions}});
      auto pose=[&](const geometry_msgs::msg::Pose& p) {x["poses"].push_back({p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w});};
      pose(o.pose); for(auto& p:o.primitive_poses) pose(p);
      // Include complete geometry for detecting external scene edits.
      x["meshes"]=Json::array(); x["planes"]=Json::array();
      for(auto& mesh:o.meshes) {
        Json vertices=Json::array(),triangles=Json::array();
        for(auto& v:mesh.vertices) vertices.push_back({v.x,v.y,v.z});
        for(auto& t:mesh.triangles) triangles.push_back(t.vertex_indices);
        x["meshes"].push_back({{"vertices",vertices},{"triangles",triangles}});
      }
      for(auto& p:o.mesh_poses) pose(p);
      for(auto& p:o.planes) x["planes"].push_back(p.coef);
      for(auto& p:o.plane_poses) pose(p);
      x["subframes"]=o.subframe_names; for(auto& p:o.subframe_poses) pose(p);
      return x;
    };
    auto objects=msg.world.collision_objects;
    std::sort(objects.begin(),objects.end(),[](const auto& a,const auto& b){return a.id<b.id;});
    for(auto& o:objects) result["objects"].push_back(object(o));
    for(auto& o:msg.robot_state.attached_collision_objects) result["attached"].push_back({{"link",o.link_name},{"touch_links",o.touch_links},{"object",object(o.object)}});
    result["octomap"]={{"id",msg.world.octomap.octomap.id},{"resolution",msg.world.octomap.octomap.resolution},
                        {"data",msg.world.octomap.octomap.data},{"binary",msg.world.octomap.octomap.binary}};
    auto& op=msg.world.octomap.origin;
    result["octomap"]["origin"]={op.position.x,op.position.y,op.position.z,op.orientation.x,op.orientation.y,op.orientation.z,op.orientation.w};
    result["transforms"]=Json::array();
    for(auto& t:msg.fixed_frame_transforms) result["transforms"].push_back({{"parent",t.header.frame_id},{"child",t.child_frame_id},
      {"xyz",{t.transform.translation.x,t.transform.translation.y,t.transform.translation.z}},
      {"q",{t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w}}});
    for(auto& e:msg.allowed_collision_matrix.entry_values) result["acm"].push_back(e.enabled);
    return result;
  }
  void guard(const Clock::time_point& end) {
    if(cancel_) throw Error("CANCELLED","Worker stopped; result discarded");
    if(Clock::now()>=end) throw Error("TIMEOUT","Task exceeded finite deadline; result discarded");
  }
  moveit::core::RobotState fresh_state(const Clock::time_point& deadline) {
    moveit::core::RobotState s(model_); s.setToDefaultValues(); auto until=std::min(deadline,Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_)));
    while(Clock::now()<until) {
      guard(deadline); bool complete=true;
      {std::lock_guard<std::mutex> lock(joints_mutex_);
       for(const auto* j:model_->getActiveJointModels()) for(const auto& name:j->getVariableNames()) {
         auto it=joint_values_.find(name);
         if(it==joint_values_.end() || Clock::now()-it->second.second>std::chrono::milliseconds(1500)) complete=false;
         else s.setVariablePosition(name,it->second.first);
       }}
      if(complete) {s.update(); return s;}
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    guard(deadline);
    throw Error("STATE_TIMEOUT","Fresh complete joint state unavailable (max age 1.5 seconds)");
  }
  Json model_capabilities() {
    Json c={{"execution_enabled",false},{"protocol_schema","internal-backend-v1; Qt adapter serves planning_protocol/v1"},
            {"model_id",model_id_},{"planning_frame",model_->getModelFrame()},
            {"groups",model_->getJointModelGroupNames()},{"plan_to_pose",true},{"plan_pick_place",false},
            {"tcp",node_->get_parameter("tcp_link").as_string()}, {"validation_resolution",resolution_},
            {"continuous_collision_guarantee",false},{"seed_supported",false},{"pick_place_errors",Json::array()},
            {"joint_bounds",Json::object()}};
#ifdef RBY1_HAS_MTC
    c["mtc_available"]=true;
#else
    c["mtc_available"]=false; c["pick_place_errors"].push_back("MTC_NOT_INSTALLED");
#endif
    auto tcp=c["tcp"].get<std::string>(); auto group=node_->get_parameter("arm_group").as_string();
    if(!model_->getLinkModel(tcp) || !model_->getJointModelGroup(group)) throw Error("INVALID_MODEL_CONFIG","TCP/group does not exist");
    const auto* hand=model_->getJointModelGroup("gripper_r");
    if(!hand || model_->getEndEffectors().empty()) c["pick_place_errors"].push_back("MISSING_END_EFFECTOR_SEMANTICS");
    for(const auto& name:model_->getVariableNames()) {
      auto b=model_->getVariableBounds(name);
      c["joint_bounds"][name]={{"position_bounded",b.position_bounded_},{"min",b.min_position_},{"max",b.max_position_},
                              {"velocity_bounded",b.velocity_bounded_},{"max_velocity",b.max_velocity_},
                              {"acceleration_bounded",b.acceleration_bounded_},{"max_acceleration",b.max_acceleration_}};
    }
    for(const auto* joint:model_->getJointModels()) if(joint->getMimic()) {
      const auto& follower=joint->getVariableBounds().front(); const auto& leader=joint->getMimic()->getVariableBounds().front();
      double a=leader.min_position_*joint->getMimicFactor()+joint->getMimicOffset();
      double b=leader.max_position_*joint->getMimicFactor()+joint->getMimicOffset();
      if(std::min(a,b)<follower.min_position_-1e-6 || std::max(a,b)>follower.max_position_+1e-6)
        c["pick_place_errors"].push_back("MIMIC_BOUNDS_CONFLICT:"+joint->getName());
    }
    for(const std::string touch:{"gripper_finger_r1","gripper_finger_r2"})
      if(!model_->getLinkModel(touch) || !hand || std::find(hand->getLinkModelNames().begin(),hand->getLinkModelNames().end(),touch)==hand->getLinkModelNames().end())
        c["pick_place_errors"].push_back("UNVERIFIED_TOUCH_LINK:"+touch);
    if(hand && model_->getJointModel("gripper_finger_r1_joint")) {
      moveit::core::RobotState open(model_); open.setToDefaultValues(); open.setVariablePosition("gripper_finger_r1_joint",-.04); open.update();
      if(!open.satisfiesBounds()) c["pick_place_errors"].push_back("INVALID_GRIPPER_OPEN_STATE");
    }
    c["touch_links"]={"gripper_finger_r1","gripper_finger_r2"};
    c["gripper_group"]="gripper_r";
    c["gripper_open_candidate"]={{"gripper_finger_r1_joint",-.04}};
    c["gripper_closed_candidate"]={{"gripper_finger_r1_joint",0.}};
    c["gripper_states_verified"]=c["pick_place_errors"].empty();
    c["plan_pick_place"]=c["pick_place_errors"].empty(); return c;
  }
  Json reset_scene(const Json& params) {
    auto doc=params.contains("scene")?params["scene"]:yaml_json(node_->get_parameter("scene_file").as_string());
    auto addition=scene_from_json(doc,model_->getModelFrame()); // Validate everything before changing live scene.
    auto current=get_scene(get_,timeout_);
    moveit_msgs::msg::PlanningScene msg; msg.is_diff=true; msg.robot_state.is_diff=true;
    msg.allowed_collision_matrix=baseline_acm_;
    if(!clear_octomap_->wait_for_service(std::chrono::duration<double>(timeout_))) throw Error("SCENE_TIMEOUT","clear_octomap unavailable");
    auto clear_future=clear_octomap_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    if(clear_future.wait_for(std::chrono::duration<double>(timeout_))!=std::future_status::ready) {
      clear_octomap_->remove_pending_request(clear_future); throw Error("SCENE_TIMEOUT","Clearing octomap timed out");
    }
    clear_future.get();
    for(auto& a:current.robot_state.attached_collision_objects) {a.object.operation=a.object.REMOVE; msg.robot_state.attached_collision_objects.push_back(a);}
    for(auto& o:current.world.collision_objects) {o.operation=o.REMOVE; msg.world.collision_objects.push_back(o);}
    // Separate calls ensure a same-ID remove/add never leaves stale attachment/world geometry.
    apply_scene(apply_,msg,timeout_); apply_scene(apply_,addition,timeout_);
    auto readback=get_scene(get_,timeout_);
    if(!readback.world.octomap.octomap.data.empty()) throw Error("RESET_FAILED","Octomap remains after reset");
    if(!readback.robot_state.attached_collision_objects.empty()) throw Error("RESET_FAILED","Attachments remain after reset");
    std::set<std::string> expected,actual;
    for(auto& o:addition.world.collision_objects) expected.insert(o.id);
    for(auto& o:readback.world.collision_objects) actual.insert(o.id);
    if(expected!=actual) throw Error("SCENE_VERIFY_FAILED","Applied scene IDs differ from readback");
    planning_scene::PlanningScene expected_scene(model_); expected_scene.setPlanningSceneDiffMsg(addition);
    moveit_msgs::msg::PlanningScene normalized; expected_scene.getPlanningSceneMsg(normalized);
    if(signature(normalized)["objects"]!=signature(readback)["objects"]) throw Error("SCENE_VERIFY_FAILED","Readback geometry/poses differ from requested fixture");
    ++revision_; plans_.clear(); markers_->publish(fixture_markers(doc,model_->getModelFrame()));
    return {{"revision",revision_},{"scene",signature(readback)}};
  }
  Json plan(const Json& req, const Clock::time_point& deadline) {
    auto params=req.value("params",Json::object()); auto method=req.at("method").get<std::string>();
    if(method=="plan_pick_place" && !caps_["plan_pick_place"].get<bool>()) throw Error("GRASP_CONFIGURATION_INVALID",caps_["pick_place_errors"].dump());
    auto start=fresh_state(deadline); guard(deadline);
    auto snapshot=get_scene(get_,timeout_); auto original=signature(snapshot);
    auto scene=std::make_shared<planning_scene::PlanningScene>(model_); scene->setPlanningSceneMsg(snapshot);
    auto full_start=scene->getCurrentState(); full_start.setVariablePositions(start.getVariablePositions()); full_start.update();
    start=full_start; scene->setCurrentState(start);
    if(params.contains("start_state")) { // Only a solution snapshot override; never changes live robot joints.
      for(auto it=params["start_state"].begin();it!=params["start_state"].end();++it) {
        if(std::find(model_->getVariableNames().begin(),model_->getVariableNames().end(),it.key())==model_->getVariableNames().end()) throw Error("INVALID_JOINT",it.key());
        start.setVariablePosition(it.key(),it.value().get<double>());
      }
      start.update(); scene->setCurrentState(start);
    }
    if(!start.satisfiesBounds()) throw Error("JOINT_LIMIT","Start state violates bounds");
    if(scene->isStateColliding(start,"",false)) throw Error("START_COLLISION","Start robot is in self/world collision");
    std::string group_name=params.value("group",node_->get_parameter("arm_group").as_string());
    auto* group=model_->getJointModelGroup(group_name); if(!group) throw Error("INVALID_GROUP",group_name);
    if(method=="plan_to_pose" && !group->getSolverInstance()) throw Error("IK_SOLVER_UNAVAILABLE",group_name);
    double vs=params.value("velocity_scaling",vs_), as=params.value("acceleration_scaling",as_);
    if(!std::isfinite(vs)||!std::isfinite(as)||vs<=0||vs>1||as<=0||as>1) throw Error("INVALID_SCALING","Scaling must be in (0,1]");
    std::string tcp=params.value("tcp",node_->get_parameter("tcp_link").as_string());
    if(!model_->getLinkModel(tcp) || std::find(group->getLinkModelNames().begin(),group->getLinkModelNames().end(),tcp)==group->getLinkModelNames().end())
      throw Error("INVALID_TCP","TCP must belong to planning group: "+tcp);
    std::unique_ptr<StoredPlan> stored=std::make_unique<StoredPlan>(start);
    stored->display.model_id=model_->getName(); moveit::core::robotStateToRobotStateMsg(start,stored->display.trajectory_start);
    Json stages=Json::array(); double planning_time=0;
    if(method=="plan_to_pose") {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id=params.value("frame",model_->getModelFrame()); goal.pose=parse_pose(params.at("pose"));
      if(!scene->knowsFrameTransform(goal.header.frame_id)) throw Error("INVALID_FRAME","Missing TF/frame: "+goal.header.frame_id);
      Eigen::Isometry3d p; tf2::fromMsg(goal.pose,p); p=scene->getFrameTransform(goal.header.frame_id)*p;
      moveit::core::RobotState target(start);
      bool found=false, collided=false;
      for(int trial=0;trial<20 && !found;++trial) {
        guard(deadline);
        if(target.setFromIK(group,p,tcp,.05)) {
          target.update();
          if(target.satisfiesBounds() && !scene->isStateColliding(target,"",false)) found=true;
          else collided=true;
        }
      }
      if(!found) throw Error(collided?"GOAL_COLLISION":"IK_UNREACHABLE",collided?"IK goal is in collision":"No valid IK solution within bounded search");
      planning_interface::MotionPlanRequest request;
      request.group_name=group_name; request.planner_id="RRTConnectkConfigDefault";
      request.allowed_planning_time=std::min(30.,std::chrono::duration<double>(deadline-Clock::now()).count());
      if(request.allowed_planning_time<=0) throw Error("TIMEOUT","No planning budget left");
      request.num_planning_attempts=1; request.max_velocity_scaling_factor=vs; request.max_acceleration_scaling_factor=as;
      moveit::core::robotStateToRobotStateMsg(start,request.start_state);
      request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(target,group,1e-4));
      planning_interface::MotionPlanResponse response;
      bool success=pipeline_->generatePlan(scene,request,response); guard(deadline);
      if(!success || response.error_code_.val!=moveit_msgs::msg::MoveItErrorCodes::SUCCESS || !response.trajectory_)
        throw Error("PLANNING_FAILED","MoveIt error code: "+std::to_string(response.error_code_.val));
      planning_time=response.planning_time_;
      trajectory_processing::IterativeParabolicTimeParameterization retime;
      if(!retime.computeTimeStamps(*response.trajectory_,vs*.5,as*.5)) throw Error("RETIMING_FAILED","MoveIt IPTP failed");
      auto validation=validate_trajectory(scene,*response.trajectory_,resolution_,vs,as,&start,[&]{guard(deadline);});
      moveit_msgs::msg::RobotTrajectory trajectory; response.trajectory_->getRobotTrajectoryMsg(trajectory);
      stored->display.trajectory.push_back(trajectory);
      stages.push_back({{"name","plan_to_pose"},{"event",nullptr},{"validation",validation},
                        {"start",state_json(start)},{"final",state_json(response.trajectory_->getLastWayPoint())},
                        {"scene",original},{"goal",pose_json(p)}});
      stored->metadata["goal"]=pose_json(p);
    } else {
#ifdef RBY1_HAS_MTC
      stored->mtc=plan_mtc(node_,scene,params,task_parameters_,deadline,cancel_,resolution_);
      stored->metadata.update(stored->mtc->metadata);
      stages=stored->mtc->metadata.at("stages"); stored->display=stored->mtc->display;
      planning_time=stored->mtc->metadata.value("planning_time_s",0.);
#else
      throw Error("MTC_NOT_INSTALLED","Install MoveIt Task Constructor core to plan pick/place");
#endif
    }
    guard(deadline);
    auto after=get_scene(get_,timeout_);
    if(signature(after)!=original) throw Error("SCENE_CHANGED","Live scene changed while planning; invalidating result");
    // Transport request IDs include a fresh connection UUID, preventing plan ID reuse after restart.
    auto id="plan-"+req.at("id").get<std::string>();
    stored->scene_signature=original; stored->expires=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(ttl_));
    stored->metadata.update({{"plan_id",id},{"model_id",model_id_},{"revision",revision_},{"ttl_s",ttl_},
                             {"start_state",state_json(start)},{"scene",original},{"stages",stages},
                             {"planning_time_s",planning_time},{"execution_enabled",false},{"live_scene_unchanged",true},
                             {"retiming","MoveIt IterativeParabolicTimeParameterization"},{"retiming_scaling_margin",0.5},{"seed",nullptr}});
    auto result=stored->metadata; guard(deadline);
    for(auto it=plans_.begin();it!=plans_.end();) {if(Clock::now()>it->second->expires) it=plans_.erase(it); else ++it;}
    if(plans_.size()>=32) plans_.erase(plans_.begin());
    plans_[id]=std::move(stored); return result;
  }
  Json preview(const Json& params,const Clock::time_point& deadline) {
    auto id=params.at("plan_id").get<std::string>(); auto it=plans_.find(id);
    if(it==plans_.end()) throw Error("PLAN_NOT_FOUND","Unknown or invalidated plan");
    auto& plan=*it->second;
    if(Clock::now()>plan.expires) {plans_.erase(it); throw Error("PLAN_EXPIRED","Plan TTL elapsed");}
    if(plan.metadata["revision"]!=revision_ || plan.metadata["model_id"]!=model_id_) throw Error("PLAN_STALE","Model or revision changed");
    auto before=get_scene(get_,timeout_); if(signature(before)!=plan.scene_signature) throw Error("PLAN_STALE","Live scene changed");
    auto start=fresh_state(deadline); if(start.distance(plan.start)>1e-4) throw Error("START_STATE_CHANGED","Live start differs from stored plan start");
    guard(deadline); display_->publish(plan.display);
#ifdef RBY1_HAS_MTC
    if(plan.mtc) plan.mtc->publish();
#endif
    if(signature(get_scene(get_,timeout_))!=plan.scene_signature) throw Error("SCENE_CHANGED","Live scene changed during preview");
    return {{"plan_id",id},{"published",true},{"live_scene_unchanged",true},{"execution_enabled",false}};
  }
  Json inspect(const std::string& method,const Json& params,const Clock::time_point& deadline) {
    auto start=fresh_state(deadline); auto snapshot=get_scene(get_,timeout_);
    auto scene=std::make_shared<planning_scene::PlanningScene>(model_); scene->setPlanningSceneMsg(snapshot);
    if(params.contains("state")) for(auto it=params["state"].begin();it!=params["state"].end();++it) {
      if(std::find(model_->getVariableNames().begin(),model_->getVariableNames().end(),it.key())==model_->getVariableNames().end()) throw Error("INVALID_JOINT",it.key());
      start.setVariablePosition(it.key(),it.value().get<double>());
    }
    auto full_state=scene->getCurrentState(); full_state.setVariablePositions(start.getVariablePositions()); full_state.update();
    start=full_state; scene->setCurrentState(start);
    auto tcp=params.value("tcp",node_->get_parameter("tcp_link").as_string()); auto* link=model_->getLinkModel(tcp);
    if(!link) throw Error("INVALID_TCP",tcp);
    // Diagnostic virtual payload fixture only: no grasp claim and no live scene mutation.
    if(params.contains("virtual_attachment")) {
      if(tcp!="ee_right") throw Error("INVALID_TCP","Virtual payload diagnostic requires verified ee_right");
      auto object=scene_from_json({{"objects",Json::array({params["virtual_attachment"]})}},tcp).world.collision_objects.front();
      if(scene->getWorld()->hasObject(object.id) || start.hasAttachedBody(object.id)) throw Error("DUPLICATE_OBJECT",object.id);
      moveit_msgs::msg::AttachedCollisionObject attached; attached.link_name=tcp; attached.object=object;
      attached.touch_links={"gripper_finger_r1","gripper_finger_r2"};
      if(!scene->processAttachedCollisionObjectMsg(attached)) throw Error("ATTACH_FAILED","Diagnostic virtual attachment rejected");
      start=scene->getCurrentState(); start.update();
    }
    if(method=="check_state") {
      collision_detection::CollisionRequest request; request.contacts=true; request.max_contacts=100;
      collision_detection::CollisionResult result; scene->checkCollision(request,result,start);
      Json contacts=Json::array(); for(auto& pair:result.contacts) contacts.push_back({pair.first.first,pair.first.second});
      return {{"diagnostic_virtual_payload",params.contains("virtual_attachment")},{"grasp_claim",false},{"bounds_valid",start.satisfiesBounds()},{"collision",result.collision},{"contacts",contacts},
              {"tcp_pose",pose_json(start.getGlobalLinkTransform(tcp))},{"state",state_json(start)}};
    }
    if(params.value("frame",model_->getModelFrame())!=model_->getModelFrame()) throw Error("INVALID_FRAME","Probe requires model planning frame");
    auto* group=model_->getJointModelGroup(params.value("group",std::string("right_arm")));
    if(!group) throw Error("INVALID_GROUP","Unknown group");
    Eigen::Isometry3d goal; tf2::fromMsg(parse_pose(params.at("pose")),goal);
    std::vector<moveit::core::RobotStatePtr> path; size_t collisions=0;
    auto valid=[&](moveit::core::RobotState* state,const moveit::core::JointModelGroup* jmg,const double* values) {
      guard(deadline); state->setJointGroupPositions(jmg,values); state->update();
      bool hit=scene->isStateColliding(*state,"",false); if(hit) ++collisions;
      return !hit && state->satisfiesBounds();
    };
    double fraction=moveit::core::CartesianInterpolator::computeCartesianPath(&start,group,path,link,goal,true,
        moveit::core::MaxEEFStep(.005),moveit::core::JumpThreshold(.5,.02),valid);
    Json samples=Json::array(); for(auto& state:path) samples.push_back({{"state",state_json(*state)},{"tcp_pose",pose_json(state->getGlobalLinkTransform(tcp))}});
    return {{"diagnostic_virtual_payload",params.contains("virtual_attachment")},{"grasp_claim",false},{"fraction",fraction},{"collision_rejections",collisions},{"step_m",.005},{"samples",samples},{"complete",fraction>=1.-1e-9},{"stored_plan",false}};
  }
  void receive(const Json& req) {
    auto id=req.at("id").get<std::string>(); auto method=req.at("method").get<std::string>();
    if(req.at("v")!=1) {fail(id,"UNSUPPORTED_VERSION","Only v=1 supported"); return;}
    if(method=="get_task_status") {
      std::lock_guard<std::mutex> lock(mutex_); auto task=req.value("params",Json::object()).value("task_id",std::string());
      auto out=envelope(id,"response"); out["ok"]=statuses_.contains(task);
      if(out["ok"].get<bool>()) out["result"]=statuses_[task]; else out["error"]={{"code","TASK_NOT_FOUND"},{"message",task}};
      emit(out); return;
    }
    if(method=="cancel") {
      std::lock_guard<std::mutex> lock(mutex_); auto task=req.value("params",Json::object()).value("task_id",std::string());
      auto out=envelope(id,"response"); out["ok"]=true;
      if(active_==task && !task.empty()) {cancel_=true; out["result"]={{"task_id",task},{"state","cancelling"},{"worker_stopped",false}};}
      else out["result"]={{"task_id",task},{"state",statuses_.contains(task)?statuses_[task].value("state","unknown"):"unknown"},{"worker_stopped",true}};
      emit(out); return;
    }
    bool is_plan=method=="plan_to_pose" || method=="plan_pick_place";
    std::lock_guard<std::mutex> lock(mutex_);
    if(!active_.empty()) {fail(id,"BUSY","One planning task is active; retry after terminal event");return;}
    if(queue_.size()>=32) {fail(id,"BUSY","Worker queue full");return;}
    if(is_plan) {
      active_=id; cancel_=false; statuses_[id]={{"state","accepted"},{"task_id",id}};
      auto ack=envelope(id,"ack"); ack["task_id"]=id; ack["state"]="accepted"; emit(ack);
    }
    Json queued=req; queued["received_steady_ns"]=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
    queue_.push_back(queued); cv_.notify_one();
  }
  void run() {
    while(true) {
      Json req;
      {std::unique_lock<std::mutex> lock(mutex_); cv_.wait(lock,[&]{return stop_ || !queue_.empty();}); if(stop_) return; req=queue_.front(); queue_.pop_front();}
      auto id=req["id"].get<std::string>(), method=req["method"].get<std::string>(); bool task=method=="plan_to_pose"||method=="plan_pick_place";
      auto received=Clock::time_point(std::chrono::nanoseconds(req["received_steady_ns"].get<int64_t>()));
      double budget=30.; auto began=Clock::now(); auto out=envelope(id,task?"event":"response");
      if(task) {std::lock_guard<std::mutex> lock(mutex_); statuses_[id]["state"]="planning";}
      try {
        budget=req.value("params",Json::object()).value("deadline_s",30.);
        if(!std::isfinite(budget)||budget<=0||budget>120) throw Error("INVALID_DEADLINE","deadline_s must be in (0,120]");
        auto deadline=received+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(budget));
        Json result;
        if(method=="capabilities") result=caps_;
        else if(method=="load_scene" || method=="reset_scene") result=reset_scene(req.value("params",Json::object()));
        else if(method=="get_scene") result={{"revision",revision_},{"scene",signature(get_scene(get_,timeout_))}};
        else if(method=="get_state") {auto s=fresh_state(deadline); result={{"state",state_json(s)},{"tcp_pose",pose_json(s.getGlobalLinkTransform(node_->get_parameter("tcp_link").as_string()))},{"frame",model_->getModelFrame()}};}
        else if(method=="check_state" || method=="cartesian_probe") result=inspect(method,req.value("params",Json::object()),deadline);
        else if(task) result=plan(req,deadline);
        else if(method=="preview") result=preview(req.value("params",Json::object()),deadline);
        else throw Error("UNKNOWN_METHOD",method);
        if(task) guard(deadline);
        out["ok"]=true; out["result"]=result;
        if(task) out["state"]="succeeded";
      } catch(const Error& e) {out["ok"]=false; out["error"]={{"code",e.code},{"message",e.what()}}; if(task) out["state"]=e.code=="CANCELLED"?"cancelled":(e.code=="TIMEOUT"?"timed_out":"failed");}
      catch(const std::exception& e) {out["ok"]=false; out["error"]={{"code","INVALID_REQUEST"},{"message",e.what()}}; if(task) out["state"]="failed";}
      out["elapsed_s"]=std::chrono::duration<double>(Clock::now()-began).count();
      if(task) {
        out["terminal"]=true; out["task_id"]=id; out["worker_stopped"]=true;
        std::lock_guard<std::mutex> lock(mutex_);
        // Cancellation racing with successful completion discards all plans/results before terminal publication.
        if(cancel_) {plans_.clear(); out.erase("result"); out["ok"]=false; out["state"]="cancelled"; out["error"]={{"code","CANCELLED"},{"message","Worker stopped; late result discarded"}};}
        statuses_[id]=out; active_.clear(); cancel_=false; emit(out);
        if(statuses_.size()>256) statuses_.erase(statuses_.begin());
      } else emit(out);
    }
  }
public:
  explicit Backend(const rclcpp::Node::SharedPtr& node):node_(node) {
    loader_=std::make_shared<robot_model_loader::RobotModelLoader>(node_,"robot_description"); model_=loader_->getModel();
    if(!model_) throw std::runtime_error("Cannot load robot model");
    model_id_=node_->get_parameter("model_id").as_string();
    task_parameters_=yaml_json(node_->get_parameter("task_parameters_file").as_string());
    resolution_=task_parameters_.value("validation_resolution",.02); vs_=task_parameters_.value("velocity_scaling",.1);
    as_=task_parameters_.value("acceleration_scaling",.1); ttl_=task_parameters_.value("plan_ttl_s",120.); timeout_=task_parameters_.value("state_timeout_s",5.);
    pipeline_=std::make_shared<planning_pipeline::PlanningPipeline>(model_,node_,"ompl");
    pipeline_->displayComputedMotionPlans(false); pipeline_->publishReceivedRequests(false);
    get_=node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_=node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    clear_octomap_=node_->create_client<std_srvs::srv::Empty>("/clear_octomap");
    responses_=node_->create_publisher<std_msgs::msg::String>("/rby1_planning/responses",100);
    display_=node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",rclcpp::QoS(1).transient_local());
    markers_=node_->create_publisher<visualization_msgs::msg::MarkerArray>("/rby1_planning/markers",rclcpp::QoS(1).transient_local());
    joints_=node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",50,[this](sensor_msgs::msg::JointState::ConstSharedPtr msg){
      if(msg->name.size()!=msg->position.size()) return;
      // Reject old ROS stamps as well as stale receipt times. Simulation time is deliberately disabled in this launch.
      if(msg->header.stamp.sec==0 || std::abs((node_->now()-rclcpp::Time(msg->header.stamp)).seconds())>1.5) return;
      std::lock_guard<std::mutex> lock(joints_mutex_);
      for(size_t i=0;i<msg->name.size();++i) joint_values_[msg->name[i]]={msg->position[i],Clock::now()};
    });
    requests_=node_->create_subscription<std_msgs::msg::String>("/rby1_planning/requests",100,[this](std_msgs::msg::String::ConstSharedPtr msg){
      try {receive(Json::parse(msg->data));} catch(const std::exception& e) {fail("","INVALID_REQUEST",e.what());}
    });
    planning_scene::PlanningScene baseline(model_); 
    baseline_acm_=moveit_msgs::msg::AllowedCollisionMatrix(); baseline.getAllowedCollisionMatrix().getMessage(baseline_acm_);
    caps_=model_capabilities(); worker_=std::thread([this]{run();});
    RCLCPP_INFO(node_->get_logger(),"PLANNING_BACKEND_READY execution_enabled=false model=%s",model_id_.c_str());
  }
  ~Backend() { {std::lock_guard<std::mutex> lock(mutex_); stop_=true; cancel_=true;} cv_.notify_all(); pipeline_->terminate(); if(worker_.joinable()) worker_.join();}
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv); int code=0;
  try {auto node=std::make_shared<rclcpp::Node>("planning_worker",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rby1_motion_planning::Backend backend(node); rclcpp::executors::MultiThreadedExecutor executor; executor.add_node(node); executor.spin();
  } catch(const std::exception& e) {fprintf(stderr,"planning_worker: %s\n",e.what()); code=1;}
  rclcpp::shutdown(); return code;
}
