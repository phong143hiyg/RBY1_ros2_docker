#include <gtest/gtest.h>
#include "rby1_motion_planning/common.hpp"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <geometric_shapes/shapes.h>

using namespace rby1_motion_planning;
static moveit::core::RobotModelPtr fixture_model() {
  const std::string urdf=R"(<robot name="validator_fixture"><link name="base"/><link name="hand"><collision><geometry><sphere radius="0.02"/></geometry></collision></link><joint name="slide" type="prismatic"><parent link="base"/><child link="hand"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint></robot>)";
  auto u=urdf::parseURDF(urdf); auto s=std::make_shared<srdf::Model>();
  s->initString(*u,R"(<robot name="validator_fixture"><group name="arm"><joint name="slide"/></group></robot>)");
  return std::make_shared<moveit::core::RobotModel>(u,s);
}
static void expect_error(const std::function<void()>& fn,const std::string& code) {
  try {fn(); FAIL()<<"Expected "<<code;} catch(const Error& e) {EXPECT_EQ(e.code,code);}
}
TEST(Validator, InterpolationDetectsObstacleBetweenWaypoints) {
  auto model=fixture_model(); auto scene=std::make_shared<planning_scene::PlanningScene>(model);
  auto msg=scene_from_json({{"objects",Json::array({{{"id","thin_wall"},{"type","box"},{"dimensions",{.04,.1,.1}},{"pose",{{"position",{0.,0.,0.}}}}}})}},"base");
  scene->setPlanningSceneDiffMsg(msg);
  moveit::core::RobotState a(model); a.setToDefaultValues(); a.setVariablePosition("slide",-.3); a.update();
  auto b=a; b.setVariablePosition("slide",.3); b.update();
  robot_trajectory::RobotTrajectory tr(model,"arm"); tr.addSuffixWayPoint(a,0); tr.addSuffixWayPoint(b,1);
  EXPECT_FALSE(scene->isStateColliding(a)); EXPECT_FALSE(scene->isStateColliding(b));
  expect_error([&]{validate_trajectory(scene,tr,.01,1,1);},"COLLISION");
}
TEST(Validator, AttachedObjectBlocksBareHandPassage) {
  auto model=fixture_model(); auto scene=std::make_shared<planning_scene::PlanningScene>(model);
  auto wall=scene_from_json({{"objects",Json::array({{{"id","wall"},{"type","box"},{"dimensions",{.1,.1,.1}},{"pose",{{"position",{0.,.15,0.}}}}}})}},"base"); scene->setPlanningSceneDiffMsg(wall);
  moveit::core::RobotState a(model); a.setToDefaultValues(); a.update();
  EXPECT_FALSE(scene->isStateColliding(a));
  moveit_msgs::msg::AttachedCollisionObject attachment; attachment.link_name="hand"; attachment.touch_links={"hand"};
  attachment.object=scene_from_json({{"objects",Json::array({{{"id","payload"},{"type","box"},{"dimensions",{.1,.3,.1}},{"pose",{{"position",{0.,0.,0.}}}}}})}},"hand").world.collision_objects.front();
  a.attachBody("payload",Eigen::Isometry3d::Identity(),{shapes::ShapeConstPtr(new shapes::Box(.1,.3,.1))},{Eigen::Isometry3d::Identity()},std::set<std::string>{"hand"},"hand"); a.update();
  EXPECT_TRUE(scene->isStateColliding(a));
  robot_trajectory::RobotTrajectory tr(model,"arm"); tr.addSuffixWayPoint(a,0);
  expect_error([&]{validate_trajectory(scene,tr,.01,1,1);},"COLLISION");
}
TEST(Validator, BoundsTimeVelocityAndStageContinuity) {
  auto model=fixture_model(); auto scene=std::make_shared<planning_scene::PlanningScene>(model);
  moveit::core::RobotState a(model); a.setToDefaultValues(); a.update(); auto b=a;
  b.setVariablePosition("slide",1.5); b.update(); robot_trajectory::RobotTrajectory bounds(model,"arm"); bounds.addSuffixWayPoint(b,0);
  expect_error([&]{validate_trajectory(scene,bounds,.01,1,1);},"JOINT_LIMIT");
  b.setVariablePosition("slide",.2); b.update(); robot_trajectory::RobotTrajectory times(model,"arm"); times.addSuffixWayPoint(a,0); times.addSuffixWayPoint(b,0);
  expect_error([&]{validate_trajectory(scene,times,.01,1,1);},"TIMESTAMPS");
  times.setWayPointDurationFromPrevious(1,.01);
  expect_error([&]{validate_trajectory(scene,times,.01,1,1);},"VELOCITY_LIMIT");
  times.setWayPointDurationFromPrevious(1,1);
  expect_error([&]{validate_trajectory(scene,times,.01,1,1,&b);},"DISCONTINUITY");
  EXPECT_TRUE(validate_trajectory(scene,times,.01,1,1,&a)["valid"].get<bool>());
}
TEST(Validator, DetachUsesFinalStateAndKeepsLiveSceneUnchanged) {
  auto model=fixture_model(); auto live=std::make_shared<planning_scene::PlanningScene>(model);
  auto object=scene_from_json({{"objects",Json::array({{{"id","payload"},{"type","box"},{"dimensions",{.02,.02,.02}},{"pose",{{"position",{0.,0.,0.}}}}}})}},"base");
  live->setPlanningSceneDiffMsg(object); auto snapshot=live->diff();
  moveit_msgs::msg::AttachedCollisionObject attach; attach.link_name="hand"; attach.touch_links={"hand"}; attach.object=object.world.collision_objects.front();
  ASSERT_TRUE(snapshot->processAttachedCollisionObjectMsg(attach));
  auto final=snapshot->getCurrentState(); final.setVariablePosition("slide",.4); final.update(); snapshot->setCurrentState(final);
  attach.object.operation=attach.object.REMOVE; ASSERT_TRUE(snapshot->processAttachedCollisionObjectMsg(attach));
  ASSERT_TRUE(snapshot->getWorld()->hasObject("payload"));
  EXPECT_NEAR(snapshot->getWorld()->getObject("payload")->pose_.translation().x(),.4,1e-8);
  EXPECT_FALSE(snapshot->getCurrentState().hasAttachedBody("payload"));
  EXPECT_NEAR(live->getWorld()->getObject("payload")->pose_.translation().x(),0.,1e-8);
  EXPECT_FALSE(live->getCurrentState().hasAttachedBody("payload"));
}
TEST(Validator, AccelerationFiniteDifference) {
  auto model=fixture_model(); auto* joint=model->getJointModel("slide"); auto limits=joint->getVariableBounds().front();
  limits.acceleration_bounded_=true; limits.min_acceleration_=-.1; limits.max_acceleration_=.1; joint->setVariableBounds("slide",limits);
  auto scene=std::make_shared<planning_scene::PlanningScene>(model); moveit::core::RobotState a(model); a.setToDefaultValues(); a.update();
  auto b=a; b.setVariablePosition("slide",.2); b.update();
  robot_trajectory::RobotTrajectory tr(model,"arm"); tr.addSuffixWayPoint(a,0); tr.addSuffixWayPoint(b,.5); tr.addSuffixWayPoint(b,.5);
  expect_error([&]{validate_trajectory(scene,tr,.01,1,1);},"ACCELERATION_LIMIT");
}
