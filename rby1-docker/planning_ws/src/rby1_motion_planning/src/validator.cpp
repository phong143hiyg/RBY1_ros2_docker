#include "rby1_motion_planning/common.hpp"
#include <cmath>
#include <algorithm>

namespace rby1_motion_planning {
Json validate_trajectory(const planning_scene::PlanningSceneConstPtr& scene,
                         const robot_trajectory::RobotTrajectory& tr,
                         double resolution, double vs, double as,
                         const moveit::core::RobotState* previous, const std::function<void()>& checkpoint) {
  if(!std::isfinite(resolution)||resolution<=0) throw Error("INVALID_RESOLUTION", "Resolution must be positive");
  auto count=tr.getWayPointCount(); if(count==0) throw Error("EMPTY_TRAJECTORY", "Planner returned no waypoints");
  const auto* group=tr.getGroup(); if(!group) throw Error("INVALID_GROUP", "Trajectory has no joint group");
  if(previous && previous->distance(tr.getFirstWayPoint())>1e-5) throw Error("DISCONTINUITY", "Stage start differs from previous final state");
  size_t samples=0; Json missing=Json::array(); double duration=0;
  auto check=[&](const moveit::core::RobotState& s) {
    if(checkpoint) checkpoint();
    for(const auto& name:s.getVariableNames()) if(!std::isfinite(s.getVariablePosition(name))) throw Error("JOINT_LIMIT", "Non-finite joint: "+name);
    if(!s.satisfiesBounds(1e-6)) throw Error("JOINT_LIMIT", "Waypoint/interpolation violates joint bounds");
    if(scene->isStateColliding(s,"",false)) throw Error("COLLISION", "Self/world/attached collision in trajectory sample");
    ++samples;
  };
  for(const auto& name:group->getVariableNames()) {
    const auto& b=scene->getRobotModel()->getVariableBounds(name);
    if(!b.acceleration_bounded_) missing.push_back(name);
  }
  for(size_t i=0;i<count;++i) {
    const auto& s=tr.getWayPoint(i); check(s);
    double dt=tr.getWayPointDurationFromPrevious(i);
    if(!std::isfinite(dt)||dt<0||(i>0 && dt<=0)) throw Error("TIMESTAMPS", "Timestamps must strictly increase inside a stage");
    duration+=dt;
    for(const auto& name:group->getVariableNames()) {
      const auto& b=scene->getRobotModel()->getVariableBounds(name);
      double velocity=s.hasVelocities()?s.getVariableVelocity(name):0.;
      double acceleration=s.hasAccelerations()?s.getVariableAcceleration(name):0.;
      if(!std::isfinite(velocity)||!std::isfinite(acceleration)) throw Error("DYNAMICS_LIMIT", "Non-finite dynamics");
      if(b.velocity_bounded_ && std::abs(velocity)>std::max(std::abs(b.min_velocity_),std::abs(b.max_velocity_))*vs+1e-5) throw Error("VELOCITY_LIMIT", name);
      if(b.acceleration_bounded_ && std::abs(acceleration)>std::max(std::abs(b.min_acceleration_),std::abs(b.max_acceleration_))*as+1e-5) throw Error("ACCELERATION_LIMIT", name+" value="+std::to_string(acceleration)+" limit="+std::to_string(b.max_acceleration_*as));
      if(i>0 && b.velocity_bounded_) {
        double delta=std::abs(s.getVariablePosition(name)-tr.getWayPoint(i-1).getVariablePosition(name));
        if(delta/dt>std::max(std::abs(b.min_velocity_),std::abs(b.max_velocity_))*vs+1e-5) throw Error("VELOCITY_LIMIT", "Segment average: "+name);
      }
      if(i>=2 && b.acceleration_bounded_) {
        double dt_prev=tr.getWayPointDurationFromPrevious(i-1);
        if(dt_prev<=0) throw Error("TIMESTAMPS","Previous timestamp interval is invalid");
        double v1=(tr.getWayPoint(i-1).getVariablePosition(name)-tr.getWayPoint(i-2).getVariablePosition(name))/dt_prev;
        double v2=(s.getVariablePosition(name)-tr.getWayPoint(i-1).getVariablePosition(name))/dt;
        double a=2.*(v2-v1)/(dt_prev+dt);
        if(std::abs(a)>std::max(std::abs(b.min_acceleration_),std::abs(b.max_acceleration_))*as+1e-5)
          throw Error("ACCELERATION_LIMIT","Segment finite difference: "+name);
      }
    }
    if(i>0) {
      auto& a=tr.getWayPoint(i-1); double delta=0;
      for(const auto& name:s.getVariableNames()) delta=std::max(delta,std::abs(s.getVariablePosition(name)-a.getVariablePosition(name)));
      size_t steps=std::max<size_t>(1,static_cast<size_t>(std::ceil(delta/resolution)));
      if(steps>100000) throw Error("VALIDATION_BUDGET", "Interpolation segment too large");
      moveit::core::RobotState intermediate(a);
      for(size_t k=1;k<steps;++k) {a.interpolate(s,static_cast<double>(k)/steps,intermediate); intermediate.update(); check(intermediate);}
    }
  }
  return {{"valid",true},{"waypoints",count},{"duration_s",duration},{"collision_samples",samples},
          {"resolution",resolution},{"resolution_units","maximum variable delta: radians (revolute), metres (prismatic)"},
          {"continuous_collision_guarantee",false},{"unbounded_acceleration_joints",missing},
          {"dynamics_check","retimed waypoint velocity/acceleration plus segment-average velocity and finite-difference acceleration"}};
}
}
