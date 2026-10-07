#include <exception>
#include <iostream>
#include <string>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

int main(int argc, char** argv) {
  const std::string endpoint = argc > 1 ? argv[1] : "localhost:50051";

  try {
    auto robot = rb::Robot<rb::y1_model::M>::Create(endpoint);
    if (!robot->Connect(5, 1000)) {
      std::cerr << "READ_ONLY_SMOKE_FAILED endpoint=" << endpoint
                << " reason=connect-returned-false\n";
      return 2;
    }

    const auto info = robot->GetRobotInfo();
    const auto state = robot->GetState();
    std::cout << "READ_ONLY_SMOKE_OK"
              << " endpoint=" << endpoint
              << " model=" << info.robot_model_name
              << " model_version=" << info.robot_model_version
              << " sdk_version=" << info.sdk_version
              << " dof=" << info.degree_of_freedom
              << " state_joints=" << state.joint_states.size() << '\n';
    robot->Disconnect();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "READ_ONLY_SMOKE_FAILED endpoint=" << endpoint
              << " reason=" << error.what() << '\n';
    return 3;
  }
}
