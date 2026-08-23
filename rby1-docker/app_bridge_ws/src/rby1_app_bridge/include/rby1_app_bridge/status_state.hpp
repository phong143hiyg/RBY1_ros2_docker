#pragma once

#include <string>

namespace rby1_app_bridge
{

struct StatusInputs
{
  bool driver_available{false};
  bool connected{false};
  bool preparing{false};
  bool fault{false};
  bool joint_busy{false};
  bool driving{false};
  bool ready{false};
  std::string fault_message;
};

struct StatusDescription
{
  std::string state;
  std::string message;
};

inline StatusDescription describe_status(const StatusInputs &input)
{
  if (!input.driver_available || !input.connected)
  {
    return {
      "Disconnected",
      "ROS2 driver or robot is unavailable"
    };
  }

  if (input.fault)
  {
    return {
      "Fault",
      input.fault_message.empty()
        ? "Control Manager fault"
        : input.fault_message
    };
  }

  if (input.preparing)
  {
    return {"Preparing", "Preparing robot"};
  }

  if (input.joint_busy)
  {
    return {"JointBusy", "Joint action is running"};
  }

  if (input.driving)
  {
    return {"Driving", "Driving"};
  }

  if (input.ready)
  {
    return {"Ready", "Robot ready"};
  }

  return {"Connected", "Robot connected; waiting for ready state"};
}

}  // namespace rby1_app_bridge
