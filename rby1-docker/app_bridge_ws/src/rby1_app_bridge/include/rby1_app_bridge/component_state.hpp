#pragma once

#include <array>
#include <mutex>

namespace rby1_app_bridge
{

struct ComponentStateSnapshot
{
  bool power{false};
  bool servo{false};
  bool stream{false};

  bool all_enabled() const
  {
    return power && servo && stream;
  }
};

enum class Component
{
  Servo,
  Stream,
  Power
};

inline constexpr std::array<Component, 3>
POWER_OFF_ORDER = {
  Component::Servo,
  Component::Stream,
  Component::Power
};

template<typename Operation>
bool run_power_off_sequence(Operation operation)
{
  bool success = true;

  for (const auto component : POWER_OFF_ORDER)
  {
    const bool step_succeeded = operation(component);
    success = step_succeeded && success;
  }

  return success;
}

inline bool can_enable_power_dependent(
  const ComponentStateSnapshot &state,
  bool enabled)
{
  return !enabled || state.power;
}

inline bool can_prepare(
  const ComponentStateSnapshot &state)
{
  return state.all_enabled();
}

class ComponentState
{
public:
  ComponentStateSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return {power_, servo_, stream_};
  }

  void confirm_power(bool enabled)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    power_ = enabled;

    if (!enabled)
    {
      servo_ = false;
      stream_ = false;
    }
  }

  void confirm_servo(bool enabled)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    servo_ = enabled;
  }

  void confirm_stream(bool enabled)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stream_ = enabled;
  }

  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    power_ = false;
    servo_ = false;
    stream_ = false;
  }

  bool ready(bool connected, bool driver_ready) const
  {
    const auto state = snapshot();
    return connected && driver_ready && state.all_enabled();
  }

private:
  mutable std::mutex mutex_;
  bool power_{false};
  bool servo_{false};
  bool stream_{false};
};

}  // namespace rby1_app_bridge
