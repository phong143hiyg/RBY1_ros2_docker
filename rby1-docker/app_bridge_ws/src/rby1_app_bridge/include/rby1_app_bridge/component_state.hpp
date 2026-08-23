#pragma once

#include <mutex>

namespace rby1_app_bridge
{

struct ComponentStateSnapshot
{
  bool power{false};
  bool servo{false};
  bool stream{false};
};

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
    return connected && driver_ready && state.power && state.servo && state.stream;
  }

private:
  mutable std::mutex mutex_;
  bool power_{false};
  bool servo_{false};
  bool stream_{false};
};

}  // namespace rby1_app_bridge
