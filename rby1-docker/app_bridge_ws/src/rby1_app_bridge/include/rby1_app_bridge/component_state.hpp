#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>

namespace rby1_app_bridge
{

enum class Component
{
  Servo,
  Stream,
  Power
};

enum class ObservedState
{
  Unknown,
  Off,
  On
};

inline const char *to_string(const ObservedState state)
{
  switch (state)
  {
    case ObservedState::Off:
      return "off";
    case ObservedState::On:
      return "on";
    case ObservedState::Unknown:
    default:
      return "unknown";
  }
}

inline const char *to_string(const Component component)
{
  switch (component)
  {
    case Component::Power:
      return "power";
    case Component::Servo:
      return "servo";
    case Component::Stream:
    default:
      return "stream";
  }
}

struct ComponentSnapshot
{
  ObservedState state{ObservedState::Unknown};
  bool pending{false};
  std::string source;

  bool known() const
  {
    return state != ObservedState::Unknown;
  }

  bool enabled() const
  {
    return state == ObservedState::On;
  }

  bool disabled() const
  {
    return state == ObservedState::Off;
  }
};

struct ComponentStateSnapshot
{
  ComponentSnapshot power;
  ComponentSnapshot servo;
  ComponentSnapshot stream;

  bool all_enabled() const
  {
    return
      power.enabled()
      && servo.enabled()
      && stream.enabled();
  }

  bool all_known_disabled() const
  {
    return
      power.disabled()
      && servo.disabled()
      && stream.disabled();
  }
};

struct ObservationChange
{
  Component component{Component::Power};
  ObservedState previous{ObservedState::Unknown};
  ObservedState current{ObservedState::Unknown};
  bool state_changed{false};
  bool pending_target_observed{false};
  std::string source;
};

struct PendingTransition
{
  Component component{Component::Power};
  bool expected_enabled{false};
  std::uint64_t generation{0};
  std::chrono::steady_clock::time_point started_at{};
};

enum class ConfirmationResult
{
  Confirmed,
  TimedOut,
  Interrupted
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
  return !enabled || state.power.enabled();
}

inline bool can_prepare(
  const ComponentStateSnapshot &state)
{
  return state.all_enabled();
}

class ComponentState
{
public:
  ComponentState()
  {
    entries_[index(Component::Power)].source = "robot_api";
    entries_[index(Component::Servo)].source = "robot_api";
    entries_[index(Component::Stream)].source = "robot_state";
  }

  ComponentStateSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_unlocked();
  }

  ObservationChange observe(
    const Component component,
    const ObservedState observed,
    const std::string &source)
  {
    ObservationChange change;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto &entry = entries_[index(component)];

      change.component = component;
      change.previous = entry.state;
      change.current = observed;
      change.state_changed = entry.state != observed;
      change.source = source;

      entry.state = observed;
      entry.source = source;

      if (entry.pending)
      {
        const bool target_observed =
          observed != ObservedState::Unknown
          && (observed == ObservedState::On)
            == entry.expected_enabled;

        entry.confirmed = target_observed;
        change.pending_target_observed = target_observed;
      }
    }

    condition_.notify_all();
    return change;
  }

  PendingTransition begin_pending(
    const Component component,
    const bool expected_enabled)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto &entry = entries_[index(component)];

    ++entry.generation;
    entry.pending = true;
    entry.confirmed = false;
    entry.expected_enabled = expected_enabled;

    return {
      component,
      expected_enabled,
      entry.generation,
      std::chrono::steady_clock::now()
    };
  }

  ConfirmationResult wait_for(
    const PendingTransition &transition,
    const std::chrono::steady_clock::time_point deadline)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    auto &entry = entries_[index(transition.component)];

    const auto finished = [&]() {
      return
        entry.generation != transition.generation
        || entry.confirmed
        || !entry.pending;
    };

    if (!condition_.wait_until(lock, deadline, finished))
    {
      if (entry.generation == transition.generation)
      {
        entry.pending = false;
      }
      return ConfirmationResult::TimedOut;
    }

    if (entry.generation != transition.generation)
    {
      return ConfirmationResult::Interrupted;
    }

    if (entry.confirmed)
    {
      entry.pending = false;
      entry.confirmed = false;
      return ConfirmationResult::Confirmed;
    }

    return ConfirmationResult::Interrupted;
  }

  bool cancel_pending(
    const PendingTransition &transition)
  {
    bool canceled = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto &entry = entries_[index(transition.component)];

      if (
        entry.generation == transition.generation
        && entry.pending)
      {
        entry.pending = false;
        entry.confirmed = false;
        canceled = true;
      }
    }

    condition_.notify_all();
    return canceled;
  }

  std::array<ObservationChange, 3> mark_disconnected()
  {
    std::array<ObservationChange, 3> changes;
    const std::array<Component, 3> components = {
      Component::Power,
      Component::Servo,
      Component::Stream
    };

    {
      std::lock_guard<std::mutex> lock(mutex_);

      for (std::size_t position = 0; position < components.size(); ++position)
      {
        const auto component = components[position];
        auto &entry = entries_[index(component)];
        auto &change = changes[position];

        change.component = component;
        change.previous = entry.state;
        change.current = ObservedState::Unknown;
        change.state_changed = entry.state != ObservedState::Unknown;
        change.pending_target_observed = false;
        change.source = entry.source;

        entry.state = ObservedState::Unknown;
        entry.pending = false;
        entry.confirmed = false;
        ++entry.generation;
      }
    }

    condition_.notify_all();
    return changes;
  }

  bool ready(
    bool connected,
    bool driver_ready) const
  {
    return
      connected
      && driver_ready
      && snapshot().all_enabled();
  }

private:
  struct Entry
  {
    ObservedState state{ObservedState::Unknown};
    bool pending{false};
    bool confirmed{false};
    bool expected_enabled{false};
    std::uint64_t generation{0};
    std::string source;
  };

  static constexpr std::size_t index(const Component component)
  {
    switch (component)
    {
      case Component::Power:
        return 0;
      case Component::Servo:
        return 1;
      case Component::Stream:
      default:
        return 2;
    }
  }

  ComponentStateSnapshot snapshot_unlocked() const
  {
    const auto make_snapshot = [this](const Component component) {
      const auto &entry = entries_[index(component)];
      return ComponentSnapshot{
        entry.state,
        entry.pending,
        entry.source
      };
    };

    return {
      make_snapshot(Component::Power),
      make_snapshot(Component::Servo),
      make_snapshot(Component::Stream)
    };
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::array<Entry, 3> entries_;
};

}  // namespace rby1_app_bridge
