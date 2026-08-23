#pragma once

#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rby1_app_bridge
{

using ReadyPose = std::map<std::string, std::vector<double>>;

class ReadyPoseState
{
public:
  bool save(const ReadyPose &pose)
  {
    if (!is_valid(pose))
    {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    pose_ = pose;
    saved_ = true;
    return true;
  }

  bool saved() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return saved_;
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_.clear();
    saved_ = false;
  }

  ReadyPose snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pose_;
  }

  static bool is_valid(const ReadyPose &pose)
  {
    static const std::map<std::string, std::size_t> expected_counts = {
      {"torso", 6},
      {"head", 2},
      {"right_arm", 7},
      {"left_arm", 7}
    };

    if (pose.size() != expected_counts.size())
    {
      return false;
    }

    for (const auto &[group, expected_count] : expected_counts)
    {
      const auto iterator = pose.find(group);

      if (iterator == pose.end() || iterator->second.size() != expected_count)
      {
        return false;
      }

      for (const double position : iterator->second)
      {
        if (!std::isfinite(position))
        {
          return false;
        }
      }
    }

    return true;
  }

private:
  mutable std::mutex mutex_;
  ReadyPose pose_;
  bool saved_{false};
};

}  // namespace rby1_app_bridge
