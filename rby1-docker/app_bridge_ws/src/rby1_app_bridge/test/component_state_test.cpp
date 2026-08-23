#include <atomic>
#include <limits>
#include <thread>

#include <gtest/gtest.h>

#include "rby1_app_bridge/component_state.hpp"
#include "rby1_app_bridge/ready_pose_state.hpp"
#include "rby1_app_bridge/status_state.hpp"

using rby1_app_bridge::ComponentState;
using rby1_app_bridge::ReadyPose;
using rby1_app_bridge::ReadyPoseState;
using rby1_app_bridge::StatusInputs;
using rby1_app_bridge::describe_status;

TEST(ComponentState, PrepareThenStatusIsReady)
{
  ComponentState state;
  state.confirm_power(true);
  state.confirm_servo(true);
  state.confirm_stream(true);

  const auto snapshot = state.snapshot();
  EXPECT_TRUE(snapshot.power);
  EXPECT_TRUE(snapshot.servo);
  EXPECT_TRUE(snapshot.stream);
  EXPECT_TRUE(state.ready(true, true));
}

TEST(ComponentState, ServoOffInvalidatesReadyAndCanRecover)
{
  ComponentState state;
  state.confirm_power(true);
  state.confirm_servo(true);
  state.confirm_stream(true);

  state.confirm_servo(false);
  EXPECT_FALSE(state.snapshot().servo);
  EXPECT_FALSE(state.ready(true, true));

  state.confirm_servo(true);
  EXPECT_TRUE(state.ready(true, true));
}

TEST(ComponentState, PowerOffCascadesToServoAndCanRecover)
{
  ComponentState state;
  state.confirm_power(true);
  state.confirm_servo(true);
  state.confirm_stream(true);

  state.confirm_power(false);
  const auto powered_off = state.snapshot();
  EXPECT_FALSE(powered_off.power);
  EXPECT_FALSE(powered_off.servo);
  EXPECT_FALSE(state.ready(true, true));

  state.confirm_power(true);
  state.confirm_servo(true);
  EXPECT_TRUE(state.ready(true, true));
}

TEST(ComponentState, StreamOffInvalidatesReadyAndCanRecover)
{
  ComponentState state;
  state.confirm_power(true);
  state.confirm_servo(true);
  state.confirm_stream(true);

  state.confirm_stream(false);
  EXPECT_FALSE(state.ready(true, true));

  state.confirm_stream(true);
  EXPECT_TRUE(state.ready(true, true));
}

TEST(ComponentState, ConcurrentStatusPollingIsConsistentAndBoolean)
{
  ComponentState state;
  std::atomic<bool> stop{false};
  std::atomic<bool> started{false};
  std::atomic<int> reads{0};

  std::thread poller([&]() {
    started = true;
    while (!stop.load())
    {
      const auto snapshot = state.snapshot();
      (void)snapshot.power;
      (void)snapshot.servo;
      (void)snapshot.stream;
      ++reads;
    }
  });

  while (!started.load())
  {
    std::this_thread::yield();
  }

  for (int index = 0; index < 1000; ++index)
  {
    state.confirm_power(true);
    state.confirm_servo(true);
    state.confirm_stream(true);
    state.reset();
  }

  stop = true;
  poller.join();
  EXPECT_GT(reads.load(), 0);
  EXPECT_FALSE(state.ready(false, false));
}

TEST(StatusState, AlwaysReturnsNonEmptyStateAndMessage)
{
  EXPECT_EQ(
    describe_status({}).state,
    "Disconnected");
  EXPECT_EQ(
    describe_status({}).message,
    "ROS2 driver or robot is unavailable");

  StatusInputs input;
  input.driver_available = true;
  input.connected = true;
  EXPECT_EQ(describe_status(input).state, "Connected");

  input.preparing = true;
  EXPECT_EQ(describe_status(input).state, "Preparing");

  input.preparing = false;
  input.ready = true;
  EXPECT_EQ(describe_status(input).state, "Ready");
  EXPECT_EQ(describe_status(input).message, "Robot ready");

  input.driving = true;
  EXPECT_EQ(describe_status(input).state, "Driving");

  input.driving = false;
  input.joint_busy = true;
  EXPECT_EQ(describe_status(input).state, "JointBusy");

  input.joint_busy = false;
  input.fault = true;
  input.fault_message = "Control Manager major fault";
  EXPECT_EQ(describe_status(input).state, "Fault");
  EXPECT_EQ(
    describe_status(input).message,
    "Control Manager major fault");
}

TEST(ReadyPoseState, SavesOnlyACompleteFiniteUpperBodyPose)
{
  ReadyPoseState state;
  const ReadyPose valid_pose = {
    {"torso", {0.0, 0.1, 0.2, 0.3, 0.4, 0.5}},
    {"head", {0.6, 0.7}},
    {"right_arm", {0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4}},
    {"left_arm", {-0.8, -0.9, -1.0, -1.1, -1.2, -1.3, -1.4}}
  };

  EXPECT_TRUE(state.save(valid_pose));
  EXPECT_TRUE(state.saved());
  EXPECT_EQ(state.snapshot(), valid_pose);

  ReadyPose incomplete_pose = valid_pose;
  incomplete_pose["head"].pop_back();
  EXPECT_FALSE(state.save(incomplete_pose));
  EXPECT_EQ(state.snapshot(), valid_pose);

  ReadyPose invalid_pose = valid_pose;
  invalid_pose["torso"][0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(state.save(invalid_pose));
  EXPECT_EQ(state.snapshot(), valid_pose);

  state.clear();
  EXPECT_FALSE(state.saved());
  EXPECT_TRUE(state.snapshot().empty());
}
