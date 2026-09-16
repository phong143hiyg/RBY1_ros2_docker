#include <atomic>
#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "rby1_app_bridge/component_state.hpp"
#include "rby1_app_bridge/protocol.hpp"
#include "rby1_app_bridge/ready_pose_state.hpp"
#include "rby1_app_bridge/status_state.hpp"

using namespace std::chrono_literals;
using rby1_app_bridge::Component;
using rby1_app_bridge::ComponentState;
using rby1_app_bridge::ConfirmationResult;
using rby1_app_bridge::ObservedState;
using rby1_app_bridge::POWER_OFF_ORDER;
using rby1_app_bridge::ReadyPose;
using rby1_app_bridge::ReadyPoseState;
using rby1_app_bridge::StatusInputs;
using rby1_app_bridge::can_enable_power_dependent;
using rby1_app_bridge::can_prepare;
using rby1_app_bridge::components_status_json;
using rby1_app_bridge::describe_status;
using rby1_app_bridge::encode_ndjson_response;
using rby1_app_bridge::parse_ndjson_request;
using rby1_app_bridge::run_power_off_sequence;

namespace
{

class FakeRobotStateProvider
{
public:
  explicit FakeRobotStateProvider(ComponentState &state)
  : state_(state)
  {
  }

  void publish(
    const ObservedState power,
    const ObservedState servo,
    const bool stream)
  {
    state_.observe(Component::Power, power, "robot_api");
    state_.observe(Component::Servo, servo, "robot_api");
    state_.observe(
      Component::Stream,
      stream ? ObservedState::On : ObservedState::Off,
      "robot_state");
  }

  void disconnect()
  {
    state_.mark_disconnected();
  }

private:
  ComponentState &state_;
};

}  // namespace

TEST(ComponentObservation, StartsUnknownThenSyncsAlreadyEnabledRobot)
{
  ComponentState state;
  FakeRobotStateProvider provider(state);

  EXPECT_FALSE(state.snapshot().power.known());
  EXPECT_FALSE(state.snapshot().servo.known());
  EXPECT_FALSE(state.snapshot().stream.known());

  provider.publish(ObservedState::On, ObservedState::On, true);

  const auto snapshot = state.snapshot();
  EXPECT_TRUE(snapshot.power.enabled());
  EXPECT_TRUE(snapshot.servo.enabled());
  EXPECT_TRUE(snapshot.stream.enabled());
  EXPECT_TRUE(state.ready(true, true));
}

TEST(ComponentObservation, ExternalClientChangesReplaceObservedState)
{
  ComponentState state;
  FakeRobotStateProvider provider(state);
  provider.publish(ObservedState::On, ObservedState::On, true);

  provider.publish(ObservedState::On, ObservedState::Off, false);

  const auto snapshot = state.snapshot();
  EXPECT_TRUE(snapshot.power.enabled());
  EXPECT_TRUE(snapshot.servo.disabled());
  EXPECT_TRUE(snapshot.stream.disabled());
  EXPECT_FALSE(state.ready(true, true));
}

TEST(ComponentConfirmation, ServiceSuccessWithoutObservedChangeTimesOut)
{
  ComponentState state;
  FakeRobotStateProvider provider(state);
  provider.publish(ObservedState::Off, ObservedState::Off, false);

  const auto pending = state.begin_pending(Component::Power, true);
  const bool fake_service_success = true;
  ASSERT_TRUE(fake_service_success);
  EXPECT_TRUE(state.snapshot().power.pending);

  const auto result = state.wait_for(
    pending,
    std::chrono::steady_clock::now() + 20ms);

  EXPECT_EQ(result, ConfirmationResult::TimedOut);
  EXPECT_TRUE(state.snapshot().power.disabled());
  EXPECT_FALSE(state.snapshot().power.pending);
}

TEST(ComponentConfirmation, CallbackConfirmsPendingWithoutExecutorDeadlock)
{
  ComponentState state;
  FakeRobotStateProvider provider(state);
  provider.publish(ObservedState::Off, ObservedState::Off, false);

  const auto pending = state.begin_pending(Component::Servo, true);
  std::thread callback([&provider]() {
    std::this_thread::sleep_for(10ms);
    provider.publish(ObservedState::On, ObservedState::On, false);
  });

  const auto result = state.wait_for(
    pending,
    std::chrono::steady_clock::now() + 1s);
  callback.join();

  EXPECT_EQ(result, ConfirmationResult::Confirmed);
  EXPECT_TRUE(state.snapshot().servo.enabled());
  EXPECT_FALSE(state.snapshot().servo.pending);
}

TEST(ComponentConfirmation, PendingEndsOnlyAfterConfirmationIsConsumed)
{
  ComponentState state;
  FakeRobotStateProvider provider(state);
  provider.publish(ObservedState::Off, ObservedState::Off, false);
  const auto pending = state.begin_pending(Component::Power, true);

  provider.publish(ObservedState::On, ObservedState::Off, false);
  EXPECT_TRUE(state.snapshot().power.pending);

  EXPECT_EQ(
    state.wait_for(
      pending,
      std::chrono::steady_clock::now() + 1s),
    ConfirmationResult::Confirmed);
  EXPECT_FALSE(state.snapshot().power.pending);
}

TEST(ComponentConfirmation, LaterMismatchInvalidatesAnUnconsumedConfirmation)
{
  ComponentState state;
  FakeRobotStateProvider provider(state);
  provider.publish(ObservedState::Off, ObservedState::Off, false);
  const auto pending = state.begin_pending(Component::Power, true);

  provider.publish(ObservedState::On, ObservedState::Off, false);
  provider.publish(ObservedState::Off, ObservedState::Off, false);

  EXPECT_EQ(
    state.wait_for(
      pending,
      std::chrono::steady_clock::now() + 20ms),
    ConfirmationResult::TimedOut);
  EXPECT_TRUE(state.snapshot().power.disabled());
}

TEST(ComponentConnection, DisconnectMakesEveryComponentUnknown)
{
  ComponentState state;
  FakeRobotStateProvider provider(state);
  provider.publish(ObservedState::On, ObservedState::On, true);

  provider.disconnect();

  const auto snapshot = state.snapshot();
  EXPECT_FALSE(snapshot.power.known());
  EXPECT_FALSE(snapshot.servo.known());
  EXPECT_FALSE(snapshot.stream.known());
  EXPECT_FALSE(snapshot.all_enabled());
}

TEST(ComponentConnection, ReconnectResynchronizesFromProvider)
{
  ComponentState state;
  FakeRobotStateProvider provider(state);
  provider.publish(ObservedState::On, ObservedState::On, true);
  provider.disconnect();

  provider.publish(ObservedState::Off, ObservedState::On, false);

  const auto snapshot = state.snapshot();
  EXPECT_TRUE(snapshot.power.disabled());
  EXPECT_TRUE(snapshot.servo.enabled());
  EXPECT_TRUE(snapshot.stream.disabled());
}

TEST(ComponentObservation, StreamRemainsSourcedFromRobotState)
{
  ComponentState state;
  FakeRobotStateProvider provider(state);
  provider.publish(ObservedState::On, ObservedState::On, true);

  const auto snapshot = state.snapshot();
  EXPECT_EQ(snapshot.power.source, "robot_api");
  EXPECT_EQ(snapshot.servo.source, "robot_api");
  EXPECT_EQ(snapshot.stream.source, "robot_state");
  EXPECT_TRUE(snapshot.stream.enabled());
}

TEST(ComponentConnection, DisconnectInterruptsPendingTransition)
{
  ComponentState state;
  FakeRobotStateProvider provider(state);
  provider.publish(ObservedState::Off, ObservedState::Off, false);
  const auto pending = state.begin_pending(Component::Power, true);

  provider.disconnect();

  EXPECT_EQ(
    state.wait_for(
      pending,
      std::chrono::steady_clock::now() + 1s),
    ConfirmationResult::Interrupted);
  EXPECT_FALSE(state.snapshot().power.pending);
}

TEST(ComponentDependency, RequiresObservedPowerOn)
{
  ComponentState state;
  EXPECT_FALSE(can_enable_power_dependent(state.snapshot(), true));
  EXPECT_TRUE(can_enable_power_dependent(state.snapshot(), false));

  state.observe(Component::Power, ObservedState::Off, "robot_api");
  EXPECT_FALSE(can_enable_power_dependent(state.snapshot(), true));

  state.observe(Component::Power, ObservedState::On, "robot_api");
  EXPECT_TRUE(can_enable_power_dependent(state.snapshot(), true));
}

TEST(ComponentDependency, PowerOffUsesSafeServiceOrder)
{
  std::vector<Component> calls;

  const bool success = run_power_off_sequence(
    [&calls](const Component component) {
      calls.push_back(component);
      return true;
    });

  EXPECT_TRUE(success);
  EXPECT_EQ(
    calls,
    (std::vector<Component>{
      Component::Servo,
      Component::Stream,
      Component::Power
    }));
}

TEST(ComponentDependency, PowerOffAttemptsEveryStepAfterFailure)
{
  std::vector<Component> calls;

  const bool success = run_power_off_sequence(
    [&calls](const Component component) {
      calls.push_back(component);
      return component != Component::Servo;
    });

  EXPECT_FALSE(success);
  EXPECT_EQ(calls.size(), POWER_OFF_ORDER.size());
}

TEST(ComponentState, PrepareRequiresAllObservedComponentsOn)
{
  ComponentState state;
  FakeRobotStateProvider provider(state);
  provider.publish(ObservedState::On, ObservedState::On, true);
  EXPECT_TRUE(can_prepare(state.snapshot()));

  state.observe(Component::Servo, ObservedState::Off, "robot_api");
  EXPECT_FALSE(can_prepare(state.snapshot()));

  state.observe(Component::Servo, ObservedState::Unknown, "robot_api");
  EXPECT_FALSE(can_prepare(state.snapshot()));
}

TEST(ComponentState, ConcurrentStatusReadsAndUpdatesAreSafe)
{
  ComponentState state;
  std::atomic<bool> stop{false};
  std::atomic<bool> started{false};
  std::atomic<bool> valid{true};
  std::atomic<int> reads{0};

  std::thread poller([&]() {
    started = true;
    while (!stop.load())
    {
      const auto snapshot = state.snapshot();
      const auto status = components_status_json(snapshot);
      if (
        !status.contains("power")
        || !status.contains("servo")
        || !status.contains("stream"))
      {
        valid = false;
      }
      ++reads;
    }
  });

  while (!started.load())
  {
    std::this_thread::yield();
  }

  for (int index = 0; index < 1000; ++index)
  {
    const auto observed =
      index % 2 == 0 ? ObservedState::On : ObservedState::Off;
    state.observe(Component::Power, observed, "robot_api");
    state.observe(Component::Servo, observed, "robot_api");
    state.observe(Component::Stream, observed, "robot_state");
    if (index % 10 == 0)
    {
      state.mark_disconnected();
    }
  }

  stop = true;
  poller.join();
  EXPECT_GT(reads.load(), 0);
  EXPECT_TRUE(valid.load());
}

TEST(StatusProtocol, CanonicalSchemaDoesNotEncodeUnknownAsFalse)
{
  ComponentState state;
  const auto components = components_status_json(state.snapshot());

  EXPECT_FALSE(components["power"]["known"].get<bool>());
  EXPECT_TRUE(components["power"]["enabled"].is_null());
  EXPECT_FALSE(components["servo"]["known"].get<bool>());
  EXPECT_TRUE(components["servo"]["enabled"].is_null());
  EXPECT_FALSE(components["stream"]["known"].get<bool>());
  EXPECT_TRUE(components["stream"]["enabled"].is_null());
}

TEST(TcpProtocol, ExistingCommandRequestsRemainValidNdjson)
{
  const std::vector<std::string> commands = {
    "ping", "status", "joints_status", "velocity", "stop", "power",
    "servo", "stream", "prepare", "cancel", "joint_nudge",
    "set_ready_pose", "clear_ready_pose", "ready_pose", "arms_ready",
    "zero_pose"
  };

  for (const auto &command : commands)
  {
    const auto request = parse_ndjson_request(
      std::string("{\"command\":\"") + command + "\"}");
    EXPECT_EQ(request.at("command"), command);
  }

  const auto response = encode_ndjson_response({
    {"success", true},
    {"power", false},
    {"servo", false},
    {"stream", false}
  });
  EXPECT_EQ(response.back(), '\n');
  EXPECT_EQ(response.find('\n'), response.size() - 1);
}

TEST(StatusState, AlwaysReturnsNonEmptyStateAndMessage)
{
  EXPECT_EQ(describe_status({}).state, "Disconnected");
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
