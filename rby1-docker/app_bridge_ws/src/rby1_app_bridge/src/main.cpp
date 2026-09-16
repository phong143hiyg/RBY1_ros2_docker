#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <exception>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include <nlohmann/json.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rby1_msgs/action/rby1_joint_command.hpp>
#include <rby1_msgs/msg/joint_command.hpp>
#include <rby1_msgs/msg/robot_state.hpp>
#include <rby1_msgs/srv/state_on_off.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rby1_app_bridge/component_state.hpp"
#include "rby1_app_bridge/protocol.hpp"
#include "rby1_app_bridge/ready_pose_state.hpp"
#include "rby1_app_bridge/status_state.hpp"

using json = nlohmann::json;
using boost::asio::ip::tcp;
using namespace std::chrono_literals;

class RBY1AppBridge : public rclcpp::Node
{
public:
  using JointAction =
    rby1_msgs::action::Rby1JointCommand;

  using JointGoalHandle =
    rclcpp_action::ClientGoalHandle<JointAction>;

  RBY1AppBridge()
  : Node("rby1_app_bridge"),
    io_context_(),
    acceptor_(
      io_context_,
      tcp::endpoint(tcp::v4(), TCP_PORT))
  {
    joint_groups_ = {
      {"torso", JointGroupState{6}},
      {"right_arm", JointGroupState{7}},
      {"left_arm", JointGroupState{7}},
      {"head", JointGroupState{2}}
    };

    cmd_vel_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(
        "/rby1/cmd_vel",
        10);

    robot_state_subscriber_ =
      create_subscription<rby1_msgs::msg::RobotState>(
        "/rby1/robot_state",
        10,
        std::bind(
          &RBY1AppBridge::robot_state_callback,
          this,
          std::placeholders::_1));

    all_joint_state_subscriber_ =
      create_subscription<sensor_msgs::msg::JointState>(
        "/rby1/joint_states",
        10,
        std::bind(
          &RBY1AppBridge::all_joint_state_callback,
          this,
          std::placeholders::_1));

    torso_joint_state_subscriber_ =
      create_group_joint_subscription(
        "/rby1/joint_states/torso",
        "torso");

    right_arm_joint_state_subscriber_ =
      create_group_joint_subscription(
        "/rby1/joint_states/right_arm",
        "right_arm");

    left_arm_joint_state_subscriber_ =
      create_group_joint_subscription(
        "/rby1/joint_states/left_arm",
        "left_arm");

    head_joint_state_subscriber_ =
      create_group_joint_subscription(
        "/rby1/joint_states/head",
        "head");

    power_client_ =
      create_client<rby1_msgs::srv::StateOnOff>(
        "/rby1/robot_power");

    servo_client_ =
      create_client<rby1_msgs::srv::StateOnOff>(
        "/rby1/robot_servo");

    stream_client_ =
      create_client<rby1_msgs::srv::StateOnOff>(
        "/rby1/stream_control");

    cancel_client_ =
      create_client<std_srvs::srv::Trigger>(
        "/rby1/cancel_control");

    joint_action_client_ =
      rclcpp_action::create_client<JointAction>(
        this,
        "/rby1/robot_joint");

    velocity_timer_ =
      create_wall_timer(
        50ms,
        std::bind(
          &RBY1AppBridge::velocity_watchdog,
          this));

    tcp_thread_ =
      std::thread(
        [this]()
        {
          tcp_server_loop();
        });

    RCLCPP_INFO(
      get_logger(),
      "RBY1 app bridge listening on port %d",
      TCP_PORT);
  }

  ~RBY1AppBridge() override
  {
    stop_requested_.store(true);
    stop_robot();

    boost::system::error_code error;
    acceptor_.close(error);
    io_context_.stop();

    if (tcp_thread_.joinable())
    {
      tcp_thread_.join();
    }
  }

private:
  struct JointGroupState
  {
    explicit JointGroupState(
      std::size_t count = 0)
    : expected_count(count)
    {
    }

    std::size_t expected_count{0};
    std::vector<std::string> names;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> efforts;
  };

  static constexpr int TCP_PORT = 8081;

  static constexpr double MAX_LINEAR_SPEED = 0.20;
  static constexpr double MAX_ANGULAR_SPEED = 0.50;

  static constexpr auto ROBOT_STATE_TIMEOUT =
    std::chrono::milliseconds(1500);

  static constexpr auto VELOCITY_COMMAND_TIMEOUT =
    std::chrono::milliseconds(350);

  static constexpr auto COMPONENT_COMMAND_TIMEOUT =
    std::chrono::seconds(10);

  static rby1_app_bridge::ObservedState component_state_from_message(
    const std::uint8_t state)
  {
    if (state == rby1_msgs::msg::RobotState::COMPONENT_STATE_ON)
    {
      return rby1_app_bridge::ObservedState::On;
    }

    if (state == rby1_msgs::msg::RobotState::COMPONENT_STATE_OFF)
    {
      return rby1_app_bridge::ObservedState::Off;
    }

    return rby1_app_bridge::ObservedState::Unknown;
  }

  void log_observation_change(
    const rby1_app_bridge::ObservationChange &change)
  {
    if (change.state_changed)
    {
      RCLCPP_INFO(
        get_logger(),
        "%s observed: %s -> %s (source=%s)",
        rby1_app_bridge::to_string(change.component),
        rby1_app_bridge::to_string(change.previous),
        rby1_app_bridge::to_string(change.current),
        change.source.c_str());
    }

    if (change.pending_target_observed)
    {
      RCLCPP_INFO(
        get_logger(),
        "%s pending target observed (source=%s)",
        rby1_app_bridge::to_string(change.component),
        change.source.c_str());
    }
  }

  void log_components_unknown(
    const std::string &reason,
    const std::array<
      rby1_app_bridge::ObservationChange,
      3
    > &changes)
  {
    RCLCPP_WARN(
      get_logger(),
      "Robot connection lost: %s; component states are unknown",
      reason.c_str());

    for (const auto &change : changes)
    {
      log_observation_change(change);
    }
  }

  static std::size_t expected_joint_count(
    const std::string &group)
  {
    if (group == "torso")
    {
      return 6;
    }

    if (
      group == "right_arm"
      || group == "left_arm")
    {
      return 7;
    }

    if (group == "head")
    {
      return 2;
    }

    return 0;
  }

  rclcpp::Subscription<
    sensor_msgs::msg::JointState
  >::SharedPtr create_group_joint_subscription(
    const std::string &topic,
    const std::string &group)
  {
    return create_subscription<
      sensor_msgs::msg::JointState
    >(
      topic,
      10,
      [this, group](
        const sensor_msgs::msg::JointState::SharedPtr message)
      {
        group_joint_state_callback(
          group,
          message);
      });
  }

  void robot_state_callback(
    const rby1_msgs::msg::RobotState::SharedPtr message)
  {
    bool reconnected = false;
    rby1_app_bridge::ObservationChange power_change;
    rby1_app_bridge::ObservationChange servo_change;
    rby1_app_bridge::ObservationChange stream_change;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      reconnected =
        received_robot_state_
        && !robot_is_connected_unlocked();

      received_robot_state_ = true;

      control_manager_state_ =
        static_cast<int>(
          message->control_manager_state);

      stream_enabled_ =
        static_cast<bool>(
          message->robot_stream_state);

      collision_ =
        static_cast<bool>(
          message->collision);

      emergency_stop_ =
        static_cast<bool>(
          message->emo_state);

      robot_version_ =
        static_cast<double>(
          message->robot_version);

      last_robot_state_time_ =
        std::chrono::steady_clock::now();

      connection_loss_reported_ = false;

      power_change = component_state_.observe(
        rby1_app_bridge::Component::Power,
        component_state_from_message(
          message->power_state),
        "robot_api");

      servo_change = component_state_.observe(
        rby1_app_bridge::Component::Servo,
        component_state_from_message(
          message->servo_state),
        "robot_api");

      stream_change = component_state_.observe(
        rby1_app_bridge::Component::Stream,
        message->robot_stream_state
          ? rby1_app_bridge::ObservedState::On
          : rby1_app_bridge::ObservedState::Off,
        "robot_state");
    }

    if (reconnected)
    {
      RCLCPP_INFO(
        get_logger(),
        "Robot state reconnected; resynchronizing components");
    }

    log_observation_change(power_change);
    log_observation_change(servo_change);
    log_observation_change(stream_change);
  }

  static std::string normalize_joint_name(
    std::string name)
  {
    std::transform(
      name.begin(),
      name.end(),
      name.begin(),
      [](
        unsigned char character)
      {
        if (
          character == '/'
          || character == '-')
        {
          return '_';
        }

        return static_cast<char>(
          std::tolower(character));
      });

    return name;
  }

  static int joint_number(
    const std::string &name)
  {
    const std::string normalized =
      normalize_joint_name(name);

    std::size_t end = normalized.size();

    while (
      end > 0
      && !std::isdigit(
        static_cast<unsigned char>(
          normalized[end - 1])))
    {
      --end;
    }

    if (end == 0)
    {
      return 10000;
    }

    std::size_t begin = end;

    while (
      begin > 0
      && std::isdigit(
        static_cast<unsigned char>(
          normalized[begin - 1])))
    {
      --begin;
    }

    try
    {
      return std::stoi(
        normalized.substr(
          begin,
          end - begin));
    }
    catch (...)
    {
      return 10000;
    }
  }

  static std::string classify_joint(
    const std::string &name)
  {
    const std::string normalized =
      normalize_joint_name(name);

    if (
      normalized.find("right_arm")
      != std::string::npos)
    {
      return "right_arm";
    }

    if (
      normalized.find("left_arm")
      != std::string::npos)
    {
      return "left_arm";
    }

    if (
      normalized.find("torso")
      != std::string::npos)
    {
      return "torso";
    }

    if (
      normalized.find("head")
      != std::string::npos)
    {
      return "head";
    }

    return "";
  }

  void all_joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::map<
      std::string,
      std::vector<std::size_t>
    > grouped_indices = {
      {"torso", {}},
      {"right_arm", {}},
      {"left_arm", {}},
      {"head", {}}
    };

    for (
      std::size_t index = 0;
      index < message->name.size();
      ++index)
    {
      const std::string group =
        classify_joint(
          message->name[index]);

      if (!group.empty())
      {
        grouped_indices[group].push_back(
          index);
      }
    }

    for (
      auto &[group, indices]
      : grouped_indices)
    {
      std::sort(
        indices.begin(),
        indices.end(),
        [&message](
          std::size_t left,
          std::size_t right)
        {
          return joint_number(
            message->name[left])
            < joint_number(
              message->name[right]);
        });

      const auto expected =
        expected_joint_count(group);

      if (indices.size() != expected)
      {
        continue;
      }

      JointGroupState updated(expected);

      for (
        const std::size_t index
        : indices)
      {
        updated.names.push_back(
          message->name[index]);

        if (
          index
          < message->position.size())
        {
          updated.positions.push_back(
            message->position[index]);
        }

        if (
          index
          < message->velocity.size())
        {
          updated.velocities.push_back(
            message->velocity[index]);
        }

        if (
          index
          < message->effort.size())
        {
          updated.efforts.push_back(
            message->effort[index]);
        }
      }

      if (
        updated.positions.size()
        == expected)
      {
        std::lock_guard<std::mutex> lock(
          state_mutex_);

        joint_groups_[group] =
          std::move(updated);
      }
    }
  }

  void group_joint_state_callback(
    const std::string &group,
    const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const std::size_t expected =
      expected_joint_count(group);

    if (
      message->position.size()
      != expected)
    {
      return;
    }

    JointGroupState updated(expected);

    updated.positions.assign(
      message->position.begin(),
      message->position.end());

    updated.velocities.assign(
      message->velocity.begin(),
      message->velocity.end());

    updated.efforts.assign(
      message->effort.begin(),
      message->effort.end());

    if (
      message->name.size()
      == expected)
    {
      updated.names.assign(
        message->name.begin(),
        message->name.end());
    }
    else
    {
      for (
        std::size_t index = 0;
        index < expected;
        ++index)
      {
        updated.names.push_back(
          group
          + "_"
          + std::to_string(index));
      }
    }

    std::lock_guard<std::mutex> lock(
      state_mutex_);

    joint_groups_[group] =
      std::move(updated);
  }

  bool robot_is_connected_unlocked() const
  {
    if (!received_robot_state_)
    {
      return false;
    }

    return (
      std::chrono::steady_clock::now()
      - last_robot_state_time_)
      < ROBOT_STATE_TIMEOUT;
  }

  void check_connection_timeout()
  {
    bool connection_lost = false;
    std::array<
      rby1_app_bridge::ObservationChange,
      3
    > changes;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (
        received_robot_state_
        && !robot_is_connected_unlocked()
        && !connection_loss_reported_)
      {
        connection_loss_reported_ = true;
        connection_lost = true;
        changes = component_state_.mark_disconnected();
      }
    }

    if (connection_lost)
    {
      log_components_unknown(
        "RobotState timeout",
        changes);
    }
  }

  bool robot_is_ready_unlocked() const
  {
    return
      control_manager_state_
        == rby1_msgs::msg::RobotState::STATE_ENABLE
      ||
      control_manager_state_
        == rby1_msgs::msg::RobotState::STATE_EXECUTING;
  }

  bool control_manager_has_fault_unlocked() const
  {
    return
      control_manager_state_
        == rby1_msgs::msg::RobotState::STATE_MAJOR_FAULT
      || control_manager_state_
        == rby1_msgs::msg::RobotState::STATE_MINOR_FAULT;
  }

  std::string control_manager_fault_message_unlocked() const
  {
    if (
      control_manager_state_
      == rby1_msgs::msg::RobotState::STATE_MAJOR_FAULT)
    {
      return "Control Manager major fault";
    }

    if (
      control_manager_state_
      == rby1_msgs::msg::RobotState::STATE_MINOR_FAULT)
    {
      return "Control Manager minor fault";
    }

    if (collision_)
    {
      return "Robot collision is active";
    }

    if (emergency_stop_)
    {
      return "Emergency stop is active";
    }

    return "";
  }

  json set_velocity_command(
    double linear_x,
    double linear_y,
    double angular_z)
  {
    if (
      !std::isfinite(linear_x)
      || !std::isfinite(linear_y)
      || !std::isfinite(angular_z))
    {
      return {
        {"success", false},
        {"error", "Velocity contains invalid values"}
      };
    }

    linear_x =
      std::clamp(
        linear_x,
        -MAX_LINEAR_SPEED,
        MAX_LINEAR_SPEED);

    linear_y =
      std::clamp(
        linear_y,
        -MAX_LINEAR_SPEED,
        MAX_LINEAR_SPEED);

    angular_z =
      std::clamp(
        angular_z,
        -MAX_ANGULAR_SPEED,
        MAX_ANGULAR_SPEED);

    {
      std::lock_guard<std::mutex> lock(
        state_mutex_);

      if (!robot_is_connected_unlocked())
      {
        return {
          {"success", false},
          {"error", "Robot state is not connected"}
        };
      }

      if (!robot_is_ready_unlocked())
      {
        return {
          {"success", false},
          {"error", "Robot is not ENABLE or EXECUTING"}
        };
      }

      if (!stream_enabled_)
      {
        return {
          {"success", false},
          {"error", "Stream control is OFF"}
        };
      }

      if (joint_action_busy_)
      {
        return {
          {"success", false},
          {"error", "A joint command is running"}
        };
      }

      if (collision_)
      {
        return {
          {"success", false},
          {"error", "Robot collision is active"}
        };
      }

      if (emergency_stop_)
      {
        return {
          {"success", false},
          {"error", "Emergency stop is active"}
        };
      }

      desired_linear_x_ = linear_x;
      desired_linear_y_ = linear_y;
      desired_angular_z_ = angular_z;

      velocity_command_active_ = true;

      last_velocity_command_time_ =
        std::chrono::steady_clock::now();
    }

    publish_velocity(
      linear_x,
      linear_y,
      angular_z);

    return {{"success", true}};
  }

  void stop_robot()
  {
    {
      std::lock_guard<std::mutex> lock(
        state_mutex_);

      desired_linear_x_ = 0.0;
      desired_linear_y_ = 0.0;
      desired_angular_z_ = 0.0;

      velocity_command_active_ = false;
    }

    for (
      int index = 0;
      index < 3;
      ++index)
    {
      publish_velocity(
        0.0,
        0.0,
        0.0);
    }
  }

  void velocity_watchdog()
  {
    check_connection_timeout();

    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;

    bool command_is_valid = false;

    {
      std::lock_guard<std::mutex> lock(
        state_mutex_);

      const bool connected =
        robot_is_connected_unlocked();

      if (velocity_command_active_)
      {
        const auto elapsed =
          std::chrono::steady_clock::now()
          - last_velocity_command_time_;

        command_is_valid =
          elapsed
            <= VELOCITY_COMMAND_TIMEOUT
          && connected
          && robot_is_ready_unlocked()
          && stream_enabled_
          && !joint_action_busy_
          && !collision_
          && !emergency_stop_;
      }

      if (command_is_valid)
      {
        linear_x = desired_linear_x_;
        linear_y = desired_linear_y_;
        angular_z = desired_angular_z_;
      }
      else
      {
        desired_linear_x_ = 0.0;
        desired_linear_y_ = 0.0;
        desired_angular_z_ = 0.0;

        velocity_command_active_ = false;
      }
    }

    publish_velocity(
      linear_x,
      linear_y,
      angular_z);
  }

  void publish_velocity(
    double linear_x,
    double linear_y,
    double angular_z)
  {
    geometry_msgs::msg::Twist message;

    message.linear.x = linear_x;
    message.linear.y = linear_y;
    message.linear.z = 0.0;
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = angular_z;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "velocity: vx=%.3f vy=%.3f wz=%.3f",
      linear_x,
      linear_y,
      angular_z);

    cmd_vel_publisher_->publish(message);
  }

  json call_state_service(
    const rclcpp::Client<
      rby1_msgs::srv::StateOnOff
    >::SharedPtr &client,
    const std::string &service_name,
    bool enabled,
    const std::string &parameters,
    double value,
    const std::chrono::steady_clock::time_point deadline)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline)
    {
      return {
        {"success", false},
        {"error", "Component command deadline expired before service call"}
      };
    }

    const auto service_wait = std::min(
      deadline - now,
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(3s));

    if (!client->wait_for_service(service_wait))
    {
      return {
        {"success", false},
        {
          "error",
          "Service not available: "
          + service_name
        }
      };
    }

    auto request =
      std::make_shared<
        rby1_msgs::srv::StateOnOff::Request
      >();

    request->state = enabled;
    request->parameters = parameters;
    request->value = value;

    auto future =
      client->async_send_request(request);

    if (
      future.wait_until(deadline)
      != std::future_status::ready)
    {
      return {
        {"success", false},
        {
          "error",
          "Service timeout: "
          + service_name
        }
      };
    }

    const auto response =
      future.get();

    return {
      {"success", response->success},
      {"message", response->message},
      {"service", service_name},
      {"enabled", enabled}
    };
  }

  void log_component_state(
    const std::string &operation)
  {
    const auto state = component_state_.snapshot();

    RCLCPP_INFO(
      get_logger(),
      "%s: power=%s servo=%s stream=%s",
      operation.c_str(),
      rby1_app_bridge::to_string(state.power.state),
      rby1_app_bridge::to_string(state.servo.state),
      rby1_app_bridge::to_string(state.stream.state));
  }

  static rby1_app_bridge::ComponentSnapshot component_snapshot(
    const rby1_app_bridge::ComponentStateSnapshot &state,
    const rby1_app_bridge::Component component)
  {
    if (component == rby1_app_bridge::Component::Power)
    {
      return state.power;
    }

    if (component == rby1_app_bridge::Component::Servo)
    {
      return state.servo;
    }

    return state.stream;
  }

  json command_component_state(
    const rby1_app_bridge::Component component,
    const rclcpp::Client<
      rby1_msgs::srv::StateOnOff
    >::SharedPtr &client,
    const std::string &service_name,
    const bool enabled,
    const std::string &parameters,
    const double value)
  {
    rby1_app_bridge::PendingTransition transition;
    check_connection_timeout();

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!robot_is_connected_unlocked())
      {
        return {
          {"success", false},
          {"error", "Robot state is not connected"}
        };
      }

      transition = component_state_.begin_pending(
        component,
        enabled);
    }

    const auto deadline =
      transition.started_at
      + COMPONENT_COMMAND_TIMEOUT;

    RCLCPP_INFO(
      get_logger(),
      "%s pending started: target=%s deadline_ms=%lld",
      rby1_app_bridge::to_string(component),
      enabled ? "on" : "off",
      static_cast<long long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          COMPONENT_COMMAND_TIMEOUT).count()));

    json service_result;
    try
    {
      service_result = call_state_service(
        client,
        service_name,
        enabled,
        parameters,
        value,
        deadline);
    }
    catch (const std::exception &exception)
    {
      component_state_.cancel_pending(transition);
      RCLCPP_ERROR(
        get_logger(),
        "%s pending ended: service exception: %s",
        rby1_app_bridge::to_string(component),
        exception.what());
      return {
        {"success", false},
        {"error", exception.what()},
        {"service", service_name}
      };
    }

    if (!service_result.value("success", false))
    {
      component_state_.cancel_pending(transition);
      RCLCPP_WARN(
        get_logger(),
        "%s pending ended: service failed (%s)",
        rby1_app_bridge::to_string(component),
        service_result.value(
          "message",
          service_result.value("error", "no message")).c_str());
      return service_result;
    }

    const auto confirmation =
      component_state_.wait_for(
        transition,
        deadline);

    const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now()
        - transition.started_at).count();

    const auto observed = component_snapshot(
      component_state_.snapshot(),
      component);

    if (confirmation == rby1_app_bridge::ConfirmationResult::Confirmed)
    {
      RCLCPP_INFO(
        get_logger(),
        "%s pending ended: observed=%s source=%s elapsed_ms=%lld",
        rby1_app_bridge::to_string(component),
        rby1_app_bridge::to_string(observed.state),
        observed.source.c_str(),
        static_cast<long long>(elapsed));

      service_result["confirmed"] = true;
      service_result["component"] =
        rby1_app_bridge::component_status_json(observed);
      return service_result;
    }

    const bool timed_out =
      confirmation == rby1_app_bridge::ConfirmationResult::TimedOut;

    RCLCPP_ERROR(
      get_logger(),
      "%s pending ended: %s target=%s observed=%s source=%s elapsed_ms=%lld",
      rby1_app_bridge::to_string(component),
      timed_out ? "timeout" : "interrupted",
      enabled ? "on" : "off",
      rby1_app_bridge::to_string(observed.state),
      observed.source.c_str(),
      static_cast<long long>(elapsed));

    return {
      {"success", false},
      {
        "error",
        std::string(rby1_app_bridge::to_string(component))
          + " service succeeded but observed "
          + observed.source
          + " state did not become "
          + (enabled ? "ON" : "OFF")
          + (timed_out
            ? " before the monotonic deadline"
            : " because observation was interrupted")
      },
      {"service_result", service_result},
      {
        "component",
        rby1_app_bridge::component_status_json(observed)
      }
    };
  }

  json set_power_state(bool enabled)
  {
    const json result = command_component_state(
      rby1_app_bridge::Component::Power,
      power_client_,
      "/rby1/robot_power",
      enabled,
      "all",
      0.0);

    const bool success = result.value("success", false);
    RCLCPP_INFO(
      get_logger(),
      "Power %s: %s (%s)",
      enabled ? "ON" : "OFF",
      success ? "success" : "failed",
      result.value(
        "message",
        result.value("error", "no message")).c_str());

    log_component_state("State after Power");
    return result;
  }

  json safely_disable_power()
  {
    stop_robot();

    json steps = json::object();
    const bool all_steps_succeeded =
      rby1_app_bridge::run_power_off_sequence(
        [this, &steps](
          const rby1_app_bridge::Component component)
        {
          json result;
          std::string name;

          try
          {
            if (component == rby1_app_bridge::Component::Servo)
            {
              name = "servo";
              result = set_servo_state(false);
            }
            else if (component == rby1_app_bridge::Component::Stream)
            {
              name = "stream";
              result = set_stream_state(false);
            }
            else
            {
              name = "power";
              result = set_power_state(false);
            }
          }
          catch (const std::exception &exception)
          {
            result = {
              {"success", false},
              {"error", exception.what()}
            };
          }

          const bool success =
            result.value("success", false);

          steps[name] = std::move(result);
          return success;
        });

    const auto state = component_state_.snapshot();
    const bool fully_disabled =
      state.all_known_disabled();

    return {
      {
        "success",
        all_steps_succeeded && fully_disabled
      },
      {
        "message",
        all_steps_succeeded && fully_disabled
          ? "Servo, Stream and Power disabled"
          : "Power shutdown did not complete successfully"
      },
      {"ready", false},
      {
        "power",
        rby1_app_bridge::legacy_component_enabled(state.power)
      },
      {
        "servo",
        rby1_app_bridge::legacy_component_enabled(state.servo)
      },
      {
        "stream",
        rby1_app_bridge::legacy_component_enabled(state.stream)
      },
      {
        "components",
        rby1_app_bridge::components_status_json(state)
      },
      {"steps", steps}
    };
  }

  json set_servo_state(bool enabled)
  {
    const json result = command_component_state(
      rby1_app_bridge::Component::Servo,
      servo_client_,
      "/rby1/robot_servo",
      enabled,
      "all",
      0.0);

    const bool success = result.value("success", false);
    RCLCPP_INFO(
      get_logger(),
      "Servo %s: %s (%s)",
      enabled ? "ON" : "OFF",
      success ? "success" : "failed",
      result.value(
        "message",
        result.value("error", "no message")).c_str());

    log_component_state("State after Servo");
    return result;
  }

  json set_stream_state(bool enabled)
  {
    const json result = command_component_state(
      rby1_app_bridge::Component::Stream,
      stream_client_,
      "/rby1/stream_control",
      enabled,
      "",
      enabled ? 20.0 : 0.0);

    log_component_state("State after Stream");
    return result;
  }

  bool wait_for_ready(
    std::chrono::seconds timeout)
  {
    const auto deadline =
      std::chrono::steady_clock::now()
      + timeout;

    while (
      std::chrono::steady_clock::now()
      < deadline)
    {
      {
        std::lock_guard<std::mutex> lock(
          state_mutex_);

        if (
          robot_is_connected_unlocked()
          && robot_is_ready_unlocked())
        {
          return true;
        }
      }

      std::this_thread::sleep_for(100ms);
    }

    return false;
  }

  json prepare_robot()
  {
    RCLCPP_INFO(get_logger(), "Prepare started");

    const auto initial_components =
      component_state_.snapshot();

    if (
      !rby1_app_bridge::can_prepare(
        initial_components))
    {
      RCLCPP_WARN(
        get_logger(),
        "Prepare rejected: power=%s servo=%s stream=%s",
        rby1_app_bridge::to_string(initial_components.power.state),
        rby1_app_bridge::to_string(initial_components.servo.state),
        rby1_app_bridge::to_string(initial_components.stream.state));

      return {
        {"success", false},
        {"ready", false},
        {
          "power",
          rby1_app_bridge::legacy_component_enabled(
            initial_components.power)
        },
        {
          "servo",
          rby1_app_bridge::legacy_component_enabled(
            initial_components.servo)
        },
        {
          "stream",
          rby1_app_bridge::legacy_component_enabled(
            initial_components.stream)
        },
        {
          "components",
          rby1_app_bridge::components_status_json(
            initial_components)
        },
        {
          "message",
          "Power, Servo and Stream must be enabled before prepare"
        }
      };
    }

    stop_robot();

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      preparing_ = true;
    }

    const auto finish_prepare = [this](json result) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      preparing_ = false;
      return result;
    };

    if (!wait_for_ready(12s))
    {
      RCLCPP_ERROR(get_logger(), "Prepare failed waiting for driver ready state");
      return finish_prepare({
        {"success", false},
        {"ready", false},
        {"message", "Robot did not reach ENABLE/EXECUTING"}
      });
    }

    bool connected = false;
    bool driver_ready = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      connected = robot_is_connected_unlocked();
      driver_ready = robot_is_ready_unlocked();
    }

    const bool ready = component_state_.ready(connected, driver_ready);
    log_component_state("Prepare completed");
    RCLCPP_INFO(get_logger(), "Prepare finished: ready=%s", ready ? "true" : "false");

    if (!ready)
    {
      return finish_prepare({
        {"success", false},
        {"ready", false},
        {"message", "A prepare precondition is no longer satisfied"}
      });
    }

    return finish_prepare({{"success", true}, {"ready", true}});
  }

  std::vector<double> get_group_positions(
    const std::string &group)
  {
    std::lock_guard<std::mutex> lock(
      state_mutex_);

    const auto iterator =
      joint_groups_.find(group);

    if (
      iterator
      == joint_groups_.end())
    {
      return {};
    }

    return iterator->second.positions;
  }

  json save_ready_pose()
  {
    rby1_app_bridge::ReadyPose pose;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      for (const std::string &group : {
          "torso",
          "head",
          "right_arm",
          "left_arm"
        })
      {
        const auto iterator = joint_groups_.find(group);
        const std::size_t expected_count = expected_joint_count(group);

        if (
          iterator == joint_groups_.end()
          || iterator->second.positions.size() != expected_count)
        {
          return {
            {"success", false},
            {"error", "Joint state is not ready for " + group},
            {"ready_pose_saved", ready_pose_.saved()}
          };
        }

        pose[group] = iterator->second.positions;
      }
    }

    if (!ready_pose_.save(pose))
    {
      return {
        {"success", false},
        {"error", "Ready pose contains invalid joint positions"},
        {"ready_pose_saved", ready_pose_.saved()}
      };
    }

    RCLCPP_INFO(get_logger(), "Ready pose saved (22 upper-body joints)");

    return {
      {"success", true},
      {"message", "Ready pose saved"},
      {"ready_pose_saved", true}
    };
  }

  json restore_ready_pose(double minimum_time)
  {
    const auto pose = ready_pose_.snapshot();

    if (!rby1_app_bridge::ReadyPoseState::is_valid(pose))
    {
      return {
        {"success", false},
        {"error", "Ready pose has not been saved"},
        {"ready_pose_saved", false}
      };
    }

    json result = send_joint_goal(pose, minimum_time, 10);
    result["ready_pose_saved"] = true;
    return result;
  }

  json clear_ready_pose()
  {
    ready_pose_.clear();
    RCLCPP_INFO(get_logger(), "Ready pose cleared");

    return {
      {"success", true},
      {"message", "Ready pose cleared"},
      {"ready_pose_saved", false}
    };
  }

  json create_joint_status_response()
  {
    std::lock_guard<std::mutex> lock(
      state_mutex_);

    json groups = json::object();

    for (
      const auto &[name, state]
      : joint_groups_)
    {
      groups[name] = {
        {"names", state.names},
        {"positions", state.positions},
        {"velocities", state.velocities},
        {"efforts", state.efforts},
        {
          "expected_count",
          state.expected_count
        },
        {
          "ready",
          state.positions.size()
            == state.expected_count
        }
      };
    }

    return {
      {"success", true},
      {"groups", groups},
      {
        "action",
        {
          {"busy", joint_action_busy_},
          {"state", joint_action_state_}
        }
      }
    };
  }

  static bool validate_joint_commands(
    const std::map<
      std::string,
      std::vector<double>
    > &commands,
    std::string &error)
  {
    const std::map<
      std::string,
      std::size_t
    > expected_counts = {
      {"torso", 6},
      {"right_arm", 7},
      {"left_arm", 7},
      {"head", 2}
    };

    if (commands.empty())
    {
      error = "No joint groups were supplied";
      return false;
    }

    for (
      const auto &[group, positions]
      : commands)
    {
      const auto iterator =
        expected_counts.find(group);

      if (
        iterator
        == expected_counts.end())
      {
        error =
          "Invalid joint group: "
          + group;

        return false;
      }

      if (
        positions.size()
        != iterator->second)
      {
        error =
          group
          + " requires "
          + std::to_string(
            iterator->second)
          + " positions";

        return false;
      }

      for (
        const double value
        : positions)
      {
        if (!std::isfinite(value))
        {
          error =
            group
            + " contains an invalid value";

          return false;
        }
      }
    }

    return true;
  }

  static void assign_joint_command(
    JointAction::Goal &goal,
    const std::string &group,
    const std::vector<double> &positions,
    double minimum_time)
  {
    rby1_msgs::msg::JointCommand command;

    command.position = positions;
    command.minimum_time = minimum_time;

    if (group == "torso")
    {
      goal.torso = command;
    }
    else if (group == "right_arm")
    {
      goal.right_arm = command;
    }
    else if (group == "left_arm")
    {
      goal.left_arm = command;
    }
    else if (group == "head")
    {
      goal.head = command;
    }
  }

  json execute_joint_goal(
    const std::map<
      std::string,
      std::vector<double>
    > &commands,
    double minimum_time,
    int priority)
  {
    if (
      !joint_action_client_
        ->wait_for_action_server(3s))
    {
      return {
        {"success", false},
        {
          "error",
          "Action not available: /rby1/robot_joint"
        }
      };
    }

    JointAction::Goal goal;
    goal.priority = priority;

    for (
      const auto &[group, positions]
      : commands)
    {
      assign_joint_command(
        goal,
        group,
        positions,
        minimum_time);
    }

    auto options =
      rclcpp_action::Client<
        JointAction
      >::SendGoalOptions();

    options.feedback_callback =
      [this](
        JointGoalHandle::SharedPtr,
        const std::shared_ptr<
          const JointAction::Feedback
        > feedback)
      {
        std::lock_guard<std::mutex> lock(
          state_mutex_);

        joint_action_state_ =
          feedback->current_state;
      };

    {
      std::lock_guard<std::mutex> lock(
        state_mutex_);

      joint_action_state_ = "sending";
    }

    auto goal_future =
      joint_action_client_
        ->async_send_goal(
          goal,
          options);

    if (
      goal_future.wait_for(5s)
      != std::future_status::ready)
    {
      return {
        {"success", false},
        {"error", "Sending joint goal timed out"}
      };
    }

    const auto goal_handle =
      goal_future.get();

    if (!goal_handle)
    {
      return {
        {"success", false},
        {"error", "Joint goal was rejected"}
      };
    }

    {
      std::lock_guard<std::mutex> lock(
        state_mutex_);

      active_joint_goal_ =
        goal_handle;

      joint_action_state_ =
        "executing";
    }

    auto result_future =
      joint_action_client_
        ->async_get_result(
          goal_handle);

    const auto timeout =
      std::chrono::milliseconds(
        static_cast<int>(
          std::max(
            minimum_time + 15.0,
            20.0)
          * 1000.0));

    if (
      result_future.wait_for(timeout)
      != std::future_status::ready)
    {
      auto cancel_future =
        joint_action_client_
          ->async_cancel_goal(
            goal_handle);

      cancel_future.wait_for(2s);

      return {
        {"success", false},
        {"error", "Joint action result timed out"}
      };
    }

    const auto wrapped_result =
      result_future.get();

    const auto result =
      wrapped_result.result;

    if (!result)
    {
      return {
        {"success", false},
        {"error", "Joint action returned no result"}
      };
    }

    const bool succeeded =
      wrapped_result.code
        == rclcpp_action::ResultCode::SUCCEEDED
      && result->success;

    return {
      {"success", succeeded},
      {
        "finish_code",
        result->finish_code
      },
      {
        "action_result_code",
        static_cast<int>(
          wrapped_result.code)
      }
    };
  }

  json send_joint_goal(
    const std::map<
      std::string,
      std::vector<double>
    > &commands,
    double minimum_time,
    int priority)
  {
    std::string validation_error;

    if (
      !validate_joint_commands(
        commands,
        validation_error))
    {
      return {
        {"success", false},
        {"error", validation_error}
      };
    }

    minimum_time =
      std::clamp(
        minimum_time,
        2.0,
        15.0);

    bool stream_was_on = false;

    {
      std::lock_guard<std::mutex> lock(
        state_mutex_);

      if (!robot_is_connected_unlocked())
      {
        return {
          {"success", false},
          {"error", "Robot state is not connected"}
        };
      }

      if (!robot_is_ready_unlocked())
      {
        return {
          {"success", false},
          {"error", "Robot is not ENABLE or EXECUTING"}
        };
      }

      if (joint_action_busy_)
      {
        return {
          {"success", false},
          {"error", "Another joint action is running"}
        };
      }

      if (collision_)
      {
        return {
          {"success", false},
          {"error", "Robot collision is active"}
        };
      }

      if (emergency_stop_)
      {
        return {
          {"success", false},
          {"error", "Emergency stop is active"}
        };
      }

      joint_action_busy_ = true;
      joint_action_state_ = "preparing";
      stream_was_on = stream_enabled_;
    }

    stop_robot();

    json result;

    try
    {
      bool may_execute = true;

      if (stream_was_on)
      {
        {
          std::lock_guard<std::mutex> lock(
            state_mutex_);

          joint_action_state_ =
            "closing_stream";
        }

        const json stream_off_result =
          set_stream_state(false);

        if (
          !stream_off_result.value(
            "success",
            false))
        {
          result = {
            {"success", false},
            {"error", "Could not turn stream OFF"},
            {"stream_result", stream_off_result}
          };

          may_execute = false;
        }
      }

      if (may_execute)
      {
        result =
          execute_joint_goal(
            commands,
            minimum_time,
            priority);
      }
    }
    catch (
      const std::exception &exception)
    {
      result = {
        {"success", false},
        {"error", exception.what()}
      };
    }

    json restore_result = {
      {"success", true},
      {
        "message",
        "Stream restore was not required"
      }
    };

    if (stream_was_on)
    {
      {
        std::lock_guard<std::mutex> lock(
          state_mutex_);

        joint_action_state_ =
          "restoring_stream";
      }

      restore_result = set_stream_state(true);
    }

    {
      std::lock_guard<std::mutex> lock(
        state_mutex_);

      active_joint_goal_.reset();
      joint_action_busy_ = false;

      joint_action_state_ =
        result.value(
          "success",
          false)
          ? result.value(
              "finish_code",
              "completed")
          : "failed";
    }

    result["stream_restore"] =
      restore_result;

    return result;
  }

  json call_cancel_service()
  {
    stop_robot();

    JointGoalHandle::SharedPtr goal_handle;

    {
      std::lock_guard<std::mutex> lock(
        state_mutex_);

      goal_handle =
        active_joint_goal_;
    }

    if (goal_handle)
    {
      auto cancel_future =
        joint_action_client_
          ->async_cancel_goal(
            goal_handle);

      cancel_future.wait_for(2s);
    }

    if (
      !cancel_client_
        ->wait_for_service(3s))
    {
      return {
        {"success", false},
        {
          "error",
          "Service not available: /rby1/cancel_control"
        }
      };
    }

    auto request =
      std::make_shared<
        std_srvs::srv::Trigger::Request
      >();

    auto future =
      cancel_client_
        ->async_send_request(
          request);

    if (
      future.wait_for(10s)
      != std::future_status::ready)
    {
      return {
        {"success", false},
        {
          "error",
          "Service timeout: /rby1/cancel_control"
        }
      };
    }

    const auto response =
      future.get();

    RCLCPP_INFO(
      get_logger(),
      "Cancel completed: success=%s",
      response->success ? "true" : "false");
    log_component_state("State after Cancel");

    return {
      {"success", response->success},
      {"message", response->message},
      {"service", "/rby1/cancel_control"}
    };
  }

  json create_status_response()
  {
    check_connection_timeout();

    const bool driver_available =
      power_client_->service_is_ready()
      && servo_client_->service_is_ready()
      && stream_client_->service_is_ready()
      && cancel_client_->service_is_ready();

    bool connected = false;
    bool driver_ready = false;
    bool preparing = false;
    bool joint_action_busy = false;
    bool velocity_command_active = false;
    bool stream_enabled = false;
    bool collision = false;
    bool emergency_stop = false;
    int control_manager_state = 0;
    double robot_version = 0.0;
    double desired_linear_x = 0.0;
    double desired_linear_y = 0.0;
    double desired_angular_z = 0.0;
    std::string joint_action_state;
    std::string fault_message;
    rby1_app_bridge::ComponentStateSnapshot components;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      connected = robot_is_connected_unlocked();
      driver_ready = robot_is_ready_unlocked();
      preparing = preparing_;
      joint_action_busy = joint_action_busy_;
      velocity_command_active = velocity_command_active_;
      stream_enabled = stream_enabled_;
      collision = collision_;
      emergency_stop = emergency_stop_;
      control_manager_state = control_manager_state_;
      robot_version = robot_version_;
      desired_linear_x = desired_linear_x_;
      desired_linear_y = desired_linear_y_;
      desired_angular_z = desired_angular_z_;
      joint_action_state = joint_action_state_;
      fault_message = control_manager_fault_message_unlocked();
      components = component_state_.snapshot();
    }

    const bool ready =
      driver_available
      && connected
      && driver_ready
      && components.all_enabled();

    const bool driving =
      velocity_command_active
      && (
        std::abs(desired_linear_x) > 1e-6
        || std::abs(desired_linear_y) > 1e-6
        || std::abs(desired_angular_z) > 1e-6);

    const bool fault =
      control_manager_state
        == rby1_msgs::msg::RobotState::STATE_MAJOR_FAULT
      || control_manager_state
        == rby1_msgs::msg::RobotState::STATE_MINOR_FAULT
      || collision
      || emergency_stop;

    const auto description =
      rby1_app_bridge::describe_status({
        driver_available,
        connected,
        preparing,
        fault,
        joint_action_busy,
        driving,
        ready,
        fault_message
      });

    return {
      {"success", true},
      {"state", description.state},
      {"connected", connected},
      {"ready", ready},
      {
        "power",
        rby1_app_bridge::legacy_component_enabled(components.power)
      },
      {
        "servo",
        rby1_app_bridge::legacy_component_enabled(components.servo)
      },
      {
        "stream",
        rby1_app_bridge::legacy_component_enabled(components.stream)
      },
      {
        "components",
        rby1_app_bridge::components_status_json(components)
      },
      {"message", description.message},
      {
        "control_manager_state",
        control_manager_state
      },
      {"stream_enabled", stream_enabled},
      {"collision", collision},
      {"emergency_stop", emergency_stop},
      {"robot_version", robot_version},
      {"ready_pose_saved", ready_pose_.saved()},
      {
        "velocity",
        {
          {"linear_x", desired_linear_x},
          {"linear_y", desired_linear_y},
          {"angular_z", desired_angular_z},
          {"active", velocity_command_active}
        }
      },
      {
        "joint_action",
        {
          {"busy", joint_action_busy},
          {"state", joint_action_state}
        }
      }
    };
  }

  json process_request(
    const json &request)
  {
    std::lock_guard<std::mutex> command_lock(
      command_mutex_);

    const std::string command =
      request.value(
        "command",
        "");

    if (command == "ping")
    {
      return {
        {"success", true},
        {"message", "pong"},
        {"bridge", "rby1_app_bridge"}
      };
    }

    if (command == "status")
    {
      return create_status_response();
    }

    if (command == "joints_status")
    {
      return create_joint_status_response();
    }

    if (command == "velocity")
    {
      return set_velocity_command(
        request.value(
          "linear_x",
          0.0),
        request.value(
          "linear_y",
          0.0),
        request.value(
          "angular_z",
          0.0));
    }

    if (command == "stop")
    {
      stop_robot();

      return {
        {"success", true},
        {"message", "Robot base stopped"}
      };
    }

    if (command == "power")
    {
      const bool enabled =
        request.value(
          "enabled",
          false);

      if (!enabled)
      {
        return safely_disable_power();
      }

      return set_power_state(enabled);
    }

    if (command == "servo")
    {
      const bool enabled =
        request.value(
          "enabled",
          false);

      if (
        !rby1_app_bridge::can_enable_power_dependent(
          component_state_.snapshot(),
          enabled))
      {
        return {
          {"success", false},
          {"message", "Observed Power state must be known and ON first"}
        };
      }

      if (!enabled)
      {
        stop_robot();

        const auto components =
          component_state_.snapshot();

        if (components.stream.enabled())
        {
          RCLCPP_INFO(
            get_logger(),
            "Servo OFF requested while Stream is ON; "
            "turning Stream OFF first");

          const json stream_result =
            set_stream_state(false);

          if (!stream_result.value("success", false))
          {
            return {
              {"success", false},
              {
                "message",
                "Servo OFF aborted because Stream OFF failed"
              },
              {"stream_result", stream_result}
            };
          }
        }
      }

      return set_servo_state(enabled);
    }

    if (command == "stream")
    {
      const bool enabled =
        request.value(
          "enabled",
          false);

      if (
        !rby1_app_bridge::can_enable_power_dependent(
          component_state_.snapshot(),
          enabled))
      {
        return {
          {"success", false},
          {"message", "Observed Power state must be known and ON first"}
        };
      }

      if (!enabled)
      {
        stop_robot();
      }

      return set_stream_state(enabled);
    }

    if (command == "prepare")
    {
      return prepare_robot();
    }

    if (command == "cancel")
    {
      return call_cancel_service();
    }

    if (command == "joint_nudge")
    {
      const std::string group =
        request.value(
          "group",
          "");

      const int index =
        request.value(
          "joint_index",
          -1);

      const double delta =
        request.value(
          "delta",
          0.0);

      const double minimum_time =
        request.value(
          "minimum_time",
          4.0);

      const std::size_t expected_count =
        expected_joint_count(group);

      if (expected_count == 0)
      {
        return {
          {"success", false},
          {"error", "Invalid joint group"}
        };
      }

      if (
        index < 0
        || static_cast<std::size_t>(
          index)
          >= expected_count)
      {
        return {
          {"success", false},
          {"error", "Invalid joint index"}
        };
      }

      if (
        !std::isfinite(delta)
        || std::abs(delta) > 0.20)
      {
        return {
          {"success", false},
          {
            "error",
            "Delta must be between -0.20 and 0.20 rad"
          }
        };
      }

      auto positions =
        get_group_positions(group);

      if (
        positions.size()
        != expected_count)
      {
        return {
          {"success", false},
          {
            "error",
            "Joint state is not ready for "
            + group
          }
        };
      }

      positions[
        static_cast<std::size_t>(
          index)]
        += delta;

      const json result =
        send_joint_goal(
          {
            {
              group,
              positions
            }
          },
          minimum_time,
          10);

      json response = result;

      response["group"] = group;
      response["joint_index"] = index;
      response["delta"] = delta;
      response["target"] = positions;

      return response;
    }

    if (command == "set_ready_pose")
    {
      return save_ready_pose();
    }

    if (command == "clear_ready_pose")
    {
      return clear_ready_pose();
    }

    if (command == "ready_pose")
    {
      return restore_ready_pose(
        request.value(
          "minimum_time",
          5.0));
    }

    if (command == "arms_ready")
    {
      return send_joint_goal(
        {
          {
            "right_arm",
            {0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0}
          },
          {
            "left_arm",
            {0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0}
          }
        },
        request.value(
          "minimum_time",
          5.0),
        10);
    }

    if (command == "zero_pose")
    {
      return send_joint_goal(
        {
          {
            "torso",
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
          },
          {
            "right_arm",
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
          },
          {
            "left_arm",
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
          },
          {
            "head",
            {0.0, 0.0}
          }
        },
        request.value(
          "minimum_time",
          5.0),
        10);
    }

    return {
      {"success", false},
      {
        "error",
        "Unknown command: "
        + command
      }
    };
  }

  void handle_client(
    tcp::socket socket)
  {
    RCLCPP_INFO(
      get_logger(),
      "Desktop application connected");

    try
    {
      socket.set_option(
        tcp::no_delay(true));

      boost::asio::streambuf buffer;

      while (!stop_requested_.load())
      {
        boost::system::error_code error;

        boost::asio::read_until(
          socket,
          buffer,
          '\n',
          error);

        if (error)
        {
          break;
        }

        std::istream input(&buffer);
        std::string request_text;

        std::getline(
          input,
          request_text);

        if (request_text.empty())
        {
          continue;
        }

        json response;

        try
        {
          const json request =
            rby1_app_bridge::parse_ndjson_request(
              request_text);

          response =
            process_request(
              request);
        }
        catch (
          const std::exception &exception)
        {
          response = {
            {"success", false},
            {"error", exception.what()}
          };
        }

        const std::string response_text =
          rby1_app_bridge::encode_ndjson_response(
            response);

        boost::asio::write(
          socket,
          boost::asio::buffer(
            response_text),
          error);

        if (error)
        {
          break;
        }
      }
    }
    catch (
      const std::exception &exception)
    {
      RCLCPP_WARN(
        get_logger(),
        "TCP client error: %s",
        exception.what());
    }

    stop_robot();

    RCLCPP_INFO(
      get_logger(),
      "Desktop application disconnected");
  }

  void tcp_server_loop()
  {
    while (!stop_requested_.load())
    {
      try
      {
        tcp::socket socket(
          io_context_);

        acceptor_.accept(
          socket);

        handle_client(
          std::move(socket));
      }
      catch (
        const std::exception &exception)
      {
        if (!stop_requested_.load())
        {
          RCLCPP_ERROR(
            get_logger(),
            "TCP server error: %s",
            exception.what());
        }
      }
    }
  }

  rclcpp::Publisher<
    geometry_msgs::msg::Twist
  >::SharedPtr cmd_vel_publisher_;

  rclcpp::Subscription<
    rby1_msgs::msg::RobotState
  >::SharedPtr robot_state_subscriber_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState
  >::SharedPtr all_joint_state_subscriber_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState
  >::SharedPtr torso_joint_state_subscriber_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState
  >::SharedPtr right_arm_joint_state_subscriber_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState
  >::SharedPtr left_arm_joint_state_subscriber_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState
  >::SharedPtr head_joint_state_subscriber_;

  rclcpp::Client<
    rby1_msgs::srv::StateOnOff
  >::SharedPtr power_client_;

  rclcpp::Client<
    rby1_msgs::srv::StateOnOff
  >::SharedPtr servo_client_;

  rclcpp::Client<
    rby1_msgs::srv::StateOnOff
  >::SharedPtr stream_client_;

  rclcpp::Client<
    std_srvs::srv::Trigger
  >::SharedPtr cancel_client_;

  rclcpp_action::Client<
    JointAction
  >::SharedPtr joint_action_client_;

  rclcpp::TimerBase::SharedPtr
    velocity_timer_;

  std::mutex state_mutex_;
  std::mutex command_mutex_;

  rby1_app_bridge::ComponentState
    component_state_;

  rby1_app_bridge::ReadyPoseState
    ready_pose_;

  bool received_robot_state_{false};
  bool connection_loss_reported_{false};
  int control_manager_state_{0};

  bool stream_enabled_{false};
  bool collision_{false};
  bool emergency_stop_{false};

  double robot_version_{0.0};

  std::chrono::steady_clock::time_point
    last_robot_state_time_{};

  std::map<
    std::string,
    JointGroupState
  > joint_groups_;

  bool joint_action_busy_{false};

  std::string joint_action_state_{
    "idle"
  };

  JointGoalHandle::SharedPtr
    active_joint_goal_;

  double desired_linear_x_{0.0};
  double desired_linear_y_{0.0};
  double desired_angular_z_{0.0};

  bool velocity_command_active_{false};

  bool preparing_{false};

  std::chrono::steady_clock::time_point
    last_velocity_command_time_{};

  boost::asio::io_context io_context_;
  tcp::acceptor acceptor_;

  std::atomic<bool>
    stop_requested_{false};

  std::thread tcp_thread_;
};

int main(
  int argc,
  char *argv[])
{
  rclcpp::init(
    argc,
    argv);

  auto node =
    std::make_shared<
      RBY1AppBridge
    >();

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),
    4);

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
