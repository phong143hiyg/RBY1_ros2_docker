#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
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
    std::lock_guard<std::mutex> lock(state_mutex_);

    const bool connection_was_stale =
      received_robot_state_
      && !robot_is_connected_unlocked();

    if (connection_was_stale)
    {
      component_state_.reset();
    }

    received_robot_state_ = true;

    control_manager_state_ =
      static_cast<int>(
        message->control_manager_state);

    stream_enabled_ =
      static_cast<bool>(
        message->robot_stream_state);

    component_state_.confirm_stream(
      stream_enabled_);

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
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;

    bool command_is_valid = false;

    {
      std::lock_guard<std::mutex> lock(
        state_mutex_);

      const bool connected =
        robot_is_connected_unlocked();

      if (!connected)
      {
        component_state_.reset();
      }

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
    std::chrono::seconds timeout = 10s)
  {
    if (!client->wait_for_service(3s))
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
      future.wait_for(timeout)
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
      state.power ? "true" : "false",
      state.servo ? "true" : "false",
      state.stream ? "true" : "false");
  }

  json set_power_state(bool enabled)
  {
    const json result = call_state_service(
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

    if (success)
    {
      component_state_.confirm_power(enabled);
    }

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
      !state.power
      && !state.servo
      && !state.stream;

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
      {"power", state.power},
      {"servo", state.servo},
      {"stream", state.stream},
      {"steps", steps}
    };
  }

  json set_servo_state(bool enabled)
  {
    const json result = call_state_service(
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

    if (success)
    {
      component_state_.confirm_servo(enabled);
    }

    log_component_state("State after Servo");
    return result;
  }

  json set_stream_state(bool enabled)
  {
    const json result = call_state_service(
      stream_client_,
      "/rby1/stream_control",
      enabled,
      "",
      enabled ? 20.0 : 0.0);

    const bool service_succeeded =
      result.value("success", false);

    bool confirmed = false;
    if (service_succeeded)
    {
      confirmed = wait_for_stream(enabled, 5s);
    }

    RCLCPP_INFO(
      get_logger(),
      "Stream %s: service=%s confirmed=%s",
      enabled ? "ON" : "OFF",
      service_succeeded ? "success" : "failed",
      confirmed ? "true" : "false");

    if (!service_succeeded)
    {
      log_component_state("State after Stream");
      return result;
    }

    if (!confirmed)
    {
      log_component_state("State after Stream");
      return {
        {"success", false},
        {
          "error",
          std::string("Stream state did not become ")
            + (enabled ? "ON" : "OFF")
        },
        {"result", result}
      };
    }

    component_state_.confirm_stream(enabled);
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

  bool wait_for_stream(
    bool expected,
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
          && stream_enabled_ == expected)
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
        initial_components.power ? "true" : "false",
        initial_components.servo ? "true" : "false",
        initial_components.stream ? "true" : "false");

      return {
        {"success", false},
        {"ready", false},
        {"power", initial_components.power},
        {"servo", initial_components.servo},
        {"stream", initial_components.stream},
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
    std::lock_guard<std::mutex> lock(
      state_mutex_);

    const bool connected =
      robot_is_connected_unlocked();

    if (!connected)
    {
      component_state_.reset();
    }

    const auto components =
      component_state_.snapshot();

    const bool driver_available =
      power_client_->service_is_ready()
      && servo_client_->service_is_ready()
      && stream_client_->service_is_ready()
      && cancel_client_->service_is_ready();

    const bool ready =
      driver_available
      && component_state_.ready(
        connected,
        robot_is_ready_unlocked());

    const bool driving =
      velocity_command_active_
      && (
        std::abs(desired_linear_x_) > 1e-6
        || std::abs(desired_linear_y_) > 1e-6
        || std::abs(desired_angular_z_) > 1e-6);

    const bool fault =
      control_manager_has_fault_unlocked()
      || collision_
      || emergency_stop_;

    const auto description =
      rby1_app_bridge::describe_status({
        driver_available,
        connected,
        preparing_,
        fault,
        joint_action_busy_,
        driving,
        ready,
        control_manager_fault_message_unlocked()
      });

    return {
      {"success", true},
      {"state", description.state},
      {"connected", connected},
      {"ready", ready},
      {"power", components.power},
      {"servo", components.servo},
      {"stream", components.stream},
      {"message", description.message},
      {
        "control_manager_state",
        control_manager_state_
      },
      {"stream_enabled", stream_enabled_},
      {"collision", collision_},
      {"emergency_stop", emergency_stop_},
      {"robot_version", robot_version_},
      {"ready_pose_saved", ready_pose_.saved()},
      {
        "velocity",
        {
          {"linear_x", desired_linear_x_},
          {"linear_y", desired_linear_y_},
          {"angular_z", desired_angular_z_},
          {"active", velocity_command_active_}
        }
      },
      {
        "joint_action",
        {
          {"busy", joint_action_busy_},
          {"state", joint_action_state_}
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
          {"message", "Power must be enabled first"}
        };
      }

      if (!enabled)
      {
        stop_robot();

        const auto components =
          component_state_.snapshot();

        if (components.stream)
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
          {"message", "Power must be enabled first"}
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
            json::parse(
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
          response.dump()
          + "\n";

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
