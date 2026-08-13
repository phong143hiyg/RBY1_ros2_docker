#!/usr/bin/env python3
"""
RBY1 Local Web Controller
=========================

Cầu nối giữa giao diện web local và ROS 2 Driver của RB-Y1.

Các chức năng:
- Đọc trạng thái robot qua:
  /rby1/robot_state
  /rby1/odom
  /rby1/joint_states
  /rby1/joint_states/torso
  /rby1/joint_states/right_arm
  /rby1/joint_states/left_arm
  /rby1/joint_states/head

- Điều khiển đế qua:
  /rby1/cmd_vel

- Gọi các service:
  /rby1/robot_power
  /rby1/robot_servo
  /rby1/stream_control
  /rby1/cancel_control

- Điều khiển thân, tay và đầu qua action:
  /rby1/robot_joint

Chạy:
    python3 server.py

Mở:
    http://localhost:8000
"""

from __future__ import annotations

import asyncio
import math
import threading
import time
from pathlib import Path
from typing import Any, Literal, Optional

import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pydantic import BaseModel, Field
from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.msg import JointCommand, RobotState
from rby1_msgs.srv import StateOnOff
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


# ============================================================
# Đường dẫn và cấu hình
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"

WEB_HOST = "0.0.0.0"
WEB_PORT = 8000

# Giới hạn vận tốc bảo thủ khi thử nghiệm.
MAX_LINEAR_SPEED = 0.20       # m/s
MAX_ANGULAR_SPEED = 0.50      # rad/s

# Nếu web không gửi lệnh vận tốc mới trong khoảng này,
# backend tự động gửi vận tốc 0.
COMMAND_TIMEOUT_SECONDS = 0.35

# Tần số publish cmd_vel.
VELOCITY_PUBLISH_PERIOD = 0.05  # 20 Hz

# Robot được coi là mất kết nối nếu quá thời gian này
# không nhận được /rby1/robot_state.
ROBOT_STATE_TIMEOUT_SECONDS = 1.5

# Số khớp theo từng nhóm của RB-Y1 model M.
JOINT_COUNTS: dict[str, int] = {
    "torso": 6,
    "right_arm": 7,
    "left_arm": 7,
    "head": 2,
}

# Tư thế thử nghiệm an toàn, dựa trên ví dụ joint command chính thức.
READY_POSE: dict[str, list[float]] = {
    "torso": [0.0] * 6,
    "right_arm": [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0],
    "left_arm": [0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0],
    "head": [0.0, 0.0],
}

JointGroupName = Literal[
    "torso",
    "right_arm",
    "left_arm",
    "head",
]


# ============================================================
# Pydantic request models
# ============================================================

class VelocityRequest(BaseModel):
    """Lệnh vận tốc cho đế robot."""

    linear_x: float = Field(
        default=0.0,
        ge=-MAX_LINEAR_SPEED,
        le=MAX_LINEAR_SPEED,
    )

    linear_y: float = Field(
        default=0.0,
        ge=-MAX_LINEAR_SPEED,
        le=MAX_LINEAR_SPEED,
    )

    angular_z: float = Field(
        default=0.0,
        ge=-MAX_ANGULAR_SPEED,
        le=MAX_ANGULAR_SPEED,
    )


class SwitchRequest(BaseModel):
    """Request bật hoặc tắt một chức năng."""

    enabled: bool


class JointNudgeRequest(BaseModel):
    """Tăng hoặc giảm một khớp từ vị trí hiện tại."""

    group: JointGroupName

    joint_index: int = Field(
        ge=0,
        le=6,
    )

    # Mỗi lần chỉ thay đổi tối đa 0.2 rad.
    delta: float = Field(
        ge=-0.20,
        le=0.20,
    )

    minimum_time: float = Field(
        default=3.0,
        ge=0.50,
        le=10.0,
    )


class JointPoseRequest(BaseModel):
    """
    Gửi trực tiếp vị trí đích cho một hoặc nhiều nhóm khớp.

    Trường nào để null thì nhóm đó không được điều khiển.
    """

    torso: Optional[list[float]] = None
    right_arm: Optional[list[float]] = None
    left_arm: Optional[list[float]] = None
    head: Optional[list[float]] = None

    minimum_time: float = Field(
        default=2.0,
        ge=0.50,
        le=15.0,
    )

    priority: int = Field(
        default=10,
        ge=1,
        le=100,
    )


# ============================================================
# ROS 2 Web Bridge
# ============================================================

class RBY1WebBridge(Node):
    """
    ROS 2 node chuyển đổi request từ web thành topic, service
    và action dành cho rby1_ros2_driver.
    """

    def __init__(self) -> None:
        # Dùng namespace rby1 để các tên tương đối trở thành:
        # /rby1/cmd_vel, /rby1/robot_state, ...
        super().__init__(
            "rby1_web_bridge",
            namespace="rby1",
        )

        self._lock = threading.RLock()

        # ----------------------------------------------------
        # Publisher điều khiển đế
        # ----------------------------------------------------
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "cmd_vel",
            10,
        )

        # ----------------------------------------------------
        # Service clients
        # ----------------------------------------------------
        self.power_client = self.create_client(
            StateOnOff,
            "robot_power",
        )

        self.servo_client = self.create_client(
            StateOnOff,
            "robot_servo",
        )

        self.stream_client = self.create_client(
            StateOnOff,
            "stream_control",
        )

        self.cancel_client = self.create_client(
            Trigger,
            "cancel_control",
        )

        # ----------------------------------------------------
        # Action client điều khiển khớp
        # ----------------------------------------------------
        self.joint_action_client = ActionClient(
            self,
            Rby1JointCommand,
            "robot_joint",
        )

        self._active_joint_goal_handle: Any = None
        self._joint_action_busy = False
        self._joint_action_state = "idle"

        # ----------------------------------------------------
        # Subscribers nhận trạng thái từ robot
        # ----------------------------------------------------
        self.create_subscription(
            RobotState,
            "robot_state",
            self._robot_state_callback,
            10,
        )

        self.create_subscription(
            Odometry,
            "odom",
            self._odom_callback,
            10,
        )

        self.create_subscription(
            JointState,
            "joint_states",
            self._joint_state_callback,
            10,
        )

        self.create_subscription(
            JointState,
            "joint_states/torso",
            lambda msg: self._group_joint_state_callback(
                "torso",
                msg,
            ),
            10,
        )

        self.create_subscription(
            JointState,
            "joint_states/right_arm",
            lambda msg: self._group_joint_state_callback(
                "right_arm",
                msg,
            ),
            10,
        )

        self.create_subscription(
            JointState,
            "joint_states/left_arm",
            lambda msg: self._group_joint_state_callback(
                "left_arm",
                msg,
            ),
            10,
        )

        self.create_subscription(
            JointState,
            "joint_states/head",
            lambda msg: self._group_joint_state_callback(
                "head",
                msg,
            ),
            10,
        )

        # ----------------------------------------------------
        # Trạng thái robot
        # ----------------------------------------------------
        self._control_manager_state = RobotState.STATE_NONE
        self._robot_stream_state = False
        self._collision = False
        self._emo_state = False
        self._robot_version = 0.0
        self._tool_flange_state: list[bool] = []
        self._center_of_mass = [0.0, 0.0, 0.0]

        self._last_robot_state_time = 0.0

        # ----------------------------------------------------
        # Odometry
        # ----------------------------------------------------
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

        self._odom_linear_x = 0.0
        self._odom_linear_y = 0.0
        self._odom_angular_z = 0.0

        # ----------------------------------------------------
        # Joint state
        # ----------------------------------------------------
        self._joint_names: list[str] = []
        self._joint_positions: list[float] = []
        self._joint_velocities: list[float] = []
        self._joint_efforts: list[float] = []

        self._joint_groups: dict[str, dict[str, list[Any]]] = {
            group: {
                "names": [],
                "positions": [],
                "velocities": [],
                "efforts": [],
            }
            for group in JOINT_COUNTS
        }

        # ----------------------------------------------------
        # Lệnh vận tốc và dead-man watchdog
        # ----------------------------------------------------
        self._desired_velocity = (0.0, 0.0, 0.0)
        self._last_velocity_request_time = 0.0

        self.create_timer(
            VELOCITY_PUBLISH_PERIOD,
            self._velocity_watchdog,
        )

        self.get_logger().info(
            "RBY1 web bridge đã khởi động."
        )

    # ========================================================
    # ROS subscriber callbacks
    # ========================================================

    def _robot_state_callback(
        self,
        message: RobotState,
    ) -> None:
        with self._lock:
            self._control_manager_state = int(
                message.control_manager_state
            )

            self._robot_stream_state = bool(
                message.robot_stream_state
            )

            self._collision = bool(message.collision)
            self._emo_state = bool(message.emo_state)
            self._robot_version = float(
                message.robot_version
            )

            self._tool_flange_state = list(
                message.tool_flange_state
            )

            self._center_of_mass = [
                float(value)
                for value in message.center_of_mass
            ]

            self._last_robot_state_time = (
                time.monotonic()
            )

    def _odom_callback(
        self,
        message: Odometry,
    ) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

        # Quaternion -> yaw.
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )

        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )

        yaw = math.atan2(
            sin_yaw,
            cos_yaw,
        )

        with self._lock:
            self._odom_x = float(position.x)
            self._odom_y = float(position.y)
            self._odom_yaw = float(yaw)

            self._odom_linear_x = float(
                message.twist.twist.linear.x
            )

            self._odom_linear_y = float(
                message.twist.twist.linear.y
            )

            self._odom_angular_z = float(
                message.twist.twist.angular.z
            )

    def _joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        """
        Nhận topic tổng /rby1/joint_states và tự tách thành
        torso, right_arm, left_arm và head.

        Đây là nguồn chính để giao diện hiển thị nút điều khiển
        cánh tay. Các topic joint_states/<group> chỉ là nguồn bổ sung.
        """

        names = list(message.name)
        positions = [
            float(value)
            for value in message.position
        ]
        velocities = [
            float(value)
            for value in message.velocity
        ]
        efforts = [
            float(value)
            for value in message.effort
        ]

        def classify_joint(name: str) -> Optional[str]:
            normalized = (
                name.lower()
                .replace("/", "_")
                .replace("-", "_")
            )

            if "right_arm" in normalized:
                return "right_arm"
            if "left_arm" in normalized:
                return "left_arm"
            if "torso" in normalized:
                return "torso"
            if "head" in normalized:
                return "head"

            return None

        def joint_number(name: str) -> int:
            normalized = (
                name.replace("/", "_")
                .replace("-", "_")
            )

            for token in reversed(
                normalized.split("_")
            ):
                if token.isdigit():
                    return int(token)

            return 10_000

        grouped_indices: dict[str, list[int]] = {
            group: []
            for group in JOINT_COUNTS
        }

        for index, name in enumerate(names):
            group = classify_joint(name)

            if group is not None:
                grouped_indices[group].append(index)

        for group, indices in grouped_indices.items():
            indices.sort(
                key=lambda index: joint_number(
                    names[index]
                )
            )

        def select_values(
            values: list[float],
            indices: list[int],
        ) -> list[float]:
            return [
                values[index]
                for index in indices
                if index < len(values)
            ]

        with self._lock:
            self._joint_names = names
            self._joint_positions = positions
            self._joint_velocities = velocities
            self._joint_efforts = efforts

            for group, indices in grouped_indices.items():
                expected_count = JOINT_COUNTS[group]

                # Chỉ sử dụng nhóm khi đã tìm đủ số khớp.
                # Nhờ đó tránh gửi sai lệnh nếu tên khớp không khớp.
                if len(indices) != expected_count:
                    continue

                self._joint_groups[group] = {
                    "names": [
                        names[index]
                        for index in indices
                    ],
                    "positions": select_values(
                        positions,
                        indices,
                    ),
                    "velocities": select_values(
                        velocities,
                        indices,
                    ),
                    "efforts": select_values(
                        efforts,
                        indices,
                    ),
                }

    def _group_joint_state_callback(
        self,
        group: str,
        message: JointState,
    ) -> None:
        """Nhận topic trạng thái riêng của từng nhóm, nếu có."""

        positions = [
            float(value)
            for value in message.position
        ]

        if len(positions) != JOINT_COUNTS[group]:
            return

        names = list(message.name)

        if len(names) != JOINT_COUNTS[group]:
            names = [
                f"{group}_{index}"
                for index in range(
                    JOINT_COUNTS[group]
                )
            ]

        with self._lock:
            self._joint_groups[group] = {
                "names": names,
                "positions": positions,
                "velocities": [
                    float(value)
                    for value in message.velocity
                ],
                "efforts": [
                    float(value)
                    for value in message.effort
                ],
            }

    # ========================================================
    # Trạng thái chung
    # ========================================================

    def robot_is_connected(self) -> bool:
        with self._lock:
            if self._last_robot_state_time <= 0.0:
                return False

            elapsed = (
                time.monotonic()
                - self._last_robot_state_time
            )

            return (
                elapsed
                < ROBOT_STATE_TIMEOUT_SECONDS
            )

    def robot_is_ready(self) -> bool:
        with self._lock:
            return self._control_manager_state in (
                RobotState.STATE_ENABLE,
                RobotState.STATE_EXECUTING,
            )

    def get_group_positions(
        self,
        group: str,
    ) -> list[float]:
        with self._lock:
            return list(
                self._joint_groups[group][
                    "positions"
                ]
            )

    def get_status(self) -> dict[str, Any]:
        with self._lock:
            connected = False

            if self._last_robot_state_time > 0.0:
                connected = (
                    time.monotonic()
                    - self._last_robot_state_time
                    < ROBOT_STATE_TIMEOUT_SECONDS
                )

            joint_groups = {
                group: {
                    "names": list(data["names"]),
                    "positions": list(
                        data["positions"]
                    ),
                    "velocities": list(
                        data["velocities"]
                    ),
                    "efforts": list(
                        data["efforts"]
                    ),
                    "expected_count": JOINT_COUNTS[group],
                    "ready": (
                        len(data["positions"])
                        == JOINT_COUNTS[group]
                    ),
                }
                for group, data
                in self._joint_groups.items()
            }

            return {
                "connected": connected,

                "control_manager_state":
                    self._control_manager_state,

                "robot_stream_state":
                    self._robot_stream_state,

                "collision": self._collision,
                "emo_state": self._emo_state,
                "robot_version": self._robot_version,

                "tool_flange_state": list(
                    self._tool_flange_state
                ),

                "center_of_mass": list(
                    self._center_of_mass
                ),

                "odom": {
                    "x": self._odom_x,
                    "y": self._odom_y,
                    "yaw": self._odom_yaw,

                    "linear_x":
                        self._odom_linear_x,

                    "linear_y":
                        self._odom_linear_y,

                    "angular_z":
                        self._odom_angular_z,
                },

                "joint_count": len(
                    self._joint_names
                ),

                "joint_names": list(
                    self._joint_names
                ),

                "joint_positions": list(
                    self._joint_positions
                ),

                "joint_velocities": list(
                    self._joint_velocities
                ),

                "joint_efforts": list(
                    self._joint_efforts
                ),

                "joint_groups": joint_groups,

                "joint_action": {
                    "busy":
                        self._joint_action_busy,

                    "state":
                        self._joint_action_state,
                },
            }

    # ========================================================
    # Điều khiển đế
    # ========================================================

    def set_velocity(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
    ) -> None:
        """
        Lưu lệnh vận tốc mới.

        Timer ROS tiếp tục publish lệnh ở 20 Hz. Nếu web không
        gửi lệnh mới trong COMMAND_TIMEOUT_SECONDS, watchdog
        tự chuyển về vận tốc 0.
        """

        if not self.robot_is_connected():
            raise RuntimeError(
                "Không nhận được trạng thái từ robot."
            )

        if not self.robot_is_ready():
            raise RuntimeError(
                "Robot chưa ở trạng thái ENABLE."
            )

        with self._lock:
            if not self._robot_stream_state:
                raise RuntimeError(
                    "Stream control đang tắt."
                )

            if self._joint_action_busy:
                raise RuntimeError(
                    "Một lệnh khớp đang chạy. "
                    "Hãy chờ lệnh hoàn thành."
                )

        linear_x = max(
            -MAX_LINEAR_SPEED,
            min(MAX_LINEAR_SPEED, linear_x),
        )

        linear_y = max(
            -MAX_LINEAR_SPEED,
            min(MAX_LINEAR_SPEED, linear_y),
        )

        angular_z = max(
            -MAX_ANGULAR_SPEED,
            min(MAX_ANGULAR_SPEED, angular_z),
        )

        with self._lock:
            self._desired_velocity = (
                linear_x,
                linear_y,
                angular_z,
            )

            self._last_velocity_request_time = (
                time.monotonic()
            )

        self._publish_velocity(
            linear_x,
            linear_y,
            angular_z,
        )

    def stop_robot(self) -> None:
        """Dừng đế bằng nhiều bản tin Twist bằng 0."""

        with self._lock:
            self._desired_velocity = (
                0.0,
                0.0,
                0.0,
            )

            self._last_velocity_request_time = 0.0

        for _ in range(3):
            self._publish_velocity(
                0.0,
                0.0,
                0.0,
            )

    def _velocity_watchdog(self) -> None:
        with self._lock:
            elapsed = (
                time.monotonic()
                - self._last_velocity_request_time
            )

            if (
                self._last_velocity_request_time > 0.0
                and elapsed
                <= COMMAND_TIMEOUT_SECONDS
            ):
                command = self._desired_velocity
            else:
                command = (0.0, 0.0, 0.0)
                self._desired_velocity = command

        self._publish_velocity(*command)

    def _publish_velocity(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
    ) -> None:
        message = Twist()

        message.linear.x = float(linear_x)
        message.linear.y = float(linear_y)
        message.angular.z = float(angular_z)

        self.cmd_vel_publisher.publish(message)

    # ========================================================
    # Chờ Future của rclpy
    # ========================================================

    async def _wait_ros_future(
        self,
        future: Any,
        timeout: float,
        description: str,
    ) -> Any:
        deadline = time.monotonic() + timeout

        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"{description} bị timeout."
                )

            await asyncio.sleep(0.02)

        return future.result()

    # ========================================================
    # ROS services
    # ========================================================

    async def call_state_service(
        self,
        client: Any,
        enabled: bool,
        parameters: str = "",
        value: float = 0.0,
        timeout: float = 10.0,
    ) -> dict[str, Any]:
        service_name = getattr(
            client,
            "srv_name",
            "ROS 2 service",
        )

        ready = await asyncio.to_thread(
            client.wait_for_service,
            timeout_sec=3.0,
        )

        if not ready:
            raise RuntimeError(
                f"Không tìm thấy service "
                f"{service_name}."
            )

        request = StateOnOff.Request()
        request.state = bool(enabled)
        request.parameters = str(parameters)
        request.value = float(value)

        future = client.call_async(request)

        response = await self._wait_ros_future(
            future,
            timeout=timeout,
            description=service_name,
        )

        if response is None:
            raise RuntimeError(
                f"{service_name} không phản hồi."
            )

        return {
            "success": bool(response.success),
            "message": str(response.message),
        }

    async def cancel_all_control(
        self,
        timeout: float = 10.0,
    ) -> dict[str, Any]:
        """
        Hủy goal action hiện tại nếu có, sau đó gọi
        /rby1/cancel_control.

        Lưu ý: service cancel_control cũng đóng stream.
        """

        self.stop_robot()

        with self._lock:
            goal_handle = (
                self._active_joint_goal_handle
            )

        # Cố gắng cancel action goal trước.
        if goal_handle is not None:
            try:
                cancel_future = (
                    goal_handle.cancel_goal_async()
                )

                await self._wait_ros_future(
                    cancel_future,
                    timeout=3.0,
                    description=(
                        "Hủy action robot_joint"
                    ),
                )

            except (
                RuntimeError,
                TimeoutError,
                AttributeError,
            ):
                # Vẫn tiếp tục gọi cancel_control
                # để bảo đảm an toàn.
                pass

        ready = await asyncio.to_thread(
            self.cancel_client.wait_for_service,
            timeout_sec=3.0,
        )

        if not ready:
            raise RuntimeError(
                "Không tìm thấy service "
                "/rby1/cancel_control."
            )

        future = self.cancel_client.call_async(
            Trigger.Request()
        )

        response = await self._wait_ros_future(
            future,
            timeout=timeout,
            description="cancel_control",
        )

        if response is None:
            raise RuntimeError(
                "cancel_control không phản hồi."
            )

        with self._lock:
            self._joint_action_state = (
                "cancelled"
            )

        return {
            "success": bool(response.success),
            "message": str(response.message),
        }

    # ========================================================
    # Robot preparation
    # ========================================================

    async def wait_for_control_state(
        self,
        target_states: tuple[int, ...],
        timeout: float,
    ) -> bool:
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            with self._lock:
                state = (
                    self._control_manager_state
                )

            if state in target_states:
                return True

            await asyncio.sleep(0.10)

        return False

    async def prepare_robot(
        self,
    ) -> dict[str, Any]:
        """
        Bật nguồn, bật servo và mở stream control.

        Stream cần thiết để điều khiển đế bằng cmd_vel.
        """

        if not self.robot_is_connected():
            raise RuntimeError(
                "Không nhận được /rby1/robot_state. "
                "Hãy kiểm tra simulator và driver."
            )

        results: dict[str, Any] = {}

        with self._lock:
            current_state = (
                self._control_manager_state
            )

        if current_state not in (
            RobotState.STATE_ENABLE,
            RobotState.STATE_EXECUTING,
        ):
            results["power"] = (
                await self.call_state_service(
                    self.power_client,
                    True,
                    parameters="all",
                    value=0.0,
                )
            )

            if not results["power"]["success"]:
                raise RuntimeError(
                    results["power"]["message"]
                )

            await asyncio.sleep(1.0)

            results["servo"] = (
                await self.call_state_service(
                    self.servo_client,
                    True,
                    parameters="all",
                    value=0.0,
                )
            )

            if not results["servo"]["success"]:
                raise RuntimeError(
                    results["servo"]["message"]
                )

            ready = await self.wait_for_control_state(
                (
                    RobotState.STATE_ENABLE,
                    RobotState.STATE_EXECUTING,
                ),
                timeout=12.0,
            )

            if not ready:
                with self._lock:
                    state = (
                        self._control_manager_state
                    )

                raise RuntimeError(
                    "Robot không chuyển sang ENABLE. "
                    f"Trạng thái hiện tại: {state}."
                )

        with self._lock:
            stream_already_open = (
                self._robot_stream_state
            )

        if stream_already_open:
            results["stream"] = {
                "success": True,
                "message": (
                    "Stream control đã được bật."
                ),
            }
        else:
            results["stream"] = (
                await self.call_state_service(
                    self.stream_client,
                    True,
                    parameters="",
                    value=20.0,
                )
            )

            if not results["stream"]["success"]:
                raise RuntimeError(
                    results["stream"]["message"]
                )

        return {
            "success": True,
            "message": (
                "Robot đã sẵn sàng điều khiển."
            ),
            "results": results,
        }

    # ========================================================
    # Joint action
    # ========================================================

    def _joint_feedback_callback(
        self,
        feedback_message: Any,
    ) -> None:
        try:
            state = str(
                feedback_message.feedback.current_state
            )
        except AttributeError:
            state = "executing"

        with self._lock:
            self._joint_action_state = state

    async def send_joint_goal(
        self,
        commands: dict[str, list[float]],
        minimum_time: float = 5.0,
        priority: int = 10,
    ) -> dict[str, Any]:
        """
        Gửi lệnh điều khiển tay, thân hoặc đầu.

        Quy trình:
        1. Dừng đế.
        2. Tạm tắt persistent stream.
        3. Gửi robot_joint giống lệnh terminal.
        4. Chờ kết quả chính thức từ driver.
        5. Khôi phục stream.
        """

        if not self.robot_is_connected():
            raise RuntimeError(
                "Không nhận được trạng thái robot."
            )

        if not self.robot_is_ready():
            raise RuntimeError(
                "Robot chưa ở trạng thái ENABLE."
            )

        if not commands:
            raise ValueError(
                "Không có nhóm khớp nào được gửi."
            )

        for group, positions in commands.items():
            if group not in JOINT_COUNTS:
                raise ValueError(
                    f"Nhóm khớp không hợp lệ: {group}"
                )

            expected_count = JOINT_COUNTS[group]

            if len(positions) != expected_count:
                raise ValueError(
                    f"{group} cần {expected_count} giá trị, "
                    f"nhưng nhận {len(positions)}."
                )

            if not all(
                math.isfinite(float(value))
                for value in positions
            ):
                raise ValueError(
                    f"{group} chứa giá trị không hợp lệ."
                )

        with self._lock:
            if self._joint_action_busy:
                raise RuntimeError(
                    "Một lệnh khớp khác đang chạy."
                )

            self._joint_action_busy = True
            self._joint_action_state = "preparing"

            stream_was_on = bool(
                self._robot_stream_state
            )

        try:
            # Dừng hoàn toàn chuyển động đế.
            self.stop_robot()

            # robot_joint hoạt động ổn định hơn khi
            # persistent stream điều khiển đế đã tắt.
            if stream_was_on:
                with self._lock:
                    self._joint_action_state = (
                        "closing_stream"
                    )

                stream_result = (
                    await self.call_state_service(
                        self.stream_client,
                        False,
                        parameters="",
                        value=0.0,
                    )
                )

                if not stream_result["success"]:
                    raise RuntimeError(
                        stream_result["message"]
                        or "Không tắt được stream."
                    )

                # Chờ robot_state cập nhật Stream = OFF.
                deadline = time.monotonic() + 3.0

                while time.monotonic() < deadline:
                    with self._lock:
                        stream_is_off = not bool(
                            self._robot_stream_state
                        )

                    if stream_is_off:
                        break

                    await asyncio.sleep(0.05)

                else:
                    raise TimeoutError(
                        "Stream không chuyển sang OFF."
                    )

                await asyncio.sleep(0.20)

            server_ready = await asyncio.to_thread(
                self.joint_action_client.wait_for_server,
                timeout_sec=3.0,
            )

            if not server_ready:
                raise RuntimeError(
                    "Không tìm thấy action "
                    "/rby1/robot_joint."
                )

            goal = Rby1JointCommand.Goal()
            goal.priority = int(priority)

            for group, positions in commands.items():
                joint_command = JointCommand()

                joint_command.position = [
                    float(value)
                    for value in positions
                ]

                joint_command.minimum_time = float(
                    minimum_time
                )

                # Không tự ép:
                # velocity_limit
                # acceleration_limit
                # control_hold_time
                #
                # Giữ nguyên giá trị mặc định từ message.

                setattr(
                    goal,
                    group,
                    joint_command,
                )

            with self._lock:
                self._joint_action_state = "sending"

            send_future = (
                self.joint_action_client.send_goal_async(
                    goal,
                    feedback_callback=(
                        self._joint_feedback_callback
                    ),
                )
            )

            goal_handle = await self._wait_ros_future(
                send_future,
                timeout=5.0,
                description="Gửi robot_joint goal",
            )

            if goal_handle is None:
                raise RuntimeError(
                    "Không nhận được goal handle."
                )

            if not goal_handle.accepted:
                raise RuntimeError(
                    "Driver từ chối lệnh khớp."
                )

            with self._lock:
                self._active_joint_goal_handle = (
                    goal_handle
                )

                self._joint_action_state = "executing"

            result_future = (
                goal_handle.get_result_async()
            )

            wrapped_result = await self._wait_ros_future(
                result_future,
                timeout=max(
                    minimum_time + 15.0,
                    20.0,
                ),
                description=(
                    "Chờ robot_joint hoàn thành"
                ),
            )

            if wrapped_result is None:
                raise RuntimeError(
                    "Action không trả kết quả."
                )

            result = wrapped_result.result

            if result is None:
                raise RuntimeError(
                    "Action result rỗng."
                )

            finish_code = (
                str(result.finish_code)
                if result.finish_code
                else "kOk"
            )

            if not result.success:
                raise RuntimeError(
                    "Driver báo lệnh khớp thất bại: "
                    f"{finish_code}"
                )

            # Không gọi _wait_for_joint_targets().
            with self._lock:
                self._joint_action_state = finish_code

            return {
                "success": True,
                "finish_code": finish_code,
                "target_reached": True,
            }

        finally:
            with self._lock:
                self._active_joint_goal_handle = None

            # Bật lại stream để có thể tiếp tục
            # điều khiển đế trên web.
            if stream_was_on:
                try:
                    with self._lock:
                        self._joint_action_state = (
                            "restoring_stream"
                        )

                    stream_result = (
                        await self.call_state_service(
                            self.stream_client,
                            True,
                            parameters="",
                            value=20.0,
                        )
                    )

                    if not stream_result["success"]:
                        self.get_logger().error(
                            "Không bật lại được stream: "
                            + stream_result["message"]
                        )

                except Exception as error:
                    self.get_logger().error(
                        "Không bật lại được stream: "
                        f"{error}"
                    )

            with self._lock:
                self._joint_action_busy = False

                if self._joint_action_state == (
                    "restoring_stream"
                ):
                    self._joint_action_state = "idle"
# ============================================================
# FastAPI application
# ============================================================

if not STATIC_DIR.exists():
    raise RuntimeError(
        f"Không tìm thấy thư mục static: {STATIC_DIR}"
    )

app = FastAPI(
    title="RBY1 Local Web Controller",
    version="1.0.0",
)

robot_bridge: Optional[RBY1WebBridge] = None


def get_robot_bridge() -> RBY1WebBridge:
    if robot_bridge is None:
        raise HTTPException(
            status_code=503,
            detail="ROS 2 bridge chưa khởi tạo.",
        )

    return robot_bridge


def http_error_from_exception(
    error: Exception,
    status_code: int = 409,
) -> HTTPException:
    return HTTPException(
        status_code=status_code,
        detail=str(error),
    )


# ============================================================
# Static website
# ============================================================

@app.get("/")
async def index() -> FileResponse:
    return FileResponse(
        STATIC_DIR / "index.html"
    )


@app.get("/api/health")
async def health() -> dict[str, Any]:
    bridge = get_robot_bridge()

    return {
        "success": True,
        "web": "online",
        "ros2_connected":
            bridge.robot_is_connected(),
    }


@app.get("/api/status")
async def get_status() -> dict[str, Any]:
    return get_robot_bridge().get_status()


# ============================================================
# Power, servo and stream APIs
# ============================================================

@app.post("/api/power")
async def set_power(
    command: SwitchRequest,
) -> dict[str, Any]:
    bridge = get_robot_bridge()

    try:
        result = await bridge.call_state_service(
            bridge.power_client,
            command.enabled,
            parameters="all",
            value=0.0,
        )

    except (RuntimeError, TimeoutError) as error:
        raise http_error_from_exception(
            error,
            status_code=503,
        ) from error

    if not result["success"]:
        raise HTTPException(
            status_code=409,
            detail=result["message"],
        )

    return result


@app.post("/api/servo")
async def set_servo(
    command: SwitchRequest,
) -> dict[str, Any]:
    bridge = get_robot_bridge()

    try:
        result = await bridge.call_state_service(
            bridge.servo_client,
            command.enabled,
            parameters="all",
            value=0.0,
        )

    except (RuntimeError, TimeoutError) as error:
        raise http_error_from_exception(
            error,
            status_code=503,
        ) from error

    if not result["success"]:
        raise HTTPException(
            status_code=409,
            detail=result["message"],
        )

    return result


@app.post("/api/stream")
async def set_stream(
    command: SwitchRequest,
) -> dict[str, Any]:
    bridge = get_robot_bridge()

    if not command.enabled:
        bridge.stop_robot()

    try:
        result = await bridge.call_state_service(
            bridge.stream_client,
            command.enabled,
            parameters="",
            value=(
                20.0
                if command.enabled
                else 0.0
            ),
        )

    except (RuntimeError, TimeoutError) as error:
        raise http_error_from_exception(
            error,
            status_code=503,
        ) from error

    if not result["success"]:
        raise HTTPException(
            status_code=409,
            detail=result["message"],
        )

    return result


@app.post("/api/prepare")
async def prepare_robot() -> dict[str, Any]:
    bridge = get_robot_bridge()

    try:
        return await bridge.prepare_robot()

    except (RuntimeError, TimeoutError) as error:
        bridge.stop_robot()

        raise http_error_from_exception(
            error,
            status_code=409,
        ) from error


# ============================================================
# Mobile-base APIs
# ============================================================

@app.post("/api/velocity")
async def set_velocity(
    command: VelocityRequest,
) -> dict[str, Any]:
    bridge = get_robot_bridge()

    try:
        bridge.set_velocity(
            command.linear_x,
            command.linear_y,
            command.angular_z,
        )

    except RuntimeError as error:
        raise http_error_from_exception(
            error,
            status_code=409,
        ) from error

    return {
        "success": True,

        "command": {
            "linear_x": command.linear_x,
            "linear_y": command.linear_y,
            "angular_z": command.angular_z,
        },
    }


@app.post("/api/stop")
async def stop_robot() -> dict[str, Any]:
    bridge = get_robot_bridge()
    bridge.stop_robot()

    return {
        "success": True,
        "message": "Đã gửi lệnh dừng đế.",
    }


@app.post("/api/cancel")
async def cancel_control() -> dict[str, Any]:
    bridge = get_robot_bridge()

    try:
        return await bridge.cancel_all_control()

    except (RuntimeError, TimeoutError) as error:
        raise http_error_from_exception(
            error,
            status_code=503,
        ) from error


# ============================================================
# Joint-control APIs
# ============================================================

@app.post("/api/joints/nudge")
async def nudge_joint(
    command: JointNudgeRequest,
) -> dict[str, Any]:
    """
    Thay đổi một khớp tương đối so với vị trí hiện tại.
    """

    bridge = get_robot_bridge()

    current = bridge.get_group_positions(
        command.group
    )

    expected_count = JOINT_COUNTS[
        command.group
    ]

    if len(current) != expected_count:
        raise HTTPException(
            status_code=409,
            detail=(
                f"Chưa nhận đủ trạng thái "
                f"{command.group}. "
                f"Hiện có {len(current)}/"
                f"{expected_count} khớp."
            ),
        )

    if command.joint_index >= expected_count:
        raise HTTPException(
            status_code=422,
            detail=(
                f"joint_index của "
                f"{command.group} chỉ từ 0 "
                f"đến {expected_count - 1}."
            ),
        )

    target = list(current)

    target[command.joint_index] = (
        target[command.joint_index]
        + command.delta
    )

    try:
        result = await bridge.send_joint_goal(
            {
                command.group: target,
            },
            minimum_time=command.minimum_time,
            priority=10,
        )

    except (
        RuntimeError,
        ValueError,
        TimeoutError,
    ) as error:
        raise http_error_from_exception(
            error,
            status_code=409,
        ) from error

    if not result["success"]:
        raise HTTPException(
            status_code=409,
            detail=(
                "Lệnh khớp không hoàn thành: "
                f"{result['finish_code']}"
            ),
        )

    return {
        **result,
        "group": command.group,
        "joint_index": command.joint_index,
        "delta": command.delta,
        "target": target,
    }


@app.post("/api/joints/pose")
async def set_joint_pose(
    command: JointPoseRequest,
) -> dict[str, Any]:
    """
    Gửi trực tiếp một pose cho một hoặc nhiều nhóm khớp.
    """

    bridge = get_robot_bridge()

    commands: dict[str, list[float]] = {}

    if command.torso is not None:
        commands["torso"] = command.torso

    if command.right_arm is not None:
        commands["right_arm"] = (
            command.right_arm
        )

    if command.left_arm is not None:
        commands["left_arm"] = (
            command.left_arm
        )

    if command.head is not None:
        commands["head"] = command.head

    try:
        result = await bridge.send_joint_goal(
            commands,
            minimum_time=command.minimum_time,
            priority=command.priority,
        )

    except (
        RuntimeError,
        ValueError,
        TimeoutError,
    ) as error:
        raise http_error_from_exception(
            error,
            status_code=409,
        ) from error

    if not result["success"]:
        raise HTTPException(
            status_code=409,
            detail=(
                "Lệnh pose không hoàn thành: "
                f"{result['finish_code']}"
            ),
        )

    return {
        **result,
        "commands": commands,
    }


@app.post("/api/joints/ready")
async def joint_ready_pose() -> dict[str, Any]:
    """Đưa robot đến tư thế co tay thử nghiệm."""

    bridge = get_robot_bridge()
    bridge.stop_robot()

    try:
        result = await bridge.send_joint_goal(
            {
                group: list(positions)
                for group, positions
                in READY_POSE.items()
            },
            minimum_time=5.0,
            priority=10,
        )

    except (
        RuntimeError,
        ValueError,
        TimeoutError,
    ) as error:
        raise http_error_from_exception(
            error,
            status_code=409,
        ) from error

    if not result["success"]:
        raise HTTPException(
            status_code=409,
            detail=(
                "Ready pose không hoàn thành: "
                f"{result['finish_code']}"
            ),
        )

    return result


@app.post("/api/joints/zero")
async def joint_zero_pose() -> dict[str, Any]:
    """
    Đưa thân, hai tay và đầu về 0 rad.
    """

    bridge = get_robot_bridge()

    bridge.stop_robot()

    try:
        result = await bridge.send_joint_goal(
            {
                "torso": [0.0] * 6,
                "right_arm": [0.0] * 7,
                "left_arm": [0.0] * 7,
                "head": [0.0] * 2,
            },
            minimum_time=4.0,
            priority=10,
        )

    except (
        RuntimeError,
        ValueError,
        TimeoutError,
    ) as error:
        raise http_error_from_exception(
            error,
            status_code=409,
        ) from error

    if not result["success"]:
        raise HTTPException(
            status_code=409,
            detail=(
                "Zero pose không hoàn thành: "
                f"{result['finish_code']}"
            ),
        )

    return result
#=================================================================
@app.post("/api/joints/test-arms")
async def test_both_arms() -> dict[str, Any]:
    """
    Pose thử nghiệm dễ quan sát cho cả hai tay.
    Không phụ thuộc dữ liệu joint state hiện tại.
    """

    bridge = get_robot_bridge()

    try:
        result = await bridge.send_joint_goal(
            {
                "right_arm": [
                    0.0,
                    -0.5,
                    0.0,
                    -1.0,
                    0.0,
                    0.0,
                    0.0,
                ],

                "left_arm": [
                    0.0,
                    0.5,
                    0.0,
                    -1.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            },
            minimum_time=5.0,
            priority=10,
        )

    except (
        RuntimeError,
        ValueError,
        TimeoutError,
    ) as error:
        raise HTTPException(
            status_code=409,
            detail=str(error),
        ) from error

    return result
# ============================================================
# WebSocket status
# ============================================================

@app.websocket("/ws/status")
async def status_websocket(
    websocket: WebSocket,
) -> None:
    await websocket.accept()

    bridge = get_robot_bridge()

    try:
        while True:
            await websocket.send_json(
                bridge.get_status()
            )

            await asyncio.sleep(0.10)

    except WebSocketDisconnect:
        # Mất kết nối điều khiển web thì dừng đế.
        bridge.stop_robot()

    except RuntimeError:
        # Socket đã đóng trong lúc gửi.
        bridge.stop_robot()


# Đặt mount sau các API route.
app.mount(
    "/static",
    StaticFiles(directory=STATIC_DIR),
    name="static",
)


# ============================================================
# Main
# ============================================================

def main() -> None:
    global robot_bridge

    rclpy.init()

    robot_bridge = RBY1WebBridge()

    executor = MultiThreadedExecutor(
        num_threads=4,
    )

    executor.add_node(robot_bridge)

    executor_thread = threading.Thread(
        target=executor.spin,
        name="rby1-ros2-executor",
        daemon=True,
    )

    executor_thread.start()

    try:
        uvicorn.run(
            app,
            host=WEB_HOST,
            port=WEB_PORT,
            log_level="info",
        )

    finally:
        robot_bridge.stop_robot()

        try:
            executor.shutdown()
        except Exception:
            pass

        executor_thread.join(timeout=2.0)

        robot_bridge.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()