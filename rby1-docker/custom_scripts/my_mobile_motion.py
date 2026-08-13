#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rby1_msgs.msg import RobotState
from rby1_msgs.srv import StateOnOff
from rclpy.node import Node


class CircleController(Node):
    def __init__(self) -> None:
        # Các tên tương đối bên dưới sẽ nằm trong namespace /rby1.
        super().__init__("circle_controller", namespace="rby1")

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "cmd_vel",
            10,
        )

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

        self.control_manager_state = None

        self.create_subscription(
            RobotState,
            "robot_state",
            self.robot_state_callback,
            10,
        )

    def robot_state_callback(self, message: RobotState) -> None:
        self.control_manager_state = message.control_manager_state

    def call_state_service(
        self,
        client,
        state: bool,
        parameters: str = "",
        value: float = 0.0,
    ) -> bool:
        """Gọi service bật/tắt nguồn, servo hoặc stream."""

        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f"Không tìm thấy service {client.srv_name}"
            )
            return False

        request = StateOnOff.Request()
        request.state = state
        request.parameters = parameters
        request.value = value

        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=15.0,
        )

        response = future.result()

        if response is None:
            self.get_logger().error(
                f"Không nhận được phản hồi từ {client.srv_name}"
            )
            return False

        if not response.success:
            self.get_logger().error(
                f"{client.srv_name} thất bại: {response.message}"
            )
            return False

        self.get_logger().info(
            f"{client.srv_name}: {response.message}"
        )
        return True

    def wait_for_robot_ready(self, timeout: float = 10.0) -> bool:
        """Chờ Control Manager chuyển sang ENABLE hoặc EXECUTING."""

        start_time = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.control_manager_state in (2, 3):
                return True

            if time.monotonic() - start_time >= timeout:
                return False

        return False

    def prepare_robot(self) -> bool:
        """Bật nguồn và servo nếu robot chưa ở trạng thái sẵn sàng."""

        # Đọc trạng thái hiện tại trước.
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.control_manager_state in (2, 3):
            self.get_logger().info("Robot đã ở trạng thái sẵn sàng.")
            return True

        self.get_logger().info("Đang bật nguồn robot...")

        if not self.call_state_service(
            self.power_client,
            True,
            parameters="all",
        ):
            return False

        time.sleep(1.0)

        self.get_logger().info("Đang bật servo...")

        if not self.call_state_service(
            self.servo_client,
            True,
            parameters="all",
        ):
            return False

        if not self.wait_for_robot_ready(timeout=10.0):
            self.get_logger().error(
                "Robot không chuyển sang trạng thái ENABLE."
            )
            return False

        self.get_logger().info("Robot đã sẵn sàng điều khiển.")
        return True

    def publish_velocity(
        self,
        linear_x: float,
        angular_z: float,
    ) -> None:
        command = Twist()

        command.linear.x = float(linear_x)
        command.linear.y = 0.0
        command.angular.z = float(angular_z)

        self.cmd_vel_publisher.publish(command)

    def run_circle(
        self,
        linear_speed: float,
        angular_speed: float,
    ) -> None:
        """Chạy một vòng tròn hoàn chỉnh."""

        radius = linear_speed / abs(angular_speed)
        duration = 2.0 * math.pi / abs(angular_speed)

        self.get_logger().info(
            f"Bắt đầu chạy vòng tròn: "
            f"bán kính={radius:.2f} m, "
            f"thời gian={duration:.2f} giây"
        )

        start_time = time.monotonic()

        # Gửi lệnh ở khoảng 25 Hz.
        while (
            rclpy.ok()
            and time.monotonic() - start_time < duration
        ):
            self.publish_velocity(
                linear_x=linear_speed,
                angular_z=angular_speed,
            )

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.04)

        self.get_logger().info("Đã hoàn thành vòng tròn.")

    def stop_robot(self) -> None:
        """Gửi lệnh vận tốc bằng 0 nhiều lần để dừng chắc chắn."""

        self.get_logger().info("Đang dừng robot...")

        for _ in range(15):
            self.publish_velocity(
                linear_x=0.0,
                angular_z=0.0,
            )

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.04)


def main() -> None:
    rclpy.init()

    controller = CircleController()
    stream_enabled = False

    try:
        if not controller.prepare_robot():
            raise RuntimeError(
                "Không thể chuẩn bị robot để điều khiển."
            )

        controller.get_logger().info(
            "Đang bật stream control..."
        )

        if not controller.call_state_service(
            controller.stream_client,
            True,
            parameters="",
            value=0.0,
        ):
            raise RuntimeError(
                "Không bật được stream control."
            )

        stream_enabled = True
        time.sleep(1.0)

        # Vận tốc tiến 0.10 m/s.
        linear_speed = 0.4

        # Dương: chạy vòng tròn sang trái.
        # Âm: chạy vòng tròn sang phải.
        angular_speed = 0.25

        controller.run_circle(
            linear_speed=linear_speed,
            angular_speed=angular_speed,
        )

    except KeyboardInterrupt:
        controller.get_logger().warning(
            "Chương trình bị dừng bằng Ctrl+C."
        )

    except RuntimeError as error:
        controller.get_logger().error(str(error))

    finally:
        controller.stop_robot()

        if stream_enabled:
            controller.get_logger().info(
                "Đang tắt stream control..."
            )

            controller.call_state_service(
                controller.stream_client,
                False,
                parameters="",
                value=0.0,
            )

        controller.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
