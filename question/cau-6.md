# Câu 6: Các thuật toán và kiến trúc sử dụng trong project

Ngoài **State Pattern**, project RB-Y1 Local Controller sử dụng các kiến trúc, mẫu thiết kế và thuật toán sau.

## 1. Kiến trúc hệ thống

### 1.1. Kiến trúc phân lớp ba tầng

Hệ thống được chia thành ba tầng:

1. **Tầng giao diện:** trình duyệt web và ứng dụng Qt Desktop, nơi người vận hành theo dõi trạng thái và gửi lệnh.
2. **Tầng điều khiển và tích hợp:** FastAPI Web Controller, C++ App Bridge và ROS 2 Interface.
3. **Tầng thiết bị và mô phỏng:** ROS 2 Driver, RBY1 SDK và MuJoCo Simulator.

Việc phân lớp giúp tách giao diện khỏi logic điều khiển và phần cứng. Do đó có thể thay đổi giao diện mà không cần sửa driver, đồng thời có thể chuyển từ simulator sang robot thật mà không phải thay đổi toàn bộ hệ thống.

### 1.2. Kiến trúc Client–Server

Project có hai luồng client–server:

- Trình duyệt là client, FastAPI là server. Hai bên giao tiếp bằng REST/JSON để gửi lệnh và WebSocket để nhận trạng thái liên tục.
- Ứng dụng Qt là TCP client, còn C++ App Bridge là TCP server tại cổng `8081`. Mỗi request và response là một JSON trên một dòng, tức giao thức NDJSON.

### 1.3. Bridge/Adapter Pattern

`RBY1WebBridge` và `RBY1AppBridge` chuyển đổi lệnh từ HTTP hoặc TCP/JSON thành các giao tiếp ROS 2:

```text
Web/Qt → HTTP hoặc TCP/JSON → Bridge → ROS 2 → Driver → Robot/Simulator
```

Nhờ lớp bridge, giao diện người dùng không phải phụ thuộc trực tiếp vào kiểu message, service hay action của ROS 2.

### 1.4. Kiến trúc hướng sự kiện của ROS 2

Hệ thống kết hợp ba mô hình giao tiếp:

- **Publish–Subscribe:** nhận `RobotState`, `Odometry`, `JointState` và publish vận tốc `Twist` lên `/rby1/cmd_vel`.
- **Request–Response:** gọi service để bật/tắt power, servo, stream và hủy điều khiển.
- **Action:** gửi lệnh điều khiển khớp có thời gian thực hiện dài, đồng thời hỗ trợ feedback, kết quả và hủy goal.

Các callback được thực thi khi có message hoặc phản hồi mới, vì vậy hệ thống mang tính hướng sự kiện thay vì liên tục chờ đồng bộ.

### 1.5. Kiến trúc bất đồng bộ và đa luồng

- FastAPI xử lý HTTP và WebSocket bằng `asyncio`.
- ROS 2 sử dụng `MultiThreadedExecutor`.
- C++ App Bridge chạy TCP server trên một thread riêng.
- `mutex` và `RLock` bảo vệ dữ liệu trạng thái dùng chung.
- `command_mutex` tuần tự hóa các lệnh TCP.
- Cờ `joint_action_busy` bảo đảm tại một thời điểm chỉ có một joint action được thực thi.

### 1.6. Kiến trúc container hóa

Docker Compose chia hệ thống thành bốn service:

- `rby1-sim`: chạy MuJoCo Simulator.
- `rby1-ros2`: chạy ROS 2 Driver và RBY1 SDK.
- `rby1-web`: chạy giao diện web và FastAPI backend.
- `rby1-app-bridge`: cung cấp cầu nối TCP cho ứng dụng Qt.

Đây là kiến trúc dịch vụ phân tán được container hóa. Tuy nhiên, không nên gọi đây là microservices thuần túy vì các service vẫn phụ thuộc chặt vào ROS 2 domain và cùng sử dụng host network.

## 2. Các thuật toán và cơ chế xử lý

### 2.1. Tiếp nhận vector vận tốc từ ứng dụng điều khiển

Ứng dụng Qt gửi lệnh `velocity` qua TCP/JSON với ba thành phần:

```text
linear_x  → vận tốc tịnh tiến theo X
linear_y  → vận tốc tịnh tiến theo Y
angular_z → vận tốc quay quanh Z
```

App Bridge kiểm tra các giá trị đầu vào, giới hạn chúng trong phạm vi an toàn, lưu vector vận tốc mong muốn và chuyển thành message `geometry_msgs/Twist` để publish lên `/rby1/cmd_vel`. Cơ chế này cho phép thực hiện đồng thời chuyển động tịnh tiến và quay mà không phụ thuộc vào thiết bị nhập cụ thể của giao diện.

### 2.2. Giới hạn giá trị bằng Saturation/Clamping

Trước khi publish, lệnh vận tốc được giới hạn:

```text
vx, vy ∈ [-0.20, 0.20] m/s
wz     ∈ [-0.50, 0.50] rad/s
```

Thời gian tối thiểu của joint action và độ dịch chuyển của lệnh nudge cũng được giới hạn. Cơ chế này ngăn dữ liệu sai hoặc lệnh từ client vượt quá phạm vi mà controller cho phép.

### 2.3. Dead-man Watchdog

Ứng dụng điều khiển phải gửi lại lệnh vận tốc theo chu kỳ, trong khi App Bridge chạy watchdog và publish ở tần số `20 Hz`. App Bridge lưu thời điểm nhận lệnh gần nhất:

```text
elapsed = current_time - last_command_time
```

Nếu `elapsed > 350 ms`, watchdog tự động đổi vận tốc thành:

```text
vx = 0, vy = 0, wz = 0
```

Robot cũng được yêu cầu dừng khi ứng dụng gửi lệnh `stop`, kết nối TCP bị ngắt hoặc bridge kết thúc. Lệnh zero được publish nhiều lần để tăng khả năng driver nhận được lệnh dừng.

### 2.4. Safety Gating

Trước khi chấp nhận lệnh di chuyển, hệ thống kiểm tra:

- Robot vẫn còn kết nối.
- Control Manager đang ở `ENABLE` hoặc `EXECUTING`.
- Stream control đang bật.
- Không có joint action khác đang chạy.
- Không phát hiện collision.
- Emergency stop không hoạt động.
- Các giá trị đầu vào là số hữu hạn.

Nếu một điều kiện không thỏa mãn, lệnh bị từ chối hoặc vận tốc được đưa về zero.

### 2.5. Điều phối loại trừ giữa điều khiển đế và khớp

Để tránh xung đột giữa `cmd_vel` và joint action, hệ thống thực hiện quy trình:

1. Dừng hoàn toàn chuyển động của đế.
2. Ghi nhớ trạng thái stream hiện tại.
3. Tắt persistent stream của đế.
4. Kiểm tra và gửi joint action.
5. Theo dõi feedback và chờ kết quả với timeout.
6. Hủy goal nếu bị timeout.
7. Khôi phục stream trong bước cleanup.

Đây là cơ chế loại trừ tài nguyên kết hợp phục hồi trạng thái, bảo đảm hai loại lệnh không được thực thi đồng thời.

### 2.6. Thuật toán nudge khớp

Lệnh nudge thay đổi tương đối một khớp dựa trên vị trí hiện tại:

```text
q_target[i] = q_current[i] + Δq
```

`Δq` phải là số hữu hạn và nằm trong khoảng `[-0.20, 0.20] rad`. Các khớp còn lại trong cùng nhóm được giữ nguyên.

### 2.7. Phân loại và sắp xếp khớp

Khi nhận một message `/joint_states`, chương trình:

1. Chuẩn hóa tên khớp về chữ thường.
2. Thay ký tự `/` và `-` bằng `_`.
3. Phân loại khớp vào `torso`, `right_arm`, `left_arm` hoặc `head`.
4. Trích số thứ tự ở cuối tên khớp.
5. Sắp xếp các khớp theo số thứ tự.
6. Chỉ cập nhật nhóm khi nhận đúng số khớp mong đợi.

Nếu một nhóm có `n` khớp thì bước sắp xếp có độ phức tạp `O(n log n)`.

### 2.8. Snapshot và khôi phục Ready Pose

App Bridge có thể lưu vị trí hiện tại của 22 khớp thân trên:

```text
Torso: 6 + Head: 2 + Right arm: 7 + Left arm: 7 = 22 khớp
```

Snapshot chỉ được lưu nếu có đủ bốn nhóm, đúng số lượng khớp và mọi vị trí đều hữu hạn. Khi nhận lệnh `ready_pose`, snapshot được gửi lại cho robot dưới dạng ROS 2 Action.

### 2.9. Chuyển Quaternion thành góc Yaw

Odometry trả về hướng dưới dạng quaternion. Giao diện chuyển quaternion thành góc yaw bằng công thức:

```text
sin_yaw = 2(wz + xy)
cos_yaw = 1 - 2(y² + z²)
yaw = atan2(sin_yaw, cos_yaw)
```

Góc yaw thu được biểu diễn hướng quay của robot quanh trục Z.

### 2.10. Thuật toán chạy quỹ đạo tròn

Script `my_mobile_motion.py` sử dụng vận tốc tiến `v` và vận tốc góc `ω` không đổi. Bán kính và thời gian hoàn thành một vòng được tính bằng:

```text
R = v / |ω|
T = 2π / |ω|
```
Lệnh được publish ở khoảng `25 Hz` trong thời gian `T`. Đây là điều khiển quỹ đạo tròn **open-loop**, nghĩa là chương trình chưa sử dụng odometry để hiệu chỉnh sai số quỹ đạo.

### 2.11. Quy trình chuẩn bị robot tuần tự và fail-fast

Lệnh `prepare` thực hiện theo thứ tự:

```text
Power ON → Servo ON → chờ ENABLE/EXECUTING → Stream ON
```

Mỗi bước có timeout và kiểm tra kết quả. Nếu một bước thất bại, quy trình dừng ngay và trả về nguyên nhân thay vì tiếp tục gửi các lệnh sau.

### 2.12. Tổng hợp trạng thái theo độ ưu tiên

Khi tạo trạng thái gửi cho ứng dụng Qt, hệ thống chọn trạng thái theo thứ tự ưu tiên:

```text
Disconnected → Fault → Preparing → JointBusy
→ Driving → Ready → Connected
```

Nhờ đó trạng thái lỗi hoặc mất kết nối luôn được hiển thị trước những trạng thái vận hành thông thường.

### 2.13. Timeout, polling và tự kết nối lại

- Trạng thái robot bị xem là mất kết nối nếu không được cập nhật trong khoảng thời gian quy định.
- Service và action đều có timeout để tránh chờ vô hạn.
- Các bước chờ `ENABLE`, stream ON/OFF sử dụng polling theo chu kỳ ngắn và deadline dựa trên monotonic clock.
- WebSocket của giao diện web tự kết nối lại sau khi bị ngắt.

## 3. Phạm vi thuật toán của project

Project không tự cài đặt PID, inverse kinematics, SLAM, A*, thuật toán tránh vật cản hoặc thuật toán nội suy quỹ đạo khớp. Những chức năng điều khiển mức thấp và nội suy chuyển động thực tế thuộc ROS 2 Driver và RBY1 SDK. Phần do project triển khai tập trung vào tích hợp hệ thống, chuyển đổi giao thức, điều phối lệnh và các cơ chế an toàn.
