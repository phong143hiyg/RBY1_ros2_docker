# RB-Y1 Local Controller

Bộ môi trường Docker để mô phỏng và điều khiển robot **Rainbow Robotics RB-Y1** qua ROS 2 Humble. Dự án tích hợp MuJoCo simulator, ROS 2 driver, giao diện điều khiển trên trình duyệt và một TCP bridge dành cho ứng dụng desktop.

> Dự án có thể phát lệnh chuyển động tới robot. Hãy thử nghiệm trong simulator trước, bảo đảm vùng hoạt động không có người/vật cản và luôn sẵn sàng dừng robot.

## Thành phần

| Service | Chức năng | Giao tiếp |
| --- | --- | --- |
| `rby1-sim` | Mô phỏng RB-Y1 bằng MuJoCo | Giao diện X11, host network |
| `rby1-ros2` | SDK và ROS 2 driver chính thức của RB-Y1 | ROS 2 domain `0` |
| `rby1-web` | Backend FastAPI và giao diện web điều khiển | HTTP `8000` |
| `rby1-app-bridge` | Cầu nối ROS 2 cho [ứng dụng desktop Qt](https://github.com/phong143hiyg/RBY1_Qt_Application_Controller) | TCP `8081` |

Mã nguồn ứng dụng điều khiển desktop được phát triển ở repository riêng: [RBY1 Qt Application Controller](https://github.com/phong143hiyg/RBY1_Qt_Application_Controller). Repository hiện tại cung cấp `rby1-app-bridge` để ứng dụng Qt giao tiếp với ROS 2 qua TCP.

Giao diện web hỗ trợ:

- Theo dõi kết nối, trạng thái control manager, stream, va chạm, odometry và các khớp theo thời gian thực.
- Chuẩn bị robot, bật/tắt stream và hủy lệnh điều khiển.
- Điều khiển đế bằng các nút điều hướng trên màn hình.
- Điều khiển từng khớp của thân, hai tay và đầu; chuyển sang ready pose hoặc zero pose.
- Tự dừng đế khi mất lệnh vận tốc quá 350 ms hoặc khi WebSocket bị ngắt.

## Yêu cầu

- Linux có Docker Engine và Docker Compose plugin.
- Máy hỗ trợ X11 để hiển thị simulator/RViz.
- Khuyến nghị tối thiểu 16 GB RAM và đủ dung lượng đĩa cho ROS 2, MoveIt, SDK và simulator.
- Các cổng `8000` và `8081` chưa được dịch vụ khác sử dụng.

Kiểm tra môi trường:

```bash
docker --version
docker compose version
echo "$DISPLAY"
```

## Cài đặt

Clone repository và chuyển vào thư mục Docker:

```bash
git clone <repository-url>
cd mynameisrobot/rby1-docker
```

### 1. Build image simulator

```bash
docker build \
  -f Dockerfile.sim \
  -t local/rby1-sim:0.10.6-m_v1.2 \
  .
```

### 2. Build image ROS 2

```bash
docker build \
  -f Dockerfile.ros2 \
  -t local/rby1-ros2:humble \
  .
```

Bước này clone và build `rby1-sdk` cùng `rby1-ros2`, vì vậy có thể mất khá nhiều thời gian trong lần đầu. Build mặc định được cố định ở `rby1-sdk` tag `v0.10.0` (commit `9af8a734`) và `rby1-ros2` commit `2493c8dd` để schema ROS 2 và bridge luôn khớp nhau.

Có thể chọn revision khác khi build:

```bash
docker build \
  --build-arg RBY1_SDK_REF=v0.10.0 \
  --build-arg RBY1_ROS2_REF=2493c8dd62dc835d5f3142d9d49ca7a8f8959f1b \
  -f Dockerfile.ros2 \
  -t local/rby1-ros2:humble \
  .
```

### 3. Build web controller và app bridge

Hai image này phụ thuộc vào `local/rby1-ros2:humble`:

```bash
docker compose build rby1-web rby1-app-bridge
```

## Chạy dự án

Cho phép container kết nối tới X server trong phiên hiện tại:

```bash
xhost +local:docker
```

Khởi động toàn bộ hệ thống:

```bash
docker compose up -d
```

Theo dõi trạng thái và log:

```bash
docker compose ps
docker compose logs -f
```

Mở giao diện điều khiển tại:

```text
http://localhost:8000
```

Sau khi dùng xong:

```bash
docker compose down
xhost -local:docker
```

## Quy trình điều khiển đề xuất

1. Chờ simulator và ROS 2 driver khởi động hoàn tất.
2. Mở `http://localhost:8000` và kiểm tra trạng thái kết nối.
3. Nhấn **Chuẩn bị robot**.
4. Bật stream nếu giao diện chưa báo `ON`.
5. Giữ nút điều hướng để di chuyển; thả nút để dừng.
6. Chỉ gửi lệnh khớp khi robot đứng yên và vùng chuyển động an toàn.
7. Dùng **DỪNG ĐẾ** để dừng chuyển động đế, hoặc **Hủy toàn bộ** để hủy các lệnh điều khiển.

Giới hạn vận tốc hiện tại của web controller:

- Tịnh tiến: tối đa `0.20 m/s` theo trục X/Y.
- Quay: tối đa `0.50 rad/s` quanh trục Z.

## API chính

FastAPI tự sinh tài liệu tương tác tại `http://localhost:8000/docs`.

| Method | Endpoint | Mô tả |
| --- | --- | --- |
| `GET` | `/api/health` | Kiểm tra web server và kết nối ROS 2 |
| `GET` | `/api/status` | Lấy trạng thái robot hiện tại |
| `POST` | `/api/prepare` | Chuẩn bị power, servo và control manager |
| `POST` | `/api/power` | Bật/tắt nguồn robot |
| `POST` | `/api/servo` | Bật/tắt servo |
| `POST` | `/api/stream` | Bật/tắt stream control |
| `POST` | `/api/velocity` | Gửi vận tốc đế |
| `POST` | `/api/stop` | Dừng đế ngay lập tức |
| `POST` | `/api/cancel` | Hủy toàn bộ điều khiển |
| `POST` | `/api/joints/nudge` | Dịch chuyển tương đối một khớp |
| `POST` | `/api/joints/pose` | Gửi pose cho một hoặc nhiều nhóm khớp |
| `POST` | `/api/joints/ready` | Chuyển tới tư thế co tay |
| `POST` | `/api/joints/zero` | Đưa thân, tay và đầu về `0 rad` |
| `WS` | `/ws/status` | Stream trạng thái với chu kỳ khoảng 100 ms |

Ví dụ gửi lệnh vận tốc:

```bash
curl -X POST http://localhost:8000/api/velocity \
  -H 'Content-Type: application/json' \
  -d '{"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.0}'
```

Dừng đế:

```bash
curl -X POST http://localhost:8000/api/stop
```

### TCP Ready pose động

Desktop controller giao tiếp với `rby1-app-bridge` qua TCP port `8081`; mỗi JSON request và response chiếm một dòng. Gửi `set_ready_pose` để lưu snapshot hiện tại của 22 khớp thân trên (torso 6, head 2, mỗi tay 7). Bridge không lưu mobility/wheel joints và chỉ thay snapshot khi đủ toàn bộ giá trị hữu hạn.

```json
{"command":"set_ready_pose"}
```

Kết quả thành công:

```json
{"success":true,"message":"Ready pose saved","ready_pose_saved":true}
```

Sau đó, dùng lệnh đã có `ready_pose` (có thể kèm `minimum_time`) để robot trở về snapshot này. Gửi `{"command":"clear_ready_pose"}` để xoá snapshot. Nếu snapshot chưa được lưu trong phiên Bridge hiện tại, lệnh `ready_pose` trả lỗi thay vì chạy ready pose tĩnh. Trường `ready_pose_saved` cũng có trong response `status`.

### Trạng thái component quan sát từ robot

Lệnh TCP `{"command":"status"}` trả thêm object `components`. Đây là nguồn mà ứng dụng Qt phải dùng:

```json
{
  "success": true,
  "connected": true,
  "components": {
    "power": {
      "known": true,
      "enabled": true,
      "pending": false,
      "source": "robot_api"
    },
    "servo": {
      "known": true,
      "enabled": true,
      "pending": false,
      "source": "robot_api"
    },
    "stream": {
      "known": true,
      "enabled": true,
      "pending": false,
      "source": "robot_state"
    }
  }
}
```

Nguồn sự thật của từng component:

- `power`: `Robot::GetState().power_states` của RBY1 SDK. Driver tổng hợp tất cả power device; nếu SDK trả `kUnknown` hoặc dữ liệu không đầy đủ thì trạng thái là `unknown`.
- `servo`: cùng snapshot `Robot::GetState()`, theo đúng điều kiện mà SDK `Robot::IsServoOn(".*")` sử dụng: mọi joint phải `is_ready` và dữ liệu joint không cũ quá 100 ms. Dữ liệu cũ được biểu diễn là `unknown`, không phải OFF.
- `stream`: trường `RobotState.robot_stream_state` của ROS 2 Driver.

`known=false` luôn đi cùng `enabled=null`; vì vậy trạng thái chưa đọc được không bị biểu diễn thành OFF trong schema mới. Các boolean top-level cũ `power`, `servo`, `stream` vẫn được giữ tạm thời cho client cũ và sẽ là `false` khi unknown; không dùng các field cũ để ra quyết định điều khiển.

Khi gửi lệnh Power, Servo hoặc Stream, bridge đặt `pending=true`, gọi service và chờ topic `/rby1/robot_state` xác nhận giá trị quan sát. Response command chỉ thành công khi quan sát đạt giá trị yêu cầu trước deadline monotonic 10 giây. Service trả success nhưng trạng thái vật lý không đổi sẽ trả lỗi timeout. Khi mất `RobotState` quá 1,5 giây, cả ba component chuyển sang unknown; message đầu tiên sau reconnect tự đồng bộ lại chúng.

Các interface SDK được kiểm chứng trực tiếp tại [`robot.h` của v0.10.0](https://github.com/RainbowRobotics/rby1-sdk/blob/v0.10.0/include/rby1-sdk/robot.h), [`robot_state.h` của v0.10.0](https://github.com/RainbowRobotics/rby1-sdk/blob/v0.10.0/include/rby1-sdk/robot_state.h) và implementation [`IsPowerOn`/`IsServoOn`](https://github.com/RainbowRobotics/rby1-sdk/blob/v0.10.0/src/robot.cpp). [`RobotState.msg` tại commit rby1-ros2 đã pin](https://github.com/RainbowRobotics/rby1-ros2/blob/2493c8dd62dc835d5f3142d9d49ca7a8f8959f1b/rby1_msgs/msg/RobotState.msg) chưa có field Power/Servo, nên project áp dụng patch nhỏ `rby1-ros2-observed-component-state.patch` trong lúc build driver; patch tái sử dụng lần gọi `GetState()` định kỳ sẵn có và không tăng tần suất polling Robot API.

## ROS 2 interface

Web controller và app bridge sử dụng namespace `/rby1` với các interface chính:

- Topic trạng thái: `/rby1/robot_state` (bao gồm tri-state `power_state`, `servo_state` và `robot_stream_state` sau patch), `/rby1/odom`, `/rby1/joint_states` và các topic joint state theo nhóm.
- Topic điều khiển đế: `/rby1/cmd_vel`.
- Service: `/rby1/robot_power`, `/rby1/robot_servo`, `/rby1/stream_control`, `/rby1/cancel_control`.
- Action điều khiển khớp: `/rby1/robot_joint`.

Tất cả container dùng host network, Fast DDS và `ROS_DOMAIN_ID=0`. Khi kết nối node ROS 2 bên ngoài, hãy dùng cùng domain và middleware tương thích.

## Cấu trúc thư mục

```text
.
├── README.md
└── rby1-docker/
    ├── docker-compose.yml
    ├── Dockerfile.sim
    ├── Dockerfile.ros2
    ├── Dockerfile.web
    ├── Dockerfile.app-bridge
    ├── ros_entrypoint.sh
    ├── web_control/
    │   ├── server.py
    │   └── static/
    └── app_bridge_ws/
        └── src/rby1_app_bridge/
```

## Khắc phục sự cố

### Simulator không hiển thị cửa sổ

Kiểm tra `DISPLAY`, X11 socket và quyền truy cập:

```bash
echo "$DISPLAY"
ls -la /tmp/.X11-unix
xhost +local:docker
```

### Web mở được nhưng robot báo mất kết nối

```bash
docker compose ps
docker compose logs rby1-ros2
docker compose exec rby1-ros2 ros2 topic echo /rby1/robot_state --once
```

Kiểm tra các container đều dùng `ROS_DOMAIN_ID=0` và không có firewall chặn DDS trên host network.

### Không gửi được lệnh vận tốc

Robot chỉ nhận lệnh khi đã kết nối, control manager ở trạng thái `ENABLE` hoặc `EXECUTING`, stream đang bật, không có collision và không có joint action đang chạy. Hãy nhấn **Chuẩn bị robot**, kiểm tra bảng trạng thái rồi thử lại.

### Cổng đã được sử dụng

```bash
ss -ltnp | grep -E ':8000|:8081'
```

Tắt tiến trình đang giữ cổng hoặc đổi cổng tương ứng trong mã nguồn và cấu hình Docker.

## Phát triển

Mã web được mount read-only vào container. Sau khi sửa HTML/CSS/JavaScript hoặc `server.py`, restart service:

```bash
docker compose restart rby1-web
```

Khi thay đổi TCP bridge C++, cần build lại image:

```bash
docker compose build rby1-app-bridge
docker compose up -d rby1-app-bridge
```

`Dockerfile.app-bridge` chạy unit test trong lúc build image. Có thể chạy lại riêng:

```bash
docker compose build rby1-app-bridge
```

## Giấy phép

Package `rby1_app_bridge` khai báo giấy phép MIT. Các thành phần bên thứ ba như ROS 2, `rby1-sdk`, `rby1-ros2` và image simulator tuân theo giấy phép riêng của từng dự án.


## Test MoveIt 2 planning/preview độc lập

Package mới [`rby1_motion_planning`](rby1-docker/planning_ws/src/rby1_motion_planning/README.md) có fake launch, scene YAML, C++ worker/validator, service TCP NDJSON và test thực. Dùng compose riêng, ROS domain 83 và GenericSystem; backend `execution_enabled=false`.

```bash
cd rby1-docker
docker compose -p rby1-planning -f planning-compose.yml build
docker compose -p rby1-planning -f planning-compose.yml up
```

Model M v1.2 ở đây chỉ là minh họa fake. Grasp/pick-place bị chặn rõ ràng bởi cấu hình mimic ngón kẹp không tương thích bounds. Adapter mới công bố contract Qt v1 trên `127.0.0.1:8082`; transport thử nghiệm nội bộ ở 7447 không được publish ra host. Hai file chuẩn của Qt được sao chép nguyên byte vào [`rby1-docker/planning_protocol`](rby1-docker/planning_protocol/protocol-v1.md) và đối chiếu SHA-256 trong test.

Planning launch yêu cầu `ROS_DOMAIN_ID` riêng khác 0 và không gửi command robot. Nếu Qt đang kết nối SDK trực tiếp tới robot, không chạy `docker-compose.yml` (nó khởi động `rby1_driver` và các bridge có thể gửi lệnh). Báo cáo MoveIt trong package là kết quả từ image Docker được chạy ngày 16/09/2026, trước adapter contract hiện tại; môi trường host này cần Docker/ROS trước khi tái chạy build và kiểm tra end to end.
