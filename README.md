# RB-Y1 Qt Direct SDK Deployment

Repository này chỉ giữ runtime điều khiển trực tiếp cho
[`RBY1_Qt_Application_Controller`](https://github.com/phong143hiyg/RBY1_Qt_Application_Controller):

```text
Qt process -> RBY1 C++ SDK v0.10.0 -> gRPC -> simulator hoặc robot RB-Y1
```

Qt là command owner duy nhất. Không chạy thêm SDK client khác trên cùng endpoint.
Luôn xác minh với simulator trước khi kết nối robot thật.

## Thành phần còn lại

| Tệp/thư mục | Vai trò |
| --- | --- |
| `rby1-docker/docker-compose.yml` | Khởi động duy nhất simulator Model M v1.2 |
| `rby1-docker/Dockerfile.sim` | Khóa official simulator image và digest |
| `rby1-docker/Dockerfile.sdk-dev` | Môi trường Qt 6/C++ với RBY1 SDK v0.10.0 |
| `rby1-docker/sdk_smoke/` | Probe chỉ đọc thông tin và trạng thái robot |
| `rby1-docker/verify-qt-direct.sh` | Kiểm tra compose, endpoint và smoke test |

Sơ đồ hiện tại nằm tại
[`docs/rby1_system_architecture_simple.mmd`](docs/rby1_system_architecture_simple.mmd)
và [`docs/rby1_system_architecture_and_state.drawio`](docs/rby1_system_architecture_and_state.drawio).

## Chạy simulator

Yêu cầu: Docker Engine, Compose plugin, X11 và cổng TCP `50051` còn trống.
Simulator được khóa tại
`rainbowroboticsofficial/rby1-sim:0.10.6-m_v1.2` với digest
`sha256:1a462a3e11299d773c089c0c691a7f31b011a0408a8d503a0529b085fc0c9443`.

```bash
cd rby1-docker
xhost +local:docker

docker compose build rby1-sim
docker compose up -d
docker compose ps
```

Cấu hình ứng dụng Qt:

```text
model: M
endpoint: localhost:50051
```

Kiểm tra compose chỉ có simulator và endpoint đã listen:

```bash
docker compose config --services
docker compose top
ss -ltnp | grep ':50051'
```

`config --services` phải chỉ in `rby1-sim`.

Dừng simulator:

```bash
docker compose down --remove-orphans
xhost -local:docker
```

## Môi trường SDK-dev và smoke test

`Dockerfile.sdk-dev` dùng Ubuntu 22.04, Qt 6, CMake/Ninja, Conan và
`rby1-sdk` tag `v0.10.0`, commit
`9af8a734b7bef0167545e3d9f0d559276a5b64ee`.

```bash
cd rby1-docker
docker build -f Dockerfile.sdk-dev -t local/rby1-sdk-dev:0.10.0 .
```

Để build ứng dụng Qt trong container, mount repository ứng dụng vào
`/workspace`:

```bash
docker run --rm -it --network host \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /absolute/path/to/RBY1_Qt_Application_Controller:/workspace \
  local/rby1-sdk-dev:0.10.0
```

Project Qt dùng `find_package(rby1-sdk CONFIG REQUIRED)` và link target
`rby1-sdk::rby1-sdk`. Khi configure, dùng Conan toolchain đã có trong image:

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=/opt/rby1-sdk/build/conan_toolchain.cmake \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

Probe đi kèm chỉ gọi `Connect()`, `GetRobotInfo()` và `GetState()`; không bật
power/servo/control manager và không gửi motion:

```bash
docker run --rm --network host local/rby1-sdk-dev:0.10.0 \
  rby1-readonly-smoke localhost:50051
```

Sau khi simulator và image SDK-dev đã sẵn sàng:

```bash
./verify-qt-direct.sh
```

## Simulator-first rồi mới tới robot thật

1. Đảm bảo không có SDK client khác đang giữ endpoint.
2. Khởi động simulator và chạy read-only smoke test.
3. Thử UI Qt với giới hạn an toàn trong simulator.
4. Dừng simulator và xác minh tiến trình đã kết thúc.
5. Đổi endpoint Qt sang địa chỉ robot do đơn vị vận hành cung cấp.
6. Đọc state/info trước mọi thử nghiệm motion trên robot thật.

## Phiên bản và nguồn

- RBY1 C++ SDK: `v0.10.0`, commit
  `9af8a734b7bef0167545e3d9f0d559276a5b64ee`, Apache-2.0.
- Simulator: official image `0.10.6-m_v1.2`, Model M v1.2, khóa digest.
- Simulator image chưa công bố rõ license/NOTICE trên Docker Hub; cần xác nhận
  với Rainbow Robotics trước khi phân phối lại image.

Nguồn chính thức:

- [RBY1 SDK v0.10.0](https://github.com/RainbowRobotics/rby1-sdk/tree/v0.10.0)
- [RBY1 SDK release v0.10.0](https://github.com/RainbowRobotics/rby1-sdk/releases/tag/v0.10.0)
- [RBY1 simulator Docker Hub](https://hub.docker.com/r/rainbowroboticsofficial/rby1-sim)

Kết quả audit nằm tại
[`docs/qt-direct-audit.md`](docs/qt-direct-audit.md).
