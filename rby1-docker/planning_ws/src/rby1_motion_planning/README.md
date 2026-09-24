# RBY1 MoveIt 2 planning/preview test

Package `ament_cmake`, C++17, tách khỏi `rby1_app_bridge`. Không gọi `execute()`, không gửi command controller và không launch driver/physics simulator. Backend công bố `execution_enabled=false`. Model minh họa fake đã khảo sát là **M v1.2**, không phải lựa chọn model cho robot thật.

## Cấu hình đã khảo sát

| Thành phần | Giá trị |
|---|---|
| ROS | Humble trong `local/rby1-ros2:humble`; host không có `/opt/ros` |
| MoveIt | 2.5.9; phiên bản Debian đầy đủ trong `reports/environment.json` |
| rby1 SDK | v0.10.0, `9af8a734b7bef0167545e3d9f0d559276a5b64ee` |
| rby1-ros2 | `2493c8dd62dc835d5f3142d9d49ca7a8f8959f1b`; giữ nguyên các patch đang có trong image |
| Vendor package/model | `rby1_moveit_m_1_2` / `RBY1_M_v1_2` |
| MTC Humble | `756634951326ae17ae099882f7110c6f1d0a98c0`; build optional nếu không tìm thấy |
| Frame/group/TCP | RobotModel frame `base`; `right_arm`; `ee_right` |
| IK | `kdl_kinematics_plugin/KDLKinematicsPlugin` trong vendor `kinematics.yaml` |
| Gripper | `gripper_r`; contact chỉ `gripper_finger_r1`, `gripper_finger_r2` |
| Fake | Explicit `use_fake_hardware=true`, kiểm tra URDF chỉ có `mock_components/GenericSystem` |
| Controllers hoạt động | Chỉ `joint_state_broadcaster`; MoveIt dùng `rby1_motion_planning/PlanningOnlyControllerManager` không có command handles/action clients |

Vendor SRDF thiếu `end_effector`: launch thêm vào XML trong RAM và ghi chính xác dòng thêm trong `launch_audit.json`. Không ghi đè package vendor. Bounds của `gripper_finger_r1_joint` là `[-0.05,0]`, ngón mimic `gripper_finger_r2_joint` là `[0,0.05]` với multiplier `+1`. Khi mở leader về `-0.04`, follower vượt bounds. Không sửa âm thầm mimic, không giả grasp thành công: `plan_pick_place=false`, lỗi `GRASP_CONFIGURATION_INVALID`. Vị trí marker pick/place là minh họa, không phải grasp đã xác nhận.

Launch bổ sung giới hạn gia tốc fake: 1 rad/s² cho joint không phải ngón, 0.1 m/s² cho ngón, ghi audit. Đây không phải giới hạn manufacturer. URDF có attribute gia tốc riêng, trong khi vendor joint_limits cấu hình `has_acceleration_limits=false`. Fake initial pose gập `right_arm_1=-0.3`, `right_arm_3=-1`, `right_arm_5=1`, các joint khác giữ zero; tránh cấu hình tay thẳng singular. Identity model hash bao gồm URDF expand, SRDF overlay và joint limits.

## Docker: dependency, build và launch một lệnh

Từ root repository:

```bash
cd rby1-docker
docker compose -p rby1-planning -f planning-compose.yml build
docker compose -p rby1-planning -f planning-compose.yml up
```

Dockerfile planning dùng image vendor sẵn có. Nếu chưa có image base:

```bash
cd rby1-docker
docker build -f Dockerfile.ros2 -t local/rby1-ros2:humble .
docker build -f Dockerfile.planning -t local/rby1-planning:humble .
```

Compose planning riêng dùng ROS domain 83, bridge network, publish contract Qt TCP `127.0.0.1:8082`. Transport nội bộ cũ ở `127.0.0.1:7447` chỉ phục vụ worker, CLI và fixture ROS trong container; nó không được publish qua Docker. Không chạy compose driver chính. Mỗi launch tạo robot_state_publisher, move_group, GenericSystem, một broadcaster, scene_loader, worker, transport nội bộ, adapter contract v1 và RViz nếu bật. Bật GUI bằng `PLANNING_RVIZ=true docker compose -p rby1-planning -f planning-compose.yml up`; cần X11 được cấu hình sẵn. Trong RViz có MotionPlanning/DisplayTrajectory và MarkerArray. Khi cấu hình grasp hợp lệ, thêm display **Motion Planning Tasks** để xem scene diff của MTC cùng object attached; DisplayTrajectory một mình không mô tả được mọi sự kiện attach/detach.

Các lệnh sau chạy trong container đang launch:

```bash
docker compose -p rby1-planning -f planning-compose.yml exec planning /planning_entrypoint.sh ros2 run rby1_motion_planning planning_cli.py capabilities
docker compose -p rby1-planning -f planning-compose.yml exec planning /planning_entrypoint.sh ros2 run rby1_motion_planning run_scenarios.py --output /reports --repetitions 3 --fixtures /opt/planning_ws/install/rby1_motion_planning/share/rby1_motion_planning/fixtures/scenarios.json
docker compose -p rby1-planning -f planning-compose.yml exec planning /planning_entrypoint.sh ros2 run rby1_motion_planning planning_cli.py reset_scene
```

## Build trực tiếp trong môi trường ROS đã có vendor

Dependency cho workspace ROS Humble (Ubuntu 22.04):

```bash
sudo apt-get update
sudo apt-get install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-rviz2 libyaml-cpp-dev nlohmann-json3-dev libfmt-dev python3-pytest ros-humble-py-binding-tools ros-humble-launch-testing-ament-cmake
source /opt/ros/humble/setup.bash
source /opt/rby1_ros2_ws/install/setup.bash
cd /tmp
mkdir -p rby1_mtc_ws/src
cd rby1_mtc_ws/src
git clone --recurse-submodules https://github.com/moveit/moveit_task_constructor.git
git -C moveit_task_constructor checkout --detach 756634951326ae17ae099882f7110c6f1d0a98c0
git -C moveit_task_constructor submodule update --init --recursive
cd /tmp/rby1_mtc_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-up-to moveit_task_constructor_core moveit_task_constructor_visualization --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source /tmp/rby1_mtc_ws/install/setup.bash
cd /home/phongday/MyFolder-Linux/mynameisrobot/rby1-docker/planning_ws
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 ros2 launch rby1_motion_planning planning_test.launch.py model:=m moveit_package:=rby1_moveit_m_1_2 rviz:=false
```

`model` và `moveit_package` là launch argument bắt buộc. Launch chỉ chấp nhận tổ hợp đã khảo sát; muốn thêm model khác cần audit và hiệu chỉnh fixture, không suy ra model thật từ Docker simulator hiện hữu. Không dùng `demo.launch.py` vendor với default `use_fake_hardware=false`.
Launch từ chối `ROS_DOMAIN_ID=0` hoặc biến chưa đặt. Domain riêng ngăn joint states fake xuất hiện trong ROS graph của driver và bridge điều khiển cũ. Compose planning đặt domain 83; chọn domain khác chỉ khi đã kiểm tra nó không được dùng bởi robot/driver.

## Test, preview, reset

Trong shell đã source ROS/vendor/MTC/package, cùng domain 83:

```bash
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
colcon test --packages-select rby1_motion_planning --event-handlers console_direct+
colcon test-result --verbose
ros2 run rby1_motion_planning run_scenarios.py --repetitions 3 --fixtures src/rby1_motion_planning/fixtures/scenarios.json --output src/rby1_motion_planning/reports
ros2 run rby1_motion_planning planning_cli.py get_state
ros2 run rby1_motion_planning planning_cli.py get_scene
ros2 run rby1_motion_planning planning_cli.py reset_scene
```

Runner tạo artifact `artifacts.baseline`, `artifacts.goal`, `artifacts.obstacle_scene`, trial results và protocol trace. Dùng goal đã hiệu chỉnh để preview một plan mới:

```bash
python3 - <<'PY'
import json, subprocess
report=json.load(open('src/rby1_motion_planning/reports/scenarios.json'))
params={'frame':report['artifacts']['start']['frame'],'pose':report['artifacts']['goal'],'deadline_s':30}
subprocess.run(['ros2','run','rby1_motion_planning','planning_cli.py','load_scene','--params','{"scene":{"frame":"base","objects":[]}}'],check=True)
plan=json.loads(subprocess.check_output(['ros2','run','rby1_motion_planning','planning_cli.py','plan_to_pose','--params',json.dumps(params)]))
subprocess.run(['ros2','run','rby1_motion_planning','planning_cli.py','preview','--params',json.dumps({'plan_id':plan['result']['plan_id']})],check=True)
PY
```

Để `colcon test` tự dựng fake launch, dừng launch thủ công trước:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build --packages-select rby1_motion_planning --cmake-force-configure --cmake-args -DRBY1_ENABLE_INTEGRATION_TESTS=ON
RBY1_TEST_REPORT_DIR=/tmp/rby1-launch-test-reports RBY1_TEST_REPETITIONS=3 colcon test --packages-select rby1_motion_planning --event-handlers console_direct+
colcon test-result --verbose
```

Launch test dùng port 7451, fixture actual FK/IK/collision và CLI thực. Scenario vật gắn dùng chính mesh RBY1 và một payload box ảo trong scene snapshot: tay trần đi hết đường Cartesian, còn payload va chạm vật cản và đường bị từ chối; scene live giữ nguyên. `virtual_attachment` chỉ là fixture chẩn đoán, trả `grasp_claim=false`, không chứng minh grasp và không tạo plan hoàn tất. C++ tests dùng model tối giản để kiểm chứng thuật toán nội suy/attached collision/detach transform; chúng không chứng minh khả năng grasp của RBY1. Các test grasp RBY1 bị SKIP có lý do nếu model invalid. OMPL không yêu cầu waypoint giống hệt nhau; backend hiện không expose seed, báo `seed_supported=false`, lưu phiên bản/model/start/scene, tỷ lệ thành công và thời gian qua số lượt cấu hình.

## Scene, validation và vòng đời plan

Scene YAML có sàn, bàn, box/cylinder obstacle và target_object; loader apply đồng bộ rồi readback. `load_scene`/`reset_scene` thay toàn bộ objects, bỏ attachment/octomap và trả ACM về SRDF baseline, xác minh cả geometry/pose qua readback, tăng revision và xóa cache plan. Launch dành cho môi trường fake độc lập. Tests hiệu chỉnh scene riêng; không coi default scene là bằng chứng cho mọi goal.

Worker lấy joint state có timestamp/receipt mới tối đa 1.5 giây và timeout 5 giây. Kiểm tra bounds, start collision, frame, IK goal collision trước planning. Robot state và scene snapshot theo từng stage; không attach vật vào scene live trong planning/preview. Pose place tính `world_T_tcp = world_T_place_object * inverse(tcp_T_object)`, dùng thêm `link_T_tcp` cho IK. MTC Cartesian yêu cầu fraction 1.0 và khoảng cách đầy đủ.

OMPL dùng config local `ompl_planning.yaml`: RRTConnect được khai báo rõ cho `right_arm`, `longest_valid_segment_fraction=0.0005`, `maximum_waypoint_distance=0.01`. Bỏ adapter TOTG làm thay đổi hình học path; giữ ResolveConstraintFrames/FixWorkspaceBounds và retime riêng. Không nới collision validator để nhận quỹ đạo lỗi.

Mỗi stage retime bằng MoveIt IPTP với hệ số dự phòng 0.5 của scaling yêu cầu, rồi kiểm tra waypoint và nội suy có maximum joint-variable step 0.02 rad (revolute) / m (prismatic), cấu hình trong `task_parameters.yaml`. Kiểm tra bounds, self/world/attached collision với ACM đúng stage; timestamps tăng; waypoint dynamics, segment-average velocity và finite-difference acceleration. Không công bố bảo đảm collision-free liên tục giữa sample. Scene/event metadata vẫn nằm trong cache ROS và MTC solution; không ép task thành một trajectory làm mất event.

Một planning task active; request mới bị `BUSY`, status/cancel vẫn phục vụ. Deadline `(0,120]` giây, tính từ khi backend nhận request; planner dùng budget hữu hạn. Nếu planner không ngắt ngay, kết quả muộn bị loại, chỉ phát terminal cancelled sau khi worker dừng. Plan lưu ở ROS, TTL mặc định 120 giây, model hash/revision/start/scene signature; preview từ chối TTL/scene/start cũ. Không nhận trajectory tùy ý từ client, không có method execute.

## Contract v1 của Qt

Hai file chuẩn được sao chép nguyên byte từ checkout Qt vào [`planning_protocol/protocol-v1.md`](../../../planning_protocol/protocol-v1.md) và [`contract-v1.json`](../../../planning_protocol/fixtures/contract-v1.json); test kiểm tra SHA-256 để phát hiện lệch schema. Adapter `planning_contract_service.py` mở cổng 8082, nhận `{protocol_version:1,type:"request",request_id,command,payload}`, trả response/event v1 và chuyển đổi sang transport nội bộ 7447. Nó giữ một kết nối backend qua các lần Qt reconnect, cache status 300 giây, không tự hủy task khi Qt rớt kết nối. `get_capabilities → get_task_status → get_scene` cho phép Qt đối chiếu lại. ACK planning không phải kết quả cuối; terminal chỉ được gửi sau khi worker dừng.

Adapter chỉ công bố `baseline` và `pick_place_obstacles`, là hai scenario đã ánh xạ sang scene fake hiện có. Scene snapshot có object/marker/hướng Cartesian từ `test_scene.yaml`; adapter đối chiếu ID, shape, kích thước, frame và pose với readback MoveIt trước khi công bố. `scene_revision` băm revision backend, scene và joint state. `plan_to_pose` chuyển request Qt sang MoveIt worker; terminal thành công chỉ được công bố nếu metadata chứa validation trajectory thật. `plan_pick_place` hiện không nằm trong `supported_commands` vì mimic gripper vendor chưa hợp lệ; request trực tiếp trả `VALIDATION_FAILED`, không dựng kết quả mock. `preview_plan` publish `DisplayTrajectory`/MTC display trong RViz, không execute. Adapter không nhận đường dẫn scene từ client.

Qt dùng `QTcpSocket` tới `127.0.0.1:8082` qua Docker/WSL localhost forwarding. Máy Linux riêng: trên Windows chạy `ssh -L 8082:127.0.0.1:8082 phongday@linux-host`. `request_id` không được dùng lại khi reconnect; event echo ID/command, sequence tăng theo task. Lỗi framing/JSON/UTF-8 đóng kết nối; lỗi có ID hợp lệ trả envelope với code contract. Cổng 8081 vẫn thuộc command bridge cũ.

Các fixture `fixtures/protocol_v1.ndjson`, `run_scenarios.py` và báo cáo ngày 16/09/2026 thuộc transport nội bộ cũ 7447. Chúng không phải bằng chứng tương thích Qt v1; test `test_contract_v1.py` chỉ xác minh mapping/fixture bằng Python. Cần chạy lại build, contract integration và MoveIt fake trên image mới trước khi công bố kết quả v1 đã xác nhận end to end.

## Quyền điều khiển robot

Planning launch chỉ dùng fake hardware, không có action handle gửi command, và không chạy `rby1_driver` hay `rby1_hardware`. Khi ứng dụng Qt kết nối SDK trực tiếp tới robot, không chạy stack điều khiển `docker-compose.yml` hoặc vendor driver trên cùng robot; planning service vẫn chỉ là preview. Domain DDS riêng không phải cơ chế cấp quyền đối với robot thật.

Trước khi triển khai execution, Qt và ROS 2 cần chung **một** điểm cấp quyền nằm trên đường gửi lệnh SDK. Điểm này cấp lease có `owner`, `lease_id`, `epoch`, `expires_at`; mỗi lệnh phải được kiểm tra lease và epoch tại điểm gửi, mất lease phải dừng stream và thu hồi control rights. Handoff cần chờ bên cũ xác nhận dừng, xác nhận robot đứng yên rồi mới cấp epoch mới. Nếu Qt hoặc driver vẫn có đường SDK trực tiếp bỏ qua điểm kiểm tra, không thể bảo đảm độc quyền. Contract v1 hiện không có quyền thực thi; phiên bản contract sau cần thêm trạng thái chủ quyền, acquire/renew/release, epoch và lỗi `CONTROL_NOT_OWNED`/`LEASE_EXPIRED` trước khi thêm execute. Không suy ra quyền thực thi từ `plan_id` hoặc `preview`.

## Workaround shutdown đã kiểm chứng

MoveIt 2.5.9 trong image vendor segfault khi shutdown: thư viện capability bị unload trước khi node hủy callback-group control blocks. Launch thêm `LD_PRELOAD` chỉ cho tiến trình `move_group`, trỏ tới `libmoveit_move_group_default_capabilities.so` trong prefix đã xác minh để giữ code tới lúc process thoát. Đây là workaround lifetime, không sửa thư viện vendor; đường dẫn và lý do ghi vào `launch_audit.json`. Launch test vẫn kiểm tra exit code và đã xác minh shutdown sạch với cấu hình này. Cảnh báo class_loader lúc teardown vẫn có thể xuất hiện; báo cáo ghi riêng cảnh báo này.

## Bằng chứng và giới hạn

Xem `reports/verification.json`, `reports/environment.json`, `reports/scenarios.json`, CSV và JUnit XML. PASS chỉ áp dụng các kiểm tra đã chạy; grasp RBY1 invalid và GUI không sẵn có phải được ghi SKIP/NOT_RUN. Báo cáo không xác nhận robot thật, không xác nhận physics simulator, không xác nhận collision-free liên tục.

Nguồn vendor khảo sát: [demo](https://github.com/RainbowRobotics/rby1-ros2/blob/2493c8dd62dc835d5f3142d9d49ca7a8f8959f1b/rby1_moveit/rby1_moveit_m_1_2/launch/demo.launch.py), [URDF](https://github.com/RainbowRobotics/rby1-ros2/blob/2493c8dd62dc835d5f3142d9d49ca7a8f8959f1b/rby1_description/urdf/rby1m/model_v1_2.urdf), [SRDF](https://github.com/RainbowRobotics/rby1-ros2/blob/2493c8dd62dc835d5f3142d9d49ca7a8f8959f1b/rby1_moveit/rby1_moveit_m_1_2/config/RBY1_M_v1_2.srdf). API MTC đối chiếu với [Humble documentation](https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html) và headers của commit pin; không dùng tên gripper tutorial.
