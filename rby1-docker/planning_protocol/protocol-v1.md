# Planning NDJSON v1 — Qt / ROS 2 contract

Đặc tả gốc: [prompts-test-moveit2-rby1.md](../question/prompts-test-moveit2-rby1.md), phần “Đặc tả chung”. Đây là service **mới**, không phải tính năng đã tồn tại của App Bridge. Implementation ROS 2 phải dùng cùng contract; không đưa ROS/MoveIt vào build Qt.

## Transport và envelope

Socket planning riêng, default localhost:8082; robot command legacy vẫn ở 8081. UTF-8 NDJSON, một JSON object/dòng, tối đa 1,048,576 byte trước LF. TCP có thể fragment/coalesce; CRLF được chấp nhận. Frame JSON lỗi, vượt giới hạn hoặc không phải UTF-8/object phải được từ chối. Qt đóng kết nối planning khi parser lỗi; không biến frame lỗi thành ACK. Parser robot cũ giữ default không giới hạn để không đổi wire contract cũ.

```json
{"protocol_version":1,"type":"request","request_id":"session-1","command":"plan_pick_place","payload":{}}
{"protocol_version":1,"type":"response","request_id":"session-1","command":"plan_pick_place","ok":true,"payload":{}}
{"protocol_version":1,"type":"event","request_id":"session-1","command":"plan_pick_place","event_seq":1,"status":"planning","stage":"approach","payload":{"progress":0.3}}
```

Request ID là string duy nhất trong phiên (Qt dùng UUID phiên + counter; không reset khi reconnect). Response/event phải echo ID và command. `ok` là boolean. Query trả kết quả trong response; planning response chỉ ACK. Mỗi task đã ACK có đúng một event terminal `succeeded|failed|cancelled`; event progress dùng `planning`. `event_seq` nguyên, tăng nghiêm ngặt theo task, trong [1, 2^53−1]; snapshot status có thể dùng 0 trước event đầu. Qt bỏ ID/command sai, event trùng/lùi và mọi event sau terminal. Terminal đến trước ACK vẫn được nhận, ACK muộn không đổi kết quả. `payload.progress` trong [0,1] là tùy chọn cho progress.

Lỗi ở cấp envelope: `error:{code,message,stage,details}` với `ok=false` hoặc event `failed`. Code: `INVALID_REQUEST`, `UNSUPPORTED_COMMAND`, `BUSY`, `TF_UNAVAILABLE`, `STATE_UNAVAILABLE`, `START_IN_COLLISION`, `GOAL_IN_COLLISION`, `NO_IK`, `PLANNING_FAILED`, `CARTESIAN_INCOMPLETE`, `VALIDATION_FAILED`, `TIMEOUT`, `STALE_PLAN`. Chỉ dùng `NO_IK` khi backend có bằng chứng phân biệt nguyên nhân. Mock luôn ghi `details.simulated=true`; lỗi scenario chỉ mô phỏng.

## Commands

| Command | Payload | Kết quả |
|---|---|---|
| `get_capabilities` | `{}` | Capability bên dưới |
| `load_test_scene` | `{scenario_id}` | Snapshot scene mới; không nhận đường dẫn |
| `get_scene` | `{}` | Snapshot scene hiện tại |
| `plan_to_pose` | `scene_revision,group,tcp_frame,goal_tcp_pose,planning_timeout_s,velocity_scale,acceleration_scale` | ACK + events |
| `plan_pick_place` | `scene_revision,group,tcp_frame,object_id,pick_tcp_pose,place_object_pose,approach_distance_m,lift_distance_m,retreat_distance_m,planning_timeout_s,velocity_scale,acceleration_scale` | ACK + events |
| `preview_plan` | `{plan_id}` | Metadata preview RViz trên snapshot; không execute |
| `get_task_status` | `{target_request_id}` | Snapshot task bên dưới |
| `cancel_planning` | `{target_request_id}` | ACK riêng; terminal task gốc xác nhận worker dừng |

Không có `execute_plan`; không stream waypoint tới Qt/driver. Một worker active toàn service; plan thứ hai và load scene trả `BUSY`. Query/status/cancel không chặn worker hoặc event loop.

## Capability và scene snapshot

Các tên trường cụ thể hóa đặc tả chung, xem [JSON fixture dùng chung](fixtures/contract-v1.json). Chia sẻ file này với repository ROS 2; mọi thay đổi schema phải được thống nhất hai phía.

Capability: `backend_mode` (`mock` hoặc `fake_hardware` ở mốc này), `model`, `model_version`, `robot_model_id`, `planning_frame`, `groups:[string]`, `tcp_mappings:[{group,tcp_frame,link}]`, `supported_commands:[string]`, `scenarios:[string]`, `execution_enabled:false`, `max_frame_bytes`, `suggested_planning_timeout_s`, `max_planning_timeout_s`, `plan_ttl_s`, `task_status_ttl_s`. Model ID thật phải chứa version và dấu vết/hash URDF/SRDF/config. Qt từ chối capability thiếu trường thiết yếu hoặc execution enabled, chỉ bật command được công bố. Timeout backend tối đa Qt chấp nhận: 3600 s.

Scenarios: `pick_place_obstacles`, `baseline`, `goal_in_collision`, `unreachable_goal`, `start_in_collision`, `attached_object_clearance`.

Scene: `schema_version:1`, `units:{length:"m",angle:"rad",quaternion:"xyzw"}`, `scene_revision` string opaque, `robot_model_id`, `frame_id`, `group`, `tcp_frame`, `objects`, `pick_tcp_pose`, `place_object_pose`, `markers`, `cartesian_directions`, `defaults`. Revision thay đổi khi geometry/pose/start state/config ảnh hưởng plan thay đổi. Qt không suy diễn revision theo thứ tự chữ/số; response scene/model được áp dụng theo thứ tự request, bỏ snapshot của query cũ về muộn.

Object: `{id,role,geometry,dimensions_m,pose}`. Role: `support|obstacle|target`; geometry box có [x,y,z], cylinder có [height,radius], tất cả mét, kích thước dương. Object IDs: `floor`, `table`, `obstacle_box`, `obstacle_column`, `target_object` (baseline có thể bỏ obstacles). Marker `{label,pose}` không phải collision object.

Pose: `{frame_id,position:[x,y,z],orientation_xyzw:[x,y,z,w]}`. Số hữu hạn, norm quaternion=1 (tolerance 0.001; không tự normalize). Không mặc định frame/world/TCP/model hoặc tọa độ an toàn. `pick_tcp_pose` khác tâm vật; `place_object_pose` là đích vật sau thả. Backend suy ra `T_world_tcp_place = T_world_object_place * inverse(T_tcp_object)` và TCP–link offset đã xác minh.

`cartesian_directions:{approach,lift,lower,retreat}`; mỗi hướng `{frame_id,vector:[x,y,z]}` unit vector. Các hướng thuộc snapshot/revision; request dùng snapshot này, không ghi đè bằng hướng hard-code trong Qt. `defaults` có các khoảng cách, timeout và scaling như payload plan. Khoảng cách ≥0; scaling trong (0,1]; timeout hữu hạn, >0 và ≤ capability. Thiếu defaults để ô trống; không có giá trị hợp lệ giả định.

Mốc đầu một tay, ưu tiên `right_arm` nếu có, base/torso cố định. Qt chỉ chấp nhận scene `right_arm|left_arm`; ROS chọn scene tương ứng model đã xác minh. Scene/start state thực phải hiệu chỉnh qua TF/joint state/IK/start collision, lưu fixture ROS với model/config/start state/seed; JSON mock minh họa không thay thế fixture planning thật.

## Terminal result, status và cancellation

`succeeded.payload` bắt buộc: `plan_id`, `scene_revision`, `robot_model_id`, `group`, `frame`, `tcp_frame`, `planning_time_s`, `duration_s`, `waypoint_count`, `joint_names`, `stages`, `validation`. Stages lưu metadata attach/detach/gripper/scene diffs phía ROS; Qt không ghép hoặc thực thi trajectory.

Validation thật: `simulated:false`, `joint_limits:"passed"`, `collision:"passed"`, `timing:"passed"`; thêm độ phân giải nội suy, giới hạn và phạm vi kiểm tra. Collision samples không chứng minh collision-free liên tục. Result không khớp snapshot/model/group/frame/TCP hoặc validation thiếu/sai bị Qt từ chối.

Validation mock: `simulated:true`, cả ba trường kiểm tra `"not_checked"`, message ghi không tính IK/collision/retiming/trajectory MoveIt. Duration/waypoint/joint/stages của mock chỉ metadata mô phỏng, UI có nhãn MOCK nổi bật. Preview mock trả `rviz_displayed:false,simulated:true`; không tuyên bố mở RViz.

`get_task_status.response.payload` là snapshot theo cấu trúc event: `request_id` task gốc, `command`, `event_seq`, `status`, `stage`, `payload` kết quả cuối nếu có, `error` nếu failed. Có thể giữ nguyên `protocol_version/type:"event"` trong snapshot như fixture/mock; Qt phân biệt response bên ngoài. Snapshot ongoing cùng sequence được dùng xác nhận task còn chạy; sequence thấp hơn không mở khóa thao tác.

ACK cancel không kết thúc task. Worker phải có deadline hữu hạn, chỉ emit cancelled khi đã dừng; không publish/lưu plan muộn sau khi nhận cancel. Qt giữ Cancelling đến terminal. Nếu server trả succeeded trong cancel race, Qt kết thúc với Failed, bỏ plan và ghi chưa xác nhận cancelled. Timeout/disconnect không đồng nghĩa hủy: Qt giữ ID, khóa plan/load/preview; event/ACK muộn sau timeout không tự mở khóa. Nút Đối chiếu status thực hiện lại handshake.

Reconnect: `get_capabilities → get_task_status` (nếu từng có task) `→ get_scene`; hoàn tất đối chiếu mới mở thao tác. Không tự gửi lại planning. Unknown/expired task (`STATE_UNAVAILABLE`) giữ trạng thái chưa đồng bộ; không đoán đã cancelled. Scene/model/frame/TCP thay đổi hoặc plan TTL hết làm plan mất hiệu lực; preview backend cũng kiểm tra TTL/revision/model/start snapshot. Qt polling capability/scene/status mỗi 2 s khi đồng bộ.

Plan ROS lưu theo TTL cùng start state, model ID, scene revision và snapshot/diffs nhất quán. Task status giữ theo TTL để reconnect. Preview không thay đổi scene live thành final state giả. Fake hardware/attach-detach kiểm chứng hình học/chuyển động; chưa chứng minh lực gắp, ma sát hoặc chống trượt/rơi.
