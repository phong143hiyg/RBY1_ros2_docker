# Câu 7: Tích hợp framework tính toán quỹ đạo

## 1. Có thể tích hợp một framework tính toán quỹ đạo vào hệ thống không?

Có. Framework phù hợp để tích hợp vào hệ thống là **MoveIt 2** vì project đang sử dụng ROS 2 Humble và image Docker đã cài sẵn các package `ros-humble-moveit`, `moveit-visual-tools`, `ros2-control` và `joint-trajectory-controller`.

Quy trình tích hợp dự kiến gồm:

1. Sử dụng mô hình URDF của RB-Y1.
2. Tạo SRDF để khai báo các nhóm lập kế hoạch như thân, tay trái, tay phải và đầu.
3. Cấu hình inverse kinematics, giới hạn khớp, ma trận tự va chạm và thuật toán lập kế hoạch.
4. Nhận trạng thái hiện tại của robot từ `/joint_states`.
5. Sử dụng MoveIt 2 để tính toán quỹ đạo thỏa mãn giới hạn khớp và tránh va chạm.
6. Chuyển quỹ đạo đầu ra sang `Rby1JointCommand` hoặc `FollowJointTrajectory` để ROS 2 Driver thực thi.
7. Kiểm thử trên MuJoCo Simulator trước khi chạy trên robot thật.

Tuy nhiên, cần phân biệt giữa **đã cài đặt** và **đã tích hợp hoàn chỉnh**. Trong project hiện tại, MoveIt 2 mới được cài trong Docker image nhưng chưa có package cấu hình SRDF, planning group, kinematics và controller dành riêng cho RB-Y1. Vì vậy chưa thể nói rằng project đang sử dụng MoveIt 2 để lập kế hoạch quỹ đạo.

Quỹ đạo tròn hiện tại chỉ là điều khiển vận tốc open-loop. Đối với chuyển động khớp, project gửi vị trí đích và thời gian thực hiện qua ROS 2 Action; quá trình nội suy và điều khiển mức thấp vẫn thuộc ROS 2 Driver và RBY1 SDK.

## 2. Vì sao lựa chọn MoveIt 2?

MoveIt 2 được lựa chọn vì các lý do sau:

- Hệ thống hiện tại được xây dựng trên ROS 2 Humble nên MoveIt 2 có thể tích hợp trực tiếp với topic, service, action, URDF, TF và RViz.
- Framework hỗ trợ inverse kinematics, kiểm tra giới hạn khớp, tự va chạm, va chạm với môi trường và tham số hóa thời gian cho quỹ đạo.
- MoveIt 2 phù hợp với robot nhiều bậc tự do như RB-Y1, đặc biệt khi cần điều phối thân và hai cánh tay.
- Kiến trúc plugin cho phép sử dụng nhiều planner như OMPL, Pilz hoặc CHOMP mà không phải thay đổi toàn bộ hệ thống.
- Planning Scene và RViz giúp quan sát, kiểm tra quỹ đạo trước khi gửi xuống robot.
- Đây là framework phổ biến trong hệ sinh thái ROS nên có tài liệu, ví dụ và cộng đồng hỗ trợ tương đối đầy đủ.
- Các dependency chính đã có trong Docker image của project, giúp giảm khối lượng cài đặt ban đầu.

Về kiến trúc, MoveIt 2 sẽ đảm nhiệm **lập kế hoạch chuyển động cấp cao**, còn RBY1 SDK và ROS 2 Driver tiếp tục đảm nhiệm **thực thi quỹ đạo và điều khiển cấp thấp**.

## 3. Ưu và nhược điểm của MoveIt 2 so với các framework khác

### 3.1. Ưu điểm

- Tích hợp với ROS 2 thuận tiện hơn các framework độc lập như Drake hoặc giải pháp tự xây dựng.
- Không phải tự cài đặt lại inverse kinematics, collision checking, planning scene và time parameterization.
- Hỗ trợ nhiều thuật toán lập kế hoạch thông qua cơ chế plugin.
- Có thể sử dụng cùng kiến trúc điều khiển cho simulator và robot thật.
- Hỗ trợ trực quan hóa và kiểm tra bằng RViz.
- Có thể mở rộng để nhận dữ liệu cảm biến 3D và cập nhật vật cản trong môi trường.
- So với việc chỉ gọi trực tiếp RBY1 SDK, MoveIt 2 có khả năng lập kế hoạch và tránh va chạm ở mức cao hơn.

### 3.2. Nhược điểm

- Việc cấu hình URDF, SRDF, controller, TF và planning group cho robot nhiều khớp tương đối phức tạp.
- Lập kế hoạch cho nhiều nhóm khớp đồng thời có thể tiêu tốn nhiều CPU và thời gian.
- Các planner lấy mẫu như OMPL có thể tạo kết quả khác nhau giữa các lần chạy và đôi khi không tìm được quỹ đạo trong thời gian giới hạn.
- MoveIt 2 không phải bộ điều khiển hard real-time; hệ thống vẫn cần driver hoặc `ros2_control` để thực thi quỹ đạo an toàn.
- Driver RB-Y1 hiện sử dụng custom action `Rby1JointCommand`, vì vậy có thể cần viết adapter để chuyển quỹ đạo từ MoveIt 2 sang giao diện của driver.
- So với Tesseract/TrajOpt, MoveIt 2 thuận tiện hơn khi tích hợp ROS nhưng có thể kém phù hợp hơn với một số bài toán tối ưu hóa quỹ đạo công nghiệp phức tạp.
- So với việc gọi trực tiếp RBY1 SDK, MoveIt 2 có kiến trúc nặng hơn và cần nhiều bước cấu hình hơn.

## 4. Phân biệt MoveIt 2 và Nav2

MoveIt 2 chủ yếu phù hợp với **quỹ đạo khớp, cánh tay và phần thân trên**. Nếu yêu cầu là tìm đường tự động cho đế di động trong bản đồ, tránh chướng ngại vật và định vị robot thì framework phù hợp hơn là **Nav2**.

Hai framework có thể kết hợp trong tương lai:

```text
Nav2     → lập kế hoạch và điều khiển đế di động
MoveIt 2 → lập kế hoạch chuyển động thân và cánh tay
Driver   → thực thi lệnh trên robot hoặc simulator
```

## 5. Câu trả lời ngắn gọn khi bảo vệ

> Có thể tích hợp MoveIt 2 vào hệ thống vì project đang sử dụng ROS 2 Humble và đã cài sẵn các dependency cần thiết. Em lựa chọn MoveIt 2 vì framework này hỗ trợ inverse kinematics, kiểm tra va chạm, giới hạn khớp và nhiều thuật toán lập kế hoạch thông qua plugin. So với giải pháp tự xây dựng hoặc gọi trực tiếp RBY1 SDK, MoveIt 2 có nhiều chức năng lập kế hoạch cấp cao và tích hợp ROS tốt hơn, nhưng cấu hình phức tạp, tiêu tốn tài nguyên và không phải bộ điều khiển hard real-time. Trong phạm vi project hiện tại, MoveIt 2 mới được cài đặt chứ chưa được cấu hình làm bộ lập kế hoạch chính; việc nội suy và thực thi chuyển động vẫn do ROS 2 Driver và RBY1 SDK đảm nhiệm.

## Tài liệu tham khảo

- [MoveIt 2 — Motion Planning](https://moveit.picknik.ai/main/doc/concepts/motion_planning.html)
- [MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)
- [MoveIt 2 — URDF và SRDF](https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html)
