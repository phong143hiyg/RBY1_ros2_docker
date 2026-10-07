# Qt-direct deployment audit

Audit date: 2026-10-05 (Asia/Bangkok).

## Kết quả

- Repository chỉ còn một runtime: ứng dụng Qt liên kết trực tiếp RBY1 C++ SDK
  và kết nối gRPC tới một simulator hoặc một robot.
- Compose mặc định chỉ khai báo service `rby1-sim`; không còn bridge, driver,
  web controller hay planning service.
- Các Dockerfile, workspace, patch, entrypoint, compose và tài liệu chỉ phục vụ
  kiến trúc cũ đã được loại bỏ.
- `Dockerfile.sdk-dev` khóa SDK tag `v0.10.0` và commit đầy đủ
  `9af8a734b7bef0167545e3d9f0d559276a5b64ee`.
- `rby1-readonly-smoke` chỉ đọc robot info/state, không bật nguồn, servo hoặc gửi
  lệnh chuyển động.

## Giới hạn xác minh

Các kiểm tra tĩnh xác nhận cấu trúc compose, shell syntax, liên kết nội bộ và
cấu trúc XML của sơ đồ. Môi trường hiện tại không có Docker daemon khả dụng nên
chưa build image, khởi động simulator hoặc chạy smoke test tích hợp.

Trên máy có Docker, chạy các lệnh trong README. Một probe thành công sẽ in
`READ_ONLY_SMOKE_OK`.

## License

- RBY1 SDK `v0.10.0` phát hành theo Apache License 2.0; các submodule có thể có
  license riêng.
- Trang Docker Hub của simulator chưa nêu rõ license/NOTICE cho image. Cần xác
  nhận với Rainbow Robotics trước khi phân phối lại.
