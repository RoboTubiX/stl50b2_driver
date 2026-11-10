# STL-50B2 ROS 2 Driver

Package C++17 cho ROS 2 Jazzy, đọc LiDAR STL-50B2 qua UART và xuất `sensor_msgs/msg/LaserScan` trên topic `/scan`.

Node tự gửi lệnh chạy motor khi khởi động và gửi lệnh dừng khi được hủy trong quá trình thoát bình thường.

## 1. Yêu cầu

- Linux và ROS 2 Jazzy đã được cài đặt.
- Công cụ build `colcon`, `ament_cmake`; dependency `rclcpp`, `sensor_msgs`, `launch`, `launch_ros`, `ament_index_python`.
- RViz2 để xem dữ liệu.
- USB–UART tương thích với thiết bị; mẫu đã kiểm tra có VID:PID `10c4:a001`, serial `lidar_01`, driver `cp210x`.

Các lệnh dưới đây giả định package nằm tại `~/ros2_ws/src/stl50b2_driver` và sử dụng Bash.

## 2. Kết nối phần cứng

Theo tài liệu giao tiếp STL-50B2 đi kèm workspace:

| Chân LiDAR | Kết nối |
| --- | --- |
| VCC | Nguồn 5 V phù hợp với thiết bị |
| GND | Mass chung với USB–UART |
| TX | RX của USB–UART |
| RX | TX của USB–UART |
| TIME | Nối GND để sử dụng chế độ điều tốc nội bộ mặc định |

UART dùng mức logic **TTL 3,3 V**, tốc độ **230400 baud, 8N1**, không dùng flow control. Nguồn cấp 5 V không có nghĩa là các chân UART chịu mức logic 5 V. Nếu dùng bộ cáp LiDAR tích hợp, đối chiếu pinout của bộ cáp trước khi đấu nối.

Baud rate hiện được đặt cố định trong code, không phải tham số ROS.

## 3. Cấu trúc package

```text
stl50b2_driver/
├── include/stl50b2_driver/stl50b2_node.hpp
├── src/
│   ├── stl50b2_node.cpp
│   └── main.cpp
├── config/stl50b2.yaml
├── launch/stl50b2.launch.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

- `stl50b2_node.hpp`: khai báo class, hàm và trạng thái của driver.
- `stl50b2_node.cpp`: mở serial, điều khiển motor, tách gói, kiểm tra CRC và ghép vòng quét.
- `main.cpp`: khởi tạo ROS, tạo node, chạy executor và xử lý ngoại lệ.
- `stl50b2.yaml`: cấu hình sử dụng khi chạy launch.
- `stl50b2.launch.py`: chạy executable `stl50b2_node` với file cấu hình.

## 4. Thiết lập USB và quyền truy cập

### Kiểm tra thiết bị

```bash
lsusb
lsusb -t
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB*
```

Nếu `lsusb` thấy `10c4:a001` nhưng chưa có cổng serial, thử:

```bash
sudo modprobe cp210x
echo '10c4 a001' | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
ls -l /dev/ttyUSB*
```

Khai báo `new_id` bằng tay chỉ có hiệu lực trong phiên module hiện tại. Chỉ dùng mã USB này cho thiết bị đã xác nhận là bộ chuyển CP210x tương ứng.

### Cấp quyền cho tài khoản

```bash
sudo usermod -aG dialout "$USER"
```

Đăng xuất tài khoản Ubuntu rồi đăng nhập lại. Kiểm tra:

```bash
id -nG
```

Danh sách cần có `dialout`. Có thể thử ngay trong shell mới bằng `newgrp dialout`, sau đó source lại môi trường ROS. Không cần chạy node bằng `sudo`.

### Tự nhận driver và tạo cổng cố định

Nếu máy đã được cấu hình và có `/dev/stl50b2`, bỏ qua bước tạo quy tắc.

```bash
sudo nano /etc/udev/rules.d/99-stl50b2.rules
```

Dán các quy tắc sau, mỗi quy tắc trên một dòng riêng:

```udev
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="10c4", ATTR{idProduct}=="a001", RUN+="/sbin/modprobe cp210x"
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="10c4", ATTR{idProduct}=="a001", RUN+="/bin/sh -c 'echo 10c4 a001 > /sys/bus/usb-serial/drivers/cp210x/new_id'"
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="a001", ATTRS{serial}=="lidar_01", SYMLINK+="stl50b2", GROUP="dialout", MODE="0660"
```

Quy tắc này dành cho thiết bị đã kiểm tra có serial `lidar_01`. Với thiết bị khác, cần thay thuộc tính nhận diện tương ứng; tên liên kết phải riêng nếu dùng nhiều LiDAR.

```bash
sudo udevadm control --reload-rules
```

Rút USB rồi cắm lại, chờ vài giây:

```bash
ls -l /dev/stl50b2
```

Kết quả mong đợi là liên kết tới cổng thực, ví dụ `/dev/stl50b2 -> ttyUSB0`. Kiểm tra lại sau reboot để xác nhận cấu hình tự nhận đã hoạt động.

### Vô hiệu hóa hoặc gỡ cấu hình USB

Nếu chỉ đổi sang package ROS khác nhưng vẫn dùng cùng LiDAR, thường không cần gỡ rules: package mới có thể mở `/dev/stl50b2`. Dừng node cũ trước khi chạy node mới để tránh hai chương trình cùng sử dụng cổng.

Nếu muốn chuyển sang driver USB khác hoặc ngừng sử dụng cấu hình này, dừng node bằng `Ctrl+C` và rút USB LiDAR trước.

Để tạm vô hiệu hóa, đổi đuôi file để udev không đọc nó:

```bash
sudo mv /etc/udev/rules.d/99-stl50b2.rules \
  /etc/udev/rules.d/99-stl50b2.rules.disabled
sudo udevadm control --reload-rules
```

Để gỡ hẳn thay vì giữ bản sao, xóa file quy tắc đang tồn tại và nạp lại rules:

```bash
sudo rm /etc/udev/rules.d/99-stl50b2.rules
sudo udevadm control --reload-rules
```

Hai cách trên là lựa chọn thay thế nhau. Nếu đã đổi tên thành `.rules.disabled`, file cấu hình đã ngừng hoạt động; muốn xóa bản sao thì xóa chính file `.rules.disabled`.

Việc nạp lại rules không xóa mã USB đã thêm bằng `new_id` trong phiên module hiện tại và không tự tháo driver khỏi thiết bị đang kết nối. Khi LiDAR đã được rút, có thể xóa đăng ký động bằng:

```bash
echo '10c4 a001' | sudo tee \
  /sys/bus/usb-serial/drivers/cp210x/remove_id
```

Nếu báo không tìm thấy file hoặc mã chưa được đăng ký, kiểm tra lại trạng thái module; không cần thêm `new_id` chỉ để xóa. Một cách khác là khởi động lại máy sau khi vô hiệu hóa hoặc xóa rules. Khi đó đăng ký động từ phiên trước cũng hết hiệu lực.

Các bước này không gỡ module `cp210x` khỏi hệ thống, không thay đổi firmware của LiDAR và không xóa quyền nhóm `dialout`. Đăng ký `new_id` áp dụng theo VID:PID, nên việc xóa nó cũng ảnh hưởng khả năng tự gắn driver cho các thiết bị khác có cùng `10c4:a001` trong phiên hiện tại. Để dùng driver USB khác, cần thực hiện thêm cấu hình riêng của driver đó.

### Bật lại rules đã vô hiệu hóa

Nếu vẫn giữ file `.rules.disabled`:

```bash
sudo mv /etc/udev/rules.d/99-stl50b2.rules.disabled \
  /etc/udev/rules.d/99-stl50b2.rules
sudo udevadm control --reload-rules
```

Nếu đã xóa file, tạo lại theo hướng dẫn phía trên. Cắm lại USB rồi kiểm tra:

```bash
ls -l /dev/stl50b2
```

Sau khi gỡ rules, nếu dùng cổng khác, cập nhật `port` trong YAML hoặc truyền `-p port:=...` khi chạy trực tiếp. Với launch mặc định, build lại sau khi sửa YAML nguồn để cập nhật cấu hình được cài đặt.

## 5. Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select stl50b2_driver
source ~/ros2_ws/install/setup.bash
```

Sau khi sửa file YAML hoặc launch trong thư mục nguồn, build lại để cập nhật bản được cài đặt trong workspace.

## 6. Chạy và dừng

### Chạy bằng launch (khuyến nghị)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch stl50b2_driver stl50b2.launch.py
```

Launch mặc định đọc `config/stl50b2.yaml` trong package đã cài đặt và dùng `/dev/stl50b2`.

Dùng một file YAML khác, chỉ định đường dẫn tuyệt đối:

```bash
ros2 launch stl50b2_driver stl50b2.launch.py \
  params_file:="$HOME/ros2_ws/src/stl50b2_driver/config/stl50b2.yaml"
```

### Chạy trực tiếp

```bash
ros2 run stl50b2_driver stl50b2_node \
  --ros-args -p port:=/dev/stl50b2
```

`ros2 run` không tự đọc YAML. Nếu không truyền `port`, giá trị mặc định trong code là `/dev/ttyUSB0`.

Chạy trực tiếp với YAML:

```bash
ros2 run stl50b2_driver stl50b2_node --ros-args \
  --params-file "$HOME/ros2_ws/src/stl50b2_driver/config/stl50b2.yaml"
```

Chỉ chạy một instance driver trên một cổng. Đóng các phần mềm khác đang dùng cùng cổng serial.

### Dừng

Nhấn `Ctrl+C` một lần. Khi node được hủy bình thường, driver gửi lệnh dừng motor rồi đóng serial. Log mong đợi:

```text
Stop command sent
```

Log chỉ xác nhận đã ghi đủ lệnh vào cổng, chưa phải xác nhận từ LiDAR; kiểm tra motor thực sự dừng. `kill -9`, crash hoặc mất kết nối không bảo đảm thực hiện được bước dừng.

## 7. Tham số

| Tham số | Mặc định trong code | YAML đi kèm | Ý nghĩa |
| --- | --- | --- | --- |
| `port` | `/dev/ttyUSB0` | `/dev/stl50b2` | Cổng serial |
| `frame_id` | `laser` | `laser` | Frame của bản tin scan |
| `clockwise` | `false` | `false` | Khi `true`, đổi dấu góc thô trước khi cộng góc lệch |
| `angle_offset_deg` | `0.0` | `0.0` | Góc lệch cộng thêm, đơn vị độ |
| `range_min` | `0.05` | `0.05` | Ngưỡng khoảng cách nhỏ nhất, mét |
| `range_max` | `12.0` | `12.0` | Ngưỡng khoảng cách lớn nhất, mét |

Các ngưỡng khoảng cách là cấu hình lọc, không phải cam kết dải đo phần cứng. Cần `range_min >= 0` và `range_max > range_min`; dùng giá trị hữu hạn.

Ví dụ đổi chiều và hướng quét khi khởi động:

```bash
ros2 run stl50b2_driver stl50b2_node --ros-args \
  -p port:=/dev/stl50b2 \
  -p clockwise:=true \
  -p angle_offset_deg:=90.0
```

Driver chưa có callback áp dụng lại cấu hình động cho các giá trị đã đọc lúc khởi tạo. Sửa cấu hình rồi khởi động lại node.

## 8. Topic và cách ghép scan

- Topic mặc định: `/scan` (tên tương đối `scan` trong node).
- Kiểu bản tin: `sensor_msgs/msg/LaserScan`.
- QoS: `rclcpp::SensorDataQoS()`, sử dụng Best Effort.
- Mỗi vòng có 720 ô góc, cách nhau 0,5 độ.
- Khoảng cách xuất theo mét; ô thiếu điểm là `+inf`.
- Nhiều điểm vào cùng ô: giữ khoảng cách gần nhất và cường độ tương ứng.
- Bỏ phần vòng chưa đầy đủ lúc mới chạy; chỉ bắt đầu thu từ lần góc đi qua 360° về 0°.

Luồng xử lý: UART → tìm header `54 2C` → lấy 47 byte → CRC8 đa thức `0x4D` → giải mã 12 điểm → ghép vòng → publish.

## 9. Hiển thị trong RViz2

Mở terminal khác:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```

1. Đặt **Global Options → Fixed Frame** thành `laser` (hoặc giá trị `frame_id` đã cấu hình).
2. Chọn **Add → LaserScan**.
3. Chọn **Topic** là `/scan`.
4. Đặt **Reliability Policy** thành **Best Effort**; mở rộng mục Topic nếu tùy chọn đang ẩn.

Đặt vật ở phía trước, rồi chuyển sang trái/phải để kiểm tra hướng. Nếu hình bị phản chiếu trái–phải, thử đổi `clockwise`. Nếu hình chỉ bị xoay, chỉnh `angle_offset_deg`. Đo khoảng cách từ tâm LiDAR để đối chiếu.

## 10. Kiểm tra hoạt động

Trong terminal đã source workspace:

```bash
ros2 node list
ros2 topic list
ros2 topic info /scan --verbose
ros2 topic echo /scan --once --qos-reliability best_effort
```

Kiểm tra log driver: số ô hợp lệ có dữ liệu, tần số vòng quét ổn định và bộ đếm `CRC errors` không tăng liên tục. Tần số thực tế phụ thuộc tốc độ motor, không cố định bởi timer đọc serial.

## 11. Xử lý lỗi

| Hiện tượng | Cách kiểm tra / xử lý |
| --- | --- |
| `ros2: command not found` | Source `/opt/ros/jazzy/setup.bash` |
| Không tìm thấy package / executable | Kiểm tra build thành công và source `~/ros2_ws/install/setup.bash` |
| `No such file or directory` khi mở serial | Kiểm tra `lsusb`, cổng thực và liên kết `/dev/stl50b2` |
| USB xuất hiện nhưng `Driver=[none]` | Kiểm tra `cp210x`, VID:PID và quy tắc `new_id` |
| `Permission denied` | Kiểm tra nhóm `dialout`, đăng xuất/đăng nhập sau khi thêm nhóm |
| RViz báo `RELIABILITY_QOS_POLICY` | Đổi LaserScan sang Best Effort |
| RViz không có scan | Kiểm tra `/scan`, Fixed Frame, QoS và log driver |
| Hình sai hướng | Đối chiếu vật thực, chỉnh `clockwise` và `angle_offset_deg` |
| CRC lỗi tăng liên tục | Kiểm tra nguồn, dây, mass chung, mức logic và phần mềm khác đang mở cùng cổng |
| Rút/cắm lại USB nhưng scan không trở lại | Dừng rồi khởi động lại node; chưa có tự kết nối lại |

Các dòng `Stereo is NOT SUPPORTED` của RViz không phải lỗi QoS hoặc lỗi dữ liệu LiDAR.

Log USB và udev:

```bash
sudo dmesg | tail -n 40
sudo journalctl -b -u systemd-udevd --no-pager -n 40
```

## 12. Giới hạn hiện tại và tích hợp robot

- Timestamp của scan lấy theo thời gian máy nhận tại ranh giới vòng, chưa đồng bộ với đồng hồ phần cứng.
- `scan_time` được đo bằng đồng hồ monotonic trên máy nhận; có thể chịu ảnh hưởng của việc gom dữ liệu serial.
- `time_increment = 0`: chưa cung cấp thời gian từng tia và chưa bù chuyển động khi robot chạy.
- Chưa tự kết nối lại khi mất USB; lỗi đọc serial sẽ hủy timer đọc.
- Chưa tự xuất TF. Khi tích hợp robot, cần TF `base_link → laser` đúng vị trí và góc lắp thực tế, thường khai báo trong URDF.
- Kiểm tra trục và chiều góc trước khi dùng SLAM/Nav2. Cấu hình RViz với Fixed Frame `laser` chỉ đủ để xem cảm biến độc lập.
- Code hiện tại còn hai dòng tạo timer giống nhau trong constructor. Khi chỉnh code, chỉ giữ một dòng `timer_ = create_wall_timer(...)`.

Các thử nghiệm trong quá trình phát triển đã xác nhận nhận gói CRC hợp lệ, hiển thị RViz, dừng motor và nhận USB sau reboot trên thiết bị của người dùng. Điều này chưa thay thế kiểm thử chuyển động hoặc kiểm định toàn bộ dải đo.

## 13. Tài liệu tham khảo

- Tài liệu giao tiếp `STL-50B2串口数据解析.pdf` tại gốc workspace. Một số mục trong PDF thiếu hoặc khác dữ liệu thiết bị, đặc biệt mô tả timestamp; driver hiện chưa sử dụng timestamp phần cứng.
- [udev: quy tắc và liên kết thiết bị](https://www.freedesktop.org/software/systemd/man/latest/udev.html).
- [Linux USB sysfs ABI: cơ chế new_id](https://github.com/torvalds/linux/blob/master/Documentation/ABI/testing/sysfs-bus-usb).
