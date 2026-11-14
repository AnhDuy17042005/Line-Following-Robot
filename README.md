# Major Project

## Đọc UART

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200
```

## Chọn firmware để nạp

Mỗi preset có thư mục build riêng; mã trong `Core/Robot/` không cần sửa.

| Preset | Firmware |
| --- | --- |
| `robot` | Chương trình robot |
| `test-sensor` | Test color và line sensor |

Trong VS Code, chạy **CMake: Select Configure Preset** và chọn `robot`
(hoặc `test-sensor` khi kiểm tra cảm biến), sau đó chạy **CMake: Configure**.
Thông báo "No configure preset is active" nghĩa là chưa chọn preset trong VS Code;
việc chạy `cmake --preset` ở terminal không chọn preset cho extension.

Nếu terminal chưa tìm thấy Ninja hoặc `arm-none-eabi-*`, thêm bộ công cụ STM32 vào PATH:

```bash
export PATH="$HOME/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:$HOME/.local/share/stm32cube/bundles/ninja/1.13.2+st.1/bin:$PATH"
```

Build và nạp firmware robot:

```bash
cmake --preset robot
cmake --build --preset robot
st-flash --reset write build/robot/LineRobot.bin 0x08000000
```

Build và nạp firmware test sensor:

```bash
cmake --preset test-sensor
cmake --build --preset test-sensor
st-flash --reset write build/test-sensor/LineRobot.bin 0x08000000
```

Thay `test-sensor` hoặc `robot` trong cả ba lệnh để nạp
firmware tương ứng. Mỗi lần STM32 chỉ chạy một firmware; nạp lại bản `robot`
để quay về chương trình robot.

Nếu UART in `0 0 0 0 0 ... NO LINE`, ADC đã hoàn thành lượt đọc nhưng
cả năm kênh đều nhận giá trị 0. `NO LINE` là kết quả của các giá trị này,
không phải nguyên nhân. Kiểm tra nguồn và GND chung của module TCRT5000,
đầu ra analog của từng cảm biến nối vào PA0–PA4, và đo điện áp tại các chân
này khi đưa cảm biến từ nền trắng sang vạch đen. Nếu điện áp tại chân thay đổi
nhưng số ADC vẫn bằng 0, nạp `test-sensor` và kiểm tra lại số ADC in qua UART.
Giá trị ADC kỳ vọng từ hiệu chuẩn hiện tại khoảng 45–98 trên nền trắng và
140–180 trên vạch đen; các giá trị này có thể khác trên phần cứng khác.
