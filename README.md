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
cmake --preset robot && \
cmake --build --preset robot && \
st-flash --reset write build/robot/LineRobot.bin 0x08000000
```

### Hiệu chuẩn cảm biến line

Sau khi nạp firmware robot, đặt 5 mắt TCRT5000 ở độ cao vận hành và bấm
**RESET STM32**. Trong 20 giây LED PC13 nhấp nháy mỗi 250 ms; dịch xe ngang
qua lại để từng mắt nhìn thấy cả nền trắng và vạch đen. Motor giữ dừng.
UART in tiến độ `CAL 1s/20s OK=.../5 WHITE=... BLACK=...` mỗi giây. Khi hiệu chuẩn
thành công, LED sáng liên tục và UART in `CAL DONE OK=5/5 WHITE=... BLACK=...`
cùng `LINE CAL SAVED`.
Robot tiếp tục đứng chờ. Nếu LED tắt và UART báo
`LINE CAL FAILED`, kiểm tra vị trí các mắt rồi bấm RESET để thử lại.

Sau đó **tắt nguồn điều khiển**, đặt đầu dò vào vạch bắt đầu và bật lại
nguồn điều khiển. Firmware đọc dữ liệu hiệu chuẩn từ hai trang Flash cuối,
rồi bắt đầu dò line. Nguồn điều khiển phải thực sự ngắt điện STM32; USB hoặc
ST-Link còn cấp nguồn sẽ khiến thao tác tắt/bật không khởi động lại MCU.
Bấm RESET sẽ hiệu chuẩn lại. Khi firmware robot thay đổi, dữ liệu cũ không
còn khớp mã firmware và chương trình cũng vào chế độ hiệu chuẩn.

Trên xe này, T1 ở bên phải và T5 ở bên trái theo hướng tiến; đầu dò nằm trước
trục bánh. Khi T1 đọc vạch, bánh trái phải chạy nhanh hơn bánh phải để xe
rẽ sang phải. Khi T5 đọc vạch, xe rẽ sang trái.

Build và nạp firmware test sensor:

```bash
cmake --preset test-sensor && \
cmake --build --preset test-sensor && \
st-flash --reset write build/test-sensor/LineRobot.bin 0x08000000
```

Thay `test-sensor` hoặc `robot` trong cả ba lệnh để nạp
firmware tương ứng. Mỗi lần STM32 chỉ chạy một firmware; nạp lại bản `robot`
để quay về chương trình robot.

Nếu UART in `NO LINE` và cả năm ADC gần các giá trị `WHITE` trong dòng
`CAL DONE`, đầu dò đang nhìn nền trắng. Giữ xe đứng yên, đặt T3 lên vạch đen
và kiểm tra T3 tăng gần giá trị `BLACK` của nó, còn các mắt ở nền trắng đọc
gần `WHITE`. Nếu số đọc chập chờn khi xe đứng yên, kiểm tra độ cao đầu dò,
nguồn và GND chung của module TCRT5000, cùng các dây analog vào PA0–PA4.
Nếu UART in `0 0 0 0 0 ... NO LINE`, đo điện áp tại các chân PA0–PA4 khi
đưa cảm biến từ nền trắng sang vạch đen. Nếu điện áp thay đổi nhưng ADC vẫn
bằng 0, nạp `test-sensor` và kiểm tra lại số ADC in qua UART.
