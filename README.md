# Major Project

## Đọc UART

```bash
ls /dev/ttyUSB* /dev/ttyACM*
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200
```

## Chọn firmware để nạp

Mỗi preset có thư mục build riêng; mã trong `Core/Robot/` không cần sửa.

| Preset | Firmware |
| --- | --- |
| `robot` | Chương trình robot |
| `test-sensor` | Test color và line sensor |

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
