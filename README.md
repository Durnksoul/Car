# Car：ESP32-S3 + IMU660RC

当前程序使用 ESP-IDF 5.5.4，读取逐飞 IMU660RC（LSM6DSV16XTR）的内置姿态解算结果，通过串口输出横滚角 Roll、俯仰角 Pitch、相对航向角 Yaw，单位为度。

## 接线

按模块背面丝印连接，推荐使用短线，断电接线。

| IMU660RC 引脚 | ESP32-S3 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SCL/SPC | GPIO12（SPI 时钟） |
| SDA/SDI | GPIO11（MOSI） |
| SA0/SDO | GPIO13（MISO） |
| CS | GPIO10（片选） |
| INT2 | GPIO9（姿态更新中断，必须连接） |
| INT1 | 悬空 |

以上引脚避开 ESP32-S3 的启动配置引脚、原生 USB GPIO19/20、UART0 GPIO43/44，以及常用 Flash/PSRAM 引脚。具体开发板如已占用 GPIO9～13，请修改 `components/imu660rc/include/imu660rc.h` 的 `IMU660RC_CONFIG_DEFAULT()`；该驱动独占 SPI2 总线。

## 在 VS Code 中使用

实际 ESP-IDF 工程根目录是此 README 所在的内层 `Car` 目录，内含顶层 `CMakeLists.txt`。

1. 使用当前 ESP-IDF 扩展环境，目标选择 `esp32s3`，执行 Build。
2. 按上表连接模块，再选择开发板端口，执行 Flash 和 Monitor。
3. 串口使用 **115200、8N1**。初始化成功后，按串口提示保持模块静止 **3 秒**，程序记录当前 Yaw 作为零点。看到 `Yaw zero recorded` 后再转动模块观察数值。启动期间如读取失败，将重新等待 3 秒。

业务层使用 `printf` 输出项目现有的 UART0 控制台，ESP32-S3 默认 TX=GPIO43、RX=GPIO44。带 USB 转串口的开发板可直接使用对应 USB 接口；外置 3.3V USB-TTL 的 RX 接 GPIO43、GND 接 GND。当前 sdkconfig 同时启用了 USB Serial/JTAG 辅助控制台，可通过开发板原生 USB 接口观察输出。

初始化成功并完成 Yaw 归零后，输出形式如下（数值仅为格式示例）：

```text
roll=1.23,pitch=-2.34,yaw=45.67
```

姿态更新频率是 120 Hz，串口输出最多 20 Hz。Roll 范围约 -180～180°，Pitch 约 -90～90°。串口的 Yaw 是相对启动零点的转角，范围为 **[-180°, 180°)**；每次重启都会重新记录零点。首次角度输出的 Yaw 为 0°，例如原始 Yaw 为 170° 时记录零点，之后原始值 260° 输出 +90°、80° 输出 -90°。程序处理了原始角度经过 0°/360° 时的环绕；相对角经过 ±180° 时仍会环绕，不累计整圈转数。

驱动内部保留逐飞原例程的 0～360° Yaw、轴顺序和符号约定，归零只在业务层完成。Roll/Pitch 保留原值。启动等待是固定的 3 秒有效数据时间，程序不检测模块是否运动，因此等待期间需要保持静止。实际安装方向与车体方向的映射，需要上车后通过绕各轴转动来确定。

六轴没有磁力计，Yaw 是相对航向，并非绝对北向，长期使用会有漂移；Pitch 接近 ±90° 时，欧拉角表示存在奇异性。

## 电脑上的姿态网页

双击 `tools/imu_viewer/StartViewer.cmd`，用桌面版 Chrome 或 Edge 打开 http://127.0.0.1:8765 ，即可显示三个角度和随姿态旋转的模型。

先关闭 VS Code 的 ESP-IDF Monitor 和其他串口软件，再在网页点击「连接串口」，选择开发板端口，波特率保持 115200。网页直接读取当前固件的输出，无须再次修改固件。「查看演示」可预览模拟姿态，页面会明确标注演示数据。详细说明见 `tools/imu_viewer/README.md`。

## 代码划分

- `components/imu660rc/imu660rc.c`：SPI 通信、芯片 ID 检查、寄存器初始化、INT2 通知及四元数读取。
- `components/imu660rc/imu660rc_math.c`：半精度浮点四元数解码、归一化及欧拉角转换。
- `components/imu660rc/include/imu660rc.h`：引脚配置、初始化/读取/释放接口和输出数据结构。
- `main/main.c`：业务层初始化重试、读取数据、启动 Yaw 归零和串口输出。

GPIO 中断只释放信号量，SPI 读取在任务中执行。`imu660rc_read()` 只能由一个任务调用；出错时不会将旧角度作为新样本输出。信号量合并积压的更新通知，此接口读取当前姿态，不用于记录每一帧历史数据。

## 排查

- `WHO_AM_I=... expected 0x70`：核对 RC 型号、3.3V 供电、共地，以及 MOSI/MISO/时钟/CS。初始化失败每秒重试一次。
- `ESP_ERR_TIMEOUT`：优先检查 INT2 是否接 GPIO9；没有姿态更新中断时程序不会输出角度。
- `ESP_ERR_INVALID_RESPONSE`：四元数为零或无效，启动阶段可短暂出现；持续出现则检查供电、接线和型号。
- 长杜邦线通信不稳定：先缩短线，再将默认 `clock_speed_hz` 降至 `1000000` 测试。
- 如果模块此前在 IIC 模式下使用过，刷入 SPI 程序后让 MCU 和模块一起断电重启。

## 参考及授权

产品资料：[SeekFree/IMU660RX_Product](https://gitee.com/seekfree/IMU660RX_Product)，参考提交 `d73c66c9748db4daea36c2234f9b8b792088c58c`。

该资料库通过子模块指向各 MCU 示例，本移植参考其 [RT1064_Library](https://gitee.com/seekfree/RT1064_Library)，提交 `1223cdbf763acb7d501d20164520da1f6d978e8b`，文件：

```text
Example/Motherboard_Demo/E4_imu/libraries/zf_device/zf_device_imu660rc.c
Example/Motherboard_Demo/E4_imu/libraries/zf_device/zf_device_imu660rc.h
Example/Motherboard_Demo/E4_imu/E4_04_imu660rc_demo/user/src/main.c
```

原例程默认使用硬件 SPI，10 MHz、模式 0，通过 INT2 读取内置 SFLP 四元数，示例选择 120 Hz。本移植保留寄存器配置、嵌入页读取方式、轴重排和欧拉角公式，改用 ESP-IDF SPI/GPIO/FreeRTOS 接口；补充通信错误返回、无效四元数检查、反正弦输入限幅，并消除原浮点转换中的指针类型别名问题。

移植组件保留 SEEKFREE 版权，按 GPL-3.0-or-later 授权，完整许可证见 `components/imu660rc/LICENSE`。

## 验证范围

2026-10-12：已在本机 ESP-IDF 5.5.4 环境完成 ESP32-S3 固件编译、链接和镜像生成，产物为 `build/Car.bin`。实际 C 角度转换函数通过主机测试：三个轴旋转、Pitch ±90°、全部 65536 种半精度编码，以及零四元数、NaN、Inf 等无效数据。测试脚本位于 `tests/test_angles.py`，使用 Clang 生成的主机 LLVM IR 和本机 llvmlite 执行 C 函数；这些测试工具不参与固件构建。

本次未烧录，未进行传感器实测。代码的编译和数学测试不能代替接线后的硬件验证。烧录后应确认芯片 ID 为 0x70，持续输出角度，再分别绕三个轴转动模块检查方向和响应。
