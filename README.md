# Car：ESP32-S3 + IMU660RC + TB6612FNG

当前程序使用 ESP-IDF 5.5.4，读取逐飞 IMU660RC（LSM6DSV16XTR）的内置姿态解算结果，通过串口输出横滚角 Roll、俯仰角 Pitch、相对航向角 Yaw，单位为度。

当前默认启用 **双路编码器读取 + UART/USB 串口命令控制**，BLE 命令入口暂时关闭，源码保留。初始化成功后，两路电机保持 **待机（STBY=0）**，不会自动转动。先发送 `enable`，再发送 `ab 20 20`，两轮以逻辑正向 20% 占空比、20kHz PWM 同向运行；也可用 `a`、`b` 单独控制。电机命令任务不依赖 IMU 是否初始化成功。逻辑正向在 A 路对应 IN1=1、IN2=0，在 B 路对应 IN1=0、IN2=1；按当前左右轮方向相反的反馈，已在应用层反转 B 路，无需交换接线。实际整车前进方向待实测。

**每次上电、复位均保持待机，必须通过命令启动。** `enable` 只使能驱动并进入滑行，不会自行旋转。启动后串口断开会保持最后输出，没有断连自动停机或命令超时；发送 `standby` 停止，重启后回到待机，不恢复旧命令。手动模式仍是占空比控制；直立平衡模式会让三环 PID 接管电机，但增益需实车整定，详见 [两轮平衡车 PID 与串口数据](docs/balance_pid.md)。

**已接入 LLM-PID-Tuner 在线调参。** 固件以 20 Hz 输出 `timestamp_ms,setpoint,input,pwm,error,p,i,d`，并接受 `STATUS`、`SET P:.. I:.. D:..`、`SETPOINT:..`、`RESET` 等大写命令；`SET` 可在平衡运行中直接改姿态环增益并自动接管。完整协议、推荐 `config.json`、操作顺序与安全边界见 [用 LLM-PID-Tuner 调直立环](docs/llm_pid_tuner.md)。

## 电机与编码器接线

以小车前进方向区分左右，**A 路=左电机，B 路=右电机**。

| TB6612FNG 模块引脚 | ESP32-S3 / 电源 |
| --- | --- |
| AIN1 / AIN2 / PWMA | GPIO4 / GPIO5 / GPIO6 |
| BIN1 / BIN2 / PWMB | GPIO7 / GPIO8 / GPIO15 |
| STBY | GPIO16，由 MCU 控制 |
| VCC | 3V3，逻辑供电 |
| VM | 独立电机电源，按电机额定电压及启动/堵转电流选择；不要用开发板 3V3 驱动电机 |
| GND | ESP32、编码器、电机电源共地 |
| AO1 / AO2 | 左电机 M+ / M− |
| BO1 / BO2 | 右电机 M+ / M− |

| 编码器引脚 | 左电机 | 右电机 |
| --- | --- | --- |
| A 信号 | GPIO17 | GPIO1 |
| B 信号 | GPIO18 | GPIO2 |
| GND | 公共 GND | 公共 GND |
| VCC | 按编码器额定供电电压 | 同左 |

编码器 A/B 接入 ESP32 时必须为 3.3V 兼容电平；5V 推挽输出需电平转换。程序启用弱上拉，开漏输出如需外部上拉，应上拉到 3.3V。确认开发板已引出上述 GPIO 且未用于其他外设。

编码器使用 PCNT 双通道正交四倍频计数、1µs 毛刺过滤，并在正负硬件限值处累计计数。A 超前 B 的相序计为正；实际电机正转读数若为负，断电交换该编码器 A/B 信号。累计值扩展为 64 位，约每 20ms 采样，每 200ms 输出一次带符号计数和每秒计数。尚未确认每圈计数及减速比，因此不把 `cps` 当作 RPM。

## 用 LLM-PID-Tuner 调直立环

固件默认打开 `CONFIG_CAR_PID_TUNER`，串口 115200 8N1。编译时还必须保持 `main/car_log_mode.h` 的 `CAR_ENCODER_ONLY_LOG` 为 0，否则只会输出编码器行。

```text
1. 关闭 ESP-IDF Monitor、网页查看器等占用串口的程序。
2. 上电后静止 3 s 完成 Yaw 归零，把车扶正并保持静止。
3. llm-pid-tuner.exe tune COM10
   HARDWARE_PROFILE=generic_serial_csv，波特率 115200，填写初始 PID。
4. 收到 SET 后固件自动记录直立角并接管；手扶确认前倾时两轮向前追。
5. 每轮采集约 5 s，轻推车体 ±2° 给出扰动；收敛后抄下最终 P/I/D。
```

姿态环增益、速度/转向环增益和直立参考角会自动写入 NVS，掉电或复位后恢复；上电仍处于 STANDBY，发 `balance arm` 即可接管，不需要重新 `balance zero`。要重新校准直立角，扶正后发 `balance zero` 覆盖保存值。第一次请使用牵引绳或测试架，并查看 [逐步教程：用 LLM-PID-Tuner 调本车直立环](docs/llm_pid_tuner_tutorial.md)；方向检查、限幅和常见问题见 [用 LLM-PID-Tuner 调直立环](docs/llm_pid_tuner.md)。

## 串口命令及外部无线串口

当前不启动 BLE、不广播 `Car-ESP32S3`，通过编译开关 `CONFIG_CAR_BLE_UART` 保留蓝牙代码供以后恢复。蓝牙协议不影响外部无线串口模块；模块在 MCU 侧只需要接 UART0。

图片中的 WCH-LinkW 类设备，串口侧按以下连接，设置 115200、8N1、3.3V TTL 电平：

| 无线串口从机 | ESP32-S3 |
| --- | --- |
| TXD | GPIO44（UART0 RX） |
| RXD | GPIO43（UART0 TX） |
| GND | GND |

电源按设备说明独立接好，避免并联两个电源输出。不要将 SWD 的 DIO/CLK 当作 UART；串口命令每行末尾加 CR 或 LF。

**无线烧录兼容性尚未确认。** WCH 官方手册描述的是 WCH RISC-V 和 ARM SWD/JTAG 下载，以及无线串口，不能直接据此认定支持 ESP32-S3 无线下载。ESP32-S3 可通过 UART0 与 esptool 通信，但需要在复位时将 GPIO0 拉低进入 ROM 下载模式；全程远程自动烧录还需要适配 GPIO0/EN 控制（通常由 DTR/RTS 经自动下载电路实现）。截图未证明所售版本支持这些控制信号，只有 TX/RX 时一般仍需在板端手动按 BOOT/EN。模块是否能可靠透传烧录数据也需实测。

依据：[WCH-Link 官方手册](https://img.wch.cn/uploads/file/20240302/1709344347108943.pdf)、[乐鑫 ESP32-S3 启动模式文档](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html)。

有线串口继续支持相同命令：UART0 和原生 USB Serial/JTAG，115200、8N1。UART0 默认 TX=GPIO43、RX=GPIO44；外置 3.3V USB-TTL 的 TX→GPIO44、RX→GPIO43、GND→GND。UART 和 USB 输入分别组装命令，由同一任务操作驱动，不能将一条命令拆到不同入口。

| 命令 | 功能 |
| --- | --- |
| `help` | 命令帮助 |
| `status` | 使能、两路占空比/刹车状态，以及最近一次编码器读数 |
| `enc` | 最近一次编码器读数，正常约 20ms 前采样 |
| `enczero` | 两路累计计数清零，不改变电机输出 |
| `standby` | 两路停止驱动，STBY=0，清除旧占空比 |
| `enable` | 从待机唤醒后两路滑行，需再发占空比命令才转；重复使能保持输出 |
| `a 50` / `b 50` | 左路 / 右路正向 50% |
| `ab 50 50` | 两路正向 50% |
| `ab -30 -30` | 两路反向 30%，换向前先停止并等待车轮停转 |
| `ab 0 0` / `coast all` | 两路滑行，不是短路刹车 |
| `brake a` / `brake b` / `brake all` | 指定通道短路刹车 |

占空比参数是 -100～100 的整数百分比，支持大小写，正负号改变方向；100% 为恒高 PWM。待机后需要先发 `enable` 才能设置输出。命令成功回复 `OK: ...`，错误回复 `ERR: ...`。回复与编码器数据通过串口返回，可用 `status` 确认软件记录状态。

串口自动输出以下格式，单位为四倍频计数和计数/秒：

```text
enc_left=1200,enc_right=1180,cps_left=2400.0,cps_right=2360.0,enc_us=4000000
```

IMU 的 `roll=...,pitch=...,yaw=...` 有线串口输出保留，BLE 暂停。电机任务不依赖 IMU 是否接入；IMU 启动仍要求静止 3 秒；电机上电保持待机，建议完成 Yaw 归零后再发转动命令。该逻辑不强制等待归零，需由使用者保持模块静止。

恢复 BLE 时先在 Bluetooth 菜单启用 NimBLE，再在 Car control 启用 `Enable BLE UART command input`；当前 sdkconfig 和 sdkconfig.defaults 均关闭蓝牙。

在 `idf.py menuconfig → Car control` 可关闭 `Enable encoder and serial motor control (standby at boot)`，再启用旧的 `Enable TB6612FNG serial command control for scope tests (no motors)`；旧模式上电待机，不读取编码器。两个控制任务互斥。不要同时添加第二个任务操作同一驱动。

示波器测 PWM 时，探头尖端接 GPIO6/15，地夹接 GND，预期 20kHz、周期 50µs、50% 高电平宽度约 25µs。普通台式示波器地夹不能接 AO1/AO2/BO1/BO2；电机差分电压应使用两通道对 GND 测量后相减，或使用差分探头。

## TB6612FNG 驱动接口

驱动位于 `components/tb6612fng`。默认使用 LEDC 低速 Timer0、Channel0/1、20kHz、10 位分辨率，独占相应定时器、通道和 GPIO，不能与其他 PWM 驱动共用。配置结构可修改引脚、通道、定时器和频率；频率上限 100kHz，实际频率与固定的 10 位分辨率组合需 LEDC 硬件支持。

关闭默认电机控制任务及旧示波器测试任务后，可在自己的单一控制任务中调用：

```c
#include "tb6612fng.h"

tb6612fng_handle_t motor = NULL;
const tb6612fng_config_t config = TB6612FNG_CONFIG_DEFAULT();
ESP_ERROR_CHECK(tb6612fng_init(&config, &motor));  // STBY=0
ESP_ERROR_CHECK(tb6612fng_enable(motor, true));   // 两路滑行，不恢复旧命令
ESP_ERROR_CHECK(tb6612fng_set_speeds(motor, 250, -500)); // A +25%，B -50%
// 按需调用，而非立即连续执行：
// tb6612fng_set_speed(motor, TB6612FNG_MOTOR_A, 750);
// tb6612fng_coast(motor, TB6612FNG_MOTOR_A);
// tb6612fng_brake(motor, TB6612FNG_MOTOR_B);
// tb6612fng_enable(motor, false); // 两路待机并清除命令
// tb6612fng_deinit(motor);        // 停止控制任务后释放资源，STBY保持低
```

驱动 API 的速度参数是 **-1000～1000 的有符号占空比命令**（串口命令则是 -100～100%），不是 RPM；0 为滑行，±1000 为恒高 PWM。驱动不读取编码器、不做速度闭环或机械加减速。真实电机应先减速/停止再换向，并按启动/堵转电流核对芯片及模块能力；当前命令直接设置方向和占空比，没有机械减速斜坡，实测换向前需先停止并等待车轮停转。

所有接口由调用者串行调用，不可在 ISR 中使用；同一方向调速只更新 PWM，避免每次命令都中断输出。非法参数不改变已有输出；GPIO/LEDC 操作失败时尝试拉低 STBY 并停止两路，返回错误。不能将软件返回值当作实物关断的测量证据。

依据：[东芝 TB6612FNG 数据手册](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG)、[ESP-IDF 5.5.4 ESP32-S3 LEDC 文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32s3/api-reference/peripherals/ledc.html)。

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

## BLE 恢复时的服务说明（当前关闭）

ESP32-S3 使用 BLE，**不支持经典蓝牙 SPP**，普通 SPP 串口助手不能连接。本工程使用 Nordic UART Service（NUS），设备名 **`Car-ESP32S3`**，允许一个连接，不启用配对认证。附近 BLE 客户端可以连接并发送控制命令。

1. 在支持 BLE UART/NUS 的手机工具中扫描并连接 `Car-ESP32S3`。
2. 订阅 TX 特征的 Notify；向 RX 特征写入 ASCII 文本。
3. 每条命令末尾加 LF 或 CR。支持 BLE 分包及一个写入中包含多条命令；最多 63 个 ASCII 字符，非法或超长行整行丢弃。断线不保留未完成的命令。

| 项目 | UUID |
| --- | --- |
| NUS 服务 | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| RX，手机写入 | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` |
| TX，手机订阅通知 | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |

## 代码划分

- `components/imu660rc/imu660rc.c`：SPI 通信、芯片 ID 检查、寄存器初始化、INT2 通知及四元数读取。
- `components/imu660rc/imu660rc_math.c`：半精度浮点四元数解码、归一化及欧拉角转换。
- `components/imu660rc/imu660rc_fifo.c`：完整7字节FIFO记录读取、姿态标签筛选、最新样本选择及溢出检查。
- `components/imu660rc/include/imu660rc.h`：引脚配置、初始化/读取/释放接口和输出数据结构。
- `main/main.c`：启动电机任务，IMU 初始化重试、Yaw 归零和串口输出。
- `main/car_motor_control.c`：上电待机、B路方向反转、串行执行有线/BLE 命令、编码器采样和遥测；含 LLM-PID-Tuner 的 20 Hz CSV、`STATUS`/`SET`/`SETPOINT`/`RESET` 解析、运行中改增益与自动接管。
- `main/car_encoder.c`：双路 PCNT 正交四倍频计数及每秒计数。
- `main/car_ble_uart.c`：保留的 NimBLE NUS 服务；当前不参与编译和启动。
- `main/car_command_stream.c`：分包文本组装、换行及非法行丢弃。
- `main/tb6612_command.c`：三种输入共用的命令解析。

GPIO 中断只释放信号量，SPI 读取在任务中执行。`imu660rc_read()` 只能由一个任务调用；出错时不会将旧角度作为新样本输出。信号量合并积压的更新通知，此接口读取当前姿态，不用于记录每一帧历史数据。

## 排查

- `WHO_AM_I=... expected 0x70`：核对 RC 型号、3.3V 供电、共地，以及 MOSI/MISO/时钟/CS。初始化失败每秒重试一次。
- `ESP_ERR_TIMEOUT`：优先检查 INT2 是否接 GPIO9；没有姿态更新中断时程序不会输出角度。
- `ESP_ERR_INVALID_RESPONSE`：FIFO姿态记录包含NaN/Inf或明显超出单位四元数范围；不等同于INT2超时。驱动最多每秒打印一次 `SFLP FIFO invalid quaternion: packet=...; WHO_AM_I=...`；ID预期为0x70。FIFO中的XYZ全零表示单位姿态，是有效数据。ID正常也不能单独证明完整数据链路正常；恢复有效数据时输出 `Valid quaternion samples resumed`。
- `ESP_ERR_INVALID_STATE` / `SFLP FIFO overrun`：本批FIFO出现溢出，批次被排空并丢弃，下一次等新样本；检查读取任务是否长时间被串口输出或其他任务阻塞。
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

原例程默认使用硬件 SPI，10 MHz、模式 0，通过 INT2 读取内置 SFLP 四元数，示例选择 120 Hz。2026-10-17起，本移植按ST公开接口改为主寄存器区FIFO读取，不再读取嵌入页3的0x4c内部数据。FIFO每条记录为标签加6字节，姿态标签为0x13，数据为XYZ三个半精度浮点数；按ST示例重建非负W，保留原欧拉角计算约定。加速度计、陀螺仪和SFLP均配置为120Hz，INT2改为FIFO水位1条的电平中断；读取前先检查FIFO，避免因合并中断或遗留数据而漏读。每次排空一个有界快照，选最新姿态，错误时不修改上一份样本。原始加速度/角速度仍从输出寄存器读取，量程保持±8g和±2000dps，与FIFO姿态不是严格同步采样。

ST参考：[SFLP官方示例](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsv16x_STdC/examples/lsm6dsv16x_sensor_fusion.c)、[寄存器定义及驱动](https://github.com/STMicroelectronics/lsm6dsv16x-pid)。新增 `python tests/test_imu_fifo.py` 验证完整记录读取、标签筛选、取最新帧、无效数据/溢出/读取错误和511条快照上限；`python tests/test_angles.py` 可自行编译当前数学C文件并验证新旧数据格式的轴向约定。

移植组件保留 SEEKFREE 版权，按 GPL-3.0-or-later 授权，完整许可证见 `components/imu660rc/LICENSE`。

## 验证范围

2026-10-21（LLM-PID-Tuner 对接）：新增 `CONFIG_CAR_PID_TUNER` 与 `CONFIG_CAR_PID_TUNER_AUTO_ARM`，恢复 `CAR_ENCODER_ONLY_LOG=0`，控制节拍改为 1 kHz tick，UART 发送增加 2 KB 缓冲。固件输出 20 Hz `timestamp_ms,setpoint,input,pwm,error,p,i,d`（倾角相对直立参考、增益为姿态环 °→%），接受大写 `STATUS`/`SET P:.. I:.. D:..`/`SETPOINT:..`/`RESET`/`STANDBY`；`SET` 运行中限幅到 50/20/5 并清积分，静止直立时自动校准并接管。主机测试覆盖协议、限幅、拒绝尾随参数、自动接管和模式互斥；本次未烧录，实车方向、自动接管时机与调参效果待验证。详见 [用 LLM-PID-Tuner 调直立环](docs/llm_pid_tuner.md)。

2026-10-18（独立车体倾角）：根据前后倾实测主要绕传感器Y轴，新增 `tilt_deg`、`tilt_rate_dps`、`tilt_valid` 综合遥测。倾角由世界竖直在传感器XZ平面的投影计算，不受世界航向影响；变化率包含侧倾时的其他轴修正，投影奇异时标记无效。不自动将上电姿态设为平衡点，旧网页角度输出保留。数学主机测试（含多航向同倾角及300组变化率参考）、FIFO主机测试和ESP-IDF构建通过；尚未烧录及实机验证新倾角，电机上电仍待机、PID仍未接管。

2026-10-17（SFLP FIFO修正）：针对运行约10秒后嵌入页四元数持续为Inf的实机日志，改为ST公开FIFO记录接口、XYZ重建W、120Hz配置及水位INT2中断，增加关键配置寄存器回读。当前数学C主机测试和FIFO读取C主机测试通过，ESP-IDF编译、链接、镜像生成和容量检查通过。本次未烧录，持续运行故障是否消除、实际角度轴向及精度待实机验证。

2026-10-17（恢复命令启动）：上电/复位保持STBY=0，需先 `enable`、再 `ab 20 20` 才转动；保留B路方向反转、编码器与综合遥测。UART0无线串口命令入口仍启用（115200 8N1，TX=GPIO43、RX=GPIO44）。编译、链接、镜像生成与容量检查通过，命令解析和电机驱动主机测试通过。本次未烧录，无线链路待实测。

2026-10-17（PID框架）：新增独立姿态/速度/转向三环算法及PID主机测试；应用层反转B路电机，编码器约20ms采样、200ms输出，增加带时间戳和三轴角速度的综合遥测。平衡算法尚未接管电机。ESP-IDF编译、链接、镜像和容量检查通过，PID/三环、编码器、命令解析与驱动主机测试通过；本次未烧录。接口、坐标确认及后续调参见 `docs/balance_pid.md`。

2026-10-17：改为每次上电、复位后自动使能，两路正向 20% 占空比（20kHz）；保留双路编码器每 200ms 串口输出累计计数及 CPS，以及原串口控制命令。ESP-IDF 5.5.4 编译、链接、镜像生成和分区容量检查通过，编码器主机测试通过。固件为 `build/Car.bin`，本次未烧录，实际轮向、启动和编码器读数待实物验证。

2026-10-13（命令启动版本）：取消上电自动 +50%，初始化及复位后两路保持待机。必须先 `enable`，再发送占空比命令才启动；BLE 继续关闭，编码器及串口读取保留。ESP-IDF 5.5.4 编译、链接、镜像生成及容量检查通过，命令解析和电机驱动主机测试通过。本次未烧录，实物上电及串口启动待验证。

2026-10-13（串口版本）：按要求停用 BLE 命令、通知和广播，关闭蓝牙栈；保留 UART0/USB 命令、编码器以及上电两路 +50% 输出。ESP-IDF 5.5.4 编译、链接、固件生成及分区容量检查通过；编译列表和链接映射中没有 BLE UART 模块。本次未烧录，WCH-LinkW 无线串口及 ESP32-S3 无线烧录尚未实测。

2026-10-13：已加入双路编码器和 BLE NUS 命令服务，默认 `CONFIG_CAR_MOTOR_CONTROL=y`、NimBLE 开启、旧示波器任务关闭。两路上电命令为 +50%，PWM 20kHz。ESP-IDF 5.5.4 的 ESP32-S3 编译、链接、镜像生成及分区容量检查通过，`build/Car.bin` 大小为 `0x8ae80` 字节，1MB 应用分区剩余约 46%。

主机测试通过：实际命令解析和文本组装 C 代码覆盖 `enc`/`enczero`、所有分包边界、CRLF、退格、非法字符、超长行丢弃及输入流隔离；实际编码器 C 代码在模拟 PCNT/时钟下覆盖 GPIO 分配、正反相序四倍频、计数/秒、软件清零、32 位计数环绕及各初始化失败点的资源清理；现有电机驱动主机测试通过。BLE 的空中传输、手机连接、通知流量和断线重连未实测，PCNT 模拟测试不能代替真实编码器电平及脉冲验证。本次未烧录，未测量 PWM、转向或转速。

接口依据：[ESP-IDF PCNT 文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32s3/api-reference/peripherals/pcnt.html)、[ESP-IDF NimBLE 文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32s3/api-reference/bluetooth/nimble/index.html)，并参考本机同版本官方 `pcnt/rotary_encoder` 和 `nimble/bleprph` 示例。

以下为此前版本的验证记录：

2026-10-12：TB6612FNG 测试改为串口命令控制后，ESP-IDF 5.5.4 的 ESP32-S3 编译、链接、镜像生成和分区容量检查通过，`build/Car.bin` 默认上电待机。`tests/test_tb6612_command.py` 使用 Clang 和 llvmlite 执行实际命令解析 C 代码，验证全部命令、±100% 内的整数参数、大小写与空白字符，并拒绝缺参、多参、小数、越界、超长及无效输入。`tests/test_tb6612fng.py` 使用相同方式执行实际驱动 C 代码，仅替换 GPIO/LEDC API，验证待机、方向、占空比、两路独立性、滑行/刹车、100% 后恢复 PWM、无效参数，以及初始化和双路换向过程中各硬件 API 失败后的清理与待机行为。这些主机测试依赖不参与固件构建。

TB6612FNG 本次未烧录，未验证 UART/USB 实际收发，也未测量 GPIO 或芯片输出；20kHz 和表中的波形是程序预期，仍需烧录后用示波器确认。构建环境缺少 ROM ELF 的调试符号路径时可能报告 `ESP_ROM_ELF_DIR` 警告，不影响本次固件镜像生成。

2026-10-12：已在本机 ESP-IDF 5.5.4 环境完成 ESP32-S3 固件编译、链接和镜像生成，产物为 `build/Car.bin`。实际 C 角度转换函数通过主机测试：三个轴旋转、Pitch ±90°、全部 65536 种半精度编码，以及零四元数、NaN、Inf 等无效数据。测试脚本位于 `tests/test_angles.py`，使用 Clang 生成的主机 LLVM IR 和本机 llvmlite 执行 C 函数；这些测试工具不参与固件构建。

本次未烧录，未进行传感器实测。代码的编译和数学测试不能代替接线后的硬件验证。烧录后应确认芯片 ID 为 0x70，持续输出角度，再分别绕三个轴转动模块检查方向和响应。

## 串口静止与运动平衡模式

见 [串口模式操作说明](docs/control_modes.md)。`mode hold` 使用设备已有 PID/零点启动平衡；`mode drive` 和 `move` 接入速度、转向目标。运动指令需持续更新，超时返回零速平衡。
