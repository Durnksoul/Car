# 两轮平衡车 PID 与串口数据

已实现可编译、可主机测试的三环控制算法。**算法尚未接管电机**；现有程序上电保持待机，发送 `enable` 后再发送 `ab 20 20` 才启动两轮。A 路保持原极性，B 路在应用层反转：`ab 20 20` 实际向驱动传入 A=+200、B=-200（千分比）。实际车身前进方向仍需实物确认。

## 控制关系

```text
目标速度 - 左右轮平均速度 → 速度 PID（通常先用 PI）→ 目标倾角修正
目标倾角 - 实测倾角       → 姿态 PID（通常先用 PD）→ 两轮共同输出
目标转向角速度 - 实测值   → 转向 PID             → 左右差速
左轮 = 共同输出 - 差速；右轮 = 共同输出 + 差速
```

速度环输出单位为度，姿态和转向环输出为 PWM 百分比。速度使用编码器四倍频计数/秒，尚未换算 m/s 或 RPM。转向环控制航向角速度，不是保持绝对航向。

`pid_controller.c` 使用实际 `dt_s`：`P=Kp×误差`，`I=限幅(I+Ki×误差×dt)`，`D=-Kd×滤波后的实测变化率`。姿态环通过 `pid_controller_update_with_rate()` 直接使用陀螺仪角速度；速度、转向环通过 `pid_controller_update()` 计算测量差分。D 对测量求导，避免目标跳变造成微分冲击。积分限幅以输出单位表示，输出饱和时阻止积分继续向饱和方向增长。

## 接口和默认行为

| 文件 | 用途 |
| --- | --- |
| `main/pid_controller.h` | PID 初始化、复位、更新和带外部变化率的更新 |
| `main/balance_controller.h` | 三环组合、显式 arm/disarm、输入与输出结构 |
| `tests/test_balance_controller.py` | 执行实际 C 算法的主机测试 |

- 默认全部增益为 0，`mapping_confirmed=false`，拒绝 arm。限幅数值是框架默认值，不是已调好的实车参数。
- arm 要求有效且未过期的数据、车身在允许直立范围内、姿态 Kp 大于零。
- 更新检查倾角、IMU/编码器数据年龄、IMU 时间戳单调性和更新间隔；拒绝 NaN/Inf。故障锁定为零输出，需显式重新 arm。
- 转向只使用姿态输出剩余的电机余量，避免差速挤占平衡输出。
- 该模块只计算，不访问传感器、不打印、不控制 GPIO。**未来的电机调用方必须在失败/disarm 时拉低 STBY**；算法返回零不能替代硬件停止。

后续调用流程（当前主程序未启用）：

```text
balance_config_default → 填入已确认的映射、方向和试验增益
→ balance_controller_init → balance_controller_arm
→ 每个新IMU样本调用balance_controller_update
   成功：将left_percent/right_percent转为驱动千分比，B路只反转一次
   失败：电机驱动进入standby
```

`balance_input_t.tilt_deg` 是相对名义直立位置的倾角，`tilt_rate_dps` 必须是同一角度、同一符号的角速度。输入原始左右 CPS，通过 `encoder_sign[2]` 归一化为前进为正；速度环按至少 `speed_period_s` 的实际累计间隔更新，姿态和转向环按新 IMU 时间更新。最终应由一个任务独占电机驱动，手动 PWM 与平衡模式互斥，控制环不能依赖电脑/AI 的串口往返时延。

## 串口遥测

保留网页严格匹配的角度行，最多 20Hz：

```text
roll=1.20,pitch=-0.30,yaw=0.00
```

编码器约每 20ms 采样、每 200ms 输出；`enc`/`status` 返回最新缓存：

```text
enc_left=120,enc_right=-118,cps_left=600.0,cps_right=-590.0,enc_us=4000000
```

新增综合遥测约 5Hz，在有效姿态完成启动 Yaw 归零后输出：

```text
telemetry,t_us=4008000,roll=1.20,pitch=-0.30,yaw=0.00,gx_dps=0.100,gy_dps=-0.200,gz_dps=0.000,enc_us=4000000,enc_valid=1,enc_left=120,enc_right=-118,cps_left=600.00,cps_right=-590.00,enabled=1,cmd_left=20,cmd_right=20,brake_left=0,brake_right=0,balance_active=0
```

| 字段 | 含义 |
| --- | --- |
| `t_us` | MCU 微秒时间，IMU读取完成时刻，不是精确中断时间 |
| `roll/pitch/yaw` | 驱动角度约定，单位度；Yaw相对启动方向 |
| `gx_dps/gy_dps/gz_dps` | 原始传感器XYZ角速度，按现有驱动声明 `raw/14.2857` 换算为度/秒，尚未映射车身轴 |
| `tilt_deg` | 独立前后倾角：世界竖直方向投影到传感器XZ平面，绕传感器+Y为正，范围[-180,180)；按实测前倾运动为正，不随世界航向变化 |
| `tilt_rate_dps` | 与该投影倾角匹配的角速度，纯Y轴转动时等于gy_dps，侧倾时包含gx/gz修正 |
| `tilt_valid` | 投影是否有效；轮轴接近竖直时为0，此时不得把倾角/变化率的占位0输入PID |
| `enc_us` | 最近编码器采样完成时间，可估计与姿态数据的时间差 |
| `enc_valid` | 是否有可用缓存；仍需检查其数据年龄 |
| `enc_left/right` | 原始有符号累计计数，未自动校正编码器方向 |
| `cps_left/right` | 最近约20ms窗口的计数/秒，低速有量化噪声 |
| `enabled`、`cmd_left/right` | 软件使能和逻辑占空比，不是实测电流/转速 |
| `brake_left/right` | 是否命令短路制动 |
| `balance_active` | 当前固定0，平衡算法未接管电机 |

两类数据不是同步硬件采样。5Hz综合遥测用于起步识别轴、方向和慢变化趋势，不足以分析所有快速振荡。实车姿态调参阶段应补充包含控制周期、P/I/D分量、限幅输出的高频记录。

## 后续确认与调参

1. 解决UART口USB枚举问题，恢复遥测与烧录。
2. 2026-10-18前后倾日志已确认主要绕传感器Y轴运动，前倾gy为正。新增 `tilt_deg`/`tilt_rate_dps` 供平衡环使用，保留原网页欧拉角。烧录后复测独立倾角的直立参考、前倾为正和后倾为负；不能直接沿用旧Pitch=-3.1°作为新的平衡点。
3. 手推前进确认左右编码器符号。B电机反转不意味着B编码器也必然要反转。
4. 确定整车前进方向及 `attitude_to_motor_sign`，保证向前倒时轮子追向前；确定 `speed_to_tilt_sign`，保证速度环负反馈。
5. 先只调姿态PD，速度与转向增益保持0；再加速度PI，最后调转向。机械重心、电源、电机死区都会影响参数。

后续AI根据授权采集的批次日志分析并调整参数，实时闭环在MCU上执行。当前版本没有自动调参程序、串口PID参数命令或参数持久化。

## 独立倾角与角速度

SFLP四元数将传感器坐标转到世界坐标，先由其旋转矩阵求世界向上单位向量在传感器系内的分量 `ux,uy,uz`。定义 `tilt=atan2(-ux,uz)`。该角度绕传感器Y轴，世界航向旋转不会改变它；X/Z投影过小（轮轴近乎竖直）时返回无效。角速度按向量运动关系计算：

```text
tilt_rate = gy - uy * (gx*ux + gz*uz) / (ux*ux + uz*uz)
```

这里gx/gy/gz为度/秒。因此普通前后倾时可视作gy，侧倾时则采用完整修正。姿态与原始角速度仍来自FIFO与输出寄存器两个采样路径，存在时间偏差。参考：[四元数、旋转矩阵和角速度关系](https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-attitudetran)。

`tilt_valid=1`只表示数学投影有效，不表示机械平衡点已校准或允许启动PID。零点以传感器Z轴向上为名义直立，不自动减去上电姿态；安装偏移、实际重心需要后续确定 `trim_deg`。控制调用方必须先检查该标志，再将独立倾角与变化率填入 `balance_input_t`，禁止把网页Roll/Pitch直接替代。当前没有加入运行时电机闭环调用，`balance_active`仍为0。

## 验证

```powershell
python tests/test_balance_controller.py
python tests/test_car_encoder.py
python tests/test_tb6612_command.py
python tests/test_tb6612fng.py
```

主机测试覆盖PID正负误差、积分/输出限幅、抗积分饱和、微分无目标冲击、外部角速度、滤波、非法输入、三环组合、方向符号、转向限幅、跌倒/过期/超时和重新使能。主机测试不代表实车已能直立。
