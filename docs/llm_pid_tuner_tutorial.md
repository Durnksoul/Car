# 逐步教程：用 LLM-PID-Tuner 调本车直立环

本教程按“烧录 → 校准 → 基线 → 启动调参 → 每轮操作 → 验收 → 保存”的顺序写成，每一步都有可检查的现象。协议细节见 [llm_pid_tuner.md](llm_pid_tuner.md)。

> 预计 30~60 分钟。第一次务必留 2 m 空间、用牵引绳或测试架，并保证手能随时断电。

---

## 阶段 0：准备

### 0.1 硬件

- 车体两轮悬空能自由转，放下时能直立；
- 电池电量足够（电压低会让同一组 PID 表现完全不同）；
- IMU660RC 和电机线固定，轮子不打滑；
- 串口：COM10，115200 8N1（CH343）。

### 0.2 软件

- 本仓库固件已编译（`CONFIG_CAR_PID_TUNER=y`、`CONFIG_CAR_PID_TUNER_AUTO_ARM=y`、`CAR_ENCODER_ONLY_LOG=0`）；
- VOFA+（FireWater 协议）或任意串口终端，用来发小写人工命令；
- LLM-PID-Tuner 桌面版或 `llm-pid-tuner.exe`，已配置好 API；
- `config.json` 的 `HARDWARE_PROFILE` 必须是 `generic_serial_csv`，`BAUD_RATE` 115200。

---

## 阶段 1：烧录并确认数据流（约 5 分钟）

### 步骤 1.1 编译

```powershell
cd C:\Users\soul\Desktop\ESP32-32\Car\Car
. C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1
idf.py build
```

看到 `Project build complete` 即可。

### 步骤 1.2 烧录

关闭 VOFA+ 和其他占用 COM10 的程序，然后：

```powershell
idf.py -p COM10 flash monitor
```

烧录完成后 monitor 会打印启动日志，按 `Ctrl+]` 退出 monitor，释放串口。

### 步骤 1.3 检查点 A

打开 VOFA+（COM10、115200、FireWater），应看到：

```text
# LLM-PID-Tuner ready: CSV timestamp_ms,setpoint,input,pwm,error,p,i,d at 20 Hz
# Uppercase: STATUS | SET P:.. I:.. D:.. | SETPOINT:<deg> | RESET | STANDBY
15234,0.000,0.412,0.000,-0.412,0.0000,0.0000,0.0000
15284,0.000,0.415,0.000,-0.415,0.0000,0.0000,0.0000
```

- 每行 **8 个数字**，间隔约 50 ms；
- `pwm=0`、`p/i/d=0` 是正常的（还没接管）；
- VOFA+ 的 I0~I7 分别对应 8 列。

如果只看到 `enc_left=...`：烧的是旧固件，回到 1.1 检查 `main/car_log_mode.h` 是否为 0 并重新编译。

---

## 阶段 2：校准与方向（约 3 分钟）

> 这一段不是“调参”，是给闭环一个正确的参考和符号。app 只改增益，不能替你判断方向。

### 步骤 2.1 记录直立参考

把车扶到机械直立、完全静止（轮子不转、车不晃），在 VOFA+ 发送框（`\n` 结尾）发送：

```text
balance zero
```

预期回复：

```text
OK: upright trim=0.xxx deg (saved)
```

同时 CSV 的 `input` 应从当前值跳到 **0 附近**。这就是校准成功的标志。

- 若回 `ERR: hold car upright and stationary...`：倾角超过 ±15°、车在晃，或轮子在转。扶稳重发。
- `trim` 是这次供电期间的参考，复位后需要重新做。

### 步骤 2.2 给一组基线增益

```text
pid attitude 4 0 0.3
```

预期回复两行：

```text
balance_state=0,trim=0.xxx,trim_set=1,motor_sign=-1,limit=20.0
pid attitude 4.0000 0.0000 0.3000; speed 0.00000 0.00000 0.00000
```

这组值只是“让闭环有输出”的基线，不是最终参数；LLM 会从这里继续改。

### 步骤 2.3 接管并确认方向

```text
balance arm
```

预期：

```text
OK: balance armed; hold the car for direction test
```

此时 CSV 的 `pwm` 不再恒为 0，`p,i,d` 变为 `4.0000,0.0000,0.3000`。

**手扶车体做方向测试：**

| 动作 | 正确现象 | 错误现象 |
| --- | --- | --- |
| 车体前倾 2~3° | 两轮**向前**追，`pwm` 为正 | 两轮向后，`pwm` 为负 |
| 车体后倾 2~3° | 两轮向后 | 两轮向前 |
| 左右倾斜 | 两轮基本同速 | 两轮差速明显（转向环应为 0） |

方向反了：

```text
standby
balance motor 1
balance arm
```

再测一次。**方向没确认前不要放手，也不要启动 LLM 调参。**

### 步骤 2.4 稳定的基线

手扶感受 10~20 秒：

- 车能自己站住 1~2 秒、只有轻微晃：可以进入调参；
- 明显低频摆动：`standby` 后把 P 降到 3 或 D 加到 0.5；
- 高频抖：D 加到 0.5~0.8；
- 完全没力、直接倒：检查电池、轮子是否打滑、`pwm` 是否经常顶到 ±20%。

改完后重新 `balance arm`。注意：`pid attitude` 只能在待机时改，先 `standby`。

---

## 阶段 3：启动仓库的调参流程（约 10 分钟）

### 步骤 3.1 交接串口

1. **不要断电**，车会继续平衡；
2. 关闭 VOFA+（释放 COM10）；
3. 打开 LLM-PID-Tuner，选择 COM10、115200、`generic_serial_csv`。

> CH343 打开串口时可能触发 DTR/RTS 复位。若 app 日志里出现启动信息，说明板子复位了：把车扶正、静止，固件会在第一条 `SET` 时自动记录直立角并接管。看到 CSV 的 `pwm` 从 0 变成非 0 即成功。

### 步骤 3.2 初始 PID

如果界面要求填写初始 PID，填 `4 / 0 / 0.3`（与基线一致）。如果不要求，它会从 CSV 的 `p,i,d` 读到同样的值。

如果填 0/0/0：第一轮 `pwm=0`，没有可分析的响应，会浪费一轮，之后 LLM 建议的 P>0 时才会接管。

### 步骤 3.3 开始

启动后，app 会：

1. 发 `STATUS`（固件回 `# STATUS: ...`，人可读，程序不解析）；
2. 发 `SET P:4.0 I:0.0 D:0.3`：如果车还没接管，固件回 `# tuner: balance armed...` 并开始闭环；如果车已经在平衡，则只更新增益、不打断闭环；
3. 收集 100 个样本（约 5 秒）→ 算指标 → 请求 LLM → 护栏 → 下发新 `SET`；
4. 循环，直到“够好”连续 3 轮或达到最大轮数。

---

## 阶段 4：每一轮你要做什么

每一轮 5 秒的采集窗口里：

1. **轻推车体 ±2° 再松手**，给一个可观测的扰动；
2. 不要一直扶着——扶着会掩盖真实响应；
3. 手保持在车边，车倒下前能接住；
4. 看 app 的指标：
   - `avg_error`：一轮平均倾角偏差（目标 1.0~1.5°）；
   - `steady_state_error`：最后 20% 样本的偏差（目标 0.5~1.0°）；
   - 过零次数/`OSCILLATING`：振荡的直接证据；
   - `overshoot` 在 setpoint=0 时**恒为 0**，不要看它。
5. 如果车倒了：它会锁定停机；把它扶正、静止，下一轮 `SET` 会自动重试接管。若连续几轮都接管不了，说明 `trim` 记歪了，需要停掉 app 回 VOFA+ 重新 `balance zero`。

### 什么时候该人工干预

| 现象 | 处理 |
| --- | --- |
| 车持续单方向加速跑 | `standby`（或断电）；检查 `trim` 是否在直立位记录 |
| 高频抖动、声音大 | 让 LLM 降 P / 加 D；固件限幅 50/20/5 会兜底 |
| 每轮都倒 | 先确认方向，再看 P 是否太小、输出是否顶到 20% |
| app 显示无数据 | 端口仍被 VOFA+ 占用，或波特率不对 |
| app 一开始就“够好”停了 | `GOOD_ENOUGH_*` 太宽，按文档收紧后重跑 |

---

## 阶段 5：收敛与验收

### 步骤 5.1 记录结果

app 停止后，抄下最终 `P / I / D`（TUI 会显示，停止前最后一行 CSV 的 `p,i,d` 也是）。

### 步骤 5.2 人工复验

1. 关闭 app，打开 VOFA+；
2. `standby`；
3. 用最终参数重新设置：

```text
pid attitude <P> <I> <D>
balance zero
balance arm
```

4. 手扶 → 松手 → 轻推 → 观察 30 秒；确认不是“只在某一轮数据里好看”。

### 步骤 5.3 保存参数

固件会自动保存，不需要再抄写：

- `pid attitude ...`、上位机 `SET P/I/D`：保存姿态/速度/转向环增益；
- `balance zero`：保存直立参考角；
- 掉电或复位后，`motor_task` 从 NVS 恢复，并在启动日志打印 `# restored P=... I=... D=... trim=...`。

上电后仍然是 STANDBY：发 `balance arm` 就能接管，**不需要重新 `balance zero`**（trim 已恢复）。要覆盖旧参考角，扶正后重新 `balance zero` 即可。

> 如果换了 IMU 安装或机械结构，建议先 `balance zero` 再 arm。想彻底恢复编译默认值，重新烧录并擦除 NVS 分区（`idf.py erase-flash`）即可。

---

## 故障排查

| 现象 | 最可能原因 | 处理 |
| --- | --- | --- |
| 只有 `enc_left=...` | 旧固件 / `CAR_ENCODER_ONLY_LOG=1` | 改回 0，重新编译烧录 |
| 没有 8 列 CSV | 波特率、端口错；VOFA+ 未选 FireWater | 115200、COM10、FireWater |
| `pwm` 一直 0 | 未接管：Kp=0、倾角>10°、车在动 | 扶正静止；填初始 PID；看 `# tuner` 回复 |
| `balance zero` 报错 | 倾角>15°、角速度>3°/s、轮子在转 | 扶稳、等轮子停 |
| `balance arm` 报错 | 没 zero、Kp=0、或已 enable | 先 `balance zero` / `pid attitude` / `standby` |
| 方向反 | 姿态输出符号 | `standby` → `balance motor 1` → `balance arm` |
| `SET` 没生效 | 行太长（已扩到 128）、负数、NaN、运行中 Kp=0 | 看 app 日志的 `# ERROR` |
| 一 `SET` 就倒 | 方向错 / P 太大 / D 太小 / 电池低 | 先方向，再降 P 加 D |
| 车越跑越远 | setpoint=0 但机械重心偏 | 重新 `balance zero`；调机械配重 |
| app 无数据 | COM10 被 VOFA+ 占用 | 关闭 VOFA+ |

---

## 安全卡（贴在手边）

1. 第一次：牵引绳/测试架 + 2 m 空间。
2. 调参中车会前后移动，手不离电源开关。
3. `Ctrl+C` 关 app **不会**让车停下；用 `standby`、复位或断电。
4. 自由平衡时不要按 `t` 改 setpoint。
5. 软件限幅：输出 ±20%、倾倒 25° 锁定；不要为了“更有力”擅自放宽。
6. 电池电压低时先充电，再调参。