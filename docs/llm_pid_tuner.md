# 用 LLM-PID-Tuner 调直立环

本文说明本固件与开源项目 [KINGSTON-115/llm-pid-tuner](https://github.com/KINGSTON-115/llm-pid-tuner) 的对接方式、已实现的协议、推荐操作顺序和安全边界。第一次操作请按 [逐步教程](llm_pid_tuner_tutorial.md) 走。

该开源工具原本面向“设定值跟踪”的温控/电机对象：固件持续上报 `timestamp,setpoint,input,pwm,error,p,i,d`，上位机收集一批样本、计算指标、请 LLM 给出新 PID，再通过串口写回。本项目的直立环只能提供“扰动恢复”数据（目标始终是直立参考角），因此它适合用来整定**姿态环 Kp/Ki/Kd**，不是替代人工方向确认和机械检查。

## 1. 固件侧已实现的内容

### 1.1 编译开关

| 开关 | 位置 | 作用 |
| --- | --- | --- |
| `CONFIG_CAR_PID_TUNER` | `idf.py menuconfig → Car control` | 打开上位机协议、20 Hz CSV，并关闭会干扰解析的角度/编码器/telemetry 行 |
| `CONFIG_CAR_PID_TUNER_AUTO_ARM` | 同上 | 允许上位机的 `SET` 在车体直立静止时自动校准直立角并接管 |
| `CAR_ENCODER_ONLY_LOG` | `main/car_log_mode.h` | 必须为 `0`，否则只有编码器行 |

`CONFIG_CAR_PID_TUNER` 默认打开，串口仍为 **115200 8N1**，UART0（GPIO43 TX / GPIO44 RX）。

### 1.2 上报格式（20 Hz）

每 50 ms 一行，单次 `printf` 输出，避免被其他任务拆行：

```text
timestamp_ms,setpoint,input,pwm,error,p,i,d
```

| 字段 | 本项目含义 | 单位 |
| --- | --- | --- |
| `timestamp_ms` | `esp_timer` 毫秒 | ms |
| `setpoint` | 目标倾角 − 直立参考角（正常为 0；速度环有输出时会叠加） | ° |
| `input` | 当前 `tilt_deg` − 直立参考角 | ° |
| `pwm` | 姿态 PID 输出（左右轮公共量） | % |
| `error` | `setpoint - input`，与实际 PID 误差一致 | ° |
| `p,i,d` | 当前姿态环增益 | ° → % |

于是上位机的 PID 增益可直接对应固件的姿态环增益，无需换算。

### 1.3 上位机命令（只认大写）

| 命令 | 行为 |
| --- | --- |
| `STATUS` | 回一行 `# STATUS: Kp=... Ki=... Kd=... Setpoint=... Temp=... Error=... Armed=... State=... Trim=...` |
| `SET P:x I:y D:z` | 运行中直接更新姿态环增益，清空积分/微分状态，不退出平衡 |
| `SET KP:x KI:y KD:z` | 同上 |
| `SETPOINT:x` | 把目标倾角设为“直立参考 + x°”，x 限幅 ±5°，只建议在有人扶车时使用 |
| `RESET` | 清空三个 PID 的积分/微分状态，保留当前增益 |
| `STANDBY` | 退出平衡并 STBY=0（上位机不会自动发送，供人工使用） |

安全限制：

- 增益限幅：`Kp ≤ 50`、`Ki ≤ 20`、`Kd ≤ 5`，负值/NaN/Inf 拒绝；实际值会出现在下一行 CSV 的 `p,i,d` 中，上位机会据此更新自己的记录。
- 平衡运行中拒绝 `Kp = 0`（会立刻失去直立能力）。
- 只有小写 `status` / `standby` 仍是原人工命令；两种入口互不影响。

### 1.4 自动接管（`CAR_PID_TUNER_AUTO_ARM`）

上位机启动时只发 `STATUS` 和 `SET`，不会发 `balance zero` / `balance arm`，因此：

1. 收到第一条合法 `SET` 且 `Kp > 0`；
2. 若尚未做过 `balance zero`，且倾角在 ±10°、角速度 ≤ 5°/s、两轮 ≤ 10 counts/s，则把当前倾角记为直立参考；
3. 通过 `balance_controller_arm` 的全部门槛后使能驱动并进入闭环。

若车体未扶正、在晃动或 `Kp = 0`，则不接管，CSV 中 `pwm` 保持 0。放正扶稳后下一次 `SET`（下一轮）会重试。倾倒、IMU 失效、数据超时仍按原逻辑故障锁定并 `STBY=0`，不会自动重启。

## 2. 调参前的硬性检查

这些不通过就不要启动自动调参，否则 LLM 会在错误被控对象上“调参”：

1. **姿态方向**：前倾时 `tilt_deg` 增大；`balance motor -1`（固件默认）下，前倾应使两轮向前追。
2. **电机方向**：`enable` 后 `ab 20 20`，两轮都应让车向前。B 路已在应用层反相。
3. **编码器方向**：手推前进时 `cps_left/right` 都为正；右编码器已在采集层反相。
4. **直立参考**：扶正后 `balance zero` 成功（或让自动接管记录），`trim_deg` 合理。
5. **机械**：轮胎不打滑、电池电压足够、IMU 与电机线固定、车体不会碰到轮子。
6. **安全**：第一次务必手扶或使用测试架/牵引绳；软件限幅 20%、倾倒 25° 停机只是最后一道保护。

## 3. 推荐的 `config.json`

`HARDWARE_PROFILE` 必须是 `generic_serial_csv`。建议把 PID 限幅收紧到本项目量纲（LLM 默认允许 P 到 1000，对 °→% 的直立环过于激进）：

```json
{
  "SERIAL_PORT": "COM10",
  "BAUD_RATE": 115200,
  "HARDWARE_PROFILE": "generic_serial_csv",
  "BUFFER_SIZE": 100,
  "MIN_ERROR_THRESHOLD": 0.8,
  "GOOD_ENOUGH_AVG_ERROR": 1.2,
  "GOOD_ENOUGH_STEADY_STATE_ERROR": 0.8,
  "GOOD_ENOUGH_OVERSHOOT": 10.0,
  "REQUIRED_STABLE_ROUNDS": 3,
  "MAX_TUNING_ROUNDS": 30,
  "PID_MAX_INCREASE_RATIO": 0.0,
  "PID_LIMITS": {
    "default": {
      "p": { "min": 0.0, "max": 20.0, "max_increase_ratio": 2.0 },
      "i": { "min": 0.0, "max": 8.0, "max_increase_ratio": 2.0 },
      "d": { "min": 0.0, "max": 2.0, "max_increase_ratio": 2.0 }
    }
  }
}
```

说明：

- `BUFFER_SIZE=100`、20 Hz → 每轮约 5 s，`MIN_SAMPLES_PER_ROUND=50` 约 2.5 s。
- `GOOD_ENOUGH_*` 的单位是“度”。取 1.2°/0.8° 比默认 0.1/0.2 更符合实车噪声；若一开始就满足，工具会直接进入观察并停止。
- 固件还有一层 50/20/5 的硬限幅，即使上位机配置更宽也不会越界。
- 上位机界面里要求填“初始 PID”时，填一组已知能站的低增益（例如 `P=4, I=0, D=0.3`），否则第一轮可能因 `Kp=0` 无法接管而浪费一轮。

## 4. 一次完整调参流程

```text
1. 烧录带 CONFIG_CAR_PID_TUNER 的固件，确认串口出现
   "# LLM-PID-Tuner ready: CSV timestamp_ms,setpoint,input,pwm,error,p,i,d at 20 Hz"
2. 上电后保持静止 3 s 完成 Yaw 归零；把车扶到直立、静止。
3. 关闭其他串口工具（Monitor、网页查看器），运行：
   llm-pid-tuner.exe tune COM10
   选择串口硬件 → 输入初始 PID → 开始调参。
4. 第一条 SET 后固件自动记录直立角并接管；手扶感受方向：
   前倾 → 车轮向前追。方向反了立刻断电/standby，先改 balance motor 符号，
   不要继续调参。
5. 每轮数据采集时（约 5 s）用手轻推车体 ±2° 再松手，给闭环一个可观测的扰动。
6. 观察 TUI 的指标和 LLM 决策；车辆会为了保持平衡而前后移动，
   第一次请用牵引绳/测试架，并留出至少 2 m 空间。
7. 收敛后从工具里抄下最终 P/I/D。当前固件不写 NVS，复位即丢；
   把数值写进固件默认值，或每次上电用 pid attitude 重新设置。
8. 用第 2 节的检查流程人工复跑一遍，确认不是只在某一轮数据里“看起来稳定”。
```

## 5. 仓库原生调参循环（本项目逐步对照）

开源项目的真实硬件流程是“采样 → 算指标 → LLM 改参 → 护栏/回退 → 稳定即停”，不是人工试凑。对应到本车：

| 仓库步骤 | 本车实际发生的事 |
| --- | --- |
| 1. （可选）预调参对话 | 用自然语言写约束，例如“不能持续超调、车不能跑出 2 m、电机不能堵转”；这些会进 LLM 提示词 |
| 2. 连接串口并发 `STATUS` | 固件回 `# STATUS: ...`，供人看；上位机只解析 CSV |
| 3. 读取/填写初始 PID | 若界面要求填写，填当前基线；否则第一轮 `pwm=0`，没有可分析响应 |
| 4. 下发 `SET P:I:D` | 固件**自动记录直立角并接管**（`CAR_PID_TUNER_AUTO_ARM`），开始 20 Hz CSV |
| 5. 累积 `BUFFER_SIZE` 样本 | 默认 100 点 × 50 ms ≈ 5 s 一轮 |
| 6. 计算指标 | `avg_error`、`max_error`、`steady_state_error`、`zero_crossings`，`status` 判 STABLE/OSCILLATING/SLOW_RESPONSE |
| 7. LLM 返回新 P/I/D | 先经 `PID_LIMITS` 与 `max_increase_ratio` 限幅 |
| 8. 下发新 `SET` | 固件运行中直接改增益（再限 Kp≤50/Ki≤20/Kd≤5），下一行 CSV 回读实际值 |
| 9. 恶化则回退，够好则观察 | 连续 `REQUIRED_STABLE_ROUNDS`（默认 3）轮达标后停止，不会无限调 |
| 10. 结束 | 最终 P/I/D 和 trim 自动写入 NVS，上电恢复；不需要重新下发 |

### 5.1 指标在平衡车上的含义

| 指标 | 本车含义 | 建议阈值 |
| --- | --- | --- |
| `avg_error` | 一轮内平均倾角偏差（°） | 1.0~1.5 |
| `steady_state_error` | 最后 20% 样本的平均偏差（°） | 0.5~1.0 |
| `overshoot` | **setpoint=0 时恒为 0**（源码只在 setpoint≠0 时计算），不要依赖它 | — |
| `zero_crossings` | 误差过零次数，>30% 样本判 OSCILLATING；这是调姿态环最有用的指标 | 越低越好 |
| `status` | STABLE / OSCILLATING / SLOW_RESPONSE | 目标是 STABLE |

因此：

- 姿态环用 **setpoint=0 + 人工轻推扰动** 即可，LLM 能从 20 点采样里看到振荡形状；
- 想要真正的“阶跃”，在 TUI 里按 `t` 给 ±1~2° 的 `SETPOINT`，但车会朝该方向持续加速，必须有人扶或牵引绳；
- `system_id.py` 的阶跃辨识适合以后整定**速度环**（轮子架空、给固定 PWM 测 CPS），不适合开环辨识会倒的直立环。

### 5.2 建议的首次仓库流程（最少人工干预）

```text
balance zero                 # 只做一次，给固件一个正确的直立参考
pid attitude 4 0 0.3         # 基线，避免第一轮 pwm=0；不是最终值
balance arm                  # 手扶确认前倾→向前，方向反了先 balance motor 1
（关闭 VOFA+ 等占用 COM10 的工具）
llm-pid-tuner.exe tune COM10 # 初始 PID 填 4 / 0 / 0.3
```

之后每一轮采集的 5 s 内用手轻推车体 ±2°，让 `zero_crossings` 和 `avg_error` 有内容；不要在自由平衡时按 `t` 改 setpoint。

## 6. 已知限制与结果解读

- 本工具的核心指标是设定值阶跃响应。平衡车的设定值固定为 0°，看到的是**扰动恢复**：
  `average error` 近似平均倾角偏差，`steady_state_error` 近似静态偏差，
  `overshoot` 对平衡车不如“恢复时间/振荡次数”直观。LLM 仍能根据曲线和统计量给出方向性建议，但不要迷信它的“DONE”。
- `pwm` 只上报姿态环公共输出，不含转向差速；转向环当前增益为 0。
- 速度环默认全零，车会缓慢漂移。直立环稳定后再单独整定速度环
  （需要另做一套“CPS → 目标倾角”的上报/命令映射，当前未接）。
- 固件没有上位机心跳：串口断开后闭环继续运行。这是有意设计，但也意味着
  `Ctrl+C` 不会让车停下，必须用 `standby`、复位或断电。
- 上位机打开串口时若触发 DTR/RTS 复位，固件会回到待机；自动接管会在下一条
  `SET` 重新校准，但直立时应有人扶住。无线 UART（只有 TX/RX/GND）不会复位。
- 自动接管依赖“当前扶正姿态就是平衡点”。若机械重心明显偏前/偏后，
  先人工 `balance zero` 再启动工具，或在调参后用 `SETPOINT` 微调。

## 7. 相关文件

- 协议实现：`main/car_motor_control.c`（`tuner_command` / `tuner_try_arm` / `tuner_stream`）
- 开关：`main/Kconfig.projbuild`、`main/car_log_mode.h`
- 直立环：`main/balance_controller.c`、`main/pid_controller.c`
- 主机测试：`tests/test_car_balance_runtime.py`
- 调参原理与人工命令：`docs/balance_pid.md`