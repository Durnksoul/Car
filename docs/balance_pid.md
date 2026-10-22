# 两轮平衡车 PID 与串口使用

完整日志已恢复：角度行、编码器行、telemetry、ESP 日志和指令回复均正常输出。
PID 已接入电机控制任务；上电保持 STANDBY，增益为零，不自动运行。
当前版本未经过实车直立调参，主机测试和编译通过不代表小车已经能自行站立。

上位机自动调参已接入 LLM-PID-Tuner：`CONFIG_CAR_PID_TUNER` 打开后以 20 Hz 输出
`timestamp_ms,setpoint,input,pwm,error,p,i,d`，并接受大写 `STATUS` / `SET P:.. I:.. D:..` /
`SETPOINT:..` / `RESET`。`SET` 可在平衡运行中直接改增益并自动接管，协议与安全说明见
[用 LLM-PID-Tuner 调直立环](llm_pid_tuner.md)。

## 已确认的方向

- 左轮编码器 GPIO17/18，右轮 GPIO1/2，两轮前进均为正。
- 编码器只计 A/B 两相下降沿，为二倍频；CPS 是当前计数方式下的计数/秒。
- 右编码器的方向已在采集层取反，PID 的 encoder_sign 保持 `{1, 1}`。
- 电机逻辑输出正值使用 A 正、B 反的驱动极性。
- 前倾使独立 `tilt_deg` 增大，使用匹配的 `tilt_rate_dps`，不用网页 Roll/Pitch 作为平衡角。
- 姿态 PID 内部为 `目标-测量`，电机输出符号默认 -1，使前倾对应逻辑正输出。
  仍需手扶确认正输出是否确实使整车前进；不能只看两轮转向一致。

## 控制逻辑

```text
目标速度 0 - 两轮平均 CPS → 速度 PI/PID → 倾角目标修正（最多 ±6°）
直立参考 + 倾角修正 - tilt → 姿态 PID → 两轮共同 PWM
```

姿态微分使用陀螺仪角速度，带低通；积分限幅，输出饱和时阻止积分继续增加。
速度环按至少 20 ms 的实际累计时间运行，默认增益为零；转向环保留在算法中，当前串口不启用转向调参。
先调姿态 PD，再调速度 PI；当前目标速度、目标转向均为零。
电机只由一个任务操作，手动 PWM 和平衡模式互斥。
IMU 任务发布最新姿态，电机任务读取最新样本，相同时间戳不会重复积分。
当前 FreeRTOS tick 为 10 ms，电机任务至少每 tick 检查一次；实际周期也受任务调度和串口输出影响。
PID 使用实际 IMU 时间差，不承诺硬实时 120 Hz；遥测约 5 Hz，不能凭这些低频行分析全部快速振荡。

## 指令

每条以回车或换行结束。新增命令使用下列小写拼写。

| 指令 | 作用 |
| --- | --- |
| `balance zero` | 待机时把手扶直立角设为本次运行参考；要求新鲜数据、角度在 ±15°、角速度不超过 3°/s、两轮速度不超过 10 CPS |
| `pid attitude Kp Ki Kd` | 待机设置姿态参数；非负有限值，最大 100 |
| `pid speed Kp Ki Kd` | 待机设置速度参数；单位 CPS→度，应从远小于姿态增益的数值试起 |
| `pid show` / `balance status` | 查看状态、参考角、方向、输出限幅和增益 |
| `balance motor -1` | 默认姿态输出方向 |
| `balance motor 1` | 反转姿态输出方向；必须在待机修改 |
| `balance arm` | 检查后使能并让 PID 接管；需待机、已校准、姿态 Kp>0、传感器新鲜且偏离参考不超过 5° |
| `balance off` / `standby` | 退出平衡并关闭驱动 |

大写 `STATUS`、`SET P:x I:y D:z`、`SET KP:x KI:y KD:z`、`SETPOINT:x`、`RESET`、`STANDBY` 属于上位机调参入口，与上表小写命令互不冲突；`SET` 不要求待机，并会清空姿态环积分/微分状态。

增益和直立参考角保存在 NVS，复位后自动恢复；上电仍是 STANDBY，发 `balance arm` 即可接管。重新 `balance zero` 会覆盖保存的参考角。
手动模式仍使用 `enable` 后 `ab 20 20`；进入平衡模式不用先发 `enable`。
平衡运行时拒绝手动速度、刹车、重复 enable、编码器清零和参数修改，查询仍可使用。

## 首次方向实验

扶稳车身并让车轮有转动空间，先保持直立静止。以下是一组低增益方向试验参数，不是已调好的直立参数：

```text
standby
balance zero
pid attitude 0.5 0 0.02
pid speed 0 0 0
pid show
balance arm
```

小幅前倾应两轮向前追，小幅后倾应反向。
若反应方向错误，先发 `standby`，再用 `balance motor 1` 切换并重新测试。
方向确认前不要放手。方向正确后才分批调整姿态 Kp/Kd，并记录输出与响应；不要把上述小增益是否能站立作为通过标准。
停止使用 `standby`，不要只关闭串口软件。

## 停机条件与遥测

- 输出暂限 ±20%；偏离校准点达到 25°停机。
- IMU 无效、数据年龄超过 50 ms、编码器年龄超过 100 ms、控制样本间隔超过 50 ms，以及驱动失败，均退出闭环并请求 STBY=0。
- 故障锁定，不因姿态恢复自动重启，需要重新 `balance arm`。
- 串口中断本身不退出平衡；本机闭环独立运行，此版未加入上位机心跳。

`telemetry` 保留原字段，新增真实状态及输出：

- `balance_active`：1 代表平衡已接管；0 代表未运行。
- `balance_state`：0 待机、1 运行、2 输入无效、3 过期、4 时序故障、5 倾倒。
- `trim_deg`：当前校准参考；`target_tilt_deg`：叠加速度环后的目标。
- `pid_left/right`：PID 输出的逻辑百分比，保留小数；`cmd_left/right` 为四舍五入后的指令百分比。
- `enabled` 为软件使能状态，不是实际电流或 STBY 电压测量。

临时编码器专用日志的源文件备份仍保留在 `.local-backups/`；恢复完整日志采用开关置零，不覆盖后来修正的编码器方向。

## 主机验证

```powershell
python tests/test_car_balance_runtime.py
python tests/test_balance_controller.py
python tests/test_car_encoder.py
python tests/test_tb6612_command.py
python tests/test_tb6612fng.py
```

运行时测试执行实际命令和闭环函数，模拟时钟、传感器及电机接口，覆盖校准/启动门槛、参数拒绝、方向、模式互斥、失效停机、显式重新启动及电机接口失败。
主机测试不替代烧录和实物方向、供电、机械平衡点及增益测试。
