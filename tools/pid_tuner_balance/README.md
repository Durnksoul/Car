# 平衡车调参工具补丁（2026-10-23）

上游：[KINGSTON-115/llm-pid-tuner](https://github.com/KINGSTON-115/llm-pid-tuner)，固定版本 `v2.5.0`（提交 `6f6e9c4570f03ca22508c31409d9ac675c359d12`）。
`balance-v2.5.0.patch` 保存本次电脑端全部源码改动和回归测试，不包含 API 配置、可执行文件或 Python 环境。上游许可请遵守其 LICENSE。

## 恢复到现有 Windows 安装

原目录布局：

```text
llm-pid-tuner-v2.5.0-windows-x64/
  llm-pid-tuner-desktop/config.json     # 私有原配置，不上传
  balance-source/
    .venv/
    llm-pid-tuner-2.5.0/                # 打补丁后的上游源码
```

下载上游 v2.5.0 源码，解压到上述 balance-source 目录，在源码目录执行（PATCH_PATH 替换为本文件夹绝对路径）：

```powershell
git apply --check PATCH_PATH/balance-v2.5.0.patch
git apply PATCH_PATH/balance-v2.5.0.patch
python -m venv ../.venv
../.venv/Scripts/python.exe -m pip install requests pyserial "textual>=8.1.1,<9" "PySide6-Essentials>=6.7,<6.11" pytest
../.venv/Scripts/python.exe balance_main.py
```

入口沿用原桌面程序旁的 config.json；每台机器的 API Key 自行填写，不要提交。源码 BALANCE-README.md 有完整操作说明。

## 本次修复

- 近零目标不再用百分比超调；使用角度 RMS、摆幅、角速度差分估计和 PWM 变化评分。
- 不允许模型 DONE 跳过实测验收；等待 AI 后丢弃旧串口数据；最后一轮不下发未测的新参数。
- 每轮核对 Armed/State，记录实测 PID、建议 PID、阶段边界及历史最佳试验轮次。
- 静止检查后提示单次轻推，采集 10 秒恢复过程，连续三次完整抗扰试验通过才声明收敛；没有扰动只能报告未验证。
- 固件 CSV setpoint 包含速度环内部倾角反馈，允许变化；抗扰评分仍以物理直立零点 SP=0 为参考。
- 静止检查显示具体失败指标；测试时不自动更改速度/转向 PID。

## 当前实车候选与证据

报告：`docs/tuning-reports/pid-session-20261023-005857.json`。
姿态 PID **P=4.85、I=0.1、D=0.053** 是第 10 轮实测、程序结束时恢复的候选；没有改成固件默认值。
第 10 轮恢复末四秒摆幅 0.405 度、估计角速度 RMS 0.728 度/秒、平均直立角度误差 1.104 度，衰减比 0.352；10 秒内未满足持续进入 ±0.8 度的条件，因此并未通过完整抗扰验收。
第 11/12 轮 D=0.058/0.066 出现较大摆动，但推动和地面条件没有严格控制，不能单独认定参数或地面是原因。
速度 PID 的运行时数值没有保存在当前报告中，不从 setpoint 变化反推其具体值。
报告内 AI 的 analysis_summary 是模型意见，不是经过验证的因果结论。
后续先固定候选，在平整硬地面重复验证，再决定是否调整零点或继续调参。未宣称位置锁定、完全无漂移或已完成抗扰验收。

## 已验证

电脑端相关回归：87 项通过；离屏桌面启动、顶部提示和 JSON 导出通过。实车结果以所附报告与现场观察为准。
