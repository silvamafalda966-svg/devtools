# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目定位

嵌入式开发工具集。每个工具独立目录，自包含。设计模式：**脚本当手，AI 当大脑** — 脚本采集数据/下发指令，AI 分析决策。

## 工具架构约定

- 每个工具一个目录，包含入口脚本和可能的辅助模块
- 入口脚本支持双模式：交互模式（无参数双击运行） + CLI 模式（带参数供 AI 调用）
- CLI 模式输出 JSON 结构化数据，方便 AI 解析后决策下一步
- 依赖尽量精简（首选 pyserial + stdlib）

## 现有工具

### pid-tuner

STM32 PID 自动调参脚本，通过串口与 STM32 通信。

运行:
```
python pid_tuner.py COM5 monitor           # 实时速度显示
python pid_tuner.py COM5 step -w 0 -s 0.3 --json  # 阶跃测试，JSON 输出
python pid_tuner.py COM5 tune -w 4 -n 5    # 全自动扫参
python pid_tuner.py COM5 set -w 3 --ff 40  # 设前馈补偿
python pid_tuner.py COM5 raw -c "L0.5"     # 发原始串口命令
```

核心依赖: `pyserial`

## 串口协议知识（所有工具共用）

与 STM32 通信的两套协议，其他工具如用到串口交互应复用此模式：

**STM32 → PC (JustFloat 二进制):**
- 8 float = 32 字节/帧, 100Hz
- ch0~3 = 四轮速度(m/s), ch7 = 0x7F800000 (NaN 帧尾)
- 帧同步靠 NaN 尾识别

**PC → STM32 (文本命令):**
- ASCII + `\n` 结尾, 115200 8N1
- W0~W3/WA: 选轮, P/I/D: PID 参数, T: 目标速度(DEBUG 状态)
- M0~M9: 状态切换, MD: 进 DEBUG, F: 设前馈, L: 设滤波 α

**注意:** Windows 开串口会触发 DTR，可能复位 STM32。交互模式下保持串口常开，CLI 模式每次开关需处理 STM32 重连时序。

## 远期方向

网页版 UI（FastAPI + WebSocket + Chart.js），实时波形 + 滑块调参。工具内部逻辑从脚本迁移到后端时保持模块化，前端只做展示和交互。
