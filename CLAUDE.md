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

### car-studio

调车综合平台（Web 版），FastAPI + WebSocket + Chart.js + Canvas 轨迹图。

运行:
```
cd car-studio
pip install -r requirements.txt
python server.py COM5                     # 连串口启动
python server.py COM5 --track 0.18        # 指定轮距
python server.py --no-serial              # 离线回放模式
```
→ 浏览器打开 `http://localhost:8000`

五大模块:
1. **仪表盘** — 四轮速度条、传感器读数、里程计轨迹图（缩放/拖拽/悬停）、航向罗盘
2. **PID 调参** — 滑块即时调 P/I/D/FF/α、阶跃测试、实时波形
3. **录制回放** — 一键录制全通道数据、回放波形+轨迹动画
4. **参数方案** — 保存/加载/对比多套 PID 参数组合
5. **对比分析** — 多轮数据叠图对比、自动指标表格

轨迹图基于四轮编码器里程计（差速模型）实时推算车体位姿 (x, y, θ)。
与 pid-tuner 共享 JustFloat 协议 + 文本命令协议，固件无需改动。

核心依赖: `fastapi + uvicorn + pyserial`（后端）; Chart.js CDN（前端）

## 远期方向

- 更多工具：通用串口调试器、数据分析等
- 网页版 UI 持续完善：赛道地图校准、自动标注弯道/直道、多车对比
