# devtools

> 嵌入式开发工具集 —— **脚本当手，AI 当大脑**

为 STM32 嵌入式开发打造的一套调试工具链：脚本负责采集数据、下发指令，AI 负责分析决策。
目标是把"人工试凑调参"变成"自动扫参 + 数据驱动决策"。

## 工具一览

| 工具 | 说明 | 技术栈 |
|---|---|---|
| [**car-studio**](car-studio/) | 调车综合平台（Web）：实时波形、状态机可视化、里程计轨迹、录制回放、参数方案管理 | FastAPI · WebSocket · Canvas |
| [**pid-tuner**](pid-tuner/) | PID 自动扫参 CLI：六种工作模式，自动完成 Kp/Ki/Kd 三维寻优 | Python · pyserial |
| [**bt_debug**](bt_debug/) | 蓝牙模块诊断：多 COM 口 × 多波特率自动扫描，自动识别 JustFloat 帧 | Python · pyserial |

## 背景

这套工具是在 **全国大学生嵌入式芯片与系统设计竞赛（ST 赛道）** 草莓采摘机器人的开发过程中沉淀下来的。

整车调试中遇到三个痛点：

1. **调参靠试凑** —— 四轮速度环 PID 手动调一组参数要好几分钟，且无法系统搜索
2. **数据看不见** —— 串口只会打印数字，速度波形、传感器状态、车体轨迹无法直观观察
3. **状态机不可追溯** —— 状态跳转靠猜，出问题难以定位是哪一步

对应的三个工具就长出来了。

## 设计约定

- 每个工具一个独立目录，**自包含**，可单独使用
- 入口脚本支持**双模式**：
  - 交互模式 —— 无参数运行，供人直接使用
  - CLI 模式 —— 带参数运行，输出 JSON 结构化数据，供 AI 解析后决策下一步
- 依赖尽量精简：首选 `pyserial` + 标准库

## 快速开始

```bash
pip install pyserial
```

### car-studio —— 调车综合平台

```bash
cd car-studio
pip install -r requirements.txt
python server.py COM5                    # 连接串口启动
python server.py COM5 --track 0.18       # 指定轮距
python server.py --no-serial             # 离线回放模式（无需硬件）
```

浏览器打开 `http://localhost:8000`

### pid-tuner —— PID 自动扫参

```bash
cd pid-tuner
python pid_tuner.py COM5 monitor                      # 实时速度显示
python pid_tuner.py COM5 step -w 0 -s 0.3 --json      # 阶跃测试，JSON 输出
python pid_tuner.py COM5 tune -w 4 -n 5               # 全自动扫参
python pid_tuner.py COM5 set -w 3 --ff 40             # 设置前馈补偿
python pid_tuner.py COM5 raw -c "L0.5"                # 发送原始串口命令
```

### bt_debug —— 蓝牙诊断

```bash
cd bt_debug
python check_bt.py
```

扫描 COM9 / COM10 × 5 种波特率，自动识别是否有数据、是否为 JustFloat 帧，输出原始 HEX。
用于排查蓝牙模块波特率不匹配、接线错误等问题。

## 串口协议

三个工具共用同一套与 STM32 通信的协议。

**STM32 → PC（JustFloat 二进制）**

| 项 | 说明 |
|---|---|
| 帧长 | 8 × float = 32 字节 |
| 频率 | 100 Hz |
| 通道 | ch0~3 = 四轮速度 (m/s)，ch4 = 红外，ch5 = 超声波，ch6 = 编码器，ch7 = `0x7F800000` |
| 帧同步 | 靠尾部的 NaN (`0x7F800000`) 识别帧边界 |

部分工具还使用**文本行**上报额外信息（状态变更 `S:`、决策 `D:`、统计 `C:`），与二进制帧混流发送。

**PC → STM32（文本命令）**

ASCII + `\n` 结尾，115200 8N1。

| 命令 | 作用 |
|---|---|
| `W0`~`W3` / `WA` | 选择轮次（单轮 / 全部） |
| `P` / `I` / `D` | 设置 PID 参数 |
| `T` | 设置目标速度 |
| `M0`~`M9` | 状态机切换 |
| `F` | 设置前馈补偿 |
| `L` | 设置滤波系数 α |

> ⚠️ Windows 打开串口会触发 DTR，可能复位 STM32。交互模式下应保持串口常开。

## 目录结构

```
devtools/
├── car-studio/          # 调车综合平台（Web）
│   ├── server.py        # FastAPI 服务 + WebSocket 推送
│   ├── protocol.py      # 串口协议解析 + 里程计推算
│   ├── analyze.py       # 录制数据分析（轮速差异 / 朝向漂移）
│   ├── read_ir_adc.py   # 红外传感器阈值调试辅助
│   ├── static/          # 前端（Canvas 轨迹图 / 状态机图）
│   └── profiles/        # 参数方案存储
├── pid-tuner/           # PID 自动扫参 CLI
├── bt_debug/            # 蓝牙诊断工具
└── CLAUDE.md            # AI 协作约定
```

## 相关项目

- **Strawberry_car** —— 配套的 STM32 固件（四轮 PID 闭环、陀螺仪定姿、岔路巡线、端侧视觉集成）

## License

MIT
