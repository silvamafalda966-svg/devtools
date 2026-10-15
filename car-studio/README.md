# car-studio

> 调车综合平台 —— 把串口里看不见的数据变成看得见的图

基于 **FastAPI + WebSocket + Canvas** 的嵌入式调车实时监控平台。
连接 STM32 后，四轮速度、传感器状态、状态机跳转、车体轨迹全部实时可视化。

## 功能模块

### 1. 仪表盘
- 四轮速度实时条 + 目标速度对比
- **6 路红外传感器阵列**实时显示（黑线 / 白底）
- 超声波测距、K210 帧计数
- 陀螺仪航向罗盘 + 里程计位姿 (x, y, θ)
- 数据帧率 / 丢帧统计

### 2. 实时波形
- 四轮速度波形（Chart.js）
- 状态切换时间线彩条，直观看出状态何时跳转
- 可调时间窗口

### 3. 赛道轨迹图（Canvas）
- 基于**四轮编码器差速里程计**实时推算车体位姿
- 支持缩放 / 拖拽 / 悬停查看坐标
- 叠加赛道底图（直道 + 弯道 + 分支线）
- 陀螺仪航向与里程计航向融合，抑制累积漂移
- 右键清空轨迹

### 4. 状态机可视化
- Canvas 绘制状态转移图，当前状态脉冲高亮
- 显示最近 20 条状态转移日志
- 支持一键切换状态（M 命令）

### 5. 录制与回放
- 一键录制全通道数据到 JSON
- 回放时波形 + 轨迹动画同步
- 录制列表支持 **CSV 导出**，可用 Excel / Python 二次分析

### 6. 参数方案管理
- 保存 / 加载多套 PID 参数组合
- 多轮数据叠图对比 + 自动指标表格（超调、上升时间、稳态误差）

## 运行

```bash
pip install -r requirements.txt

python server.py COM5                 # 连接串口启动
python server.py COM5 --track 0.18    # 指定轮距（米）
python server.py --no-serial          # 离线模式，回放已有录制
python server.py COM5 -p 8000         # 指定端口
```

浏览器打开 `http://localhost:8000`

## 架构

```
STM32 ──串口──> server.py ──WebSocket──> 浏览器
                    │
                    └── protocol.py
                        ├── 二进制帧解析 (JustFloat)
                        ├── 文本行解析 (S: / D: / C: / K:)
                        └── 里程计推算 (差速模型)
```

**关键实现：二进制流与文本行混流解析**

STM32 同时发送 JustFloat 二进制帧（32 字节）和 ASCII 文本行（状态变更等）。
二进制帧中的字节可能恰好等于 `\n`，导致纯按行切分的解析器把二进制垃圾混进文本缓冲区，
使 `S:5` 被解析成 `S:5\x99` 而静默丢弃。

解决方案：按 `\n` 切分原始字节流后，在每行内**搜索前缀**而非匹配行首，
并只保留 `[0-9,-]` 字符截断后缀 —— 见 `protocol.py` 的 `_clean_line()`。

## 文件说明

| 文件 | 说明 |
|---|---|
| `server.py` | FastAPI 服务、串口读取线程、WebSocket 广播 |
| `protocol.py` | 协议解析 + 里程计推算 + 位姿修正 |
| `analyze.py` | 录制数据分析：四轮速度差异、朝向漂移率 |
| `read_ir_adc.py` | 红外传感器阈值调试辅助 |
| `static/index.html` | 前端单页应用（Canvas + Chart.js） |
| `profiles/` | 参数方案持久化存储 |
| `recordings/` | 录制数据（已 gitignore） |

## 界面预览

> 📷 待补充截图

| 仪表盘 | 赛道轨迹 |
|---|---|
| _screenshot-dashboard_ | _screenshot-track_ |

## 相关

- [devtools](../) —— 所属工具集
- **Strawberry_car** —— 配套 STM32 固件
