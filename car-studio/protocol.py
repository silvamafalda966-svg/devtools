"""
Strawberry_car 串口协议层
- JustFloat 32B 帧解析 (STM32 → PC)
- 文本命令编码 (PC → STM32)
"""

import struct
import math
from dataclasses import dataclass, field
from typing import Optional

# ── 常量 ──────────────────────────────────────────
MAGIC_TAIL = 0x7F800000          # 帧尾标识 (IEEE 754 +Inf)
MAGIC_BYTES = struct.pack("<I", 0x7F800000)  # b'\x00\x00\x80\x7f'
FRAME_SIZE = 32                   # 8 float × 4 bytes
FLOAT_COUNT = 8


# ── 解析结果 ──────────────────────────────────────
@dataclass
class CarData:
    """单帧解析结果 + 里程计位姿"""
    # 原始通道
    fl_speed: float = 0.0
    fr_speed: float = 0.0
    bl_speed: float = 0.0
    br_speed: float = 0.0
    ir: float = 0.0
    ultrasonic: float = 0.0
    encoder_total: float = 0.0

    # 状态机（从帧尾嵌入提取）
    state: int = 0

    # 里程计 (世界坐标系, y 轴为前进方向)
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0          # 航向角 (rad), 0 = 正前方

    # 衍生
    v_forward: float = 0.0        # 车体前进速度 (m/s)
    v_angular: float = 0.0        # 车体角速度 (rad/s)
    timestamp: float = 0.0        # Unix time

    def to_dict(self) -> dict:
        return {
            "fl": round(self.fl_speed, 4),
            "fr": round(self.fr_speed, 4),
            "bl": round(self.bl_speed, 4),
            "br": round(self.br_speed, 4),
            "ir": round(self.ir, 2),
            "us": round(self.ultrasonic, 2),
            "enc": int(self.encoder_total),
            "state": self.state,
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "h": round(self.heading, 4),
            "v": round(self.v_forward, 4),
            "w": round(self.v_angular, 4),
            "ts": self.timestamp,
        }


@dataclass
class DecisionData:
    """K210 分支判定结果（D: 文本行解析）"""
    pick: int = 0          # 1=采摘, 0=跳过
    cls: int = 0           # 0=红(ripe), 1=绿(unripe)
    area: int = 0          # 选中目标面积百分比
    red_cnt: int = 0       # 视野中红草莓总数
    green_cnt: int = 0     # 视野中绿草莓总数

    def to_dict(self) -> dict:
        return {
            "pick": self.pick,
            "cls": self.cls,
            "area": self.area,
            "red_cnt": self.red_cnt,
            "green_cnt": self.green_cnt,
        }


# ── 帧解析器 ─────────────────────────────────────
class JustFloatParser:
    """
    JustFloat 帧同步解析器。

    帧格式 (32B LE):
      ch0~ch3: 四轮速度 (m/s)
      ch4:     红外传感器
      ch5:     超声波距离 (cm)
      ch6:     编码器总计
      ch7:     0x7F800000 (帧尾标识)

    用法:
      parser = JustFloatParser()
      for frame in parser.feed(incoming_bytes):
          process(frame)
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[dict]:
        """喂入原始字节，返回解析完成的帧列表"""
        self._buf.extend(data)
        frames = []

        # 在缓冲区中搜索帧尾标识（精确 4 字节匹配）
        while len(self._buf) >= FRAME_SIZE:
            idx = self._buf.find(MAGIC_BYTES)
            if idx < 0:
                keep = min(len(self._buf), 3)
                self._buf = self._buf[-keep:] if keep else bytearray()
                break

            if idx < FRAME_SIZE - 4:
                self._buf = self._buf[idx + 4:]
                continue

            frame_start = idx - (FRAME_SIZE - 4)
            frame_bytes = self._buf[frame_start:idx + 4]
            self._buf = self._buf[idx + 4:]

            parsed = self._parse_frame(frame_bytes)
            if parsed is not None:
                frames.append(parsed)

        # 防止缓冲区无限增长
        if len(self._buf) > 2048:
            self._buf = self._buf[-512:]

        return frames

    def _parse_frame(self, data: bytes) -> Optional[dict]:
        """解析单个 32B 帧为字典，失败返回 None"""
        try:
            floats = struct.unpack("<8f", data)
        except struct.error:
            return None

        # 验证帧尾
        tail_bits = struct.unpack("<I", data[28:32])[0]
        if tail_bits != MAGIC_TAIL:
            return None

        # ── 帧数据验证：拒绝明显失步的垃圾帧 ──
        # 检查 ch0~ch6 是否都是有限值（非 NaN/Inf）
        for i in range(7):
            if not math.isfinite(floats[i]):
                return None

        # 四轮速度应在合理范围（MG310 电机最高 ~1.5m/s，留 3× 余量）
        for i in range(4):
            if abs(floats[i]) > 5.0:
                return None

        # 超声波 0~400cm（HC-SR04 量程）
        if floats[5] < 0 or floats[5] > 450:
            return None

        return {
            "fl_speed": floats[0],
            "fr_speed": floats[1],
            "bl_speed": floats[2],
            "br_speed": floats[3],
            "ir": floats[4],
            "ultrasonic": floats[5],
            "encoder_total": floats[6],
        }

    def reset(self):
        """重置缓冲区（串口重连时调用）"""
        self._buf = bytearray()


# ── 里程计 ────────────────────────────────────────
class Odometry:
    """
    四轮差速里程计。

    运动学模型 (差速轮系):
      v_left  = (v_FL + v_BL) / 2
      v_right = (v_FR + v_BR) / 2
      v       = (v_left + v_right) / 2
      ω       = (v_right - v_left) / wheel_track

      θ += ω × Δt
      x += v × cos(θ) × Δt
      y += v × sin(θ) × Δt

    坐标系: 标准数学约定，θ 从 +x 逆时针量
      θ = π/2 → 正前方 (+y)
      θ = 0   → 正右方 (+x)
    """

    def __init__(self, wheel_track: float = 0.18):
        """
        Args:
            wheel_track: 左右轮距 (m)，默认 18cm
        """
        self.wheel_track = wheel_track
        self.x = -1.30            # 左直道中心线 X = offsetX - CR = -0.65 - 0.65
        self.y = 0.15             # START线 Y=0.15（上移15cm）
        self.heading = -math.pi/2  # 赛道实测: 初始朝向+80°修正
        self._last_ts: Optional[float] = None
        # 陀螺仪融合：航向 = 初始航向 + (当前yaw - 初始yaw)
        self.GYRO_SCALE = 1.0      # 陀螺仪比例修正（1.0=不修正，微调用）
        self._gyro_yaw = None     # 最新陀螺仪 yaw (rad)
        self._gyro_yaw_ts = 0.0   # 最后收到陀螺仪数据的时间
        self._gyro_first_yaw = None     # 第一帧陀螺仪 yaw
        self._gyro_first_heading = None # 第一帧时的编码器航向（仅做初始对齐）
        self._gyro_refreshed = False    # 本轮是否有新数据

    def feed_gyro_yaw(self, yaw_deg: float, ts: float):
        """接收陀螺仪 Y: 行的 yaw 角度（度）"""
        yaw_rad = math.radians(yaw_deg * self.GYRO_SCALE)
        if self._gyro_first_yaw is None:
            self._gyro_first_yaw = yaw_rad
            self._gyro_first_heading = self.heading  # 对齐起点
        self._gyro_yaw = yaw_rad
        self._gyro_yaw_ts = ts
        self._gyro_refreshed = True

    def update(self, fl: float, fr: float, bl: float, br: float,
               timestamp: float) -> CarData:
        """输入四轮速度和当前时间戳，返回带位姿的 CarData"""
        data = CarData(
            fl_speed=fl, fr_speed=fr, bl_speed=bl, br_speed=br,
        )

        # 计算车体速度
        v_left = (fl + bl) / 2.0
        v_right = (fr + br) / 2.0

        # 左右轮速偏差补偿：仅在车运动时生效，静止时免漂移
        # （MPU6050 陀螺仪已接管航向，此补偿仅影响 v_forward 位置积分）
        avg_speed = (abs(v_left) + abs(v_right)) / 2.0
        if avg_speed > 0.01:
            v_right -= 0.0059

        data.v_forward = (v_left + v_right) / 2.0

        if abs(self.wheel_track) > 1e-6:
            data.v_angular = (v_right - v_left) / self.wheel_track
        else:
            data.v_angular = 0.0

        # 积分位姿 — 固定 dt=0.01s（PID 控制周期 10ms）
        dt = 0.01

        # 航向 100% 由陀螺仪控制：heading = 初始航向 + (当前yaw - 初始yaw)
        if (self._gyro_refreshed and self._gyro_first_yaw is not None):
            delta = self._gyro_yaw - self._gyro_first_yaw
            self.heading = self._gyro_first_heading + delta
            self._gyro_refreshed = False
        # 陀螺仪未就绪 → 航向不变

        self.x += data.v_forward * math.cos(self.heading) * dt
        self.y += data.v_forward * math.sin(self.heading) * dt

        data.x = self.x
        data.y = self.y
        data.heading = self.heading
        data.timestamp = timestamp
        return data

    def reset(self):
        """重置位姿到原点，heading=π/2=正前方"""
        self.x = -1.30           # 左直道中心线
        self.y = 0.15            # START线，与 __init__ 一致
        self.heading = -math.pi/2  # 和 __init__ 一致
        self._last_ts = None
        self._gyro_yaw = None
        self._gyro_first_yaw = None
        self._gyro_first_heading = None
        self._gyro_refreshed = False

    def set_position(self, x: float, y: float, heading: float):
        """强制设置位姿（用于标记修正）"""
        self.x = x
        self.y = y
        self.heading = heading
        self._last_ts = None

    def set_wheel_track(self, value: float):
        """动态修改轮距"""
        self.wheel_track = value


# ── 命令编码 ──────────────────────────────────────
def encode_command(cmd: str) -> bytes:
    """
    编码文本命令为串口发送格式。

    支持的命令:
      W0~W3/WA   - 选轮
      P/I/D       - 设 PID 参数
      T           - 设目标速度
      M0~M9       - 状态切换
      MD          - 进 DEBUG
      F           - 设前馈
      L           - 设滤波 α
      raw string  - 任意原始命令
    """
    cmd = cmd.strip()
    if not cmd.endswith("\n"):
        cmd += "\n"
    return cmd.encode("ascii", errors="replace")


def decode_response(data: bytes) -> str:
    """解码 STM32 文本响应"""
    return data.decode("ascii", errors="replace").strip()


def _clean_line(line: str, prefix: str) -> Optional[str]:
    """从可能被二进制污染的 line 中提取 prefix 之后的内容"""
    idx = line.find(prefix)
    if idx < 0:
        return None
    rest = line[idx + len(prefix):]
    # 截断到第一个非预期字符（只保留数字、逗号、负号）
    i = 0
    while i < len(rest) and rest[i] in '0123456789,-':
        i += 1
    return rest[:i] if i > 0 else None


def parse_decision_line(line: str) -> Optional[DecisionData]:
    """
    解析 STM32 发送的 D: 决策文本行。
    格式: D:<pick>,<cls>,<area>,<red_cnt>,<green_cnt>[,<frames_in>,<frames_ok>]
    """
    line = line.strip()
    clean = _clean_line(line, "D:")
    if not clean:
        return None
    try:
        parts = clean.split(",")
        if len(parts) < 5:
            return None
        return DecisionData(
            pick=int(parts[0]),
            cls=int(parts[1]),
            area=int(parts[2]),
            red_cnt=int(parts[3]),
            green_cnt=int(parts[4]),
        )
    except (ValueError, IndexError):
        return None


def parse_yaw_line(line: str) -> Optional[dict]:
    """
    解析 STM32 发送的 Y: 陀螺仪航向角文本行。
    格式: Y:<yaw_milli_deg>,<alive>,<fail_cnt>  （yaw_deg × 100）
    返回: {"yaw": float, "alive": int, "fail_cnt": int} 或 None
    """
    line = line.strip()
    clean = _clean_line(line, "Y:")
    if not clean:
        return None
    try:
        parts = clean.split(",")
        yaw_deg = float(parts[0]) / 100.0  # 毫度 → 度
        alive = int(parts[1]) if len(parts) > 1 else 0
        fail_cnt = int(parts[2]) if len(parts) > 2 else 0
        return {"yaw": yaw_deg, "alive": alive, "fail_cnt": fail_cnt}
    except (ValueError, IndexError):
        return None


def parse_mpu_line(line: str) -> Optional[dict]:
    """
    解析 STM32 发送的 M: MPU6050 初始化信息。
    格式: M:<mpu_ok>,<gyro_z_st_pct>,<bias_centi_dps>,<alive>
    返回: {"ok": int, "st_z_pct": int, "bias_dps": float, "alive": int} 或 None
    """
    line = line.strip()
    clean = _clean_line(line, "M:")
    if not clean:
        return None
    try:
        parts = clean.split(",")
        ok = int(parts[0])
        st_z_pct = int(parts[1]) if len(parts) > 1 else 0
        bias_dps = float(parts[2]) / 100.0 if len(parts) > 2 else 0.0
        alive = int(parts[3]) if len(parts) > 3 else 0
        return {"ok": ok, "st_z_pct": st_z_pct, "bias_dps": bias_dps, "alive": alive}
    except (ValueError, IndexError):
        return None


def parse_warmup_line(line: str) -> Optional[dict]:
    """
    解析 STM32 发送的 W: 预热进度文本行。
    格式: W:<pct>,<temp_decideg>
    返回: {"pct": int, "temp_c": float} 或 None
    """
    line = line.strip()
    clean = _clean_line(line, "W:")
    if not clean:
        return None
    try:
        parts = clean.split(",")
        pct = int(parts[0])
        temp_c = float(parts[1]) / 10.0 if len(parts) > 1 else 0.0
        return {"pct": pct, "temp_c": temp_c}
    except (ValueError, IndexError):
        return None


def parse_marker_line(line: str) -> Optional[int]:
    """
    解析 STM32 发送的 K: 标记检测文本行。
    格式: K:<encoder_count>
    """
    line = line.strip()
    clean = _clean_line(line, "K:")
    if not clean:
        return None
    try:
        return int(clean)
    except ValueError:
        return None


def parse_state_line(line: str) -> Optional[int]:
    """
    解析 STM32 发送的 S: 状态文本行。
    格式: S:<state_id>
    """
    line = line.strip()
    clean = _clean_line(line, "S:")
    if not clean:
        return None
    try:
        state_id = int(clean)
        if 0 <= state_id <= 9:
            return state_id
    except ValueError:
        pass
    return None


# ── 累计统计（C: 文本行） ──────────────────────────

@dataclass
class SummaryData:
    """Zone 完成后的累计统计（C: 文本行解析）"""
    zone: int = 0           # 刚完成的区域编号
    red_total: int = 0      # 累计红草莓采摘数
    green_total: int = 0    # 累计绿草莓跳过数

    def to_dict(self) -> dict:
        return {
            "zone": self.zone,
            "red_total": self.red_total,
            "green_total": self.green_total,
        }


def parse_summary_line(line: str) -> Optional[SummaryData]:
    """
    解析 STM32 发送的 C: 累计统计文本行。
    格式: C:<zone>,<red_total>,<green_total>
    """
    line = line.strip()
    clean = _clean_line(line, "C:")
    if not clean:
        return None
    try:
        parts = clean.split(",")
        if len(parts) != 3:
            return None
        return SummaryData(
            zone=int(parts[0]),
            red_total=int(parts[1]),
            green_total=int(parts[2]),
        )
    except (ValueError, IndexError):
        return None
