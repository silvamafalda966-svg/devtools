"""
Car Studio — 调车综合平台后端
FastAPI + WebSocket + 串口管理

用法:
  python server.py COM5
  python server.py COM5 --baud 115200 --track 0.18
  python server.py --no-serial          # 纯回放模式，不连串口
"""

import argparse
import asyncio
import json
import math
import os
import queue
import sys
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import serial

# ── 项目根路径处理 ────────────────────────────────
ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
PROFILES_DIR = ROOT / "profiles"
RECORDINGS_DIR = ROOT / "recordings"


def ensure_dirs():
    PROFILES_DIR.mkdir(exist_ok=True)
    RECORDINGS_DIR.mkdir(exist_ok=True)


# ── 协议层 (本地 import，避免路径问题) ──────────────
# 直接从同目录导入
import protocol as proto

# ── FastAPI ────────────────────────────────────────
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse

app = FastAPI(title="Car Studio", version="0.1.0")

# ── 全局状态 ──────────────────────────────────────
serial_conn: serial.Serial | None = None
serial_running = False
parser = proto.JustFloatParser()
odom = proto.Odometry(wheel_track=0.1412)  # 物理左右轮距 14.12cm

# 线程间数据桥
data_queue: queue.Queue = queue.Queue(maxsize=1000)   # (timestamp, dict)
cmd_queue: queue.Queue = queue.Queue(maxsize=50)       # str

# WebSocket 客户端
ws_clients: set[WebSocket] = set()
ws_lock = asyncio.Lock()

# 录制状态
recording_active = False
recording_buf: list[dict] = []
recordings: dict[str, dict] = {}   # {id: {meta, frames}}

# K210 决策数据 & 当前状态（D:/S: 文本行解析，线程安全由 GIL 保证）
latest_decision: proto.DecisionData | None = None
latest_summary: proto.SummaryData | None = None
latest_state: int = 0


# ══════════════════════════════════════════════════════
# 串口线程
# ══════════════════════════════════════════════════════

def serial_reader_thread(port: str, baud: int):
    """后台线程：持续读取串口、解析帧、放入队列"""
    global serial_conn, serial_running, parser

    def log(msg):
        print(f"[SERIAL] {msg}")

    try:
        serial_conn = serial.Serial(port, baud, timeout=0.05)
        serial_conn.reset_input_buffer()
        serial_running = True
        parser.reset()
        log(f"已连接 {port} @ {baud}")
    except Exception as e:
        log(f"打开串口失败: {e}")
        serial_running = False
        return

    frame_count = 0
    byte_count = 0
    last_report = time.time()
    text_line_buf = bytearray()  # 文本行累积缓冲

    try:
        while serial_running:
            # 读取串口数据
            try:
                n = serial_conn.in_waiting
                if n > 0:
                    raw = serial_conn.read(n)
                    byte_count += len(raw)


                    # ── 文本行提取 ──
                    # 按 \n 切分 raw bytes，在每行中搜索 S:/D:/C:/K: 前缀
                    # 即使 VOFA+ 二进制字节混入，也能正确解析
                    raw_str = raw.decode("ascii", errors="replace")
                    lines = raw_str.split("\n")
                    for line in lines:
                        line = line.strip()
                        if not line:
                            continue
                        # V: 调试行（K210 帧计数）— 用 find 防二进制污染
                        v_idx = line.find("V:")
                        if v_idx >= 0:
                            try: log(f"K210帧计数: {int(line[v_idx+2:])}")
                            except: pass
                            continue
                        for prefix, parser_fn, msg_type in [
                            ("W:", proto.parse_warmup_line, "warmup"),
                            ("M:", proto.parse_mpu_line, "mpu"),
                            ("S:", proto.parse_state_line, "state"),
                            ("D:", proto.parse_decision_line, "decision"),
                            ("C:", proto.parse_summary_line, "summary"),
                            ("K:", proto.parse_marker_line, "marker"),
                            ("Y:", proto.parse_yaw_line, "yaw"),
                        ]:
                            idx = line.find(prefix)
                            if idx >= 0:
                                clean = line[idx:]
                                result = parser_fn(clean)
                                if result is not None:
                                    if msg_type == "warmup":
                                        log(f"预热: {result['pct']}%  {result['temp_c']:.1f}°C")
                                        try: data_queue.put_nowait((time.time(), {"_type": "warmup", "pct": result["pct"], "temp_c": result["temp_c"]}))
                                        except queue.Full: pass
                                    elif msg_type == "mpu":
                                        log(f"MPU6050: init={result['ok']}, st_z={result['st_z_pct']}%, bias={result['bias_dps']:.3f}°/s, alive={result['alive']}")
                                        try: data_queue.put_nowait((time.time(), {"_type": "mpu", "init_ok": result["ok"], "st_z_pct": result["st_z_pct"], "bias_dps": result["bias_dps"], "alive": result["alive"]}))
                                        except queue.Full: pass
                                    elif msg_type == "state":
                                        log(f"状态变更: S:{result}")
                                        try: data_queue.put_nowait((time.time(), {"_type": "state", "state": result}))
                                        except queue.Full: pass
                                    elif msg_type == "marker":
                                        log(f"标记检测: enc={result}")
                                        try: data_queue.put_nowait((time.time(), {"_type": "marker", "enc": result}))
                                        except queue.Full: pass
                                    elif msg_type == "yaw":
                                        try: data_queue.put_nowait((time.time(), {"_type": "yaw", "yaw": result["yaw"], "gyro_alive": result["alive"]}))
                                        except queue.Full: pass
                                    elif msg_type == "vdbg":
                                        log(f"K210帧计数: {int(result)}")
                                        break  # 纯日志，不入队列
                                    elif msg_type == "decision":
                                        global latest_decision
                                        latest_decision = result
                                        log(f"决策: {clean}")  # 打印原始行，含 frames_in/ok
                                        try: data_queue.put_nowait((time.time(), {"_type": "decision", "decision": result.to_dict()}))
                                        except queue.Full: pass
                                    elif msg_type == "summary":
                                        global latest_summary
                                        latest_summary = result
                                        log(f"统计: Zone={result.zone} R={result.red_total} G={result.green_total}")
                                        try: data_queue.put_nowait((time.time(), {"_type": "summary", "summary": result.to_dict()}))
                                        except queue.Full: pass
                                    break
                    frames = parser.feed(raw)
                    ts = time.time()
                    for f in frames:
                        frame_count += 1
                        try:
                            data_queue.put_nowait((ts, f))
                        except queue.Full:
                            try:
                                data_queue.get_nowait()
                                data_queue.put_nowait((ts, f))
                            except queue.Empty:
                                pass

                    # 每秒输出统计
                    now = time.time()
                    if now - last_report >= 1.0:
                        log(f"收到 {frame_count} 帧, {byte_count} 字节, 队列积压 {data_queue.qsize()}")
                        frame_count = 0
                        byte_count = 0
                        last_report = now
            except serial.SerialException as e:
                log(f"串口读错误: {e}")
                break

            # 处理命令队列
            try:
                cmd = cmd_queue.get_nowait()
                if serial_conn and serial_conn.is_open:
                    encoded = proto.encode_command(cmd)
                    serial_conn.write(encoded)
                    log(f"发送: {cmd.strip()}")
            except queue.Empty:
                pass

            time.sleep(0.001)  # ~1kHz 轮询

    finally:
        if serial_conn and serial_conn.is_open:
            serial_conn.close()
        serial_running = False
        serial_conn = None
        log("串口已关闭")


# ══════════════════════════════════════════════════════
# 数据广播协程
# ══════════════════════════════════════════════════════

async def broadcast_loop():
    """从 data_queue 取数据，广播到所有 WebSocket 客户端"""
    global recording_active, recording_buf, ws_clients

    while True:
        # 批量取帧（减少锁竞争）
        frames_to_send = []
        for _ in range(20):  # 一次最多取 20 帧
            try:
                frames_to_send.append(data_queue.get_nowait())
            except queue.Empty:
                break

        if frames_to_send:
            # 组装 JSON 消息
            items = []
            for ts, raw in frames_to_send:
                # 处理标记检测消息
                if raw.get("_type") == "marker":
                    items.append({
                        "_type": "marker",
                        "ts": ts,
                        "enc": raw.get("enc", 0),
                        "odom_y": odom.y,
                        "odom_x": odom.x,
                    })
                    continue
                # 处理状态消息 — 独立广播，不依赖数据帧搭车
                if raw.get("_type") == "state":
                    global latest_state
                    latest_state = raw.get("state", 0)
                    items.append({
                        "_type": "state",
                        "ts": ts,
                        "state": latest_state,
                    })
                    continue
                # 处理决策消息
                if raw.get("_type") == "decision":
                    items.append({
                        "_type": "decision",
                        "ts": ts,
                        **raw["decision"],
                    })
                    continue
                # 处理陀螺仪 yaw 消息
                if raw.get("_type") == "yaw":
                    odom.feed_gyro_yaw(raw.get("yaw", 0.0), ts)
                    items.append({
                        "_type": "yaw",
                        "ts": ts,
                        "yaw": raw.get("yaw", 0.0),
                    })
                    continue
                # 处理预热进度
                if raw.get("_type") == "warmup":
                    items.append({
                        "_type": "warmup",
                        "ts": ts,
                        "pct": raw.get("pct", 0),
                        "temp_c": raw.get("temp_c", 0.0),
                    })
                    continue
                # 处理 MPU6050 初始化信息
                if raw.get("_type") == "mpu":
                    items.append({
                        "_type": "mpu",
                        "ts": ts,
                        "init_ok": raw.get("init_ok", 0),
                        "st_z_pct": raw.get("st_z_pct", 0),
                        "bias_dps": raw.get("bias_dps", 0.0),
                        "alive": raw.get("alive", 0),
                    })
                    continue
                # 处理累计统计消息
                if raw.get("_type") == "summary":
                    items.append({
                        "_type": "summary",
                        "ts": ts,
                        **raw["summary"],
                    })
                    continue

                # 里程计积分
                fd = odom.update(
                    raw["fl_speed"], raw["fr_speed"],
                    raw["bl_speed"], raw["br_speed"],
                    ts
                )
                item = fd.to_dict()
                item["ir"] = raw["ir"]
                item["us"] = raw["ultrasonic"]
                item["enc"] = int(raw["encoder_total"])
                item["state"] = latest_state
                items.append(item)

                # 录制
                if recording_active:
                    recording_buf.append(item)

            msg = json.dumps({"type": "data", "frames": items}, ensure_ascii=False)

            # 广播
            async with ws_lock:
                dead = set()
                for ws in ws_clients:
                    try:
                        await ws.send_text(msg)
                    except Exception:
                        dead.add(ws)
                ws_clients -= dead

        await asyncio.sleep(0.008)  # ~120Hz 广播频率


# ══════════════════════════════════════════════════════
# FastAPI 路由
# ══════════════════════════════════════════════════════

@app.get("/")
async def root():
    return FileResponse(STATIC / "index.html")


@app.get("/api/status")
async def get_status():
    """获取串口连接状态和里程计"""
    return {
        "connected": serial_running,
        "port": serial_conn.port if serial_conn else None,
        "baud": serial_conn.baudrate if serial_conn else None,
        "recording": recording_active,
        "odometry": {
            "x": round(odom.x, 4),
            "y": round(odom.y, 4),
            "heading": round(odom.heading, 4),
            "track": odom.wheel_track,
        },
    }


@app.post("/api/connect")
async def api_connect(data: dict):
    """连接/切换串口"""
    global serial_running

    port = data.get("port", "COM5")
    baud = data.get("baud", 115200)

    # 先断开旧连接
    if serial_running:
        serial_running = False
        await asyncio.sleep(0.3)

    # 启动新连接
    t = threading.Thread(target=serial_reader_thread, args=(port, baud), daemon=True)
    t.start()
    await asyncio.sleep(0.5)

    if serial_running:
        return {"ok": True, "port": port, "baud": baud}
    else:
        return {"ok": False, "error": f"无法打开 {port}"}


@app.post("/api/disconnect")
async def api_disconnect():
    """断开串口"""
    global serial_running
    if serial_running:
        serial_running = False
        return {"ok": True}
    return {"ok": False, "error": "未连接"}


@app.post("/api/command")
async def send_command(data: dict):
    """发送文本命令到 STM32"""
    cmd = data.get("cmd", "")
    if not cmd:
        raise HTTPException(400, "cmd required")

    if not serial_running:
        raise HTTPException(503, "串口未连接")

    try:
        cmd_queue.put_nowait(cmd)
    except queue.Full:
        raise HTTPException(429, "命令队列满")

    return {"ok": True, "cmd": cmd}


@app.post("/api/odom/reset")
async def reset_odometry():
    """重置里程计原点"""
    odom.reset()
    return {"ok": True}


@app.post("/api/odom/set")
async def set_odometry(data: dict):
    """强制设置里程计位姿（标记修正用）"""
    x = data.get("x", 0.0)
    y = data.get("y", 0.0)
    heading = data.get("heading", math.pi / 2)
    odom.set_position(x, y, heading)
    return {"ok": True, "x": x, "y": y, "heading": heading}


@app.post("/api/odom/track")
async def set_wheel_track(data: dict):
    """设置轮距"""
    val = data.get("track", 0.18)
    odom.set_wheel_track(val)
    return {"ok": True, "track": val}


@app.post("/api/gyro/scale")
async def set_gyro_scale(data: dict):
    """设置陀螺仪比例修正"""
    val = data.get("scale", 1.0)
    odom.GYRO_SCALE = float(val)
    odom._gyro_first_yaw = None       # 清空初始对齐，下帧重新对齐
    odom._gyro_first_heading = None
    return {"ok": True, "gyro_scale": odom.GYRO_SCALE}


# ── 录制 ──────────────────────────────────────────

@app.post("/api/record/start")
async def start_recording():
    global recording_active, recording_buf
    recording_active = True
    recording_buf = []
    return {"ok": True}


@app.post("/api/record/stop")
async def stop_recording(data: dict | None = None):
    global recording_active, recording_buf, recordings
    if not recording_active:
        raise HTTPException(400, "未在录制")

    recording_active = False
    rid = str(uuid.uuid4())[:8]

    label = (data or {}).get("label", f"record_{rid}")
    recordings[rid] = {
        "id": rid,
        "label": label,
        "frames": recording_buf.copy(),
        "count": len(recording_buf),
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    # 存盘
    ensure_dirs()
    fpath = RECORDINGS_DIR / f"{rid}.json"
    fpath.write_text(json.dumps(recordings[rid], ensure_ascii=False, indent=2),
                     encoding="utf-8")

    return {"ok": True, "id": rid, "frames": len(recording_buf)}


@app.get("/api/recordings")
async def list_recordings():
    """列出所有已保存的录制"""
    ensure_dirs()
    result = []
    for f in sorted(RECORDINGS_DIR.glob("*.json"), key=os.path.getmtime, reverse=True):
        try:
            meta = json.loads(f.read_text(encoding="utf-8"))
            result.append({
                "id": meta["id"],
                "label": meta.get("label", f.stem),
                "count": meta.get("count", len(meta.get("frames", []))),
                "time": meta.get("time", ""),
            })
        except Exception:
            pass
    return result


@app.get("/api/recordings/{rid}")
async def get_recording(rid: str):
    """获取录制数据"""
    fpath = RECORDINGS_DIR / f"{rid}.json"
    if not fpath.exists():
        raise HTTPException(404, "录制不存在")
    return JSONResponse(json.loads(fpath.read_text(encoding="utf-8")))


@app.delete("/api/recordings/{rid}")
async def delete_recording(rid: str):
    fpath = RECORDINGS_DIR / f"{rid}.json"
    if fpath.exists():
        fpath.unlink()
    return {"ok": True}


@app.get("/api/recordings/{rid}/csv")
async def export_csv(rid: str):
    """导出录制为 CSV 文件"""
    fpath = RECORDINGS_DIR / f"{rid}.json"
    if not fpath.exists():
        raise HTTPException(404, "录制不存在")

    rec = json.loads(fpath.read_text(encoding="utf-8"))
    frames = rec.get("frames", [])

    import io
    import csv as csv_mod

    output = io.StringIO()
    writer = csv_mod.writer(output)

    # 表头
    writer.writerow([
        "time_s", "fl_mps", "fr_mps", "bl_mps", "br_mps",
        "ir", "us_cm", "enc", "x_m", "y_m", "heading_rad",
        "state", "v_forward_mps",
    ])

    # 数据行
    for i, f in enumerate(frames):
        writer.writerow([
            round(i / 100.0, 3),
            f.get("fl", 0),
            f.get("fr", 0),
            f.get("bl", 0),
            f.get("br", 0),
            f.get("ir", 0),
            f.get("us", 0),
            f.get("enc", 0),
            f.get("x", 0),
            f.get("y", 0),
            f.get("h", 0),
            f.get("state", 0),
            f.get("v", 0),
        ])

    from fastapi.responses import Response
    csv_content = output.getvalue()
    output.close()

    filename = f"{rec.get('label', rid)}.csv"
    return Response(
        content=csv_content,
        media_type="text/csv; charset=utf-8",
        headers={"Content-Disposition": f"attachment; filename={filename}"},
    )


# ── 标记位置管理 ──────────────────────────────────

_markers_config: list[dict] = [
    # 左直道 (X=-1.30, 车下行), 从START每隔34cm
    {"label": "L1", "x": -1.30, "y":  0.00},
    {"label": "L2", "x": -1.30, "y": -0.34},
    {"label": "L3", "x": -1.30, "y": -0.68},
    {"label": "L4", "x": -1.30, "y": -1.02},
    {"label": "L5", "x": -1.30, "y": -1.36},
    {"label": "L6", "x": -1.30, "y": -1.70},
    {"label": "L7", "x": -1.30, "y": -2.04},
    # 右直道 (X=0.00, 车上行), 对称位置
    {"label": "R1", "x":  0.00, "y": -2.04},
    {"label": "R2", "x":  0.00, "y": -1.70},
    {"label": "R3", "x":  0.00, "y": -1.36},
    {"label": "R4", "x":  0.00, "y": -1.02},
    {"label": "R5", "x":  0.00, "y": -0.68},
    {"label": "R6", "x":  0.00, "y": -0.34},
    {"label": "R7", "x":  0.00, "y":  0.00},
]

@app.get("/api/markers")
async def get_markers():
    return _markers_config

@app.post("/api/markers")
async def save_markers(data: dict):
    global _markers_config
    _markers_config = data.get("markers", [])
    return {"ok": True, "count": len(_markers_config)}

# ── 参数方案管理 ───────────────────────────────────

@app.get("/api/profiles")
async def list_profiles():
    ensure_dirs()
    result = []
    for f in sorted(PROFILES_DIR.glob("*.json"), key=os.path.getmtime, reverse=True):
        try:
            p = json.loads(f.read_text(encoding="utf-8"))
            result.append({"name": f.stem, **p})
        except Exception:
            pass
    return result


@app.post("/api/profiles")
async def save_profile(data: dict):
    """保存参数方案"""
    name = data.get("name", "").strip()
    if not name:
        raise HTTPException(400, "name required")

    ensure_dirs()
    profile = {
        "name": name,
        "params": data.get("params", {}),
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "note": data.get("note", ""),
    }

    fpath = PROFILES_DIR / f"{name}.json"
    fpath.write_text(json.dumps(profile, ensure_ascii=False, indent=2),
                     encoding="utf-8")
    return {"ok": True, "name": name}


@app.delete("/api/profiles/{name}")
async def delete_profile(name: str):
    fpath = PROFILES_DIR / f"{name}.json"
    if fpath.exists():
        fpath.unlink()
    return {"ok": True}


# ══════════════════════════════════════════════════════
# WebSocket
# ══════════════════════════════════════════════════════

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    async with ws_lock:
        ws_clients.add(ws)

    try:
        while True:
            # 接收前端消息
            msg = await ws.receive_text()
            data = json.loads(msg)
            action = data.get("action", "")

            if action == "command":
                cmd = data.get("cmd", "")
                if cmd and serial_running:
                    try:
                        cmd_queue.put_nowait(cmd)
                    except queue.Full:
                        await ws.send_text(json.dumps(
                            {"type": "error", "msg": "命令队列满"}
                        ))
            elif action == "reset_odom":
                odom.reset()
            elif action == "set_track":
                odom.set_wheel_track(data.get("track", 0.18))

    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"[WS] error: {e}")
    finally:
        async with ws_lock:
            ws_clients.discard(ws)


# ══════════════════════════════════════════════════════
# 启动
# ══════════════════════════════════════════════════════

@app.on_event("startup")
async def startup():
    """启动后台广播任务"""
    asyncio.create_task(broadcast_loop())


def main():
    parser_arg = argparse.ArgumentParser(description="Car Studio Server")
    parser_arg.add_argument("port", nargs="?", default=None,
                            help="串口号，如 COM5 (默认 COM5)")
    parser_arg.add_argument("--baud", type=int, default=115200,
                            help="波特率 (默认 115200)")
    parser_arg.add_argument("--track", type=float, default=0.22,
                            help="有效轮距 (m) (默认 0.22=22cm)")
    parser_arg.add_argument("--no-serial", action="store_true",
                            help="不连串口，纯回放模式")
    parser_arg.add_argument("--host", default="localhost",
                            help="监听地址")
    parser_arg.add_argument("--port", "-p", type=int, default=8000,
                            dest="http_port", help="HTTP 端口 (默认 8000)")

    args = parser_arg.parse_args()

    # 设置轮距
    odom.set_wheel_track(args.track)
    print(f"轮距: {args.track}m")

    # 连接串口
    if not args.no_serial and args.port:
        t = threading.Thread(
            target=serial_reader_thread,
            args=(args.port, args.baud),
            daemon=True
        )
        t.start()
        time.sleep(0.5)
    elif not args.no_serial:
        # 尝试默认 COM5
        default_port = "COM5"
        print(f"未指定串口，尝试 {default_port}...")
        t = threading.Thread(
            target=serial_reader_thread,
            args=(default_port, args.baud),
            daemon=True
        )
        t.start()
        time.sleep(0.5)

    # 启动 HTTP
    import uvicorn
    print(f"\n  Car Studio 启动")
    print(f"  -> http://{args.host}:{args.http_port}")
    print(f"  -> 串口: {'已连接' if serial_running else '未连接'}")
    print()

    uvicorn.run(app, host=args.host, port=args.http_port, log_level="info")


if __name__ == "__main__":
    main()
