"""分析录制文件: 四轮速度差异 + 朝向漂移

用法:
    python analyze.py                         # 自动分析 recordings/ 下最新的录制
    python analyze.py recordings/xxx.json     # 指定录制文件
"""
import json, math, sys, os, glob

TRACK_WIDTH = 0.1412  # 轮距 (m)，用于把左右速差换算成角速度


def latest_recording():
    files = glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings", "*.json"))
    return max(files, key=os.path.getmtime) if files else None


path = sys.argv[1] if len(sys.argv) > 1 else latest_recording()
if not path or not os.path.exists(path):
    sys.exit("未找到录制文件。用法: python analyze.py recordings/xxx.json")

with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)

print(f"分析文件: {path}\n")

frames = data["frames"]
N = len(frames)
print(f"总帧数: {N}, 录制时长: {N/100:.1f}s\n")

# 1. 找车开始运动的帧 (速度 > 0.01)
start_idx = 0
for i, fr in enumerate(frames):
    if abs(fr["v"]) > 0.01:
        start_idx = i
        break

# 2. 分段: 运动帧
moving = frames[start_idx:]
print(f"静止帧: {start_idx}, 运动帧: {len(moving)}, 运动时长: {len(moving)/100:.1f}s")

# 3. 四轮速度统计
def wheel_stats(label, key):
    vals = [f[key] for f in moving if abs(f["v"]) > 0.05]
    if not vals: return
    avg = sum(vals) / len(vals)
    mx = max(vals)
    mn = min(vals)
    print(f"  {key}: 均值={avg:.4f}  范围=[{mn:.4f}, {mx:.4f}]")

print("\n=== 四轮速度统计 ===")
for k in ["fl","fr","bl","br"]:
    wheel_stats("", k)

# 4. 左/右侧速度差异 (关键!)
print("\n=== 左右轮速不平衡 ===")
diffs = []
straight_diffs = []
for f in moving:
    v_left = (f["fl"] + f["bl"]) / 2
    v_right = (f["fr"] + f["br"]) / 2
    diff = v_right - v_left
    diffs.append(diff)
    # 低角速度 = 近似直线
    if abs(f["w"]) < 0.3:
        straight_diffs.append(diff)

avg_diff = sum(diffs) / len(diffs)
avg_straight_diff = sum(straight_diffs) / len(straight_diffs) if straight_diffs else 0
print(f"全程左右速差均值: {avg_diff:.4f} m/s")
print(f"直道左右速差均值: {avg_straight_diff:.4f} m/s")
print(f"  → 直道朝向漂移率: {avg_straight_diff / TRACK_WIDTH * 57.3:.2f} °/s")

# 5. 朝向变化分析
print("\n=== 朝向漂移 ===")
h_start = moving[0]["h"]
h_end = moving[-1]["h"]
h_delta = h_end - h_start
print(f"起始朝向: {h_start:.4f} rad ({math.degrees(h_start):.1f}°)")
print(f"结束朝向: {h_end:.4f} rad ({math.degrees(h_end):.1f}°)")
print(f"总漂移: {h_delta:.4f} rad ({math.degrees(h_delta):.1f}°)")

# 6. 前100帧直道朝向漂移
straight_h = []
for f in moving[:300]:  # 前3秒
    if abs(f["w"]) < 0.3:
        straight_h.append(f["h"])
if len(straight_h) > 10:
    h_drift = straight_h[-1] - straight_h[0]
    print(f"前3秒直道朝向漂移: {math.degrees(h_drift):.2f}°")

# 7. 位置终点
print(f"\n=== 轨迹 ===")
print(f"起点: ({moving[0]['x']:.3f}, {moving[0]['y']:.3f})")
print(f"终点: ({moving[-1]['x']:.3f}, {moving[-1]['y']:.3f})")
print(f"总位移: ({moving[-1]['x']-moving[0]['x']:.3f}, {moving[-1]['y']-moving[0]['y']:.3f})")
