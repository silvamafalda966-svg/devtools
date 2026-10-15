#!/usr/bin/env python3
"""
Strawberry_car PID Auto-Tuner
==============================
Reads VOFA+ JustFloat data from STM32 USART1, analyzes step response,
and auto-tunes incremental PID parameters.

Protocol:
  STM32 -> PC:  JustFloat 8×float (32 bytes) at 100Hz
                ch0~3: speed FL/FR/BL/BR (m/s)
                ch4:   IR byte (as float)
                ch5:   ultrasonic distance (cm)
                ch6:   encoder odometry
                ch7:   0x7F800000 (NaN frame tail)
  PC -> STM32:  Text commands, '\n' terminated
                W0~W3/WA: select wheel
                P/I/D: set Kp/Ki/Kd
                T: set Target (DEBUG only)
                MD: enter DEBUG, M0~M9: switch states

Usage:
  python pid_tuner.py COM5 monitor              # real-time data display
  python pid_tuner.py COM5 collect -t 3.0       # collect 3s of data, print JSON
  python pid_tuner.py COM5 step -w 4 -s 0.3     # run step test and analyze
  python pid_tuner.py COM5 tune -w 4 -s 0.3 -n 5  # auto-tune (5 iterations)
  python pid_tuner.py COM5 set -w 0 -p 0.35 -i 0.06  # manually set PID
"""

import serial
import struct
import time
import sys
import json
import argparse
import textwrap
from collections import deque
from dataclasses import dataclass, asdict
from typing import Optional, List, Tuple

# ── Constants ──
FRAME_SIZE = 32          # 8 floats × 4 bytes
FRAME_TAIL = 0x7F800000  # NaN marker for channel 7
BAUD_RATE = 115200
READ_TIMEOUT = 0.01      # 10ms non-blocking read
POLL_INTERVAL = 0.005    # 5ms polling loop

# ── Data Structures ──

@dataclass
class PIDParams:
    Kp: float
    Ki: float
    Kd: float

@dataclass
class WheelData:
    """One sample frame from STM32"""
    speed_fl: float
    speed_fr: float
    speed_bl: float
    speed_br: float
    ir_byte: int
    distance_cm: float
    enc_total: int
    timestamp: float

    def get_speed(self, wheel_idx: int) -> float:
        """0=FL, 1=FR, 2=BL, 3=BR"""
        return [self.speed_fl, self.speed_fr, self.speed_bl, self.speed_br][wheel_idx]

    def to_dict(self):
        d = asdict(self)
        d['timestamp'] = round(d['timestamp'], 3)
        for k in ['speed_fl', 'speed_fr', 'speed_bl', 'speed_br', 'distance_cm']:
            d[k] = round(d[k], 4)
        return d

@dataclass
class StepMetrics:
    """Step response analysis result"""
    target_mps: float
    wheel: str
    rise_time_ms: float        # 10%→90% rise time
    overshoot_pct: float       # overshoot percentage (0 = no overshoot)
    settling_time_ms: float    # ±2% settling time
    steady_state_err_mps: float
    dead_time_ms: float        # delay before response
    peak_mps: float            # peak speed reached
    final_mps: float           # final steady speed
    initial_mps: float         # initial speed before step
    num_samples: int           # samples used
    score: float               # quality score (lower = better)

# ── Serial Bridge ──

class SerialBridge:
    """Manages USART1 communication with STM32"""

    def __init__(self, port: str):
        self.port = port
        self.ser = serial.Serial(port, BAUD_RATE, timeout=READ_TIMEOUT)
        self.buf = bytearray()
        self.history: deque[WheelData] = deque(maxlen=500)
        self.latest: Optional[WheelData] = None
        self._frame_count = 0

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        print(f"\n[SerialBridge] closed {self.port}")

    def send(self, cmd: str):
        """Send text command + newline to STM32"""
        raw = (cmd.strip() + '\n').encode('ascii')
        self.ser.write(raw)
        self.ser.flush()

    def set_pid(self, wheel: int, Kp: float = None, Ki: float = None, Kd: float = None):
        """
        Set PID params. wheel: 0=FL, 1=FR, 2=BL, 3=BR, 4=ALL.
        Sends W first, then P/I/D as needed.
        """
        w = 'A' if wheel == 4 else str(wheel)
        self.send(f'W{w}')
        time.sleep(0.01)
        if Kp is not None: self.send(f'P{Kp:.3f}')
        if Ki is not None: self.send(f'I{Ki:.3f}')
        if Kd is not None: self.send(f'D{Kd:.3f}')

    def set_target(self, speed_mps: float, wheel: int = 4):
        """Set target speed. Only works in DEBUG state!"""
        w = 'A' if wheel == 4 else str(wheel)
        self.send(f'W{w}')
        time.sleep(0.01)
        self.send(f'T{speed_mps:.3f}')

    def set_feedforward(self, value: float):
        """Set BR friction feedforward (F command). Works in any state."""
        self.send(f'F{value:.1f}')

    def set_filter_alpha(self, alpha: float):
        """Set speed EMA filter coefficient 0~0.99 (L command)."""
        self.send(f'L{alpha:.2f}')

    def enter_debug(self):
        """Switch STM32 to DEBUG state (required for T command)"""
        self.send('MD')

    def flush(self, wait: float = 0.5):
        """Clear serial buffers and wait for fresh data"""
        time.sleep(wait)
        self.ser.reset_input_buffer()
        self.buf.clear()
        self.history.clear()
        self.latest = None

    def read_frame(self) -> Optional[WheelData]:
        """
        Try to read one complete JustFloat frame.
        Uses NaN tail (0x7F800000) at ch7 for frame sync.
        Returns None if no complete frame available.
        """
        # Read new bytes
        try:
            if self.ser.in_waiting:
                chunk = self.ser.read(self.ser.in_waiting)
                self.buf.extend(chunk)
        except serial.SerialException:
            return None

        # Search for valid frame
        while len(self.buf) >= FRAME_SIZE:
            # Check if position 28..31 is NaN tail
            tail_bytes = self.buf[28:32]
            tail = struct.unpack('<I', tail_bytes)[0]

            if tail == FRAME_TAIL:
                # Parse full frame
                try:
                    floats = struct.unpack('<8f', self.buf[:FRAME_SIZE])
                except struct.error:
                    self.buf.pop(0)
                    continue

                wd = WheelData(
                    speed_fl=floats[0],
                    speed_fr=floats[1],
                    speed_bl=floats[2],
                    speed_br=floats[3],
                    ir_byte=int(floats[4]),
                    distance_cm=floats[5],
                    enc_total=int(floats[6]),
                    timestamp=time.time(),
                )
                self.buf = self.buf[FRAME_SIZE:]
                self.latest = wd
                self.history.append(wd)
                self._frame_count += 1
                return wd
            else:
                # Not a valid frame - skip one byte and retry
                self.buf.pop(0)

        return None

    def read_batch(self, duration_sec: float, verbose: bool = False) -> List[WheelData]:
        """
        Collect frames for a given duration.
        Returns list of frames collected.
        """
        result = []
        t0 = time.time()
        while time.time() - t0 < duration_sec:
            frame = self.read_frame()
            if frame:
                result.append(frame)
                if verbose and len(result) % 50 == 0:
                    spd = frame.get_speed(0)
                    sys.stdout.write(f"\r  frames: {len(result)} | FL={spd:.3f} m/s  ")
                    sys.stdout.flush()
            else:
                time.sleep(POLL_INTERVAL)
        if verbose:
            sys.stdout.write("\r" + " " * 50 + "\r")
        return result

# ── Step Analyzer ──

class StepAnalyzer:
    """Analyzes step response from collected wheel data"""

    @staticmethod
    def analyze(data: List[WheelData], target_mps: float, wheel_idx: int) -> StepMetrics:
        """
        Analyze step response for one wheel.
        Computes rise time, overshoot, settling time, etc.
        """
        WHEEL_NAMES = ['FL', 'FR', 'BL', 'BR']
        speeds = [abs(d.get_speed(wheel_idx)) for d in data]

        if len(speeds) < 20 or target_mps == 0:
            return StepMetrics(
                target_mps=target_mps, wheel=WHEEL_NAMES[wheel_idx],
                rise_time_ms=0, overshoot_pct=0, settling_time_ms=0,
                steady_state_err_mps=0, dead_time_ms=0,
                peak_mps=0, final_mps=0, initial_mps=0,
                num_samples=len(speeds), score=999,
            )

        # Initial & final values (median of tail ends)
        n = min(10, len(speeds) // 4)
        initial = abs(sum(speeds[:n]) / n)
        final = abs(sum(speeds[-n:]) / n)

        # Dead time: first point exceeding 5% of step
        step = final - initial
        if abs(step) < 0.001:
            step = target_mps  # fallback in case motor not spinning
        dead_5 = initial + 0.05 * abs(step)
        dead_time = 0.0
        for i, s in enumerate(speeds):
            if abs(s) >= dead_5:
                dead_time = data[i].timestamp - data[0].timestamp
                break

        # Rise time: 10% → 90%
        rise_10 = initial + 0.10 * abs(step)
        rise_90 = initial + 0.90 * abs(step)
        t_10 = None
        t_90 = None
        for i, s in enumerate(speeds):
            t = data[i].timestamp - data[0].timestamp
            if t_10 is None and abs(s) >= rise_10:
                t_10 = t
            if t_90 is None and abs(s) >= rise_90:
                t_90 = t
            if t_10 is not None and t_90 is not None:
                break
        rise_time = ((t_90 - t_10) * 1000) if (t_10 and t_90) else 0

        # Peak & overshoot
        peak_val = max(speeds)
        overshoot = max(0, (peak_val - final) / abs(step) * 100) if abs(step) > 0.001 else 0

        # Settling time: enter and stay within ±2% of final
        band = 0.02 * abs(final) + 0.005  # +5mm/s floor
        settling_time = 0.0
        for i in range(len(speeds)):
            if abs(speeds[i] - final) <= band:
                # check next 20 samples all in band
                ok = True
                for j in range(i, min(i + 20, len(speeds))):
                    if abs(speeds[j] - final) > band:
                        ok = False
                        break
                if ok:
                    settling_time = data[i].timestamp - data[0].timestamp
                    break

        # Steady-state error
        steady_err = final - abs(target_mps)

        # Quality score: lower = better
        # Heavily penalize overshoot (user prefers overdamped)
        overshoot_score = overshoot * 15
        rise_score = max(0, rise_time - 250) / 10  # target < 250ms
        error_score = abs(steady_err) * 200
        settling_score = max(0, settling_time * 1000 - 500) / 20
        score = overshoot_score + rise_score + error_score + settling_score

        return StepMetrics(
            target_mps=target_mps, wheel=WHEEL_NAMES[wheel_idx],
            rise_time_ms=rise_time, overshoot_pct=round(overshoot, 2),
            settling_time_ms=round(settling_time * 1000, 0),
            steady_state_err_mps=round(steady_err, 4),
            dead_time_ms=round(dead_time * 1000, 0),
            peak_mps=round(peak_val, 4), final_mps=round(final, 4),
            initial_mps=round(initial, 4),
            num_samples=len(speeds), score=round(score, 1),
        )

# ── PID Tuner Engine ──

class PIDTuner:
    """
    Iterative auto-tuner using FOPDT model + CHR tuning rules.
    """

    WHEELS = ['FL', 'FR', 'BL', 'BR']

    def __init__(self, bridge: SerialBridge):
        self.bridge = bridge
        self.analyzer = StepAnalyzer()

    # ── FOPDT model fitting ──

    @staticmethod
    def fit_fopdt(data: List[WheelData], target_mps: float, wheel_idx: int) -> Tuple[float, float, float]:
        """
        Fit First-Order-Plus-Dead-Time model from step response.
        Model: G(s) = K * exp(-θs) / (τs + 1)

        Returns (K, tau, theta):
          K     = steady-state gain (Δspeed / Δtarget)
          tau   = time constant (seconds)
          theta = dead time (seconds)
        """
        speeds = [abs(d.get_speed(wheel_idx)) for d in data]
        if len(speeds) < 20:
            return 1.0, 0.1, 0.01

        n = min(10, len(speeds) // 4)
        initial = sum(speeds[:n]) / n
        final = sum(speeds[-n:]) / n
        step = final - initial

        if abs(step) < 0.001:
            return 1.0, 0.1, 0.01

        K = abs(step) / abs(target_mps) if abs(target_mps) > 0 else 1.0

        # Dead time θ: first exceed 5% of step
        dead_5 = initial + 0.05 * abs(step)
        theta = 0.01
        for i, s in enumerate(speeds):
            if abs(s) >= dead_5:
                theta = data[i].timestamp - data[0].timestamp
                break

        # Time constant τ: time to reach 63.2% of step
        thresh_63 = initial + 0.632 * abs(step)
        t_63 = theta + 0.05
        for i, s in enumerate(speeds):
            if abs(s) >= thresh_63:
                t_63 = data[i].timestamp - data[0].timestamp
                break

        tau = max(0.01, t_63 - theta)

        return K, tau, max(theta, 0.005)

    # ── CHR tuning (0% overshoot, setpoint tracking) ──

    @staticmethod
    def chr_tune(K: float, tau: float, theta: float) -> PIDParams:
        """
        Chien-Hrones-Reswick tuning for 0% overshoot.

        Rules (setpoint, 0% overshoot):
          Kp = 0.6 * τ / (K * θ)
          Ti = τ  →  Ki = Kp / τ
          Td = 0   (no derivative for first-order plants)
        """
        if theta <= 0.001:
            theta = 0.001
        if tau <= 0:
            tau = 0.01

        Kp = 0.6 * tau / (K * theta)
        Ki = Kp / tau
        Kd = 0.0

        # Clamp to safe ranges for this motor/PWM system
        # Typical values: Kp 0.2~1.5, Ki 0.02~0.3, Kd 0~0.1
        Kp = max(0.10, min(1.5, Kp))
        Ki = max(0.02, min(0.30, Ki))
        Kd = max(0.0, min(0.10, Kd))

        return PIDParams(Kp=round(Kp, 3), Ki=round(Ki, 3), Kd=round(Kd, 3))

    # ── Step test ──

    def step_test(self, target_mps: float = 0.3, duration: float = 2.0,
                  wheel: int = 4) -> List[WheelData]:
        """
        Run a complete step test:
        1. Ensure stopped (target=0)
        2. Apply step
        3. Collect data for `duration` seconds
        4. Stop

        Returns collected frames.
        """
        w_str = 'ALL' if wheel == 4 else self.WHEELS[wheel]
        print(f"\n  [Step Test] target={target_mps} m/s, wheel={w_str}, T={duration}s")

        # Stop & flush
        self.bridge.set_target(0.0, wheel)
        self.bridge.flush(0.8)

        # Apply step
        self.bridge.set_target(target_mps, wheel)
        t0 = time.time()

        # Collect
        data = self.bridge.read_batch(duration, verbose=True)

        # Stop
        self.bridge.set_target(0.0, wheel)
        time.sleep(0.3)

        print(f"  [Step Test] collected {len(data)} frames in {time.time()-t0:.1f}s")
        return data

    # ── Full auto-tune loop ──

    def auto_tune(self, wheel: int = 4, target_mps: float = 0.3,
                  max_iter: int = 8, target_overshoot: float = 0.5) -> PIDParams:
        """
        Run iterative auto-tuning using model-free metric-driven adjustment.

        Algorithm per iteration:
          1. Run step test with current gains
          2. Analyze response → score (lower = better)
          3. Adjust gains based on simple rules:
             - Overshoot > 1%  → reduce Kp by 10%, reduce Ki by 10%
             - Rise time > 250ms → increase Kp by 10%, increase Ki by 10%
             - Steady err > 0.02 → increase Ki by 20%
             - Overshoot = 0 AND rise < 150ms AND err < 0.005 → converged
          4. Track best, apply, repeat

        Returns best PID params found.
        """
        w_str = 'ALL' if wheel == 4 else self.WHEELS[wheel]
        print(f"\n{'#'*60}")
        print(f"#  PID AUTO-TUNE (metric-driven)")
        print(f"#  Wheel: {w_str}  |  Target: {target_mps} m/s  |  Max iter: {max_iter}")
        print(f"#  Goal: overshoot ~ 0%, rise < 250ms, |err| < 0.02")
        print(f"{'#'*60}")

        # Enter DEBUG state (required for T command)
        self.bridge.enter_debug()
        self.bridge.flush(0.3)

        # ── Start from known good baseline ──
        # Use previous tune best as starting point
        current = PIDParams(Kp=0.28, Ki=0.05, Kd=0.04)
        self.bridge.set_pid(wheel, current.Kp, current.Ki, current.Kd)
        print(f"\n  Start gains: Kp={current.Kp:.3f}, Ki={current.Ki:.3f}, Kd={current.Kd:.3f}")

        all_results = []
        best_params = current
        best_score = float('inf')
        w_idx = 0 if wheel == 4 else wheel  # FL as representative

        for it in range(1, max_iter + 1):
            print(f"\n{'─'*50}")
            print(f"  Iteration {it}/{max_iter}")
            print(f"{'─'*50}")

            # 1. Run step test
            data = self.step_test(target_mps, duration=2.0, wheel=wheel)

            if len(data) < 30:
                print(f"  WARN: Too few samples ({len(data)}), retrying...")
                continue

            # 2. Analyze
            metrics = self.analyzer.analyze(data, target_mps, w_idx)
            all_results.append(metrics)

            print(f"  [Metrics]")
            print(f"     Rise: {metrics.rise_time_ms:.0f}ms | Overshoot: {metrics.overshoot_pct:.1f}%")
            print(f"     Settle: {metrics.settling_time_ms:.0f}ms | Steady err: {metrics.steady_state_err_mps:.4f} m/s")
            print(f"     Score: {metrics.score} (lower=better)")

            # Track best
            if metrics.score < best_score:
                best_score = metrics.score
                best_params = PIDParams(current.Kp, current.Ki, current.Kd)
                print(f"  ** New best!")

            # 3. Convergence check
            if (metrics.overshoot_pct <= 1.0 and
                metrics.rise_time_ms <= 200 and
                abs(metrics.steady_state_err_mps) < 0.01):
                print(f"\n  CONVERGED! overshoot={metrics.overshoot_pct:.1f}%, "
                      f"rise={metrics.rise_time_ms:.0f}ms, err={metrics.steady_state_err_mps:.4f}")
                break

            # 4. Adjust gains based on metric rules (v2: +Kd)
            adj = PIDParams(current.Kp, current.Ki, current.Kd)

            if metrics.overshoot_pct > 2.0:
                # Too much overshoot -> reduce P, increase D (damping)
                adj.Kp = round(current.Kp * 0.88, 3)
                adj.Ki = round(current.Ki * 0.92, 3)
                adj.Kd = round(current.Kd * 1.30 + 0.005, 3)
                print(f"  [ADJ] Overshoot {metrics.overshoot_pct:.1f}% > 2% -> Kp-12% Ki-8% Kd+30%")
            elif metrics.rise_time_ms > 250:
                # Too slow -> increase P, reduce D (D slows response)
                adj.Kp = round(current.Kp * 1.10, 3)
                adj.Ki = round(current.Ki * 1.10, 3)
                adj.Kd = round(current.Kd * 0.85, 3)
                print(f"  [ADJ] Rise {metrics.rise_time_ms:.0f}ms > 250ms -> Kp+10% Ki+10% Kd-15%")
            elif abs(metrics.steady_state_err_mps) > 0.02:
                # Steady error -> boost I
                adj.Ki = round(current.Ki * 1.20, 3)
                print(f"  [ADJ] Steady err {metrics.steady_state_err_mps:.4f} > 0.02 -> boost Ki by 20%")
            else:
                # Close to target, fine-tune: bump Kp slightly, trim Kd
                adj.Kp = round(current.Kp * 1.05, 3)
                adj.Kd = round(current.Kd * 0.90, 3)
                print(f"  [ADJ] Close to target -> Kp+5% Kd-10% to speed up")

            # Clamp
            adj.Kp = max(0.10, min(1.5, adj.Kp))
            adj.Ki = max(0.02, min(0.5, adj.Ki))
            adj.Kd = max(0.005, min(0.15, adj.Kd))

            # 5. Apply
            self.bridge.set_pid(wheel, adj.Kp, adj.Ki, adj.Kd)
            print(f"  Applied: Kp={adj.Kp:.3f}, Ki={adj.Ki:.3f}, Kd={adj.Kd:.3f}")
            current = adj

            time.sleep(0.3)  # let new params settle

        # ── Final report ──
        print(f"\n{'='*60}")
        print(f"  TUNING COMPLETE")
        print(f"  Best: Kp={best_params.Kp:.3f}, Ki={best_params.Ki:.3f}, Kd={best_params.Kd:.3f}")
        print(f"  Score: {best_score:.0f}")
        print(f"{'='*60}")

        # Print history
        print(f"\n  Iteration history:")
        for i, r in enumerate(all_results):
            print(f"    {i+1}. score={r.score:5.0f} | rise={r.rise_time_ms:5.0f}ms "
                  f"overshoot={r.overshoot_pct:4.1f}% err={r.steady_state_err_mps:+.4f}")

        # C header snippet
        print(f"\n  Code to paste (C header):")
        print(f"  #define PID_KP  {best_params.Kp:.2f}f")
        print(f"  #define PID_KI  {best_params.Ki:.2f}f")
        print(f"  #define PID_KD  {best_params.Kd:.2f}f")

        return best_params

# ── CLI ──

def cmd_monitor(args, bridge: SerialBridge):
    """Real-time data display (minimal VOFA+)"""
    print("PID Monitor — press Ctrl+C to stop\n")
    print(f"{'Time':>8s}  {'FL':>8s}  {'FR':>8s}  {'BL':>8s}  {'BR':>8s}  {'IR':>4s}  {'Dist':>6s}  {'Enc':>8s}")
    print("-" * 80)
    t0 = time.time()
    try:
        while True:
            frame = bridge.read_frame()
            if frame:
                t = frame.timestamp - t0
                print(f"{t:8.1f}  {frame.speed_fl:8.4f}  {frame.speed_fr:8.4f}  "
                      f"{frame.speed_bl:8.4f}  {frame.speed_br:8.4f}  "
                      f"0x{frame.ir_byte:02X}  {frame.distance_cm:5.1f}cm  "
                      f"{frame.enc_total:8d}", end='\r')
            else:
                time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        print("\n\nStopped.")

def cmd_collect(args, bridge: SerialBridge):
    """Collect data and output JSON summary"""
    print(f"Collecting {args.time}s of data...")
    data = bridge.read_batch(args.time, verbose=True)
    print(f"Collected {len(data)} frames")

    if not data:
        print("{}")
        return

    # Summary stats
    last = data[-1]
    summary = {
        "num_frames": len(data),
        "duration_s": round(data[-1].timestamp - data[0].timestamp, 3),
        "latest": last.to_dict(),
        "averages": {
            "speed_fl": round(sum(d.speed_fl for d in data) / len(data), 4),
            "speed_fr": round(sum(d.speed_fr for d in data) / len(data), 4),
            "speed_bl": round(sum(d.speed_bl for d in data) / len(data), 4),
            "speed_br": round(sum(d.speed_br for d in data) / len(data), 4),
            "distance_cm": round(sum(d.distance_cm for d in data) / len(data), 2),
        },
        "last_10_samples": [d.to_dict() for d in data[-10:]],
    }
    print(json.dumps(summary, indent=2, ensure_ascii=False))

def cmd_step(args, bridge: SerialBridge):
    """Run one step test and print analysis"""
    tuner = PIDTuner(bridge)
    bridge.enter_debug()
    bridge.flush(0.3)

    # Set current PID params
    if args.kp is not None or args.ki is not None or args.kd is not None:
        kp = args.kp if args.kp is not None else 0.20
        ki = args.ki if args.ki is not None else 0.10
        kd = args.kd if args.kd is not None else 0.0
        bridge.set_pid(args.wheel, kp, ki, kd)

    data = tuner.step_test(args.speed, args.duration, args.wheel)

    w_idx = 0 if args.wheel == 4 else args.wheel
    metrics = StepAnalyzer.analyze(data, args.speed, w_idx)
    print(f"\n{'='*50}")
    print(f"  Step Response Analysis")
    print(f"{'='*50}")
    print(f"  Target:     {metrics.target_mps:.3f} m/s")
    print(f"  Wheel:      {metrics.wheel}")
    print(f"  Samples:    {metrics.num_samples}")
    print(f"  Initial:    {metrics.initial_mps:.4f} m/s")
    print(f"  Final:      {metrics.final_mps:.4f} m/s")
    print(f"  Peak:       {metrics.peak_mps:.4f} m/s")
    print(f"  ──────────────────────────")
    print(f"  Dead time:  {metrics.dead_time_ms:.0f} ms")
    print(f"  Rise time:  {metrics.rise_time_ms:.0f} ms")
    print(f"  Overshoot:  {metrics.overshoot_pct:.1f} %")
    print(f"  Settling:   {metrics.settling_time_ms:.0f} ms")
    print(f"  Steady err: {metrics.steady_state_err_mps:.4f} m/s")
    print(f"  ──────────────────────────")
    print(f"  Score:      {metrics.score} (0=perfect)")

    # FOPDT fit
    K, tau, theta = PIDTuner.fit_fopdt(data, args.speed, w_idx)
    print(f"\n  FOPDT model: K={K:.3f}, τ={tau*1000:.0f}ms, θ={theta*1000:.0f}ms")
    new_params = PIDTuner.chr_tune(K, tau, theta)
    print(f"  CHR suggests: Kp={new_params.Kp:.3f}, Ki={new_params.Ki:.3f}, Kd={new_params.Kd:.3f}")

    # Output JSON for Claude to parse
    if args.json:
        # Include raw time series (downsampled to ~33Hz)
        raw_series = []
        for i, d in enumerate(data):
            if i % 3 == 0:
                raw_series.append({
                    "t_ms": round((d.timestamp - data[0].timestamp) * 1000, 0),
                    "v": round(abs(d.get_speed(w_idx)), 4),
                })
        result = {
            "metrics": asdict(metrics),
            "fopdt": {"K": round(K, 3), "tau_ms": round(tau*1000, 0), "theta_ms": round(theta*1000, 0)},
            "chr_pid": asdict(new_params),
            "raw_series": raw_series,
        }
        print(f"\n<!-- JSON FOR CLAUDE -->")
        print(json.dumps(result, indent=2))

def cmd_tune(args, bridge: SerialBridge):
    """Run full auto-tune sequence"""
    tuner = PIDTuner(bridge)
    best = tuner.auto_tune(
        wheel=args.wheel,
        target_mps=args.speed,
        max_iter=args.iterations,
        target_overshoot=0.5,
    )

    # Output JSON
    print(f"\n<!-- JSON FOR CLAUDE -->")
    print(json.dumps(asdict(best)))

def cmd_set(args, bridge: SerialBridge):
    """Manually set PID params + feedforward + filter alpha"""
    if args.kp is not None or args.ki is not None or args.kd is not None:
        bridge.set_pid(args.wheel, args.kp, args.ki, args.kd)
        print(f"Set wheel {args.wheel}: Kp={args.kp}, Ki={args.ki}, Kd={args.kd}")
    if args.ff is not None:
        bridge.set_feedforward(args.ff)
        print(f"Set feedforward: {args.ff}")
    if args.alpha is not None:
        bridge.set_filter_alpha(args.alpha)
        print(f"Set filter alpha: {args.alpha}")

def cmd_raw(args, bridge: SerialBridge):
    """Send raw command to STM32"""
    bridge.send(args.cmd)
    print(f"Sent: {args.cmd}")

# ── Serial port helpers ──

def list_serial_ports():
    """List all available serial ports."""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:")
    for p in ports:
        desc = f"  {p.device}"
        if p.description:
            desc += f"  -- {p.description}"
        if p.hwid:
            desc += f"  [VID:PID={p.vid:04X}:{p.pid:04X}]" if p.vid else f"  [{p.hwid}]"
        print(desc)

def prompt_port() -> str:
    """Ask user to pick a COM port."""
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found! Is the STM32 connected?")
        port = input("Enter port manually (e.g., COM5): ").strip()
        return port

    print("\nAvailable serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  -- {p.description}")

    while True:
        choice = input(f"Select port [0-{len(ports)-1}] or type name: ").strip()
        if choice.isdigit() and 0 <= int(choice) < len(ports):
            return ports[int(choice)].device
        elif choice.upper().startswith('COM'):
            return choice.upper()
        else:
            print("  Invalid choice, try again.")

# ── Interactive mode ──

def interactive_mode():
    """Double-click friendly: prompt for port, then show menu."""
    print("""
╔══════════════════════════════════════════╗
║   Strawberry_car PID Auto-Tuner v1.0    ║
╚══════════════════════════════════════════╝
""")
    port = prompt_port()
    print(f"\nConnecting to {port}...")
    bridge = SerialBridge(port)

    try:
        while True:
            print(f"""
┌── Menu ──────────────────────────────────┐
│  1. monitor  - Real-time data display     │
│  2. collect  - Collect data + JSON dump   │
│  3. step     - Single step test & analyze │
│  4. tune     - Full auto-tune (iterative) │
│  5. set      - Manually set PID params    │
│  6. raw      - Send raw command           │
│  0. exit                                  │
└───────────────────────────────────────────┘""")
            choice = input("  Choice: ").strip()

            if choice == '0':
                break
            elif choice == '1':
                cmd_monitor(None, bridge)
            elif choice == '2':
                t = float(input("  Duration (seconds) [3.0]: ").strip() or '3.0')
                args = type('Args', (), {'time': t})()
                cmd_collect(args, bridge)
            elif choice == '3':
                w = int(input("  Wheel (0=FL,1=FR,2=BL,3=BR,4=ALL) [4]: ").strip() or '4')
                s = float(input("  Target speed m/s [0.3]: ").strip() or '0.3')
                d = float(input("  Duration seconds [2.0]: ").strip() or '2.0')
                json_out = input("  JSON output? (y/n) [n]: ").strip().lower() == 'y'
                args = type('Args', (), {
                    'wheel': w, 'speed': s, 'duration': d,
                    'kp': None, 'ki': None, 'kd': None, 'json': json_out,
                })()
                cmd_step(args, bridge)
            elif choice == '4':
                w = int(input("  Wheel (0=FL,1=FR,2=BL,3=BR,4=ALL) [4]: ").strip() or '4')
                s = float(input("  Target speed m/s [0.3]: ").strip() or '0.3')
                n = int(input("  Max iterations [5]: ").strip() or '5')
                args = type('Args', (), {'wheel': w, 'speed': s, 'iterations': n})()
                cmd_tune(args, bridge)
            elif choice == '5':
                w = int(input("  Wheel (0-3 or 4=ALL): ").strip())
                kp = input("  Kp [skip]: ").strip()
                ki = input("  Ki [skip]: ").strip()
                kd = input("  Kd [skip]: ").strip()
                args = type('Args', (), {
                    'wheel': w,
                    'kp': float(kp) if kp else None,
                    'ki': float(ki) if ki else None,
                    'kd': float(kd) if kd else None,
                })()
                cmd_set(args, bridge)
            elif choice == '6':
                c = input("  Command: ").strip()
                if c:
                    args = type('Args', (), {'cmd': c})()
                    cmd_raw(args, bridge)
            else:
                print("  Invalid choice.")

    except KeyboardInterrupt:
        print("\n")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bridge.close()
        input("\nPress Enter to close...")

# ── Main ──

def main():
    parser = argparse.ArgumentParser(
        description='Strawberry_car PID Auto-Tuner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent('''
        Examples:
          python pid_tuner.py COM5 monitor
          python pid_tuner.py COM5 collect -t 3
          python pid_tuner.py COM5 step -w 4 -s 0.3
          python pid_tuner.py COM5 tune -w 4 -s 0.3 -n 5
          python pid_tuner.py COM5 set -w 0 -p 0.35 -i 0.06
          python pid_tuner.py COM5 raw -c "MD"
        '''))
    parser.add_argument('port', nargs='?', default=None,
                        help='Serial port name (e.g., COM5). Omit for interactive mode.')
    parser.add_argument('--list-ports', action='store_true',
                        help='List available serial ports and exit')

    sub = parser.add_subparsers(dest='command', help='Commands')

    # monitor
    p_mon = sub.add_parser('monitor', help='Real-time data display')
    p_mon.set_defaults(func=cmd_monitor)

    # collect
    p_col = sub.add_parser('collect', help='Collect data and output JSON')
    p_col.add_argument('-t', '--time', type=float, default=3.0, help='Duration in seconds')
    p_col.set_defaults(func=cmd_collect)

    # step
    p_step = sub.add_parser('step', help='Run step test and analyze')
    p_step.add_argument('-w', '--wheel', type=int, default=4, help='Wheel 0-3 or 4=ALL')
    p_step.add_argument('-s', '--speed', type=float, default=0.3, help='Target speed m/s')
    p_step.add_argument('-d', '--duration', type=float, default=2.0, help='Test duration')
    p_step.add_argument('--kp', type=float, help='Override Kp')
    p_step.add_argument('--ki', type=float, help='Override Ki')
    p_step.add_argument('--kd', type=float, help='Override Kd')
    p_step.add_argument('--json', action='store_true', help='Output JSON for Claude')
    p_step.set_defaults(func=cmd_step)

    # tune
    p_tune = sub.add_parser('tune', help='Auto-tune PID')
    p_tune.add_argument('-w', '--wheel', type=int, default=4, help='Wheel 0-3 or 4=ALL')
    p_tune.add_argument('-s', '--speed', type=float, default=0.3, help='Target speed m/s')
    p_tune.add_argument('-n', '--iterations', type=int, default=5, help='Max iterations')
    p_tune.set_defaults(func=cmd_tune)

    # set
    p_set = sub.add_parser('set', help='Manually set PID/FF/Alpha params')
    p_set.add_argument('-w', '--wheel', type=int, required=True, help='Wheel 0-3 or 4=ALL')
    p_set.add_argument('-p', '--kp', type=float, help='Kp value')
    p_set.add_argument('-i', '--ki', type=float, help='Ki value')
    p_set.add_argument('-d', '--kd', type=float, help='Kd value')
    p_set.add_argument('--ff', type=float, help='BR feedforward (F command)')
    p_set.add_argument('--alpha', type=float, help='EMA filter alpha 0~0.99 (L command)')
    p_set.set_defaults(func=cmd_set)

    # raw
    p_raw = sub.add_parser('raw', help='Send raw command')
    p_raw.add_argument('-c', '--cmd', type=str, required=True, help='Command to send')
    p_raw.set_defaults(func=cmd_raw)

    args = parser.parse_args()

    # --list-ports
    if args.list_ports:
        list_serial_ports()
        sys.exit(0)

    # No command + no port → interactive mode
    if not args.command:
        interactive_mode()
        sys.exit(0)

    # No port specified → ask
    if not args.port:
        port = prompt_port()
    else:
        port = args.port

    bridge = SerialBridge(port)
    try:
        args.func(args, bridge)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bridge.close()

if __name__ == '__main__':
    main()
