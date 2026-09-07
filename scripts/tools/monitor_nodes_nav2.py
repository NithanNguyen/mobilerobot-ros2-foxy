#!/usr/bin/env python3
"""
monitor_nodes_nav2.py
=====================
ROS 2 Foxy — Nav2 Node Performance Monitor for Jetson Xavier AGX
Measures per-node CPU / RAM over time with automatic stage detection
via /robot/state topic subscription.

Usage:
    # Basic — monitor default Nav2 nodes
    python3 monitor_nodes_nav2.py

    # Custom node list
    python3 monitor_nodes_nav2.py --nodes amcl controller_server planner_server

    # Custom duration and output dir
    python3 monitor_nodes_nav2.py --duration 120 --output /home/nguyenan/logs

Dependencies (install once):
    pip3 install psutil pandas matplotlib --break-system-packages

Author: NguyenAn — Jetson Xavier AGX / ROS 2 Foxy
"""

# ─── stdlib ────────────────────────────────────────────────────────────────
import argparse
import csv
import os
import signal
import subprocess
import sys
import threading
import time
from collections import defaultdict, deque
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ─── third-party ───────────────────────────────────────────────────────────
try:
    import psutil
except ImportError:
    sys.exit("[ERROR] psutil not found. Run: pip3 install psutil --break-system-packages")

try:
    import pandas as pd
    import matplotlib
    matplotlib.use("Agg")           # headless-safe; swap to TkAgg if display available
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import matplotlib.ticker as ticker
    from matplotlib.gridspec import GridSpec
    from matplotlib.lines import Line2D
except ImportError:
    sys.exit("[ERROR] pandas / matplotlib not found. Run: pip3 install pandas matplotlib --break-system-packages")

# ─── ROS 2 (optional — graceful degradation if not sourced) ────────────────
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String as StringMsg
    ROS2_AVAILABLE = True
except ImportError:
    pass  # Monitor will run without ROS 2 state subscription


# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ══════════════════════════════════════════════════════════════════════════════

DEFAULT_NODES = [
    "amcl",
    "controller_server",
    "planner_server",
    "bt_navigator",
    "ekf_filter_node",
    "slam_toolbox",
    "navigator",
]

# CPU sampling: psutil needs two calls separated by interval to give accurate %
CPU_INTERVAL     = 0.2      # seconds between cpu_percent polls (per process)
SAMPLE_RATE_HZ   = 2        # how many samples/sec we write to log
ROLLING_WINDOW   = 6        # samples for smoothing (~3s at 2Hz)

# Tegrastats path on Jetson
TEGRASTATS_BIN   = "/usr/bin/tegrastats"
TEGRASTATS_MS    = 500      # tegrastats polling interval in ms

# Nav2 state topic (published by navigator.py)
ROBOT_STATE_TOPIC = "/robot/state"

# Severity thresholds (per-node)
CPU_WARN_PCT  = 70.0        # yellow highlight on chart
CPU_CRIT_PCT  = 90.0        # red highlight
RAM_WARN_MB   = 400.0
RAM_CRIT_MB   = 800.0

# Chart aesthetics
PALETTE = [
    "#4e9af1", "#f97316", "#22c55e", "#a855f7",
    "#ec4899", "#14b8a6", "#eab308", "#ef4444",
    "#8b5cf6", "#06b6d4",
]
STAGE_COLORS = {
    "IDLE":           "#64748b",
    "COMPUTING_PATH": "#f97316",
    "PRE_ROTATING":   "#eab308",
    "NAVIGATING":     "#22c55e",
    "STOPPED":        "#ef4444",
    "WAITING_RESET":  "#a855f7",
    "RETURNING_HOME": "#14b8a6",
    "unknown":        "#94a3b8",
}


# ══════════════════════════════════════════════════════════════════════════════
#  PROCESS FINDER
# ══════════════════════════════════════════════════════════════════════════════

class ProcessRegistry:
    """
    Resolves ROS 2 node names → psutil.Process objects.
    Handles:
      - Python nodes  (python3 … navigator.py)
      - C++ nodes     (amcl, controller_server, …)
      - Component containers sharing one PID
    Caches PID→Process; refreshes when a process disappears.
    """

    def __init__(self, node_names: List[str]):
        self.node_names = node_names
        self._cache: Dict[str, psutil.Process] = {}   # node_name → Process
        self._lock  = threading.Lock()

    def _scan(self) -> Dict[str, psutil.Process]:
        found: Dict[str, psutil.Process] = {}
        for proc in psutil.process_iter(["pid", "name", "cmdline", "status"]):
            try:
                info = proc.info
                if info["status"] == psutil.STATUS_ZOMBIE:
                    continue
                cmdline = info.get("cmdline") or []
                cmd_str = " ".join(cmdline)

                for target in self.node_names:
                    if target in found:
                        continue
                    # Match C++ executables by process name
                    if info["name"] == target:
                        found[target] = proc
                        continue
                    # Match Python nodes by cmdline
                    if f"__node:={target}" in cmd_str:
                        found[target] = proc
                        continue
                    # ROS 2 Foxy passes node name differently
                    if f"--ros-args" in cmd_str and target in cmd_str:
                        # Avoid false positives: check target is a "word" in cmdline
                        tokens = cmd_str.split()
                        if target in tokens or any(
                            tok.endswith(f"/{target}") or tok.endswith(f"/{target}.py")
                            for tok in tokens
                        ):
                            found[target] = proc
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return found

    def get_processes(self) -> Dict[str, psutil.Process]:
        """Return cached mapping; re-scan if any cached process is gone."""
        with self._lock:
            # Check liveness
            stale = [n for n, p in self._cache.items() if not p.is_running()]
            if stale or len(self._cache) < len(self.node_names):
                self._cache = self._scan()
            return dict(self._cache)


# ══════════════════════════════════════════════════════════════════════════════
#  TEGRASTATS READER  (Jetson-specific: GPU, EMC, thermal)
# ══════════════════════════════════════════════════════════════════════════════

class TegrastatsReader:
    """
    Spawns tegrastats as a subprocess, parses output in a background thread.
    Provides latest snapshot via .get().

    Example tegrastats line (Xavier AGX):
      RAM 3412/15822MB (lfb 1x2MB) SWAP 0/7911MB ...
      CPU [12%@1907,8%@1907,15%@1907,6%@1907,4%@1907,3%@1907,7%@1907,5%@1907]
      GPU 23%@522 EMC_FREQ 39% APE 150 PLL@42C CPU@44C PMIC@100C GPU@43C AO@39.5C thermal@43.4C
    """

    def __init__(self, interval_ms: int = 500):
        self._interval_ms = interval_ms
        self._latest: dict = {}
        self._lock   = threading.Lock()
        self._proc   = None
        self._thread = None
        self._stop   = threading.Event()

    def start(self):
        if not Path(TEGRASTATS_BIN).exists():
            print(f"[WARN] tegrastats not found at {TEGRASTATS_BIN}. Jetson metrics disabled.")
            return
        cmd = [TEGRASTATS_BIN, "--interval", str(self._interval_ms)]
        self._proc   = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                        stderr=subprocess.DEVNULL, text=True)
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self):
        for line in self._proc.stdout:
            if self._stop.is_set():
                break
            parsed = self._parse(line.strip())
            with self._lock:
                self._latest = parsed

    def _parse(self, line: str) -> dict:
        data: dict = {}
        try:
            # CPU cores: CPU [12%@1907, ...]
            import re
            cpu_match = re.search(r"CPU \[([^\]]+)\]", line)
            if cpu_match:
                cores = cpu_match.group(1).split(",")
                percents = []
                freqs    = []
                for c in cores:
                    c = c.strip()
                    if "@" in c:
                        pct, freq = c.split("@")
                        percents.append(float(pct.replace("%", "").replace("off", "0")))
                        freqs.append(float(freq))
                data["cpu_cores_pct"]  = percents
                data["cpu_avg_pct"]    = sum(percents) / len(percents) if percents else 0.0
                data["cpu_max_pct"]    = max(percents) if percents else 0.0
                data["cpu_freq_mhz"]   = freqs[0] if freqs else 0.0
                data["cpu_n_active"]   = sum(1 for p in percents if p > 0)

            # GPU: GPU 23%@522
            gpu_match = re.search(r"GPU\s+(\d+)%@(\d+)", line)
            if gpu_match:
                data["gpu_pct"]      = float(gpu_match.group(1))
                data["gpu_freq_mhz"] = float(gpu_match.group(2))

            # RAM: RAM 3412/15822MB
            ram_match = re.search(r"RAM\s+(\d+)/(\d+)MB", line)
            if ram_match:
                data["ram_used_mb"]  = float(ram_match.group(1))
                data["ram_total_mb"] = float(ram_match.group(2))
                data["ram_pct"]      = 100.0 * float(ram_match.group(1)) / float(ram_match.group(2))

            # EMC (memory controller): EMC_FREQ 39%
            emc_match = re.search(r"EMC_FREQ\s+(\d+)%", line)
            if emc_match:
                data["emc_pct"] = float(emc_match.group(1))

            # Thermals: CPU@44C GPU@43C
            for sensor in ("CPU", "GPU", "thermal", "PLL", "AO"):
                t_match = re.search(rf"{sensor}@([\d.]+)C", line)
                if t_match:
                    data[f"temp_{sensor.lower()}_c"] = float(t_match.group(1))

        except Exception:
            pass
        return data

    def get(self) -> dict:
        with self._lock:
            return dict(self._latest)

    def stop(self):
        self._stop.set()
        if self._proc:
            self._proc.terminate()


# ══════════════════════════════════════════════════════════════════════════════
#  ROS 2 STATE SUBSCRIBER
# ══════════════════════════════════════════════════════════════════════════════

class RobotStateWatcher:
    """
    Subscribes to /robot/state (std_msgs/String) in a background thread.
    Stores (timestamp, state) transitions for stage markers on chart.
    """

    def __init__(self):
        self.current_state = "unknown"
        self.transitions: List[Tuple[float, str]] = []   # (elapsed_s, state)
        self._start_time   = 0.0
        self._node         = None
        self._executor     = None
        self._thread       = None
        self._active       = False

    def start(self, start_time: float):
        if not ROS2_AVAILABLE:
            print("[INFO] ROS 2 not available — state subscription disabled.")
            return
        self._start_time = start_time
        self._active     = True
        self._thread     = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def _spin_loop(self):
        try:
            rclpy.init(args=None)
            self._node = rclpy.create_node("perf_monitor_state_watcher")
            self._node.create_subscription(
                StringMsg, ROBOT_STATE_TOPIC,
                self._cb, 10
            )
            rclpy.spin(self._node)
        except Exception as e:
            print(f"[WARN] ROS 2 state watcher error: {e}")
        finally:
            if self._node:
                self._node.destroy_node()

    def _cb(self, msg: "StringMsg"):
        new_state = msg.data.strip()
        if new_state != self.current_state:
            elapsed = time.time() - self._start_time
            self.transitions.append((elapsed, new_state))
            self.current_state = new_state
            print(f"\n[STATE] {new_state} @ t={elapsed:.1f}s")

    def stop(self):
        self._active = False
        if ROS2_AVAILABLE and rclpy.ok():
            rclpy.shutdown()


# ══════════════════════════════════════════════════════════════════════════════
#  ANOMALY DETECTOR
# ══════════════════════════════════════════════════════════════════════════════

class AnomalyDetector:
    """
    Detects and records threshold violations per node.
    Produces a summary table for the CSV and chart annotations.
    """

    def __init__(self):
        self.events: List[dict] = []   # {time, node, metric, value, level}

    def check(self, elapsed: float, node: str,
              cpu_pct: float, ram_mb: float):
        if cpu_pct >= CPU_CRIT_PCT:
            self.events.append(dict(time=elapsed, node=node, metric="CPU",
                                    value=cpu_pct, level="CRITICAL"))
        elif cpu_pct >= CPU_WARN_PCT:
            self.events.append(dict(time=elapsed, node=node, metric="CPU",
                                    value=cpu_pct, level="WARNING"))
        if ram_mb >= RAM_CRIT_MB:
            self.events.append(dict(time=elapsed, node=node, metric="RAM",
                                    value=ram_mb, level="CRITICAL"))
        elif ram_mb >= RAM_WARN_MB:
            self.events.append(dict(time=elapsed, node=node, metric="RAM",
                                    value=ram_mb, level="WARNING"))

    def summary(self) -> str:
        if not self.events:
            return "No anomalies detected."
        lines = [f"{'Time':>7} {'Node':<25} {'Metric':<6} {'Value':>8} {'Level'}"]
        lines.append("-" * 60)
        for e in self.events[-40:]:      # last 40 to keep summary short
            lines.append(
                f"{e['time']:>7.1f} {e['node']:<25} {e['metric']:<6} "
                f"{e['value']:>8.1f} {e['level']}"
            )
        return "\n".join(lines)


# ══════════════════════════════════════════════════════════════════════════════
#  SAMPLER  (main data collection loop)
# ══════════════════════════════════════════════════════════════════════════════

class Sampler:
    """Collects per-node CPU/RAM + tegrastats + anomalies at SAMPLE_RATE_HZ."""

    def __init__(self, node_names: List[str],
                 tegrastats: TegrastatsReader,
                 anomaly: AnomalyDetector):
        self.node_names = node_names
        self.registry   = ProcessRegistry(node_names)
        self.tegrastats = tegrastats
        self.anomaly    = anomaly
        self.records: List[dict] = []
        self._stop      = threading.Event()

        # Per-process CPU smoothing (psutil needs interval=None after first call)
        self._cpu_procs: Dict[int, psutil.Process] = {}

    def run(self, duration_s: float, start_time: float):
        interval = 1.0 / SAMPLE_RATE_HZ
        # Prime cpu_percent (first call always returns 0.0)
        for proc in self.registry.get_processes().values():
            try:
                proc.cpu_percent(interval=None)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        time.sleep(CPU_INTERVAL)

        deadline = start_time + duration_s
        while not self._stop.is_set():
            t0      = time.time()
            elapsed = t0 - start_time

            if elapsed > duration_s:
                break

            row = self._sample(elapsed)
            self.records.append(row)

            # Console status line
            total_cpu = sum(row.get(f"{n}_cpu_pct", 0) for n in self.node_names)
            total_ram = sum(row.get(f"{n}_ram_mb",  0) for n in self.node_names)
            tstat     = self.tegrastats.get()
            sys_cpu   = tstat.get("cpu_avg_pct", 0.0)
            sys_ram   = tstat.get("ram_used_mb",  0.0)
            gpu_pct   = tstat.get("gpu_pct",       0.0)
            temp_cpu  = tstat.get("temp_cpu_c",    0.0)

            sys.stdout.write(
                f"\r[{elapsed:6.1f}s] "
                f"SYS CPU:{sys_cpu:5.1f}% "
                f"GPU:{gpu_pct:5.1f}% "
                f"RAM:{sys_ram:6.0f}MB "
                f"T_CPU:{temp_cpu:4.1f}°C | "
                f"NAV2 CPU:{total_cpu:6.1f}% "
                f"RAM:{total_ram:6.0f}MB    "
            )
            sys.stdout.flush()

            # Sleep for remainder of interval
            elapsed_this = time.time() - t0
            sleep_s = max(0.0, interval - elapsed_this)
            time.sleep(sleep_s)

    def _sample(self, elapsed: float) -> dict:
        row: dict = {"elapsed_s": round(elapsed, 2)}

        # Tegrastats system-level metrics
        tstat = self.tegrastats.get()
        row["sys_cpu_avg_pct"]  = tstat.get("cpu_avg_pct",    0.0)
        row["sys_cpu_max_pct"]  = tstat.get("cpu_max_pct",    0.0)
        row["sys_cpu_freq_mhz"] = tstat.get("cpu_freq_mhz",   0.0)
        row["sys_n_active_core"]= tstat.get("cpu_n_active",   0)
        row["sys_gpu_pct"]      = tstat.get("gpu_pct",         0.0)
        row["sys_gpu_freq_mhz"] = tstat.get("gpu_freq_mhz",   0.0)
        row["sys_ram_used_mb"]  = tstat.get("ram_used_mb",    0.0)
        row["sys_ram_pct"]      = tstat.get("ram_pct",         0.0)
        row["sys_emc_pct"]      = tstat.get("emc_pct",         0.0)
        row["temp_cpu_c"]       = tstat.get("temp_cpu_c",     0.0)
        row["temp_gpu_c"]       = tstat.get("temp_gpu_c",     0.0)
        row["temp_thermal_c"]   = tstat.get("temp_thermal_c", 0.0)

        # Per-node metrics
        processes = self.registry.get_processes()
        for name in self.node_names:
            proc = processes.get(name)
            cpu_pct = 0.0
            ram_mb  = 0.0
            n_threads = 0
            if proc:
                try:
                    cpu_pct   = proc.cpu_percent(interval=None) / psutil.cpu_count()
                    mem       = proc.memory_info()
                    ram_mb    = mem.rss / (1024 * 1024)
                    n_threads = proc.num_threads()
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            row[f"{name}_cpu_pct"]   = round(cpu_pct,   2)
            row[f"{name}_ram_mb"]    = round(ram_mb,    2)
            row[f"{name}_threads"]   = n_threads
            row[f"{name}_running"]   = 1 if proc else 0
            self.anomaly.check(elapsed, name, cpu_pct, ram_mb)

        return row

    def stop(self):
        self._stop.set()


# ══════════════════════════════════════════════════════════════════════════════
#  CHART BUILDER
# ══════════════════════════════════════════════════════════════════════════════

def build_charts(df: pd.DataFrame,
                 node_names: List[str],
                 state_transitions: List[Tuple[float, str]],
                 anomaly: AnomalyDetector,
                 output_dir: str,
                 timestamp: str):

    # ── Smoothing ────────────────────────────────────────────────────────────
    for name in node_names:
        df[f"{name}_cpu_smooth"] = (
            df[f"{name}_cpu_pct"].rolling(ROLLING_WINDOW, min_periods=1).mean()
        )
        df[f"{name}_ram_smooth"] = (
            df[f"{name}_ram_mb"].rolling(ROLLING_WINDOW, min_periods=1).mean()
        )

    sys_cols = ["sys_cpu_avg_pct", "sys_cpu_max_pct",
                "sys_gpu_pct", "sys_ram_used_mb",
                "sys_emc_pct", "temp_cpu_c", "temp_gpu_c"]
    for col in sys_cols:
        if col in df.columns:
            df[f"{col}_smooth"] = df[col].rolling(ROLLING_WINDOW, min_periods=1).mean()

    nav2_total_cpu = df[[f"{n}_cpu_smooth" for n in node_names]].sum(axis=1)
    nav2_total_ram = df[[f"{n}_ram_smooth" for n in node_names]].sum(axis=1)

    t = df["elapsed_s"]

    # ── Layout ───────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(18, 22), facecolor="#0f1117")
    gs  = GridSpec(5, 2, figure=fig,
                   hspace=0.42, wspace=0.28,
                   left=0.07, right=0.97, top=0.94, bottom=0.05)

    ax_cpu_node  = fig.add_subplot(gs[0, :])   # full width — per-node CPU
    ax_cpu_sys   = fig.add_subplot(gs[1, 0])   # system CPU + GPU
    ax_ram_node  = fig.add_subplot(gs[1, 1])   # per-node RAM
    ax_ram_sys   = fig.add_subplot(gs[2, 0])   # system RAM + EMC
    ax_temp      = fig.add_subplot(gs[2, 1])   # temperatures
    ax_total     = fig.add_subplot(gs[3, :])   # Nav2 total CPU + RAM dual-axis
    ax_cores     = fig.add_subplot(gs[4, :])   # per-core CPU heatmap (from tegrastats)

    def style(ax, title: str, ylabel: str):
        ax.set_facecolor("#161b22")
        ax.set_title(title, color="#e2e8f0", fontsize=11, fontweight="bold", pad=6)
        ax.set_ylabel(ylabel, color="#94a3b8", fontsize=9)
        ax.tick_params(colors="#64748b", labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor("#1e2530")
        ax.grid(True, color="#1e2530", linewidth=0.6, alpha=0.8)
        ax.xaxis.set_major_locator(ticker.MultipleLocator(10))
        ax.set_xlabel("Time (s)", color="#64748b", fontsize=8)

    # ── Helper: draw state band shading ──────────────────────────────────────
    def draw_state_bands(ax):
        if not state_transitions:
            return
        t_max = t.max()
        transitions = [(0.0, "unknown")] + list(state_transitions)
        for i, (t_start, state) in enumerate(transitions):
            t_end = transitions[i + 1][0] if i + 1 < len(transitions) else t_max
            color = STAGE_COLORS.get(state, "#94a3b8")
            ax.axvspan(t_start, t_end, alpha=0.08, color=color, zorder=0)
            ax.axvline(t_start, color=color, linewidth=0.8, alpha=0.4, linestyle="--", zorder=1)
            ax.text(t_start + 0.3, ax.get_ylim()[1] * 0.97, state,
                    color=color, fontsize=6, va="top", alpha=0.7)

    # ── Helper: annotate critical anomalies on an axis ───────────────────────
    def draw_anomaly_markers(ax, metric: str, y_max: float):
        crits = [e for e in anomaly.events
                 if e["metric"] == metric and e["level"] == "CRITICAL"]
        for e in crits:
            ax.axvline(e["time"], color="#ef4444", linewidth=0.6, alpha=0.5)

    # ╔══════════════════════════════════════════════════════════════════════╗
    # ║  PLOT 1 — Per-node CPU                                               ║
    # ╚══════════════════════════════════════════════════════════════════════╝
    style(ax_cpu_node, "Per-Node CPU Usage  (% per core)", "CPU %")
    for i, name in enumerate(node_names):
        col = PALETTE[i % len(PALETTE)]
        ax_cpu_node.plot(t, df[f"{name}_cpu_smooth"],
                         label=name, color=col, linewidth=1.6, alpha=0.9)
    ax_cpu_node.axhline(CPU_WARN_PCT, color="#eab308", linewidth=0.8,
                        linestyle=":", alpha=0.7, label=f"WARN {CPU_WARN_PCT}%")
    ax_cpu_node.axhline(CPU_CRIT_PCT, color="#ef4444", linewidth=0.8,
                        linestyle=":", alpha=0.7, label=f"CRIT {CPU_CRIT_PCT}%")
    ax_cpu_node.set_ylim(bottom=0)
    ax_cpu_node.legend(loc="upper right", fontsize=8,
                       facecolor="#161b22", edgecolor="#1e2530",
                       labelcolor="#e2e8f0", ncol=4)
    draw_state_bands(ax_cpu_node)
    draw_anomaly_markers(ax_cpu_node, "CPU", ax_cpu_node.get_ylim()[1])

    # ╔══════════════════════════════════════════════════════════════════════╗
    # ║  PLOT 2 — System CPU + GPU                                           ║
    # ╚══════════════════════════════════════════════════════════════════════╝
    style(ax_cpu_sys, "System CPU avg/max  +  GPU  (tegrastats)", "Usage %")
    if "sys_cpu_avg_pct_smooth" in df.columns:
        ax_cpu_sys.plot(t, df["sys_cpu_avg_pct_smooth"],
                        color="#4e9af1", linewidth=1.8, label="CPU avg")
        ax_cpu_sys.plot(t, df["sys_cpu_max_pct_smooth"],
                        color="#f97316", linewidth=1.2, linestyle="--", label="CPU max")
    if "sys_gpu_pct_smooth" in df.columns:
        ax_cpu_sys.plot(t, df["sys_gpu_pct_smooth"],
                        color="#a855f7", linewidth=1.5, label="GPU")
    ax_cpu_sys.set_ylim(0, 105)
    ax_cpu_sys.legend(fontsize=8, facecolor="#161b22", edgecolor="#1e2530", labelcolor="#e2e8f0")
    draw_state_bands(ax_cpu_sys)

    # ╔══════════════════════════════════════════════════════════════════════╗
    # ║  PLOT 3 — Per-node RAM                                               ║
    # ╚══════════════════════════════════════════════════════════════════════╝
    style(ax_ram_node, "Per-Node RAM Usage", "MB")
    for i, name in enumerate(node_names):
        col = PALETTE[i % len(PALETTE)]
        ax_ram_node.plot(t, df[f"{name}_ram_smooth"],
                         label=name, color=col, linewidth=1.6, alpha=0.9)
    ax_ram_node.axhline(RAM_WARN_MB, color="#eab308", linewidth=0.8, linestyle=":", alpha=0.7)
    ax_ram_node.axhline(RAM_CRIT_MB, color="#ef4444", linewidth=0.8, linestyle=":", alpha=0.7)
    ax_ram_node.set_ylim(bottom=0)
    ax_ram_node.legend(loc="upper right", fontsize=8,
                       facecolor="#161b22", edgecolor="#1e2530", labelcolor="#e2e8f0", ncol=2)
    draw_state_bands(ax_ram_node)

    # ╔══════════════════════════════════════════════════════════════════════╗
    # ║  PLOT 4 — System RAM + EMC                                           ║
    # ╚══════════════════════════════════════════════════════════════════════╝
    style(ax_ram_sys, "System RAM  +  EMC bandwidth  (tegrastats)", "RAM MB")
    if "sys_ram_used_mb_smooth" in df.columns:
        ax_ram_sys.plot(t, df["sys_ram_used_mb_smooth"],
                        color="#22c55e", linewidth=1.8, label="RAM used (MB)")
    ax_ram_sys.set_ylim(bottom=0)
    ax2_emc = ax_ram_sys.twinx()
    ax2_emc.set_facecolor("#161b22")
    if "sys_emc_pct_smooth" in df.columns:
        ax2_emc.plot(t, df["sys_emc_pct_smooth"],
                     color="#14b8a6", linewidth=1.2, linestyle="--", alpha=0.7, label="EMC %")
        ax2_emc.set_ylabel("EMC %", color="#14b8a6", fontsize=8)
        ax2_emc.tick_params(colors="#14b8a6", labelsize=8)
        ax2_emc.set_ylim(0, 105)
    lines1, lbl1 = ax_ram_sys.get_legend_handles_labels()
    lines2, lbl2 = ax2_emc.get_legend_handles_labels()
    ax_ram_sys.legend(lines1 + lines2, lbl1 + lbl2, fontsize=8,
                      facecolor="#161b22", edgecolor="#1e2530", labelcolor="#e2e8f0")
    draw_state_bands(ax_ram_sys)

    # ╔══════════════════════════════════════════════════════════════════════╗
    # ║  PLOT 5 — Temperatures                                               ║
    # ╚══════════════════════════════════════════════════════════════════════╝
    style(ax_temp, "Thermal — CPU / GPU / Board  (°C)", "°C")
    temp_map = {
        "temp_cpu_c_smooth":     ("#f97316", "CPU"),
        "temp_gpu_c_smooth":     ("#a855f7", "GPU"),
        "temp_thermal_c_smooth": ("#ec4899", "Board"),
    }
    for col, (color, lbl) in temp_map.items():
        if col in df.columns and df[col].max() > 0:
            ax_temp.plot(t, df[col], color=color, linewidth=1.6, label=lbl)
    ax_temp.axhline(75, color="#ef4444", linewidth=0.8, linestyle=":", alpha=0.6, label="Throttle 75°C")
    ax_temp.set_ylim(bottom=0)
    ax_temp.legend(fontsize=8, facecolor="#161b22", edgecolor="#1e2530", labelcolor="#e2e8f0")
    draw_state_bands(ax_temp)

    # ╔══════════════════════════════════════════════════════════════════════╗
    # ║  PLOT 6 — Nav2 total CPU + RAM (dual axis)                           ║
    # ╚══════════════════════════════════════════════════════════════════════╝
    style(ax_total, "Nav2 Stack — Total CPU  +  Total RAM", "CPU %")
    ax_total.plot(t, nav2_total_cpu.rolling(ROLLING_WINDOW, min_periods=1).mean(),
                  color="#4e9af1", linewidth=2.2, label="Nav2 total CPU %")
    ax_total.set_ylim(bottom=0)

    ax_total_r = ax_total.twinx()
    ax_total_r.set_facecolor("#161b22")
    ax_total_r.plot(t, nav2_total_ram.rolling(ROLLING_WINDOW, min_periods=1).mean(),
                    color="#22c55e", linewidth=2.0, linestyle="--", label="Nav2 total RAM (MB)")
    ax_total_r.set_ylabel("RAM MB", color="#22c55e", fontsize=9)
    ax_total_r.tick_params(colors="#22c55e", labelsize=8)
    ax_total_r.set_ylim(bottom=0)

    # Shade by Nav2 state
    if state_transitions:
        transitions = [(0.0, "unknown")] + list(state_transitions)
        t_max = t.max()
        for i, (ts, state) in enumerate(transitions):
            te    = transitions[i + 1][0] if i + 1 < len(transitions) else t_max
            color = STAGE_COLORS.get(state, "#94a3b8")
            ax_total.axvspan(ts, te, alpha=0.12, color=color, zorder=0)

    lines1, lbl1 = ax_total.get_legend_handles_labels()
    lines2, lbl2 = ax_total_r.get_legend_handles_labels()

    # State legend patches
    seen_states = list(dict.fromkeys([s for _, s in state_transitions]))
    state_patches = [
        mpatches.Patch(color=STAGE_COLORS.get(s, "#94a3b8"), alpha=0.5, label=s)
        for s in seen_states
    ]
    ax_total.legend(lines1 + lines2 + state_patches,
                    lbl1 + lbl2 + seen_states,
                    fontsize=8, facecolor="#161b22",
                    edgecolor="#1e2530", labelcolor="#e2e8f0", ncol=6)

    # ╔══════════════════════════════════════════════════════════════════════╗
    # ║  PLOT 7 — Per-core CPU heatmap (if tegrastats data available)        ║
    # ╚══════════════════════════════════════════════════════════════════════╝
    # Build per-core columns from cpu_cores_pct stored per row
    core_cols = [c for c in df.columns if c.startswith("core_")]
    if core_cols:
        core_data = df[core_cols].values.T  # shape (n_cores, n_samples)
        im = ax_cores.imshow(core_data, aspect="auto", origin="upper",
                             cmap="RdYlGn_r", vmin=0, vmax=100,
                             extent=[t.iloc[0], t.iloc[-1], len(core_cols) + 0.5, 0.5])
        ax_cores.set_yticks(range(1, len(core_cols) + 1))
        ax_cores.set_yticklabels([f"Core {i}" for i in range(len(core_cols))],
                                  color="#94a3b8", fontsize=8)
        ax_cores.set_facecolor("#161b22")
        ax_cores.set_title("Per-Core CPU Heatmap (tegrastats)",
                            color="#e2e8f0", fontsize=11, fontweight="bold")
        ax_cores.set_xlabel("Time (s)", color="#64748b", fontsize=8)
        ax_cores.tick_params(colors="#64748b", labelsize=8)
        cbar = fig.colorbar(im, ax=ax_cores, orientation="vertical", pad=0.01)
        cbar.ax.tick_params(colors="#94a3b8", labelsize=8)
        cbar.set_label("CPU %", color="#94a3b8", fontsize=8)
    else:
        style(ax_cores, "Per-Core CPU (no tegrastats data)", "")
        ax_cores.text(0.5, 0.5, "tegrastats not available on this system",
                      transform=ax_cores.transAxes,
                      ha="center", va="center", color="#64748b", fontsize=11)

    # ── Super title ──────────────────────────────────────────────────────────
    fig.suptitle(
        f"Nav2 Performance Monitor — Jetson Xavier AGX  |  {timestamp}",
        color="#e2e8f0", fontsize=14, fontweight="bold", y=0.97
    )

    # ── Save ─────────────────────────────────────────────────────────────────
    chart_path = os.path.join(output_dir, f"benchmark_nav2_{timestamp}.png")
    fig.savefig(chart_path, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"\n[CHART] Saved → {chart_path}")
    return chart_path


# ══════════════════════════════════════════════════════════════════════════════
#  CSV WRITER
# ══════════════════════════════════════════════════════════════════════════════

def save_csv(records: List[dict],
             state_transitions: List[Tuple[float, str]],
             anomaly: AnomalyDetector,
             node_names: List[str],
             output_dir: str,
             timestamp: str) -> str:
    """Write main data CSV + anomaly summary CSV."""

    # ── Merge state column into records ──────────────────────────────────────
    state_map: Dict[float, str] = {}
    if state_transitions:
        transitions = [(0.0, "unknown")] + list(state_transitions)
        for i, (ts, state) in enumerate(transitions):
            te = transitions[i + 1][0] if i + 1 < len(transitions) else float("inf")
            for rec in records:
                if ts <= rec["elapsed_s"] < te:
                    state_map[rec["elapsed_s"]] = state

    for rec in records:
        rec["robot_state"] = state_map.get(rec["elapsed_s"], "unknown")

    # Main CSV
    main_path = os.path.join(output_dir, f"benchmark_nav2_{timestamp}.csv")
    if records:
        keys = list(records[0].keys())
        with open(main_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(records)
    print(f"[CSV]   Saved → {main_path}")

    # Anomaly CSV
    if anomaly.events:
        anom_path = os.path.join(output_dir, f"anomalies_nav2_{timestamp}.csv")
        with open(anom_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["time", "node", "metric", "value", "level"])
            writer.writeheader()
            writer.writerows(anomaly.events)
        print(f"[CSV]   Anomalies → {anom_path}")

    return main_path


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Nav2 Node Performance Monitor — Jetson Xavier AGX / ROS 2 Foxy",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--nodes", nargs="+", default=DEFAULT_NODES,
        help="ROS 2 node names to monitor (default: Nav2 core nodes)"
    )
    parser.add_argument(
        "--duration", type=float, default=0,
        help="Recording duration in seconds (0 = run until Ctrl+C)"
    )
    parser.add_argument(
        "--output", default=".",
        help="Output directory for CSV and PNG files"
    )
    parser.add_argument(
        "--no-ros", action="store_true",
        help="Disable ROS 2 state subscription (pure psutil + tegrastats mode)"
    )
    args = parser.parse_args()

    # Resolve output dir
    out_dir = Path(args.output).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    duration  = args.duration if args.duration > 0 else float("inf")

    print("=" * 60)
    print("  Nav2 Performance Monitor  —  Jetson Xavier AGX")
    print("=" * 60)
    print(f"  Nodes      : {args.nodes}")
    print(f"  Duration   : {'until Ctrl+C' if duration == float('inf') else f'{duration}s'}")
    print(f"  Sample rate: {SAMPLE_RATE_HZ} Hz")
    print(f"  Output dir : {out_dir}")
    print(f"  ROS 2 state: {'disabled' if args.no_ros else ROBOT_STATE_TOPIC}")
    print("=" * 60)
    print()

    # ── Init subsystems ───────────────────────────────────────────────────────
    tegrastats  = TegrastatsReader(interval_ms=TEGRASTATS_MS)
    anomaly     = AnomalyDetector()
    state_watcher = RobotStateWatcher()
    sampler       = Sampler(args.nodes, tegrastats, anomaly)

    tegrastats.start()

    start_time = time.time()

    if not args.no_ros:
        state_watcher.start(start_time)

    # ── Ctrl+C handler ────────────────────────────────────────────────────────
    def _sigint(sig, frame):
        print("\n\n[INFO] Ctrl+C — stopping monitor...")
        sampler.stop()

    signal.signal(signal.SIGINT, _sigint)

    # ── Run sampler (blocks until done or Ctrl+C) ─────────────────────────────
    try:
        sampler.run(duration_s=duration, start_time=start_time)
    except Exception as e:
        print(f"\n[ERROR] Sampler crashed: {e}")

    # ── Teardown ──────────────────────────────────────────────────────────────
    tegrastats.stop()
    state_watcher.stop()

    print(f"\n\n[INFO] Collected {len(sampler.records)} samples.")

    if not sampler.records:
        print("[WARN] No data collected. Exiting.")
        return

    # ── Anomaly summary ───────────────────────────────────────────────────────
    print("\n── Anomaly Summary ──────────────────────────────────────")
    print(anomaly.summary())

    # ── Save outputs ──────────────────────────────────────────────────────────
    df = pd.DataFrame(sampler.records)

    # Expand cpu_cores_pct list into per-column (if available)
    # (tegrastats data is not per-row yet; placeholder for future extension)

    csv_path = save_csv(
        sampler.records,
        state_watcher.transitions,
        anomaly,
        args.nodes,
        str(out_dir),
        timestamp,
    )

    chart_path = build_charts(
        df,
        args.nodes,
        state_watcher.transitions,
        anomaly,
        str(out_dir),
        timestamp,
    )

    print(f"\n[DONE]")
    print(f"  CSV   → {csv_path}")
    print(f"  Chart → {chart_path}")

if __name__ == "__main__":
    main()