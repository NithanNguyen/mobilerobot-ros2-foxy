#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bag_to_path.py
==============================================================================
Trích path THỰC TẾ của robot từ một bag ROS 2 (offline) và vẽ đè lên bản đồ SLAM.

Nguồn dữ liệu (đã xác minh trong project X221):
  - Pose robot trong hệ bản đồ  = TF (map -> base_footprint)
        map  -> odom            do AMCL publish trên /tf
        odom -> base_footprint  do EKF  publish trên /tf   (ekf.yaml: publish_tf=true,
                                                            base_link_frame=base_footprint)
  - Ranh giới từng chặng        = /robot/status_message
        bắt đầu chặng: "Navigating -> [id] 'name'"   (navigator.py dòng 801)
        kết thúc chặng: "Arrived..."/"At Home..."/"Aborted..." hoặc chặng kế tiếp
  - Nền ảnh                     = map_<floor>.pgm + map_<floor>.yaml (map_server)

Các topic cần có trong bag (nav_v3_launch.py dòng 157-170):
  /tf, /tf_static, /robot/status_message   (bắt buộc)
  /robot/state, /amcl_pose                 (tùy chọn, để đối chiếu)

Output:
  <out>/full_path.csv                 : t_sec, x, y, yaw  (toàn bộ run, THÔ)
  <out>/path_settled.csv              : như trên nhưng đã cắt pha định vị đầu
  <out>/segments/seg_XX_cp<ID>.csv    : path từng chặng
  <out>/overview.png                  : map + path (theo --path-mode / --draw-mode)
  <out>/segments/seg_XX_cp<ID>.png    : (chỉ khi --path-mode segment) từng chặng, XANH BIỂN

Tham số vẽ (xem --help):
  --path-mode {full,segment}   full: 1 ảnh path tổng | segment: overview đa màu + N PNG chặng
  --full-color {blue,time}     (chỉ full) đơn sắc xanh biển | gradient tím→vàng theo thời gian
  --draw-mode {keep,valid}     keep: vẽ tất cả, cạnh-nhảy = NÉT ĐỨT xám(khởi động)/đỏ(sau định vị)
                               valid: chỉ vẽ path hợp lệ (ngắt & BỎ cạnh-nhảy)

Xử lý gián đoạn AMCL: pose map->base_footprint "nhảy" mỗi khi AMCL hiệu chỉnh
(hội tụ lúc đầu / relocalize). Ta KHÔNG sửa số liệu gốc — chỉ thể hiện cú nhảy
bằng nét đứt (keep) hoặc loại bỏ cạnh-nhảy (valid). Ranh giới xám/đỏ ở chế độ
keep = settle_index (từ --settle-window); trong keep KHÔNG cắt pha đầu (vẽ từ
pose 0) — settle_index chỉ dùng để phân màu. full_path.csv luôn là dữ liệu thô.

Chạy được trên ROS 2 Foxy (Jetson) và Humble (laptop). Chỉ dùng message chuẩn
(tf2_msgs, std_msgs, geometry_msgs) nên bag Foxy đọc trên Humble không lỗi type.

Cách chạy:
  source /opt/ros/<distro>/setup.bash
  python3 bag_to_path.py --bag ~/mbrobot_ws/bags/sS1_run01_YYYYMMDD_HHMMSS \
                         --map ~/mbrobot_ws/install/mobile_robot/share/mobile_robot/maps/map_e6_v1.yaml \
                         --out ./out_sS1_run01

Phụ thuộc: rclpy, rosbag2_py (từ ROS 2) + numpy, matplotlib, pyyaml (pip).
==============================================================================
"""

import argparse
import math
import os
import re
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")  # không cần màn hình (chạy được qua SSH trên Jetson)
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.collections import LineCollection


def _get_cmap(name):
    """Lấy colormap tương thích mọi phiên bản matplotlib.
    matplotlib >= 3.6 gỡ cm.get_cmap -> dùng matplotlib.colormaps[name].
    Giữ portable giữa Foxy (matplotlib cũ) và Humble/pip mới."""
    try:
        return matplotlib.colormaps[name]        # matplotlib >= 3.6
    except (AttributeError, KeyError):
        return cm.get_cmap(name)                 # matplotlib cũ (Foxy)


# ─────────────────────────────────────────────────────────────────────────────
# 1) Toán học TF 2D (đã self-test)
# ─────────────────────────────────────────────────────────────────────────────
def quat_to_yaw(x, y, z, w):
    """Quaternion -> yaw (rad). Chỉ cần thành phần z,w cho robot phẳng."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compose_2d(a, b):
    """
    Ghép hai phép biến đổi 2D: kết quả = a ∘ b.
      a = map->odom          (ax, ay, ayaw)
      b = odom->base_footprint (bx, by, byaw)
      => map->base_footprint (x, y, yaw)
    """
    ax, ay, ayaw = a
    bx, by, byaw = b
    ca, sa = math.cos(ayaw), math.sin(ayaw)
    x = ax + ca * bx - sa * by
    y = ay + sa * bx + ca * by
    yaw = math.atan2(math.sin(ayaw + byaw), math.cos(ayaw + byaw))  # wrap [-pi,pi]
    return (x, y, yaw)


# ─────────────────────────────────────────────────────────────────────────────
# 2) Đọc bản đồ SLAM (.pgm/.png + .yaml)
# ─────────────────────────────────────────────────────────────────────────────
def _read_pgm_p5_p2(path):
    """Đọc PGM nhị phân (P5) hoặc ASCII (P2), bỏ qua dòng comment (#). Trả numpy 2D."""
    with open(path, "rb") as f:
        data = f.read()
    # Tokenize header, bỏ comment
    idx = 0
    tokens = []

    def _skip_ws_comments(i):
        while i < len(data):
            c = data[i:i + 1]
            if c in b" \t\r\n":
                i += 1
            elif c == b"#":
                while i < len(data) and data[i:i + 1] != b"\n":
                    i += 1
            else:
                break
        return i

    def _read_token(i):
        i = _skip_ws_comments(i)
        j = i
        while j < len(data) and data[j:j + 1] not in b" \t\r\n":
            j += 1
        return data[i:j], j

    magic, idx = _read_token(idx)
    magic = magic.decode("ascii", "ignore")
    if magic not in ("P5", "P2"):
        raise ValueError(f"Không phải PGM P5/P2: magic={magic!r} ({path})")
    w_tok, idx = _read_token(idx)
    h_tok, idx = _read_token(idx)
    max_tok, idx = _read_token(idx)
    width, height, maxval = int(w_tok), int(h_tok), int(max_tok)

    if magic == "P5":
        idx += 1  # đúng 1 whitespace sau maxval, phần còn lại là raster
        raster = data[idx: idx + width * height]
        arr = np.frombuffer(raster, dtype=np.uint8).astype(np.float32)
        arr = arr.reshape((height, width))
    else:  # P2 ASCII
        vals = data[idx:].split()
        arr = np.array([int(v) for v in vals[:width * height]], dtype=np.float32)
        arr = arr.reshape((height, width))
    return arr / float(maxval)  # chuẩn hoá 0..1


def load_map(map_yaml):
    """Trả (image_2d_0..1, resolution, origin=[ox,oy,oyaw])."""
    with open(map_yaml, "r") as f:
        meta = yaml.safe_load(f)
    img_rel = meta["image"]
    img_path = img_rel if os.path.isabs(img_rel) else os.path.join(
        os.path.dirname(os.path.abspath(map_yaml)), img_rel)
    if not os.path.exists(img_path):
        raise FileNotFoundError(f"Không thấy ảnh map: {img_path}")

    ext = os.path.splitext(img_path)[1].lower()
    if ext in (".pgm",):
        img = _read_pgm_p5_p2(img_path)
    else:
        img = plt.imread(img_path)
        if img.ndim == 3:
            img = img[..., :3].mean(axis=2)  # -> grayscale
        img = img.astype(np.float32)
        if img.max() > 1.0:
            img = img / 255.0

    resolution = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"]]  # [x, y, yaw]
    return img, resolution, origin


# ─────────────────────────────────────────────────────────────────────────────
# 3) Đọc bag ROS 2 bằng rosbag2_py
# ─────────────────────────────────────────────────────────────────────────────
def _detect_storage_id(bag_uri):
    """Đọc metadata.yaml để lấy storage_identifier (sqlite3 với Foxy)."""
    meta_path = os.path.join(bag_uri, "metadata.yaml")
    if os.path.exists(meta_path):
        with open(meta_path, "r") as f:
            m = yaml.safe_load(f)
        try:
            return m["rosbag2_bagfile_information"]["storage_identifier"]
        except Exception:
            pass
    return "sqlite3"


def iter_bag(bag_uri):
    """Yield (topic, msg, t_ns) cho mọi message trong bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage_id = _detect_storage_id(bag_uri)

    # StorageOptions/ConverterOptions: keyword args tương thích Foxy & Humble.
    try:
        storage = rosbag2_py.StorageOptions(uri=bag_uri, storage_id=storage_id)
    except TypeError:
        storage = rosbag2_py.StorageOptions(bag_uri, storage_id)
    try:
        conv = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr")
    except TypeError:
        conv = rosbag2_py.ConverterOptions("cdr", "cdr")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage, conv)
    typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}

    cache = {}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in typemap:
            continue
        if topic not in cache:
            cache[topic] = get_message(typemap[topic])
        yield topic, deserialize_message(raw, cache[topic]), t_ns


# ─────────────────────────────────────────────────────────────────────────────
# 4) Trích trajectory (map->base_footprint) + các sự kiện status
# ─────────────────────────────────────────────────────────────────────────────
START_RE = re.compile(r"Navigating.*?\[(-?\d+)\]\s*'([^']*)'")
COMPUTE_RE = re.compile(r"Computing path.*?\[(-?\d+)\]\s*'([^']*)'")
END_RE = re.compile(
    r"(Arrived|At Home|Aborted|canceled|CRITICAL|rejected|IDLE)", re.IGNORECASE)


def extract(bag_uri, map_frame, odom_frame, base_frame, start_marker="navigating"):
    """
    Trả:
      traj  : ndarray Nx4  (t_sec, x, y, yaw)   -- map->base_footprint
      starts: list[(t_sec, cp_id, name)]        -- mốc bắt đầu mỗi chặng
      ends  : list[t_sec]                       -- mốc kết thúc/tới nơi
    """
    latest_map_odom = None    # (x,y,yaw)
    traj = []
    starts, ends = [], []
    t0_ns = None

    start_pat = COMPUTE_RE if start_marker == "computing" else START_RE

    for topic, msg, t_ns in iter_bag(bag_uri):
        if t0_ns is None:
            t0_ns = t_ns
        t_sec = (t_ns - t0_ns) * 1e-9

        if topic == "/tf" or topic == "/tf_static":
            for tr in msg.transforms:
                parent = tr.header.frame_id.lstrip("/")
                child = tr.child_frame_id.lstrip("/")
                q = tr.transform.rotation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                px = tr.transform.translation.x
                py = tr.transform.translation.y
                if parent == map_frame and child == odom_frame:
                    latest_map_odom = (px, py, yaw)
                elif parent == odom_frame and child == base_frame:
                    if latest_map_odom is not None:
                        x, y, yw = compose_2d(latest_map_odom, (px, py, yaw))
                        traj.append((t_sec, x, y, yw))

        elif topic == "/robot/status_message":
            text = msg.data
            m = start_pat.search(text)
            if m:
                starts.append((t_sec, int(m.group(1)), m.group(2)))
            elif END_RE.search(text):
                ends.append(t_sec)

    traj = np.array(traj, dtype=float) if traj else np.empty((0, 4))
    return traj, starts, ends


def build_segments(traj, starts, ends):
    """Cắt traj thành các chặng [start_i, end_i)."""
    segs = []
    if traj.shape[0] == 0 or not starts:
        return segs
    t_end_bag = traj[-1, 0]
    for i, (t_start, cp_id, name) in enumerate(starts):
        t_next_start = starts[i + 1][0] if i + 1 < len(starts) else t_end_bag + 1.0
        # kết thúc = terminal-event sớm nhất trong (t_start, t_next_start), else t_next_start
        cands = [e for e in ends if t_start < e <= t_next_start]
        t_stop = min(cands) if cands else min(t_next_start, t_end_bag)
        mask = (traj[:, 0] >= t_start) & (traj[:, 0] <= t_stop)
        pts = traj[mask]
        if pts.shape[0] >= 2:
            segs.append({"idx": i, "cp_id": cp_id, "name": name,
                         "t0": t_start, "t1": t_stop, "pts": pts})
    return segs


# ─────────────────────────────────────────────────────────────────────────────
# 5) Vẽ
# ─────────────────────────────────────────────────────────────────────────────
def map_extent(img, resolution, origin):
    """extent world cho imshow(origin='upper')."""
    h, w = img.shape
    ox, oy = origin[0], origin[1]
    return [ox, ox + w * resolution, oy, oy + h * resolution]


# ── Xử lý gián đoạn AMCL (jump) ───────────────────────────────────────────────
# Bối cảnh: pose map->base_footprint = (map->odom) ∘ (odom->base_footprint).
# Mỗi khi AMCL cập nhật map->odom (hội tụ lúc đầu / relocalize), pose "nhảy" một
# đoạn. Nối liền nét LIỀN qua cú nhảy sẽ vẽ ra đường "teleport" xuyên tường KHÔNG
# có thật. Ta KHÔNG sửa số liệu — bước > --jump-thresh coi là cạnh-nhảy và được:
#   • draw-mode 'keep' : vẽ NÉT ĐỨT (xám nếu còn trong pha định vị đầu, đỏ nếu sau)
#   • draw-mode 'valid': BỎ hẳn cạnh đó (ngắt nét, chỉ giữ path hợp lệ)
# Không dùng "stitching" vì nó làm sai vị trí. Ngưỡng --jump-thresh dùng chung cho
# CẢ hai draw-mode.

def step_dist(traj):
    """Khoảng cách giữa các pose liên tiếp (m). traj: Nx4 (t,x,y,yaw)."""
    if traj.shape[0] < 2:
        return np.empty((0,))
    return np.hypot(np.diff(traj[:, 1]), np.diff(traj[:, 2]))


def detect_settle_index(traj, big_jump=1.0, window_s=10.0):
    """Chỉ số pose đầu tiên SAU khi AMCL định vị ổn định = ngay sau cú nhảy
    >big_jump cuối cùng nằm trong window_s giây đầu (pha hội tụ định vị: có thể
    do global_localizer quay 360°, HOẶC do publish initial pose rồi AMCL tinh
    chỉnh — tùy launch/run). Nếu không có cú nhảy lớn nào ở đầu -> 0 (không cắt)."""
    d = step_dist(traj)
    if d.size == 0:
        return 0
    t = traj[:, 0]
    early = [i for i in range(d.size) if d[i] > big_jump and t[i] < t[0] + window_s]
    return (early[-1] + 1) if early else 0


def clean_length(traj, jump_thresh):
    """Tổng quãng đường, LOẠI các bước-jump (tránh thổi phồng do teleport)."""
    d = step_dist(traj)
    return float(d[d <= jump_thresh].sum()) if d.size else 0.0


def _draw_path(ax, traj, jump_thresh, draw_mode, color_mode,
               settle_time=float("-inf"), base_color="royalblue",
               cmap="plasma", lw=1.8, zorder=3):
    """Vẽ MỘT quỹ đạo lên ax theo cấu hình. Trả số cạnh-nhảy.

      draw_mode:
        'keep'  -> vẽ toàn bộ; cạnh-nhảy (bước > jump_thresh) tô NÉT ĐỨT:
                   xám nếu pose-trước có t < settle_time (còn trong pha định vị),
                   đỏ  nếu t >= settle_time (nhảy sau khi đã định vị).
        'valid' -> ngắt nét tại cạnh-nhảy rồi BỎ cạnh đó (chỉ path hợp lệ).
      color_mode (áp cho cạnh KHÔNG nhảy):
        'single' -> đơn sắc base_color.
        'time'   -> gradient cmap theo thời gian (chuẩn hoá toàn traj).

    Dùng settle_time (giây, tuyệt đối theo t_sec) thay vì chỉ số -> đúng cho cả
    traj tổng lẫn sub-array từng chặng.
    """
    n = traj.shape[0]
    if n < 2:
        return 0
    d = step_dist(traj)
    is_jump = d > jump_thresh                     # cạnh i nối pose i -> i+1
    n_jump = int(is_jump.sum())

    pts = traj[:, 1:3]
    seg_pts = np.stack([pts[:-1], pts[1:]], axis=1)   # (n-1, 2, 2)
    t_before = traj[:-1, 0]                            # t của pose-trước mỗi cạnh
    tspan = traj[:, 0] - traj[0, 0]
    denom = max(tspan[-1], 1e-9)

    # 1) Cạnh KHÔNG nhảy
    normal = ~is_jump
    if normal.any():
        if color_mode == "time":
            lc = LineCollection(seg_pts[normal], cmap=cmap,
                                linewidths=lw, zorder=zorder)
            lc.set_array(tspan[:-1][normal] / denom)
            lc.set_clim(0, 1)
        else:
            lc = LineCollection(seg_pts[normal], colors=base_color,
                                linewidths=lw, zorder=zorder)
        ax.add_collection(lc)

    # 2) Cạnh-nhảy — chỉ vẽ ở 'keep' (nét đứt xám/đỏ); 'valid' bỏ hẳn
    if draw_mode == "keep" and n_jump:
        gray = is_jump & (t_before < settle_time)
        red = is_jump & (t_before >= settle_time)
        if gray.any():
            ax.add_collection(LineCollection(
                seg_pts[gray], colors="0.6", linewidths=lw,
                linestyles="--", zorder=zorder + 1))
        if red.any():
            ax.add_collection(LineCollection(
                seg_pts[red], colors="red", linewidths=lw,
                linestyles="--", zorder=zorder + 1))
    return n_jump


def draw_overview_full(img, res, origin, traj, out_png, jump_thresh,
                       draw_mode, full_color, settle_index):
    """Overview 1 ảnh cho PATH TỔNG.
    keep  -> vẽ từ pose 0 (thấy cả pha định vị); jump = nét đứt xám/đỏ.
    valid -> vẽ từ settle_index (đã cắt pha định vị); jump bị bỏ."""
    ext = map_extent(img, res, origin)
    fig, ax = plt.subplots(figsize=(11, 12))
    ax.imshow(img, cmap="gray", extent=ext, origin="upper", vmin=0, vmax=1)

    trimmed = (draw_mode == "valid" and settle_index > 0)
    tr = traj[settle_index:] if trimmed else traj
    settle_time = (float(traj[settle_index, 0])
                   if (draw_mode == "keep" and settle_index) else float("-inf"))
    color_mode = "time" if full_color == "time" else "single"

    n_jump = _draw_path(ax, tr, jump_thresh, draw_mode, color_mode,
                        settle_time=settle_time, base_color="royalblue",
                        cmap="plasma", lw=1.8, zorder=3)

    ax.plot(tr[0, 1], tr[0, 2], "o", color="lime", ms=12, mec="k",
            label="Start" + (" (đã định vị)" if trimmed else ""), zorder=7)
    ax.plot(tr[-1, 1], tr[-1, 2], "s", color="red", ms=12, mec="k",
            label="End", zorder=7)

    if color_mode == "time":
        sm = plt.cm.ScalarMappable(cmap="plasma", norm=plt.Normalize(0, 1))
        cb = fig.colorbar(sm, ax=ax, fraction=0.046, pad=0.04)
        cb.set_label("Thời gian (0→1)")

    title = (f"Path map→base_footprint | ≈{clean_length(tr, jump_thresh):.0f} m"
             f" | {n_jump} AMCL jump [{draw_mode}]")
    if trimmed:
        title += f" | cắt từ t={tr[0,0]:.0f}s"
    ax.set_xlabel("x [m] (map frame)")
    ax.set_ylabel("y [m] (map frame)")
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


def draw_overview_segments(img, res, origin, segs, out_png, jump_thresh,
                           draw_mode, settle_time):
    """Overview đa màu: mỗi chặng 1 màu tab10 (đơn sắc). draw_mode áp trong từng
    chặng. (segs đã được caller đảm bảo khác rỗng.)"""
    ext = map_extent(img, res, origin)
    fig, ax = plt.subplots(figsize=(11, 12))
    ax.imshow(img, cmap="gray", extent=ext, origin="upper", vmin=0, vmax=1)

    colors = _get_cmap("tab10")
    for s in segs:
        c = colors(s["idx"] % 10)
        _draw_path(ax, s["pts"], jump_thresh, draw_mode, "single",
                   settle_time=settle_time, base_color=c, lw=2.2, zorder=3)
        ax.plot(s["pts"][0, 1], s["pts"][0, 2], "o", color=c, ms=7, zorder=4,
                label=f"Seg {s['idx']:02d} → CP{s['cp_id']} '{s['name']}'")
        ax.plot(s["pts"][-1, 1], s["pts"][-1, 2], "s", color=c, ms=7, zorder=4)

    ax.set_xlabel("x [m] (map frame)")
    ax.set_ylabel("y [m] (map frame)")
    ax.set_title(f"Actual robot path per segment (map → base_footprint) [{draw_mode}]")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


def draw_segment(img, res, origin, seg, out_png, jump_thresh,
                 draw_mode, settle_time=float("-inf")):
    """PNG cho MỘT chặng — path đơn sắc XANH BIỂN (theo --draw-mode)."""
    ext = map_extent(img, res, origin)
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(img, cmap="gray", extent=ext, origin="upper", vmin=0, vmax=1)
    n_jump = _draw_path(ax, seg["pts"], jump_thresh, draw_mode, "single",
                        settle_time=settle_time, base_color="royalblue",
                        lw=2.4, zorder=3)
    ax.plot(seg["pts"][0, 1], seg["pts"][0, 2], "o", color="lime", ms=9,
            mec="k", label="start", zorder=7)
    ax.plot(seg["pts"][-1, 1], seg["pts"][-1, 2], "s", color="red", ms=9,
            mec="k", label="end", zorder=7)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(f"Seg {seg['idx']:02d} → CP{seg['cp_id']} '{seg['name']}'  "
                 f"({seg['t1'] - seg['t0']:.1f}s, ≈{clean_length(seg['pts'], jump_thresh):.1f} m"
                 f", {n_jump} jump)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────
# 6) Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Bag ROS2 -> path + ảnh đè map SLAM")
    ap.add_argument("--bag", required=True, help="Thư mục bag (chứa metadata.yaml)")
    ap.add_argument("--map", required=True, help="Đường dẫn map_<floor>.yaml")
    ap.add_argument("--out", default="./bag_path_out", help="Thư mục output")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default="base_footprint")
    ap.add_argument("--start-marker", choices=["navigating", "computing"],
                    default="navigating",
                    help="Mốc bắt đầu chặng: 'navigating' (bắt đầu chạy) hay "
                         "'computing' (gồm cả pre-rotate)")
    ap.add_argument("--path-mode", choices=["full", "segment"], default="full",
                    help="'full': 1 ảnh path tổng. 'segment': overview đa màu "
                         "+ N PNG chặng (xanh biển). Mặc định full.")
    ap.add_argument("--full-color", choices=["blue", "time"], default="blue",
                    help="Màu path tổng (CHỈ khi --path-mode full): 'blue' đơn sắc "
                         "xanh biển, 'time' gradient tím→vàng theo thời gian. "
                         "Mặc định blue.")
    ap.add_argument("--draw-mode", choices=["keep", "valid"], default="keep",
                    help="'keep': vẽ toàn bộ, cạnh-nhảy = nét đứt xám(khởi động)/"
                         "đỏ(sau định vị). 'valid': chỉ vẽ path hợp lệ (ngắt & bỏ "
                         "cạnh-nhảy). Mặc định keep.")
    ap.add_argument("--jump-thresh", type=float, default=0.30,
                    help="Bước > ngưỡng (m) coi là jump AMCL (dùng cho CẢ hai "
                         "draw-mode). Mặc định 0.30")
    ap.add_argument("--no-trim-settle", action="store_true",
                    help="Ép settle_index=0: không cắt pha định vị, không có nét xám.")
    ap.add_argument("--settle-jump", type=float, default=1.0,
                    help="Ngưỡng nhảy (m) để nhận diện pha định vị (mặc định 1.0)")
    ap.add_argument("--settle-window", type=float, default=10.0,
                    help="Cửa sổ (s) đầu run để dò pha định vị (mặc định 10.0)")
    args = ap.parse_args()

    if not os.path.isdir(args.bag):
        sys.exit(f"[ERR] --bag không phải thư mục: {args.bag}")
    if not os.path.exists(args.map):
        sys.exit(f"[ERR] không thấy map yaml: {args.map}")

    # ── Đặt tên folder output theo timestamp trong TÊN folder bag ──────────────
    # Quy ước: out cuối cùng = "<--out>_<timestamp>", với <timestamp> là chuỗi
    # khớp mẫu \d{8}_\d{6} (GIỮ NGUYÊN VĂN thứ tự số, không đảo lại) trích từ
    # tên folder bag.
    #   VD bag  "sS1_run02_20260710_165402"        -> ts "20260710_165402"
    #      --out ".../path_run"                     -> ".../path_run_20260710_165402"
    # Nếu tên bag KHÔNG khớp mẫu -> giữ nguyên --out (không nối gì).
    bag_name = os.path.basename(os.path.normpath(args.bag))
    m = re.search(r"\d{8}_\d{6}", bag_name)
    if m:
        args.out = f"{args.out}_{m.group(0)}"
        print(f"[out] timestamp='{m.group(0)}' (từ bag '{bag_name}') "
              f"-> out='{args.out}'")
    else:
        print(f"[out] bag '{bag_name}' không chứa timestamp dạng "
              f"\\d{{8}}_\\d{{6}}; giữ nguyên --out='{args.out}'")

    seg_dir = os.path.join(args.out, "segments")
    os.makedirs(seg_dir, exist_ok=True)

    print("[1/4] Đọc map ...")
    img, res, origin = load_map(args.map)
    print(f"      map {img.shape[1]}x{img.shape[0]} px | res={res} m/px | "
          f"origin={origin}")

    print("[2/4] Đọc bag & trích path (map -> base_footprint) ...")
    traj, starts, ends = extract(args.bag, args.map_frame, args.odom_frame,
                                 args.base_frame, args.start_marker)
    if traj.shape[0] == 0:
        sys.exit("[ERR] Không dựng được pose nào. Kiểm tra: bag có /tf không, "
                 "AMCL đã localize chưa (có TF map->odom?), tên frame đúng chưa.")
    print(f"      {traj.shape[0]} pose | {len(starts)} mốc-bắt-đầu | "
          f"{len(ends)} mốc-kết-thúc")

    print("[3/4] Cắt chặng, phân tích jump & ghi CSV ...")
    np.savetxt(os.path.join(args.out, "full_path.csv"), traj,
               delimiter=",", header="t_sec,x,y,yaw", comments="")

    # Thống kê bước dịch chuyển để bạn kiểm ngưỡng jump có hợp lý không
    d = step_dist(traj)
    if d.size:
        pct = np.percentile(d, [50, 95, 99])
        n_jump = int((d > args.jump_thresh).sum())
        print(f"      Bước dịch chuyển (m): p50={pct[0]:.3f} p95={pct[1]:.3f} "
              f"p99={pct[2]:.3f} max={d.max():.2f}")
        print(f"      Số bước > jump_thresh({args.jump_thresh}) = {n_jump} "
              f"| quãng đường 'sạch' ≈ {clean_length(traj, args.jump_thresh):.1f} m")

    # Cắt pha định vị chưa ổn định (tùy chọn)
    settle_index = 0 if args.no_trim_settle else detect_settle_index(
        traj, args.settle_jump, args.settle_window)
    settle_time = float(traj[settle_index, 0]) if settle_index else float("-inf")
    if settle_index:
        tr_settled = traj[settle_index:]
        np.savetxt(os.path.join(args.out, "path_settled.csv"), tr_settled,
                   delimiter=",", header="t_sec,x,y,yaw", comments="")
        print(f"      Cắt {settle_index} pose đầu (định vị) -> "
              f"path_settled.csv bắt đầu t={tr_settled[0,0]:.1f}s")

    segs = build_segments(traj, starts, ends)
    for s in segs:
        fn = os.path.join(seg_dir, f"seg_{s['idx']:02d}_cp{s['cp_id']}.csv")
        np.savetxt(fn, s["pts"], delimiter=",", header="t_sec,x,y,yaw", comments="")
    print(f"      {len(segs)} chặng hợp lệ (>=2 pose)")

    if args.path_mode == "segment" and args.full_color != "blue":
        print("[WARN] --full-color chỉ áp dụng cho --path-mode full; "
              "bỏ qua ở chế độ segment.")

    print("[4/4] Vẽ ảnh ...")
    overview_png = os.path.join(args.out, "overview.png")
    if args.path_mode == "segment":
        if not segs:
            sys.exit("[ERR] Không tìm thấy mốc chặng (segs rỗng) -> dừng: không "
                     "tạo overview lẫn PNG chặng. (Kiểm tra --start-marker và topic "
                     "/robot/status_message trong bag — xem full_path.csv để dò.)")
        draw_overview_segments(img, res, origin, segs, overview_png,
                               jump_thresh=args.jump_thresh,
                               draw_mode=args.draw_mode, settle_time=settle_time)
        for s in segs:
            draw_segment(img, res, origin, s,
                         os.path.join(seg_dir, f"seg_{s['idx']:02d}_cp{s['cp_id']}.png"),
                         jump_thresh=args.jump_thresh, draw_mode=args.draw_mode,
                         settle_time=settle_time)
    else:  # full
        draw_overview_full(img, res, origin, traj, overview_png,
                           jump_thresh=args.jump_thresh, draw_mode=args.draw_mode,
                           full_color=args.full_color, settle_index=settle_index)

    n_seg_png = len(segs) if args.path_mode == "segment" else 0
    print(f"\n[OK] Output → {os.path.abspath(args.out)}")
    print(f"     overview.png | full_path.csv"
          + ("" if args.no_trim_settle or not settle_index else " | path_settled.csv")
          + f" | segments/ ({len(segs)} CSV"
          + (f", {n_seg_png} PNG)" if n_seg_png else ")"))


if __name__ == "__main__":
    main()
