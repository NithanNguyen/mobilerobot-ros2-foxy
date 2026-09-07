#!/usr/bin/env python3
"""
esp32_hoverboard.py
===================
Port trực tiếp từ ESP32 Arduino sang Python chạy trên Jetson.
Kết nối BT qua /dev/ttyUltrasonic, hoverboard qua /dev/ttyWheel.

Packet format (little-endian):
  [START_FRAME: uint16] [steer: int16] [speed: int16] [checksum: uint16]
  checksum = START_FRAME ^ steer ^ speed (16-bit XOR)
"""

import threading
import time
import struct
import serial
from dataclasses import dataclass, field

# ── Cấu hình cổng ─────────────────────────────────────────────────────────
BT_PORT    = "/dev/ttyUltrasonic"
BT_BAUD    = 115200

HOVER_PORT = "/dev/ttyWheel"
HOVER_BAUD = 115200

# --- 1. CẤU HÌNH TỐC ĐỘ ---
SPEED_MAX  = 150
STEER_MAX  = 80
SPEED_STEP = 20

# --- 2. CẤU HÌNH LÀM MƯỢT ---
ACCEL_RATE = 0.5
DECEL_RATE = 1.5
STEER_RATE = 1.0

# --- 3. CẤU HÌNH KHỞI ĐỘNG ---
MIN_START_POWER = 25

# --- UART ---
TIME_SEND        = 0.020          # 50 Hz
START_FRAME      = 0xABCD
CMD_FORMAT       = '<HhhH'        # start(u16), steer(s16), speed(s16), checksum(u16)

# --- WATCHDOG ---
WATCHDOG_TIMEOUT  = 0.5           # giây — dừng xe nếu không nhận lệnh
RECONNECT_TIMEOUT = 1.0           # giây — ngưỡng coi là mất kết nối BT


# ════════════════════════════════════════════════════════════════════════════
# STATE — gom toàn bộ biến điều khiển vào một dataclass
# ════════════════════════════════════════════════════════════════════════════
@dataclass
class RobotState:
    go: int            = 40        # mức tốc độ hiện tại (10–SPEED_MAX)
    target_speed: float = 0.0
    target_steer: float = 0.0
    current_speed: float = 0.0
    current_steer: float = 0.0
    last_cmd_time: float = field(default_factory=time.monotonic)


state = RobotState()
lock  = threading.Lock()

# ════════════════════════════════════════════════════════════════════════════
# GỬI LỆNH — send()
# ════════════════════════════════════════════════════════════════════════════
def send(hover_ser: serial.Serial, steer: int, speed: int):
    """
    Đóng gói và gửi lệnh điều khiển đến hoverboard.
    struct.pack('<HhhH') xử lý signed int16 trực tiếp — không cần chuyển đổi thủ công.
    """
    checksum = (START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF
    pkt = struct.pack(CMD_FORMAT, START_FRAME, steer, speed, checksum)
    hover_ser.write(pkt)
    print(f"[SEND] speed={speed:+4d}  steer={steer:+4d}  checksum=0x{checksum:04X}  pkt={pkt.hex(' ')}")


# ════════════════════════════════════════════════════════════════════════════
# XỬ LÝ LỆNH — process_command()
# ════════════════════════════════════════════════════════════════════════════
def process_command(cmd: str, bt_ser: serial.Serial):
    cmd = cmd.strip().upper()
    if not cmd:
        return

    with lock:
        state.last_cmd_time = time.monotonic()

        def do_1():
            state.go = min(state.go + 10, SPEED_MAX)
            bt_ser.write(f"{state.go}\n".encode())

        def do_2():
            state.go = max(state.go - 10, 10)
            bt_ser.write(f"{state.go}\n".encode())

        def do_F():
            state.target_speed = state.go
            bt_ser.write(f"Tien: {state.go}\n".encode())

        def do_B():
            state.target_speed = -state.go
            bt_ser.write(f"Lui: {state.go}\n".encode())

        def do_L():
            state.target_steer = -state.go
            bt_ser.write(b"Trai\n")

        def do_R():
            state.target_steer = state.go
            bt_ser.write(b"Phai\n")

        def do_S():
            state.target_speed = 0.0
            state.target_steer = 0.0
            bt_ser.write(b"Dung\n")

        dispatch = {
            '1': do_1, '2': do_2,
            'F': do_F, 'B': do_B,
            'L': do_L, 'R': do_R,
            'S': do_S,
        }

        handler = dispatch.get(cmd)
        if handler:
            handler()
            
        else:
            state.target_speed = 0.0
            state.target_steer = 0.0

        # Clamp lần cuối (chỉ speed vì steer đã clamp trong handler)
        state.target_speed = max(-SPEED_MAX, min(SPEED_MAX, state.target_speed))


# ════════════════════════════════════════════════════════════════════════════
# THUẬT TOÁN LÀM MƯỢT — smooth_control()
# GỌI BÊN TRONG lock — không gọi từ ngoài lock
# ════════════════════════════════════════════════════════════════════════════
def smooth_control():
    """
    Phải được gọi bên trong `with lock`.
    Cập nhật current_speed / current_steer tiến dần về target.
    """
    # ── Kick-start: chỉ áp dụng khi xe đang đứng yên (current == 0) ──
    if state.current_speed == 0.0:
        if state.target_speed > 0:
            state.current_speed = MIN_START_POWER
        elif state.target_speed < 0:
            state.current_speed = -MIN_START_POWER

    # ── Tăng/giảm tốc ──
    # Giảm tốc khi target gần 0 hơn current (về cường độ)
    is_decelerating = abs(state.target_speed) < abs(state.current_speed)
    rate = DECEL_RATE if is_decelerating else ACCEL_RATE

    if state.current_speed < state.target_speed:
        state.current_speed = min(state.current_speed + rate, state.target_speed)
    elif state.current_speed > state.target_speed:
        state.current_speed = max(state.current_speed - rate, state.target_speed)

    # ── Làm mượt quay ──
    if state.current_steer < state.target_steer:
        state.current_steer = min(state.current_steer + STEER_RATE, state.target_steer)
    elif state.current_steer > state.target_steer:
        state.current_steer = max(state.current_steer - STEER_RATE, state.target_steer)


# ════════════════════════════════════════════════════════════════════════════
# THREAD ĐỌC BT
# ════════════════════════════════════════════════════════════════════════════
def read_bt_loop(bt_ser: serial.Serial, stop_event: threading.Event):
    last_data_time   = time.monotonic()
    was_disconnected = False

    while not stop_event.is_set():
        try:
            data = bt_ser.read(bt_ser.in_waiting or 1)

            if data:
                now = time.monotonic()

                # Phát hiện reconnect — reset buffer, dừng xe
                if was_disconnected or (now - last_data_time) > RECONNECT_TIMEOUT:
                    print("[BT] Phat hien ket noi lai → Flush buffer + Dung xe")
                    bt_ser.reset_input_buffer()
                    with lock:
                        state.target_speed  = 0.0
                        state.target_steer  = 0.0
                        state.last_cmd_time = time.monotonic()
                    was_disconnected = False
                    last_data_time   = now
                    # KHÔNG continue — tiếp tục xử lý data vừa nhận
                    # (byte đầu tiên sau reconnect không bị bỏ mất)

                last_data_time = now
                for ch in data.decode(errors="ignore"):
                    process_command(ch, bt_ser)

            else:
                # Không có data — kiểm tra timeout
                if (time.monotonic() - last_data_time) > RECONNECT_TIMEOUT:
                    if not was_disconnected:
                        was_disconnected = True
                        with lock:
                            state.target_speed  = 0.0
                            state.target_steer  = 0.0
                            state.last_cmd_time = time.monotonic()
                        print("[BT] Mat ket noi → Dung xe")

        except serial.SerialException as e:
            print(f"[BT] Loi: {e}")
            was_disconnected = True
            with lock:
                state.target_speed  = 0.0
                state.target_steer  = 0.0
                state.last_cmd_time = time.monotonic()
            time.sleep(1.0)


# ════════════════════════════════════════════════════════════════════════════
# THREAD GỬI LỆNH 50Hz
# ════════════════════════════════════════════════════════════════════════════
def send_loop(hover_ser: serial.Serial, stop_event: threading.Event):
    """
    Chạy ở 50 Hz với deadline tuyệt đối để tránh timer drift.
    Toàn bộ đọc/ghi state được bảo vệ bởi lock.
    """
    next_tick = time.monotonic()

    while not stop_event.is_set():
        next_tick += TIME_SEND

        with lock:
            # Watchdog: dừng xe nếu lâu không nhận lệnh
            if time.monotonic() - state.last_cmd_time > WATCHDOG_TIMEOUT:
                state.target_speed = 0.0
                state.target_steer = 0.0

            smooth_control()
            spd      = int(state.current_speed)
            steer_val = int(state.current_steer)

        try:
            send(hover_ser, steer_val, spd)
        except serial.SerialException as e:
            print(f"[HOVER] Loi gui: {e} — Thu mo lai cong...")
            hover_ser = _reopen_serial(HOVER_PORT, HOVER_BAUD)

        sleep_t = next_tick - time.monotonic()
        if sleep_t > 0:
            time.sleep(sleep_t)


# ════════════════════════════════════════════════════════════════════════════
# HELPER — mở lại cổng serial sau lỗi
# ════════════════════════════════════════════════════════════════════════════
def _reopen_serial(port: str, baud: int, retries: int = 10) -> serial.Serial:
    for i in range(retries):
        try:
            time.sleep(1.0)
            ser = serial.Serial(port, baud, timeout=0.1)
            print(f"[SERIAL] Mo lai {port} thanh cong (lan {i+1})")
            return ser
        except serial.SerialException as e:
            print(f"[SERIAL] Thu {i+1}/{retries} that bai: {e}")
    raise RuntimeError(f"Khong the mo lai {port} sau {retries} lan thu")


# ════════════════════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════════════════════
def main():
    print(f"[HOVER] Mo cong {HOVER_PORT} @ {HOVER_BAUD} ...")
    hover_ser = serial.Serial(HOVER_PORT, HOVER_BAUD, timeout=0.1)
    print("[HOVER] OK")

    print(f"[BT] Mo cong {BT_PORT} @ {BT_BAUD} ...")
    bt_ser = serial.Serial(BT_PORT, BT_BAUD, timeout=0.1)
    print("[BT] OK — San sang nhan lenh!")
    bt_ser.write(b"Ready! F=Tien B=Lui L=Trai R=Phai S=Dung 1=Tang 2=Giam\n")

    stop_event = threading.Event()

    t_read = threading.Thread(target=read_bt_loop,
                              args=(bt_ser, stop_event), daemon=True)
    t_send = threading.Thread(target=send_loop,
                              args=(hover_ser, stop_event), daemon=True)
    t_read.start()
    t_send.start()

    print("[MAIN] Dang chay. Nhan Ctrl+C de dung.")
    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[MAIN] Dung...")
    finally:
        stop_event.set()
        try:
            send(hover_ser, 0, 0)
        except Exception:
            pass
        hover_ser.close()
        bt_ser.close()
        print("[MAIN] Da dong cong.")


if __name__ == '__main__':
    main()