#!/usr/bin/env bash
# =============================================================================
# pre_check.sh — Mobile Robot Pre-Flight System Check
# Platform : Jetson AGX Xavier, Ubuntu 20.04, ROS 2 Foxy
# Middleware: FastRTPS  (RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
# Usage     : ./pre_check.sh [--floor FLOOR] [--skip-build] [--check-topics]
#                            [--skip-serial-probe] [--help]
# -----------------------------------------------------------------------------
# CHANGELOG v2 (2026-08-07)
#   PHASE 2 được vá lại sau sự cố exp_sS1_run01_20260807_144048:
#   phiên bản cũ báo 33/33 PASS trong khi cả /dev/ttyWheel lẫn /dev/ttyUltrasonic
#   đều không mở được (Errno 5 / EIO). Nguyên nhân: _check_device() chỉ dùng
#   `[[ -e ]]` + `[[ -r/-w ]]`, tức chỉ stat inode và đọc bit permission — nó
#   KHÔNG BAO GIỜ open() cổng. Một thiết bị CP2102 đã enumerate nhưng không trả
#   lời control transfer vẫn có device node + symlink + quyền rw hợp lệ.
#
#   Bổ sung:
#     2.x  _probe_serial()   — thực sự open() cổng, đo thời gian, phân loại
#                              errno (EIO/EBUSY/EACCES/ENOENT) thành thông điệp
#                              chẩn đoán cụ thể. ~5.0s + EIO ⇒ control transfer
#                              timeout ở tầng USB.
#     2.x  _usb_report()     — dump topology USB (by-path, lsusb -t, bMaxPower,
#                              removable) vào log để truy vết cổng vật lý.
#     2.x  _usb_error_scan() — quét dmesg tìm cp210x/-110/over-current/
#                              disabled by hub, và đếm số lần re-enumerate.
#     2.x  _check_port_holder() — phát hiện tiến trình đang giữ cổng (BT.py,
#                              ModemManager, brltty) trước khi kết luận.
#
#   Cờ mới: --skip-serial-probe  (bỏ qua open() test, ~10s nhanh hơn)
# =============================================================================

# ── Script-level safety: NOT set globally; applied per function ──────────────
WS="$HOME/mbrobot_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WS/log_check"
LOG_FILE="$LOG_DIR/pre_check_${STAMP}.log"

# ── Counters ─────────────────────────────────────────────────────────────────
COUNT_PASS=0; COUNT_WARN=0; COUNT_FAIL=0
COUNT_AUTOFIX=0; COUNT_AUTOKILL=0

# ── Result storage (parallel arrays: tag | description) ─────────────────────
declare -a RESULT_TAGS=()
declare -a RESULT_MSGS=()

# ── Background launch PID for Phase 3 ────────────────────────────────────────
LAUNCH_PID=""
LAUNCH_PGID=""

# ── ANSI colors (no external dependencies) ───────────────────────────────────
C_GREEN="\033[0;32m";  C_YELLOW="\033[0;33m"; C_RED="\033[0;31m"
C_CYAN="\033[0;36m";   C_MAGENTA="\033[0;35m"; C_RESET="\033[0m"
C_BOLD="\033[1m"

# ── Floor (default e6, overridable via CLI) ───────────────────────────────────
FLOOR="e6"
SKIP_BUILD=0
CHECK_TOPICS=0
SKIP_SERIAL_PROBE=0

# ── Serial probe tuning ──────────────────────────────────────────────────────
# 5000 ms là USB_CTRL_SET_TIMEOUT mặc định của kernel Linux. Một open() mất
# xấp xỉ 5.0s rồi trả EIO là chữ ký của control-transfer timeout, KHÔNG phải
# lỗi phần mềm. Ngưỡng dưới đây dùng để nhận diện chữ ký đó.
SERIAL_PROBE_BAUD=115200
SERIAL_PROBE_TIMEOUT=12      # timeout cứng cho toàn bộ lần probe (giây)
SERIAL_SLOW_OPEN_SEC=3.0     # open() lâu hơn ngưỡng này ⇒ nghi USB timeout

# =============================================================================
# PARSE CLI
# =============================================================================
parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                echo "Usage: $0 [--floor FLOOR] [--skip-build] [--check-topics]"
                echo "          [--skip-serial-probe] [--help]"
                echo ""
                echo "  --floor FLOOR         Floor identifier (default: e6)"
                echo "  --skip-build          Skip PHASE 0 (colcon build + source)"
                echo "  --check-topics        Enable PHASE 3 launch + topic/TF checks"
                echo "                        (zombie kill always runs regardless)"
                echo "  --skip-serial-probe   Skip the real open() test on serial ports"
                echo "                        (faster, but CANNOT detect EIO/USB faults)"
                echo "  --help, -h            Show this message and exit"
                echo ""
                echo "Examples:"
                echo "  ./pre_check.sh                         # full check, floor e6"
                echo "  ./pre_check.sh --floor e1              # full check, floor e1"
                echo "  ./pre_check.sh --floor e1 --check-topics"
                echo "  ./pre_check.sh --floor e1 --skip-build"
                exit 0
                ;;
            --floor=*)      FLOOR="${1#--floor=}" ;;
            --floor)        FLOOR="$2"; shift ;;
            --skip-build)         SKIP_BUILD=1        ;;
            --check-topics)       CHECK_TOPICS=1      ;;
            --skip-serial-probe)  SKIP_SERIAL_PROBE=1 ;;
            -*)
                echo "Unknown flag: $1  (use --help for usage)"
                exit 1
                ;;
            *)
                echo "Unknown argument: $1  (use --help for usage)"
                exit 1
                ;;
        esac
        shift
    done
}

# =============================================================================
# LOGGING INFRASTRUCTURE
# =============================================================================
# Log directory must exist before we can tee; bootstrapped in main()
_ts()  { date +"%H:%M:%S"; }
_date(){ date +"%Y-%m-%d %H:%M:%S"; }

# Write to terminal (with ANSI) AND log file (plain text) simultaneously.
# Usage: _log COLOR "[TAG]" "message" [raw_details]
_log() {
    local color="$1" tag="$2" msg="$3" details="${4:-}"
    local plain_line="[$(  _ts)] ${tag} ${msg}"
    # Terminal: colored
    printf "${color}${tag}${C_RESET} %s\n" "$msg"
    # Log file: plain (strip ANSI just in case)
    printf "%s\n" "$plain_line" >> "$LOG_FILE"
    if [[ -n "$details" ]]; then
        printf "       %s\n" "$details" >> "$LOG_FILE"
    fi
}

log_pass()     { _log "$C_GREEN"   "[PASS]    " "$1"; }
log_warn()     { _log "$C_YELLOW"  "[WARN]    " "$1" "${2:-}"; }
log_fail()     { _log "$C_RED"     "[FAIL]    " "$1" "${2:-}"; }
log_info()     { _log "$C_CYAN"    "[INFO]    " "$1"; }
log_autofix()  { _log "$C_MAGENTA" "[AUTO-FIX]" "$1"; }
log_autokill() { _log "$C_MAGENTA" "[AUTO-KILL]" "$1"; }

# Store result in arrays for final report
store() {
    local tag="$1" msg="$2"
    RESULT_TAGS+=("$tag")
    RESULT_MSGS+=("$msg")
    case "$tag" in
        PASS)     ((COUNT_PASS++))     ;;
        WARN)     ((COUNT_WARN++))     ;;
        FAIL)     ((COUNT_FAIL++))     ;;
        AUTOFIX)  ((COUNT_AUTOFIX++))  ;;
        AUTOKILL) ((COUNT_AUTOKILL++)) ;;
    esac
}

# =============================================================================
# CLEANUP TRAP — always kill Phase 3 background launch on exit
# =============================================================================
cleanup_launch() {
    if [[ -n "$LAUNCH_PGID" ]]; then
        log_info "Cleaning up background launch (PGID=$LAUNCH_PGID)..."
        kill -- "-$LAUNCH_PGID" 2>/dev/null
        sleep 2
        # Force kill stragglers
        kill -9 -- "-$LAUNCH_PGID" 2>/dev/null || true
        printf "[$(  _ts)] [INFO]     Background launch cleaned up (PGID=$LAUNCH_PGID)\n" >> "$LOG_FILE"
    fi
}
trap cleanup_launch EXIT INT TERM

# =============================================================================
# PHASE 0 — Build & Source Workspace
# =============================================================================
phase0_build() {
    local phase_label="PHASE 0 — Build & Source"
    printf "\n${C_BOLD}%s${C_RESET}\n" "$phase_label"
    printf "%s\n" "$(printf '─%.0s' {1..60})"
    printf "%s\n" "$phase_label" >> "$LOG_FILE"

    # 0.1 colcon build
    # log_info "Running colcon build --symlink-install in $WS ..."
    log_info "Running colcon build for 'mobile_robot' in $WS ..."
    local build_out
    # build_out=$(cds"$WS" && colcon build --symlink-install 2>&1)
    build_out=$(cd "$WS" && colcon build --symlink-install --packages-select mobile_robot 2>&1)
    local rc=$?
    if [[ $rc -ne 0 ]]; then
        log_fail "colcon build FAILED — aborting pre-check"
        printf "%s\n" "$build_out" >> "$LOG_FILE"
        store FAIL "colcon build failed"
        exit 1
    fi
    log_pass "colcon build succeeded"
    store PASS "colcon build succeeded"

    # 0.2 Source workspace
    if ! source "$WS/install/setup.bash" 2>/dev/null; then
        log_fail "Failed to source $WS/install/setup.bash"
        store FAIL "source install/setup.bash failed"
        exit 1
    fi
    log_pass "Workspace sourced ($WS/install/setup.bash)"
    store PASS "Workspace sourced"
}

# =============================================================================
# PHASE 1 — System Prerequisites
# =============================================================================
phase1_system() {
    local phase_label="PHASE 1 — System Prerequisites"
    printf "\n${C_BOLD}%s${C_RESET}\n" "$phase_label"
    printf "%s\n" "$(printf '─%.0s' {1..60})"
    printf "%s\n" "$phase_label" >> "$LOG_FILE"

    # 1.1 ROS 2 Foxy sourced
    if [[ -f /opt/ros/foxy/setup.bash ]]; then
        # shellcheck disable=SC1091
        source /opt/ros/foxy/setup.bash 2>/dev/null
        log_pass "ROS 2 Foxy found and sourced (/opt/ros/foxy/setup.bash)"
        store PASS "ROS 2 Foxy sourced"
    else
        log_fail "/opt/ros/foxy/setup.bash not found — ROS 2 Foxy not installed"
        store FAIL "ROS 2 Foxy not found"
    fi

    # 1.2 RMW_IMPLEMENTATION
    if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_fastrtps_cpp" ]]; then
        log_pass "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
        store PASS "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    else
        log_warn "RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-<unset>}' (expected: rmw_fastrtps_cpp)" \
                 "Run: export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
        store WARN "RMW_IMPLEMENTATION not set correctly"
    fi

    # 1.4 Required ROS packages
    local ros_pkgs=("sllidar_ros2" "bno055" "robot_localization" "laser_filters" "slam_toolbox" "nav2_bringup")
    local all_ros_ok=1
    for pkg in "${ros_pkgs[@]}"; do
        if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
            log_pass "ROS package installed: $pkg"
            store PASS "ROS package: $pkg"
        else
            log_fail "ROS package NOT found: $pkg"
            store FAIL "Missing ROS package: $pkg"
            all_ros_ok=0
        fi
    done

    # 1.5 Python packages
    local py_pkgs=("serial" "yaml" "numpy")
    local py_names=("pyserial (serial)" "pyyaml (yaml)" "numpy")
    for i in "${!py_pkgs[@]}"; do
        if python3 -c "import ${py_pkgs[$i]}" 2>/dev/null; then
            log_pass "Python package available: ${py_names[$i]}"
            store PASS "Python: ${py_names[$i]}"
        else
            log_warn "Python package NOT found: ${py_names[$i]}"
            store WARN "Missing Python package: ${py_names[$i]}"
        fi
    done
}

# =============================================================================
# PHASE 2 — Hardware & Device Check
# =============================================================================
phase2_hardware() {
    local phase_label="PHASE 2 — Hardware & Devices"
    printf "\n${C_BOLD}%s${C_RESET}\n" "$phase_label"
    printf "%s\n" "$(printf '─%.0s' {1..60})"
    printf "%s\n" "$phase_label" >> "$LOG_FILE"

    # ── Helper: check device exists + permissions ─────────────────────────────
    # LƯU Ý: hàm này CHỈ kiểm tra inode + permission. Nó không chứng minh cổng
    # dùng được. Phải luôn đi kèm _probe_serial() bên dưới.
    _check_device() {
        local dev="$1" desc="$2"
        if [[ ! -e "$dev" ]]; then
            log_fail "$desc not found at $dev"
            store FAIL "$desc not found: $dev"
            return 1
        fi

        # Phân giải symlink → thiết bị thật, ghi vào log để truy vết cổng vật lý
        local real_dev by_path
        real_dev=$(readlink -f "$dev" 2>/dev/null || echo "$dev")
        by_path=$(find /dev/serial/by-path -lname "*${real_dev##*/}" -printf '%f\n' 2>/dev/null | head -1)
        if [[ -n "$by_path" ]]; then
            log_pass "$dev found → $real_dev  [$by_path]"
            store PASS "$dev found ($( basename "$real_dev"))"
        else
            log_pass "$dev found → $real_dev"
            store PASS "$dev found ($( basename "$real_dev"))"
        fi
        printf "[$(  _ts)] [INFO]     %s -> %s (by-path: %s)\n" \
               "$dev" "$real_dev" "${by_path:-n/a}" >> "$LOG_FILE"

        # Phải là character device, không phải file thường / symlink treo
        if [[ ! -c "$real_dev" ]]; then
            log_fail "$dev resolves to '$real_dev' which is NOT a character device"
            store FAIL "$dev: not a char device"
            return 1
        fi

        if [[ ! -r "$dev" || ! -w "$dev" ]]; then
            log_autofix "Applying chmod a+rw $dev"
            chmod a+rw "$dev" 2>/dev/null
            store AUTOFIX "chmod a+rw $dev"
            printf "[$(  _ts)] [AUTO-FIX] chmod a+rw %s\n" "$dev" >> "$LOG_FILE"
        fi
        # Verify after potential fix
        if [[ -r "$dev" && -w "$dev" ]]; then
            log_pass "$dev has read/write permission"
            store PASS "$dev permissions OK"
        else
            log_fail "$dev still not accessible after chmod"
            store FAIL "$dev permission fix failed"
        fi
        return 0
    }

    # ── Helper: phát hiện tiến trình đang giữ cổng ────────────────────────────
    # BT.py (About_MobileRobot.md §"BT.py") chiếm CẢ ttyWheel lẫn ttyUltrasonic.
    # ModemManager/brltty là hai thủ phạm kinh điển cướp cổng USB-serial.
    _check_port_holder() {
        local dev="$1"
        local holders=""
        if command -v fuser >/dev/null 2>&1; then
            holders=$(fuser "$dev" 2>/dev/null | tr -d ' ')
        fi
        if [[ -n "$holders" ]]; then
            local names
            names=$(ps -o comm=,args= -p $holders 2>/dev/null | head -5)
            log_fail "$dev is HELD by another process (PID: $holders)"
            printf "  Holder(s):\n%s\n" "$names" >> "$LOG_FILE"
            store FAIL "$dev held by PID $holders"
            return 1
        fi
        return 0
    }

    # ── Helper: PROBE THẬT — mở cổng bằng pyserial, đo thời gian, phân loại lỗi ──
    # Trả về: 0 = OK, 1 = WARN, 2 = FAIL
    _probe_serial() {
        local dev="$1" desc="$2"

        if (( SKIP_SERIAL_PROBE == 1 )); then
            log_warn "$dev open-probe SKIPPED (--skip-serial-probe) — existence check alone proves nothing"
            store WARN "$dev open-probe skipped"
            return 1
        fi

        # Không probe nếu đang có tiến trình giữ cổng (sẽ gây dương tính giả)
        _check_port_holder "$dev" || return 2

        log_info "Probing $dev — attempting real open() @ ${SERIAL_PROBE_BAUD} baud ..."

        local probe_out rc
        probe_out=$(timeout "$SERIAL_PROBE_TIMEOUT" python3 - "$dev" "$SERIAL_PROBE_BAUD" <<'PYEOF' 2>&1
import sys, time
port, baud = sys.argv[1], int(sys.argv[2])
t0 = time.time()
try:
    import serial
except ImportError:
    print("RESULT=NOPYSERIAL ELAPSED=0.00 ERRNO=0 MSG=pyserial_not_installed")
    sys.exit(3)
try:
    s = serial.Serial(port, baud, timeout=0.2)
    el = time.time() - t0
    s.reset_input_buffer()
    s.close()
    print("RESULT=OK ELAPSED=%.2f ERRNO=0 MSG=opened_and_closed_cleanly" % el)
    sys.exit(0)
except Exception as e:
    el = time.time() - t0
    eno = getattr(e, "errno", None)
    try:
        eno = int(eno)
    except (TypeError, ValueError):
        eno = -1
    msg = str(e).replace(" ", "_").replace("\n", " ")
    print("RESULT=FAIL ELAPSED=%.2f ERRNO=%d MSG=%s" % (el, eno, msg))
    sys.exit(1)
PYEOF
        )
        rc=$?

        if (( rc == 124 )); then
            log_fail "$dev — open() HUNG for >${SERIAL_PROBE_TIMEOUT}s (hard timeout, driver wedged)"
            store FAIL "$dev: open() hung >${SERIAL_PROBE_TIMEOUT}s"
            _usb_error_scan "$dev"
            return 2
        fi

        local result elapsed errno_val
        result=$(sed -n 's/.*RESULT=\([A-Z]*\).*/\1/p'   <<<"$probe_out" | head -1)
        elapsed=$(sed -n 's/.*ELAPSED=\([0-9.]*\).*/\1/p' <<<"$probe_out" | head -1)
        errno_val=$(sed -n 's/.*ERRNO=\(-\?[0-9]*\).*/\1/p' <<<"$probe_out" | head -1)
        elapsed="${elapsed:-0.00}"; errno_val="${errno_val:--1}"

        printf "  Probe raw: %s\n" "$probe_out" >> "$LOG_FILE"

        case "$result" in
            OK)
                log_pass "$(printf '%-22s open() OK in %5.2fs' "$dev" "$elapsed")"
                store PASS "$dev open OK (${elapsed}s)"
                return 0
                ;;
            NOPYSERIAL)
                log_warn "$dev — cannot probe: pyserial not installed"
                store WARN "$dev probe skipped (no pyserial)"
                return 1
                ;;
        esac

        # ── Phân loại errno thành chẩn đoán cụ thể ────────────────────────────
        local slow
        slow=$(awk "BEGIN{print ($elapsed >= $SERIAL_SLOW_OPEN_SEC) ? 1 : 0}")
        local diag=""
        case "$errno_val" in
            5)  # EIO
                if [[ "$slow" == "1" ]]; then
                    diag="EIO after ${elapsed}s => USB control-transfer TIMEOUT. Device enumerated but not answering EP0. Root cause is PHYSICAL (VBUS droop / EMI / hub / cable / adapter), NOT software. Check: dmesg | grep -i 'cp210x.*status: -110'"
                else
                    diag="EIO (I/O error) — driver rejected open. Check dmesg for the underlying USB/driver error."
                fi
                ;;
            2)  diag="ENOENT — symlink resolves to a device node that no longer exists (device unplugged / re-enumerated on a different path). Check udev rules + /dev/serial/by-path/." ;;
            13) diag="EACCES — permission denied. Add user to 'dialout' group, or set MODE=\"0666\" in the udev rule." ;;
            16) diag="EBUSY — port already opened exclusively. Kill BT.py / ModemManager / brltty first." ;;
            11) diag="EAGAIN — port locked by another process (flock)." ;;
            *)  diag="Unclassified errno=${errno_val}. See raw probe output in log." ;;
        esac

        log_fail "$(printf '%-22s open() FAILED in %5.2fs (errno=%s)' "$dev" "$elapsed" "$errno_val")"
        log_fail "  → $desc: $diag"
        printf "  Diagnosis: %s\n" "$diag" >> "$LOG_FILE"
        store FAIL "$dev open FAILED (errno=$errno_val, ${elapsed}s)"

        _usb_error_scan "$dev"
        return 2
    }

    # ── Helper: quét dmesg tìm bằng chứng lỗi tầng USB ────────────────────────
    _usb_error_scan() {
        local dev="${1:-}"
        log_info "Scanning kernel log for USB-layer faults ..."

        local dm
        dm=$(dmesg -T 2>/dev/null || sudo -n dmesg -T 2>/dev/null || echo "")
        if [[ -z "$dm" ]]; then
            log_warn "Cannot read dmesg (need root) — run: sudo dmesg -T | grep -iE 'cp210x|-110|over-current|disabled by hub'"
            store WARN "dmesg not readable for USB scan"
            return
        fi

        local recent
        recent=$(echo "$dm" | grep -iE "cp210x|ch34|ftdi|cdc_acm|status: -110|status: -71|over-?current|disabled by hub|USB disconnect|not accepting address|insufficient|rejected .* configuration" | tail -40)

        if [[ -n "$recent" ]]; then
            printf "  ── dmesg USB excerpt ──\n%s\n" "$recent" >> "$LOG_FILE"

            # Các chữ ký quyết định
            if echo "$recent" | grep -q "status: -110"; then
                log_fail "dmesg: control transfer TIMEOUT (-110 ETIMEDOUT) — USB device not answering. PHYSICAL fault."
                store FAIL "dmesg: USB control timeout (-110)"
            fi
            if echo "$recent" | grep -qi "Unable to enable UART"; then
                log_fail "dmesg: 'cp210x_open - Unable to enable UART' — CP2102 rejected IFC_ENABLE (req 0x00)"
                store FAIL "dmesg: cp210x IFC_ENABLE failed"
            fi
            if echo "$recent" | grep -qiE "over-?current"; then
                log_fail "dmesg: USB OVER-CURRENT detected — VBUS budget exceeded on that port/hub"
                store FAIL "dmesg: USB over-current"
            fi
            if echo "$recent" | grep -qi "disabled by hub"; then
                log_fail "dmesg: 'disabled by hub (EMI?)' — signal-integrity / power fault on the hub branch"
                store FAIL "dmesg: port disabled by hub (EMI)"
            fi
            if echo "$recent" | grep -qiE "insufficient|rejected .* configuration"; then
                log_fail "dmesg: configuration rejected due to insufficient bus power"
                store FAIL "dmesg: insufficient USB bus power"
            fi
        else
            log_info "No USB-layer errors found in kernel log"
        fi

        # Đếm số lần re-enumerate: device number cao = USB tree không ổn định
        local maxdev
        maxdev=$(echo "$dm" | grep -oP 'new (full|high|low)-speed USB device number \K[0-9]+' | sort -n | tail -1)
        if [[ -n "$maxdev" ]] && (( maxdev > 12 )); then
            log_warn "USB enumeration churn: highest device number = $maxdev (repeated disconnect/reconnect)"
            store WARN "USB churn: device number reached $maxdev"
        fi
    }

    # ── Helper: dump topology USB vào log (không tính PASS/FAIL) ──────────────
    _usb_report() {
        {
            printf "\n──────── USB TOPOLOGY SNAPSHOT ────────\n"
            printf "\n[ls -l /dev/tty{Wheel,Ultrasonic}]\n"
            ls -l /dev/ttyWheel /dev/ttyUltrasonic 2>&1
            printf "\n[/dev/serial/by-path/]\n"
            ls -l /dev/serial/by-path/ 2>&1
            printf "\n[/dev/serial/by-id/]\n"
            ls -l /dev/serial/by-id/ 2>&1
            printf "\n[lsusb]\n";    lsusb    2>&1
            printf "\n[lsusb -t]\n"; lsusb -t 2>&1
            printf "\n[USB power attributes]\n"
            for d in /sys/bus/usb/devices/*/; do
                local n; n=$(basename "$d")
                [[ "$n" == usb* || "$n" == *:* ]] && continue
                printf "  %-10s bMaxPower=%-8s removable=%-10s product=%s\n" \
                    "$n" \
                    "$(cat "$d/bMaxPower" 2>/dev/null || echo '?')" \
                    "$(cat "$d/removable" 2>/dev/null || echo '?')" \
                    "$(cat "$d/product"   2>/dev/null || echo '?')"
            done
            printf "\n[udev rules in /etc/udev/rules.d/]\n"
            grep -Hn "ttyWheel\|ttyUltrasonic" /etc/udev/rules.d/*.rules 2>&1
            printf "───────────────────────────────────────\n\n"
        } >> "$LOG_FILE" 2>&1
        log_info "USB topology snapshot written to log"
    }

    # 2.0 Snapshot topology TRƯỚC khi kiểm tra — để đối chiếu khi debug
    _usb_report

    # 2.1–2.3 /dev/ttyWheel  (tồn tại → quyền → MỞ THẬT)
    if _check_device "/dev/ttyWheel" "STM32 wheel controller (USB-TTL)"; then
        _probe_serial "/dev/ttyWheel" "STM32 wheel controller" ||
            log_warn "wheel_odom_node WILL crash on startup — do not launch nav_v3 yet"
    else
        log_warn "Check USB-TTL cable for STM32 wheel controller"
    fi

    # 2.4–2.6 /dev/ttyUltrasonic
    if _check_device "/dev/ttyUltrasonic" "Ultrasonic MCU (USB-TTL)"; then
        _probe_serial "/dev/ttyUltrasonic" "Ultrasonic ESP32 bridge" ||
            log_warn "jetson_sensor_bridge WILL crash-loop (respawn=True) — do not launch nav_v3 yet"
    else
        log_warn "Check USB-TTL cable for ultrasonic MCU"
    fi

    # 2.7 Cảnh báo trùng serial number giữa các adapter USB-serial
    # CP2102 xuất xưởng với SerialNumber="0001". Hai adapter cùng "0001" khiến
    # /dev/serial/by-id/ chỉ tạo được MỘT symlink → không thể phân biệt bằng
    # ATTRS{serial} trong udev; bắt buộc phải dùng ATTRS{devpath} (port path).
    local n_serial_dev n_byid
    n_serial_dev=$(ls /sys/bus/usb-serial/devices/ 2>/dev/null | wc -l)
    n_byid=$(ls /dev/serial/by-id/ 2>/dev/null | wc -l)
    if (( n_serial_dev > 0 && n_byid < n_serial_dev )); then
        log_warn "USB-serial serial-number COLLISION: ${n_serial_dev} tty device(s) but only ${n_byid} by-id link(s)"
        log_warn "  → adapters share the same SerialNumber. udev must use ATTRS{devpath}, not ATTRS{serial}."
        store WARN "USB-serial by-id collision (${n_byid}/${n_serial_dev})"
    fi

    # 2.5–2.6 /dev/i2c-8
    if [[ ! -e "/dev/i2c-8" ]]; then
        log_fail "I2C bus 8 not found — check hardware or kernel module"
        store FAIL "/dev/i2c-8 not found"
    else
        log_pass "/dev/i2c-8 found"
        store PASS "/dev/i2c-8 found"

        if [[ ! -r "/dev/i2c-8" || ! -w "/dev/i2c-8" ]]; then
            log_autofix "Applying chmod a+rw /dev/i2c-8"
            chmod a+rw /dev/i2c-8 2>/dev/null
            store AUTOFIX "chmod a+rw /dev/i2c-8"
            printf "[$(  _ts)] [AUTO-FIX] chmod a+rw /dev/i2c-8\n" >> "$LOG_FILE"
        fi

        if [[ -r "/dev/i2c-8" && -w "/dev/i2c-8" ]]; then
            log_pass "/dev/i2c-8 has read/write permission"
            store PASS "/dev/i2c-8 permissions OK"
        else
            log_fail "/dev/i2c-8 still not accessible after chmod"
            store FAIL "/dev/i2c-8 permission fix failed"
        fi

        # 2.7 BNO055 I2C detect 
        log_info "Running i2cdetect on bus 8 (addr 0x28)..."
        local i2c_out
        i2c_out=$(timeout 5 i2cdetect -y 8 2>&1)
        
        # Trích xuất dòng bắt đầu bằng '20:' để thu hẹp phạm vi tìm kiếm
        local row_20
        row_20=$(echo "$i2c_out" | grep "^20:")
        
        # Kiểm tra xem có chứa '28' (trạng thái rảnh) hoặc 'UU' (đang được node/driver chiếm giữ)
        if echo "$row_20" | grep -qiE '\b(28|UU)\b'; then
            log_pass "BNO055 detected at I2C bus 8, addr 0x28 (Status: Ready/Busy)"
            store PASS "BNO055 I2C detected"
        else
            # [FALLBACK CHECK]: Đôi khi i2cdetect không hoạt động tốt trên Jetson. 
            # Thử đọc trực tiếp register CHIP_ID (0x00) của BNO055. Mã định danh luôn phải là 0xa0.
            log_info "i2cdetect failed/timeout. Attempting direct register read (CHIP_ID 0x00)..."
            local chip_id
            chip_id=$(timeout 2 i2cget -y 8 0x28 0x00 2>/dev/null || echo "FAIL")
            
            if [[ "$chip_id" == "0xa0" ]]; then
                log_pass "BNO055 detected via I2C Chip ID verification (0xA0)"
                store PASS "BNO055 I2C detected (Fallback Check)"
            else
                log_fail "BNO055 NOT detected on I2C bus 8 addr 0x28 — check wiring"
                printf "%s\n" "$i2c_out" >> "$LOG_FILE"
                store FAIL "BNO055 not detected on I2C"
            fi
        fi
    fi

    # 2.8 RPLIDAR ping (with auto-fix on failure)
    log_info "Pinging RPLIDAR S2E at 192.168.11.2 ..."
    if timeout 10 ping -c 3 -W 2 192.168.11.2 > /dev/null 2>&1; then
        log_pass "RPLIDAR S2E reachable at 192.168.11.2"
        store PASS "RPLIDAR S2E pingable"
    else
        log_warn "Cannot reach 192.168.11.2 — attempting auto-fix (ip addr/link on enp2s0) ..."
        log_autofix "sudo ip addr add 192.168.11.1/24 dev enp2s0"
        sudo ip addr add 192.168.11.1/24 dev enp2s0 2>/dev/null || true
        log_autofix "sudo ip link set enp2s0 up"
        sudo ip link set enp2s0 up 2>/dev/null || true
        store AUTOFIX "ip addr add 192.168.11.1/24 dev enp2s0 + ip link set enp2s0 up"
        printf "[$(  _ts)] [AUTO-FIX] ip addr add 192.168.11.1/24 dev enp2s0 && ip link set enp2s0 up\n" >> "$LOG_FILE"

        log_info "Retrying ping to 192.168.11.2 ..."
        sleep 2
        if timeout 10 ping -c 3 -W 2 192.168.11.2 > /dev/null 2>&1; then
            log_pass "RPLIDAR S2E reachable after auto-fix"
            store PASS "RPLIDAR S2E pingable (after auto-fix)"
        else
            log_fail "Cannot reach 192.168.11.2 even after auto-fix — Check Ethernet cable or LiDAR power"
            store FAIL "RPLIDAR S2E unreachable"
        fi
    fi
}

# =============================================================================
# PHASE 3 — ROS Node & Topic Health
# =============================================================================
phase3_ros_topics() {
    local phase_label="PHASE 3 — ROS Topics & TF"
    printf "\n${C_BOLD}%s${C_RESET}\n" "$phase_label"
    printf "%s\n" "$(printf '─%.0s' {1..60})"
    printf "%s\n" "$phase_label" >> "$LOG_FILE"

    # 3.1 Kill hung/zombie node instances (always runs)
    local zombie_nodes=("wheel_odom_node" "bno055" "sllidar_node" "ultrasonic_fusion_node" "ekf_filter_node")
    for node_name in "${zombie_nodes[@]}"; do
        local pids
        pids=$(pgrep -f "$node_name" 2>/dev/null || true)
        for pid in $pids; do
            log_autokill "Killing stale node: $node_name (PID $pid)"
            kill "$pid" 2>/dev/null || true
            sleep 2
            if kill -0 "$pid" 2>/dev/null; then
                kill -9 "$pid" 2>/dev/null || true
            fi
            store AUTOKILL "Killed stale node: $node_name (PID $pid)"
            printf "[$(  _ts)] [AUTO-KILL] kill %s (%s)\n" "$node_name" "$pid" >> "$LOG_FILE"
        done
    done

    # 3.2–3.6 Launch + topic/TF checks (only when --check-topics is given)
    if (( CHECK_TOPICS == 0 )); then
        log_info "PHASE 3 topic/TF check skipped (use --check-topics to enable)"
        printf "[$(  _ts)] [INFO]     PHASE 3 topic/TF check skipped (--check-topics not set)\n" >> "$LOG_FILE"
        return
    fi

    # 3.2 Launch nav_v3.launch.py in background (new process group for clean kill)
    log_info "Launching nav_v3.launch.py in background (floor=$FLOOR) ..."
    set -m  # enable job control / process groups temporarily
    (
        set -euo pipefail
        source /opt/ros/foxy/setup.bash
        source "$WS/install/setup.bash"
        ros2 launch mobile_robot nav_v3.launch.py floor:="$FLOOR" \
            >> "$LOG_DIR/launch_${STAMP}.log" 2>&1
    ) &
    LAUNCH_PID=$!
    LAUNCH_PGID=$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ' || echo "")
    set +m

    # Verify launch process started
    sleep 1
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        log_fail "nav_v3.launch.py failed to start (check $LOG_DIR/launch_${STAMP}.log)"
        store FAIL "launch nav_v3.launch.py failed"
        return
    fi
    log_pass "nav_v3.launch.py background launch started (PID=$LAUNCH_PID)"
    store PASS "Background launch started"

    # 3.3 Countdown wait 25 seconds
    printf "${C_CYAN}[INFO]    ${C_RESET}Waiting for nodes to initialize..."
    for ((i=25; i>0; i--)); do
        printf " %ds" "$i"
        sleep 1
    done
    printf " done\n"
    printf "[$(  _ts)] [INFO]     Node initialization wait complete (25s)\n" >> "$LOG_FILE"

    # ── Topic Hz check helper ─────────────────────────────────────────────────
    # check_topic <topic> <type_hint> <warn_hz> <fail_hz>
    check_topic_hz() {
        set -u
        local topic="$1" warn_hz="$2" fail_hz="$3"

        log_info "Checking Hz: $topic (warn<${warn_hz}, fail<${fail_hz}) ..."
        local hz_out
        hz_out=$(timeout 10 ros2 topic hz "$topic" --window 20 2>&1 | head -20 || true)

        # Extract average Hz value from output
        local measured
        measured=$(echo "$hz_out" | grep -oP 'average rate: \K[\d.]+' | head -1)

        if [[ -z "$measured" ]]; then
            log_fail "$topic — No data received (topic not publishing)"
            printf "  Raw output:\n%s\n" "$hz_out" >> "$LOG_FILE"
            store FAIL "$topic: no data"
            return 2
        fi

        # Use awk for float comparison (bash can't do floats)
        local cmp_fail cmp_warn
        cmp_fail=$(awk "BEGIN{print ($measured < $fail_hz) ? 1 : 0}")
        cmp_warn=$(awk "BEGIN{print ($measured < $warn_hz) ? 1 : 0}")

        if [[ "$cmp_fail" == "1" ]]; then
            log_fail "$(printf "%-30s %6.1f Hz  (fail threshold: %g Hz)" "$topic" "$measured" "$fail_hz")"
            printf "  Raw output:\n%s\n" "$hz_out" >> "$LOG_FILE"
            store FAIL "$topic: ${measured} Hz (below fail threshold ${fail_hz} Hz)"
            return 2
        elif [[ "$cmp_warn" == "1" ]]; then
            log_warn "$(printf "%-30s %6.1f Hz  (warn threshold: %g Hz)" "$topic" "$measured" "$warn_hz")" \
                     "$hz_out"
            store WARN "$topic: ${measured} Hz (below warn threshold ${warn_hz} Hz)"
            return 1
        else
            log_pass "$(printf "%-30s %6.1f Hz  (min: %g)" "$topic" "$measured" "$fail_hz")"
            store PASS "$topic: ${measured} Hz"
            return 0
        fi
    }

    # 3.4 Check all topics
    check_topic_hz "/scan"               10  5
    check_topic_hz "/scan_filtered"      10  5
    check_topic_hz "/imu/data"           40 20
    check_topic_hz "/imu/euler"          40 20
    check_topic_hz "/odom"               50 20
    check_topic_hz "/odometry/filtered"  30 15
    check_topic_hz "/ultrasonic_scan"     7  3
    check_topic_hz "/tf"                 30 10

    # ── TF transform check helper ─────────────────────────────────────────────
    check_tf() {
        set -u
        local src="$1" dst="$2" fail_on_missing="$3"
        log_info "Checking TF: $src → $dst ..."
        local tf_out
        tf_out=$(timeout 6 ros2 run tf2_ros tf2_echo "$src" "$dst" 2>&1 | head -5 || true)

        if echo "$tf_out" | grep -qiE "(translation|rotation|transform)"; then
            log_pass "TF: $src → $dst available"
            store PASS "TF: $src → $dst"
        else
            if [[ "$fail_on_missing" == "FAIL" ]]; then
                log_fail "TF: $src → $dst NOT available"
                printf "  Raw output:\n%s\n" "$tf_out" >> "$LOG_FILE"
                store FAIL "TF: $src → $dst missing"
            else
                log_warn "TF: $src → $dst not available (AMCL may not be running yet)"
                printf "  Raw output:\n%s\n" "$tf_out" >> "$LOG_FILE"
                store WARN "TF: $src → $dst not yet available"
            fi
        fi
    }

    # 3.5 Check TF transforms
    check_tf "odom"          "base_footprint" "FAIL"
    check_tf "map"           "odom"           "WARN"
    check_tf "base_footprint" "laser_frame"   "FAIL"
    check_tf "base_footprint" "imu_link"      "FAIL"

    # 3.6 Kill background launch (also handled by trap, explicit here for logging)
    log_info "Stopping background launch (PID=$LAUNCH_PID, PGID=$LAUNCH_PGID) ..."
    if [[ -n "$LAUNCH_PGID" ]]; then
        kill -- "-$LAUNCH_PGID" 2>/dev/null || true
        sleep 2
        kill -9 -- "-$LAUNCH_PGID" 2>/dev/null || true
        log_info "Background launch stopped"
        printf "[$(  _ts)] [INFO]     Background launch killed (PGID=%s)\n" "$LAUNCH_PGID" >> "$LOG_FILE"
    fi
    LAUNCH_PGID=""  # prevent trap from killing again
    LAUNCH_PID=""
}

# =============================================================================
# PHASE 4 — Configuration File Check
# =============================================================================
phase4_configs() {
    local phase_label="PHASE 4 — Configuration Files (floor: $FLOOR)"
    printf "\n${C_BOLD}%s${C_RESET}\n" "$phase_label"
    printf "%s\n" "$(printf '─%.0s' {1..60})"
    printf "%s\n" "$phase_label" >> "$LOG_FILE"

    # Resolve PKG_SHARE
    local PKG_SHARE
    PKG_SHARE=$(timeout 10 ros2 pkg prefix mobile_robot --share 2>/dev/null || true)
    if [[ -z "$PKG_SHARE" ]]; then
        log_fail "Cannot resolve mobile_robot package share path (is workspace sourced?)"
        store FAIL "mobile_robot package not found"
        return
    fi
    log_info "PKG_SHARE = $PKG_SHARE"

    _check_file() {
        local f="$1"
        if [[ -f "$f" ]]; then
            log_pass "Found: $f"
            store PASS "Config file: $(basename "$f")"
        else
            log_fail "Missing: $f"
            store FAIL "Missing config: $f"
        fi
    }

    # 4.1–4.6 Static configs
    _check_file "$PKG_SHARE/config/ekf.yaml"
    _check_file "$PKG_SHARE/config/nav2_params_test.yaml"
    _check_file "$PKG_SHARE/config/laser_filter.yaml"
    _check_file "$PKG_SHARE/config/bno055_params.yaml"
    _check_file "$PKG_SHARE/config/wheel_odom_params.yaml"
    _check_file "$PKG_SHARE/urdf/mobile_robot_v2.urdf.xacro"

    # 4.7–4.8 Floor-specific
    _check_file "$PKG_SHARE/maps/map_${FLOOR}.yaml"
    _check_file "$PKG_SHARE/config/checkpoints_${FLOOR}.yaml"
}

# =============================================================================
# PHASE 5 — Disk, Memory & Temperature
# =============================================================================
phase5_resources() {
    local phase_label="PHASE 5 — Disk / Memory / Temperature"
    printf "\n${C_BOLD}%s${C_RESET}\n" "$phase_label"
    printf "%s\n" "$(printf '─%.0s' {1..60})"
    printf "%s\n" "$phase_label" >> "$LOG_FILE"

    # 5.1 Disk space on /
    local disk_avail_kb disk_avail_gb
    disk_avail_kb=$(df / --output=avail | tail -1 | tr -d ' ')
    disk_avail_gb=$(awk "BEGIN{printf \"%.1f\", $disk_avail_kb/1024/1024}")

    local disk_fail disk_warn
    disk_fail=$(awk "BEGIN{print ($disk_avail_kb < 500*1024) ? 1 : 0}")
    disk_warn=$(awk "BEGIN{print ($disk_avail_kb < 2*1024*1024) ? 1 : 0}")

    if [[ "$disk_fail" == "1" ]]; then
        log_fail "Disk available: ${disk_avail_gb} GB (CRITICAL — below 500 MB)"
        store FAIL "Disk: ${disk_avail_gb} GB (< 500 MB)"
    elif [[ "$disk_warn" == "1" ]]; then
        log_warn "Disk available: ${disk_avail_gb} GB (low — below 2 GB)"
        store WARN "Disk: ${disk_avail_gb} GB (< 2 GB)"
    else
        log_pass "Disk available: ${disk_avail_gb} GB"
        store PASS "Disk: ${disk_avail_gb} GB"
    fi

    # 5.2 Available RAM
    local ram_avail_kb ram_avail_gb
    ram_avail_kb=$(awk '/MemAvailable/ {print $2}' /proc/meminfo)
    ram_avail_gb=$(awk "BEGIN{printf \"%.1f\", $ram_avail_kb/1024/1024}")

    local ram_warn
    ram_warn=$(awk "BEGIN{print ($ram_avail_kb < 2*1024*1024) ? 1 : 0}")

    if [[ "$ram_warn" == "1" ]]; then
        log_warn "RAM available: ${ram_avail_gb} GB (below 2 GB)"
        store WARN "RAM: ${ram_avail_gb} GB (< 2 GB)"
    else
        log_pass "RAM available: ${ram_avail_gb} GB"
        store PASS "RAM: ${ram_avail_gb} GB"
    fi

    # 5.3 CPU/SoC temperatures
    local max_temp_c=0
    local max_zone=""
    for zone_file in /sys/class/thermal/thermal_zone*/temp; do
        [[ -f "$zone_file" ]] || continue
        local raw_temp temp_c zone_name
        raw_temp=$(cat "$zone_file" 2>/dev/null || echo 0)
        temp_c=$(awk "BEGIN{printf \"%.0f\", $raw_temp/1000}")
        zone_name=$(basename "$(dirname "$zone_file")")
        if (( temp_c > max_temp_c )); then
            max_temp_c=$temp_c
            max_zone=$zone_name
        fi
    done

    local temp_fail temp_warn
    temp_fail=$(awk "BEGIN{print ($max_temp_c > 85) ? 1 : 0}")
    temp_warn=$(awk "BEGIN{print ($max_temp_c > 70) ? 1 : 0}")

    if [[ "$temp_fail" == "1" ]]; then
        log_fail "Max CPU/SoC temp: ${max_temp_c}°C on $max_zone (CRITICAL — above 85°C)"
        store FAIL "Temp: ${max_temp_c}°C > 85°C"
    elif [[ "$temp_warn" == "1" ]]; then
        log_warn "Max CPU/SoC temp: ${max_temp_c}°C on $max_zone (above 70°C)"
        store WARN "Temp: ${max_temp_c}°C > 70°C"
    else
        log_pass "Max CPU/SoC temp: ${max_temp_c}°C"
        store PASS "Temp: ${max_temp_c}°C"
    fi

    # 5.4 log_check/ directory
    if [[ -d "$LOG_DIR" && -w "$LOG_DIR" ]]; then
        log_pass "log_check/ exists and is writable ($LOG_DIR)"
        store PASS "log_check/ writable"
    else
        log_autofix "Creating log directory: mkdir -p $LOG_DIR"
        mkdir -p "$LOG_DIR"
        store AUTOFIX "mkdir -p $LOG_DIR"
        if [[ -d "$LOG_DIR" && -w "$LOG_DIR" ]]; then
            log_pass "log_check/ created and writable"
            store PASS "log_check/ created"
        else
            log_fail "log_check/ could not be created at $LOG_DIR"
            store FAIL "log_check/ not writable"
        fi
    fi
}

# =============================================================================
# FINAL REPORT
# =============================================================================
print_report() {
    local verdict verdict_symbol exit_code
    if (( COUNT_FAIL > 0 )); then
        # Dùng ký tự ASCII [X] thay cho Unicode ✘ để tránh lỗi căn lề Bash
        verdict="[X] NOT READY - Fix FAIL items before proceeding"
        verdict_symbol="${C_RED}"
        exit_code=1
    elif (( COUNT_WARN > 0 )); then
        verdict="[!] READY WITH WARNINGS - Review WARN items"
        verdict_symbol="${C_YELLOW}"
        exit_code=0
    else
        verdict="[v] READY - Safe to run autonomous navigation"
        verdict_symbol="${C_GREEN}"
        exit_code=0
    fi

    local total=$(( COUNT_PASS + COUNT_WARN + COUNT_FAIL ))
    local dt=$(_date)
    local hostname_str=$(hostname)
    
    # Rút gọn đường dẫn log (thay /home/user bằng ~) để tránh bị phình khung
    local display_log="${LOG_FILE/$HOME/\~}"
    local log_str="Log saved to: $display_log"

    # Giảm chiều rộng cơ bản của khung xuống cho gọn hơn
    local W=66
    (( ${#log_str} + 4 > W )) && W=$(( ${#log_str} + 4 ))

    # Helper function vẽ các đường viền ngang
    _print_top() { printf "╔"; printf '═%.0s' $(seq 1 $W); printf "╗\n"; }
    _print_sep() { printf "╠"; printf '═%.0s' $(seq 1 $W); printf "╣\n"; }
    _print_bot() { printf "╚"; printf '═%.0s' $(seq 1 $W); printf "╝\n"; }

    # Helper function in nội dung dòng và tự động lấp đầy khoảng trắng tới viền phải
    _print_line() {
        local text="$1"
        local color="${2:-}"
        local text_len=${#text}
        
        local pad_len=$(( W - text_len ))
        (( pad_len < 0 )) && pad_len=0
        local pad_str=$(printf '%*s' "$pad_len" "")
        
        # In ra Terminal (có màu sắc)
        printf "║${color}%s${C_RESET}%s║\n" "$text" "$pad_str"
        # In vào file Log (không kèm mã màu để tránh lỗi hiển thị khi đọc file)
        printf "║%s%s║\n" "$text" "$pad_str" >> "$LOG_FILE"
    }

    # Hàm thực thi đồng thời vẽ viền lên terminal và file log
    _do_top() { _print_top; _print_top >> "$LOG_FILE"; }
    _do_sep() { _print_sep; _print_sep >> "$LOG_FILE"; }
    _do_bot() { _print_bot; _print_bot >> "$LOG_FILE"; }

    # ── Bắt đầu In Báo Cáo ──────────────────────────────────────────────────────
    echo ""
    echo "" >> "$LOG_FILE"
    
    _do_top
    _print_line "       ROBOT PRE-FLIGHT CHECK REPORT"
    _print_line "   Date  : $dt"
    _print_line "   Host  : $hostname_str"
    _print_line "   Floor : $FLOOR"
    _do_sep

    for i in "${!RESULT_TAGS[@]}"; do
        local tag="${RESULT_TAGS[$i]}" msg="${RESULT_MSGS[$i]}"
        local color=""
        case "$tag" in
            PASS)     color="$C_GREEN"   ;;
            WARN)     color="$C_YELLOW"  ;;
            FAIL)     color="$C_RED"     ;;
            AUTOFIX|AUTOKILL) color="$C_MAGENTA" ;;
        esac
        # Đưa khoảng trắng ra ngoài dấu ngoặc để chữ và ngoặc sát nhau: [PASS]     
        local tag_fmt=$(printf "%-11s" "[$tag]")
        _print_line "   $tag_fmt $msg" "$color"
    done

    _do_sep
    _print_line "  SUMMARY"
    _print_line "    Total checks : $(printf "%-5d" "$total")"
    _print_line "    PASS         : $(printf "%-5d" "$COUNT_PASS")"
    _print_line "    WARN         : $(printf "%-5d" "$COUNT_WARN")"
    _print_line "    FAIL         : $(printf "%-5d" "$COUNT_FAIL")"
    _print_line "    AUTO-FIX     : $(printf "%-5d" "$COUNT_AUTOFIX")  |  AUTO-KILL : $(printf "%-5d" "$COUNT_AUTOKILL")"
    _do_sep
    _print_line "   VERDICT: $verdict" "$verdict_symbol"
    _print_line "   $log_str"
    _do_bot

    return $exit_code
}

# =============================================================================
# MAIN
# =============================================================================
main() {
    parse_args "$@"

    # Bootstrap log directory early (Phase 5 will also check/create it)
    mkdir -p "$LOG_DIR" 2>/dev/null || true

    # Re-open LOG_FILE (may not exist yet if log_dir just created)
    : > "$LOG_FILE"  # truncate / create

    printf "%s\n" "============================================================" >> "$LOG_FILE"
    printf "pre_check.sh started at %s | floor=%s | skip-build=%d | check-topics=%d\n" \
           "$(_date)" "$FLOOR" "$SKIP_BUILD" "$CHECK_TOPICS" >> "$LOG_FILE"
    printf "%s\n" "============================================================" >> "$LOG_FILE"

    printf "${C_BOLD}╔══════════════════════════════════════════════════════════╗${C_RESET}\n"
    printf "${C_BOLD}║     Mobile Robot Pre-Flight Check  [floor: %s]           ║${C_RESET}\n" "$FLOOR"
    printf "${C_BOLD}╚══════════════════════════════════════════════════════════╝${C_RESET}\n"
    log_info "Start: $(_date) | host: $(hostname) | floor: $FLOOR"

    # Source ROS + workspace upfront (needed even if skip-build)
    if [[ -f /opt/ros/foxy/setup.bash ]]; then
        # shellcheck disable=SC1091
        source /opt/ros/foxy/setup.bash 2>/dev/null
    fi
    if [[ -f "$WS/install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "$WS/install/setup.bash" 2>/dev/null
    fi

    if (( SKIP_BUILD == 0 )); then
        phase0_build
    else
        log_info "PHASE 0 skipped (--skip-build)"
        printf "[$(  _ts)] [INFO]     PHASE 0 skipped via --skip-build\n" >> "$LOG_FILE"
    fi

    phase1_system
    phase2_hardware

    # PHASE 3: zombie kill always runs; topic/TF check only with --check-topics
    phase3_ros_topics

    phase4_configs
    phase5_resources

    print_report
    local rc=$?

    log_info "pre_check.sh finished at $(_date) | exit code: $rc"
    printf "[$(  _ts)] [INFO]     pre_check.sh finished | exit=%d\n" "$rc" >> "$LOG_FILE"
    exit $rc
}

main "$@"