#!/usr/bin/env bash

set -eo pipefail

# ==================================================
# SLAM mapping runner — bag recording + auto map save
# ROS 2 Foxy | mobile_robot package
# ==================================================
# Purpose:
#   Launch slam_v3_launch.py, record a bag, and auto-save the map
#   (pgm + yaml) when the operator presses Ctrl+C.
#   Map is saved BEFORE slam_toolbox is killed.
#
# Usage:
#   ./run_slam_exp.sh [OPTIONS]
#
# Required (or use defaults):
#   --speed SPEED       Teleop speed for this run: 0.10 / 0.20 / 0.30  (default: 0.10)
#   --run NUM           Run number, zero-padded                          (default: 01)
#   --scenario ID       Scenario ID                                      (default: S6)
#
# Optional:
#   --skip-build        Skip colcon build step
#   --no-skip-build     Force build
#   --cleanup-mode MODE fast (default) or deep
#   --deep              Shorthand for --cleanup-mode deep
#   --no-hard-kill      Skip force kill on exit
#   --no-bag            Disable bag recording
#   --no-save-map       Disable auto map save on exit
#   --pre-check         Run pre-flight checks (kill stale procs, flush cache, check swap)
#   --help, -h          Show this message and exit
#
# Examples:
#   ./run_slam_exp.sh                                  # run với bag (defaults)
#   ./run_slam_exp.sh --no-bag                         # run không ghi bag
#   ./run_slam_exp.sh --speed 0.10 --run 01
#   ./run_slam_exp.sh --speed 0.20 --run 01 --skip-build
#   ./run_slam_exp.sh --speed 0.10 --run 01 --pre-check
#   ./run_slam_exp.sh --speed 0.10 --run 01 --no-save-map
#
# Env-var overrides (CLI takes precedence):
#   TELEOP_SPEED=0.20 RUN_NUMBER=02 ./run_slam_exp.sh
# ==================================================

# ==================================================
# User-configurable variables
# ==================================================
WS="${WS:-$HOME/mbrobot_ws}"
PKG="${PKG:-mobile_robot}"
LAUNCH_FILE="${LAUNCH_FILE:-slam_v3_launch.py}"

# Experiment identity
SCENARIO="${SCENARIO:-S6}"
RUN_NUMBER="${RUN_NUMBER:-01}"
TELEOP_SPEED="${TELEOP_SPEED:-0.10}"
RECORD_BAG="${RECORD_BAG:-true}"
SAVE_MAP="${SAVE_MAP:-true}"
PRE_CHECK="${PRE_CHECK:-false}"

# Derived naming (speed as zero-padded cm/s integer)
# Computed after parse_args(); placeholders here.
SPEED_CM=""
BAG_NAME=""
MAP_NAME=""
BAG_PATH=""
MAP_PATH=""
MAPS_DIR="$WS/src/$PKG/maps"
# Directory where the map is saved. The map FILENAME follows the bag name
# (same stem as $BAG_NAME) so each map pairs 1:1 with its recorded bag.
MAP_OUT_DIR="${MAP_OUT_DIR:-$MAPS_DIR/demo}"

# Nav parameters inherited (no floor/timeout for SLAM)
USE_SIM_TIME="${USE_SIM_TIME:-false}"
US_SERIAL_PORT="${US_SERIAL_PORT:-/dev/ttyUltrasonic}"
US_BAUD_RATE="${US_BAUD_RATE:-115200}"

# Build / cleanup
SKIP_BUILD="${SKIP_BUILD:-true}"
CLEANUP_MODE="${CLEANUP_MODE:-fast}"
USE_LIFECYCLE_SHUTDOWN="${USE_LIFECYCLE_SHUTDOWN:-auto}"
CLEAN_FASTDDS="${CLEAN_FASTDDS:-auto}"
STOP_ROS_DAEMON="${STOP_ROS_DAEMON:-auto}"
HARD_KILL="${HARD_KILL:-true}"
EXTRA_LAUNCH_ARGS="${EXTRA_LAUNCH_ARGS:-}"

# ==================================================
# Derived names / paths
# ==================================================
# Computed after parse_args() resolves TELEOP_SPEED
compute_names() {
    SPEED_CM=$(awk "BEGIN{printf \"%03d\", $TELEOP_SPEED * 100 + 0.5}")
    RUN_TS="$(date +%Y%m%d_%H%M%S)"
    BAG_NAME="s${SCENARIO}_v${SPEED_CM}_run${RUN_NUMBER}_${RUN_TS}"
    BAG_PATH="$WS/bags/$BAG_NAME"
    # Map filename matches the bag name → easy to tell which map came from which bag
    MAP_NAME="$BAG_NAME"
    MAP_PATH="$MAP_OUT_DIR/$MAP_NAME"
    LOG_DIR="$WS/log_run"
    LOG_FILE="$LOG_DIR/slam_${BAG_NAME}.log"
    META_FILE="$WS/bags/${BAG_NAME}_meta.txt"
}

# ==================================================
# Helper functions
# ==================================================
log()
{
    echo "$@" | tee -a "$LOG_FILE"
}

require_cmd()
{
    if ! command -v "$1" >/dev/null 2>&1; then
        log "[ERROR] Required command not found: $1"
        exit 127
    fi
}

is_true()
{
    case "${1,,}" in
        true|1|yes|y|on) return 0 ;;
        *) return 1 ;;
    esac
}

is_auto_enabled_for_deep()
{
    local value="${1,,}"
    case "$value" in
        auto)
            [ "$CLEANUP_MODE" = "deep" ]
            ;;
        true|1|yes|y|on)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

kill_soft()
{
    local pattern="$1"
    pkill -INT -f "$pattern" >/dev/null 2>&1 || true
}

kill_hard()
{
    local pattern="$1"
    pkill -9 -f "$pattern" >/dev/null 2>&1 || true
}

publish_zero_velocity_topic()
{
    local topic="$1"
    local repeat_count="${2:-2}"
    local sleep_s="${3:-0.08}"

    if ! command -v ros2 >/dev/null 2>&1; then
        return 0
    fi

    for _ in $(seq 1 "$repeat_count"); do
        timeout 0.8s ros2 topic pub --once "$topic" geometry_msgs/msg/Twist \
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
            >/dev/null 2>&1 || true
        sleep "$sleep_s"
    done
}

publish_zero_velocity_fast()
{
    # /cmd_vel is consumed by wheel_odom_node, which forwards speed/steer commands to STM32.
    publish_zero_velocity_topic "/cmd_vel" 2 0.08

    # /cmd_vel_nav may exist before ultrasonic_fusion_node gates velocity.
    # Harmless if unused.
    publish_zero_velocity_topic "/cmd_vel_nav" 1 0.05
}

publish_zero_velocity_deep()
{
    publish_zero_velocity_topic "/cmd_vel" 4 0.10
    publish_zero_velocity_topic "/cmd_vel_nav" 2 0.08
}

try_lifecycle_shutdown()
{
    local node="$1"

    if timeout 0.8s ros2 node list 2>/dev/null | grep -qx "$node"; then
        log "[CLEANUP][DEEP] Lifecycle shutdown: $node"
        timeout 2s ros2 lifecycle set "$node" shutdown >/dev/null 2>&1 || true
    fi
}

shutdown_nav2_lifecycle_nodes()
{
    # Best-effort graceful shutdown. Some nodes may not exist depending on Nav2 version.
    try_lifecycle_shutdown "/bt_navigator"
    try_lifecycle_shutdown "/controller_server"
    try_lifecycle_shutdown "/planner_server"
    try_lifecycle_shutdown "/recoveries_server"
    try_lifecycle_shutdown "/waypoint_follower"
    try_lifecycle_shutdown "/amcl"
    try_lifecycle_shutdown "/map_server"

    # Optional newer/Nav2-extension nodes. Usually absent in Foxy setups.
    try_lifecycle_shutdown "/behavior_server"
    try_lifecycle_shutdown "/smoother_server"
    try_lifecycle_shutdown "/velocity_smoother"
    try_lifecycle_shutdown "/collision_monitor"
}

soft_stop_command_generators()
{
    # Stop nodes that can create new goals or new velocity commands.
    kill_soft "navigator.py|checkpoint_navigator|checkpoint_cmd.py|checkpoint_command_sender|endurance_test.py|endurance_runner|robot_teleop.py|teleop_twist_keyboard"
}

soft_stop_non_wheel_nodes()
{
    # Keep wheel_odom_node alive until after final zero velocity is sent.
    kill_soft "bt_navigator|controller_server|planner_server|recoveries_server|waypoint_follower|lifecycle_manager"
    kill_soft "map_server|amcl"
    kill_soft "ekf_node|ekf_filter_node|robot_localization"
    kill_soft "sllidar_node|scan_to_scan_filter_chain"
    kill_soft "bno055|imu_reader"
    kill_soft "jetson_sensor_bridge|ultrasonic_fusion_node|ultrasonic_node"
    kill_soft "robot_state_publisher|joint_state_publisher"
    kill_soft "session_logger"
}

soft_stop_extra_nodes_deep()
{
    kill_soft "behavior_server|smoother_server|velocity_smoother|collision_monitor"
    kill_soft "component_container|component_container_mt|component_container_isolated"
}

hard_stop_robot_nodes_fast()
{
    kill_hard "ros2 launch ${PKG} ${LAUNCH_FILE}"
    kill_hard "navigator.py|checkpoint_navigator|checkpoint_cmd.py|checkpoint_command_sender|endurance_test.py|endurance_runner|robot_teleop.py|teleop_twist_keyboard"
    kill_hard "bt_navigator|controller_server|planner_server|recoveries_server|waypoint_follower|lifecycle_manager"
    kill_hard "map_server|amcl"
    kill_hard "ekf_node|ekf_filter_node|robot_localization"
    kill_hard "sllidar_node|scan_to_scan_filter_chain"
    kill_hard "bno055|imu_reader"
    kill_hard "jetson_sensor_bridge|ultrasonic_fusion_node|ultrasonic_node"
    kill_hard "wheel_odom_node"
    kill_hard "robot_state_publisher|joint_state_publisher"
    kill_hard "session_logger"
    kill_hard "ros2 bag record"
    kill_hard "slam_toolbox|async_slam_toolbox_node"
}

hard_stop_robot_nodes_deep()
{
    hard_stop_robot_nodes_fast
    kill_hard "behavior_server|smoother_server|velocity_smoother|collision_monitor"
    kill_hard "component_container|component_container_mt|component_container_isolated"
    kill_hard "ros2 bag record"
    kill_hard "slam_toolbox|async_slam_toolbox_node"
}

clean_fastrtps_files()
{
    log "[CLEANUP][DEEP] Remove FastDDS/FastRTPS temporary files"
    rm -rf /tmp/fastrtps_* /tmp/fastdds_* >/dev/null 2>&1 || true
    rm -rf /dev/shm/fastrtps_* /dev/shm/fastdds_* >/dev/null 2>&1 || true
}

stop_ros_daemon()
{
    log "[CLEANUP][DEEP] Stop ROS 2 daemon"
    timeout 2s ros2 daemon stop >/dev/null 2>&1 || true
}

# ==================================================
# Pre-flight checks (opt-in via --pre-check)
# ==================================================
pre_run()
{
    log ""
    log "[PRE] 1/5 Kill stale ROS processes"
    pkill -SIGTERM -f "ros2|slam_toolbox|ekf_node|rviz" >/dev/null 2>&1 || true
    sleep 2
    pkill -SIGKILL -f "ros2|slam_toolbox|ekf_node" >/dev/null 2>&1 || true

    log "[PRE] 2/5 Clear ROS temp files"
    rm -rf /tmp/ros_* /tmp/__pycache__ >/dev/null 2>&1 || true

    log "[PRE] 3/5 Flush page cache"
    sync
    sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches' >/dev/null 2>&1 \
        || log "[PRE][WARN] Cannot flush page cache (sudo required)"

    log "[PRE] 4/5 Check swap"
    if [ "$(swapon --show 2>/dev/null | wc -l)" -lt 2 ]; then
        log "[PRE][WARN] Swap disabled — OOM risk on large maps. Fix: sudo swapon /swapfile"
    else
        log "[PRE] Swap: $(free -h | grep Swap | awk '{print $2}')"
    fi

    log "[PRE] 5/5 Set fan to cool mode"
    sudo sh -c 'echo cool > /sys/devices/pwm-fan/target_pwm' >/dev/null 2>&1 \
        || sudo jetson_clocks --fan >/dev/null 2>&1 \
        || log "[PRE][WARN] Cannot set fan mode (skipping)"

    log "[PRE] Done. RAM: Used=$(free -h | grep Mem | awk '{print $3}') Available=$(free -h | grep Mem | awk '{print $4}')"
}

# ==================================================
# Cleanup functions
# ==================================================
cleanup_fast()
{
    log "[CLEANUP][FAST] 1/6 Publish zero velocity"
    publish_zero_velocity_fast

    log "[CLEANUP][FAST] 2/6 Stop command-generating nodes"
    soft_stop_command_generators

    log "[CLEANUP][FAST] 3/6 Publish zero velocity again before stopping launch"
    publish_zero_velocity_fast

    log "[CLEANUP][FAST] 4/6 Soft terminate launch and non-wheel nodes"
    kill_soft "ros2 launch ${PKG} ${LAUNCH_FILE}"
    soft_stop_non_wheel_nodes

    sleep 0.8

    log "[CLEANUP][FAST] 5/6 Final zero velocity, then stop wheel_odom_node last"
    publish_zero_velocity_fast
    kill_soft "wheel_odom_node"

    sleep 0.3

    if is_true "$HARD_KILL"; then
        log "[CLEANUP][FAST] 6/6 Force kill remaining robot processes"
        hard_stop_robot_nodes_fast
    else
        log "[CLEANUP][FAST] 6/6 Skip force kill because HARD_KILL=false"
    fi
}

cleanup_deep()
{
    log "[CLEANUP][DEEP] 1/9 Publish zero velocity"
    publish_zero_velocity_deep

    log "[CLEANUP][DEEP] 2/9 Stop command-generating nodes"
    soft_stop_command_generators

    if is_auto_enabled_for_deep "$USE_LIFECYCLE_SHUTDOWN"; then
        log "[CLEANUP][DEEP] 3/9 Best-effort Nav2 lifecycle shutdown"
        shutdown_nav2_lifecycle_nodes
    else
        log "[CLEANUP][DEEP] 3/9 Skip Nav2 lifecycle shutdown"
    fi

    log "[CLEANUP][DEEP] 4/9 Publish zero velocity again before stopping launch"
    publish_zero_velocity_deep

    log "[CLEANUP][DEEP] 5/9 Soft terminate launch and non-wheel nodes"
    kill_soft "ros2 launch ${PKG} ${LAUNCH_FILE}"
    soft_stop_non_wheel_nodes
    soft_stop_extra_nodes_deep

    sleep 1.0

    log "[CLEANUP][DEEP] 6/9 Final zero velocity, then stop wheel_odom_node last"
    publish_zero_velocity_deep
    kill_soft "wheel_odom_node"

    sleep 0.5

    if is_true "$HARD_KILL"; then
        log "[CLEANUP][DEEP] 7/9 Force kill remaining robot processes"
        hard_stop_robot_nodes_deep
    else
        log "[CLEANUP][DEEP] 7/9 Skip force kill because HARD_KILL=false"
    fi

    if is_auto_enabled_for_deep "$CLEAN_FASTDDS"; then
        log "[CLEANUP][DEEP] 8/9 Clean FastDDS/FastRTPS resources"
        clean_fastrtps_files
    else
        log "[CLEANUP][DEEP] 8/9 Skip FastDDS/FastRTPS cleanup"
    fi

    if is_auto_enabled_for_deep "$STOP_ROS_DAEMON"; then
        log "[CLEANUP][DEEP] 9/9 Stop ROS 2 daemon"
        stop_ros_daemon
    else
        log "[CLEANUP][DEEP] 9/9 Skip ROS 2 daemon stop"
    fi
}

# ==================================================
# Map save — must be called BEFORE slam_toolbox is killed
# ==================================================
save_map_on_exit()
{
    if is_true "$SAVE_MAP"; then
        log "[MAP] --no-save-map set — skipping map save."
        return 0
    fi

    log ""
    log "[MAP] Saving map → $MAP_PATH"
    log "[MAP] (slam_toolbox must still be running; saving before kill)"

    # Ensure the target directory exists (e.g. maps/demo/)
    mkdir -p "$(dirname "$MAP_PATH")"

    # map_saver_cli subscribes to /map; slam_toolbox must still publish it.
    # save_map_timeout is in milliseconds: 60000 = 60 s.
    timeout 70s ros2 run nav2_map_server map_saver_cli \
        -f "$MAP_PATH" \
        --ros-args \
        -p save_map_timeout:=60000 \
        -p map_subscribe_transient_local:=true \
        -p free_thresh_default:=0.25 \
        -p occupied_thresh_default:=0.65 \
        2>&1 | tee -a "$LOG_FILE" || true

    if [ -f "${MAP_PATH}.pgm" ] && [ -f "${MAP_PATH}.yaml" ]; then
        log "[MAP] Saved: ${MAP_PATH}.pgm"
        log "[MAP] Saved: ${MAP_PATH}.yaml"
    else
        log "[MAP][WARN] Map files not found at $MAP_PATH"
        log "[MAP][WARN] save_map_timeout may have expired or slam_toolbox was already stopped."
    fi
}

cleanup()
{
    local exit_code=$?
    set +e
    trap - EXIT
    trap - INT
    trap - TERM

    log ""
    log "=================================================="
    log "[CLEANUP] Start SLAM cleanup"
    log "=================================================="

    # ── Step 1: Save map while slam_toolbox is still alive ──────────────────
    save_map_on_exit

    # ── Step 2: Standard node shutdown ──────────────────────────────────────
    case "$CLEANUP_MODE" in
        fast) cleanup_fast ;;
        deep) cleanup_deep ;;
        *)    log "[CLEANUP][WARN] Unknown CLEANUP_MODE. Using fast."; cleanup_fast ;;
    esac

    # ── Step 3: Kill bag record process ─────────────────────────────────────
    kill_hard "ros2 bag record"

    wait 2>/dev/null || true

    # ── Step 4: Update metadata ──────────────────────────────────────────────
    if [ -f "$META_FILE" ]; then
        local result_str="STOPPED_OK"
        if [ "$exit_code" -ne 0 ] && [ "$exit_code" -ne 130 ] && [ "$exit_code" -ne 143 ]; then
            result_str="ERROR_$exit_code"
        fi
        local map_saved="false"
        [ -f "${MAP_PATH}.pgm" ] && map_saved="true"
        sed -i "s/^result=PENDING/result=$result_str/" "$META_FILE" 2>/dev/null || true
        sed -i "s/^map_saved=false/map_saved=$map_saved/" "$META_FILE" 2>/dev/null || true
        log "Metadata: result=$result_str  map_saved=$map_saved"
    fi

    log "[CLEANUP] Done"
    log "=================================================="
    log "Exit code : $exit_code"
    log "Log file  : $LOG_FILE"
    log "=================================================="

    exit "$exit_code"
}

# ==================================================
# CLI argument parsing (overrides env-var defaults)
# ==================================================
parse_args()
{
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                sed -n '/^# Usage:/,/^# Notes:/p' "$0" | sed 's/^# \{0,3\}//'
                exit 0
                ;;
            --scenario=*)   SCENARIO="${1#--scenario=}" ;;
            --scenario)     SCENARIO="$2";          shift ;;
            --run=*)        RUN_NUMBER="${1#--run=}" ;;
            --run)          RUN_NUMBER="$2";        shift ;;
            --speed=*)      TELEOP_SPEED="${1#--speed=}" ;;
            --speed)        TELEOP_SPEED="$2";      shift ;;
            --no-bag)       RECORD_BAG=false ;;
            --save-map)     SAVE_MAP=true ;;
            --pre-check)    PRE_CHECK=true ;;
            --skip-build)     SKIP_BUILD=true ;;
            --no-skip-build)  SKIP_BUILD=false ;;
            --cleanup-mode=*) CLEANUP_MODE="${1#--cleanup-mode=}" ;;
            --cleanup-mode)   CLEANUP_MODE="$2";             shift ;;
            --deep)           CLEANUP_MODE=deep ;;
            --timeout=*)      TIMEOUT_AT_CHECKPOINT="${1#--timeout=}" ;;
            --timeout)        TIMEOUT_AT_CHECKPOINT="$2";   shift ;;
            --no-hard-kill)   HARD_KILL=false ;;
            -*)
                echo "[ERROR] Unknown flag: $1  (use --help for usage)" >&2
                exit 1
                ;;
            *)
                echo "[ERROR] Unexpected argument: $1  (use --help for usage)" >&2
                exit 1
                ;;
        esac
        shift
    done
}

parse_args "$@"

compute_names
mkdir -p "$LOG_DIR" "$WS/bags" "$MAPS_DIR"

# ==================================================
# Trap signals
# ==================================================
trap cleanup EXIT
trap 'log ""; log "[SIGNAL] Ctrl+C/SIGINT received. Stopping SLAM launch..."; exit 130' INT
trap 'log ""; log "[SIGNAL] SIGTERM received. Stopping SLAM launch..."; exit 143' TERM

# ==================================================
# Startup information
# ==================================================
log "=================================================="
log "AMR SLAM Mapping Runner"
log "--------------------------------------------------"
log "Scenario            : $SCENARIO"
log "Run number          : $RUN_NUMBER"
log "Teleop speed        : $TELEOP_SPEED m/s  (v${SPEED_CM})"
log "Record bag          : $RECORD_BAG"
log "Save map on exit    : $SAVE_MAP"
log "Bag path            : $BAG_PATH"
log "Map path            : $MAP_PATH"
log "--------------------------------------------------"
log "Run timestamp       : $RUN_TS"
log "Workspace           : $WS"
log "Launch file         : $LAUNCH_FILE"
log "Cleanup mode        : $CLEANUP_MODE"
log "Skip build          : $SKIP_BUILD"
log "Log file            : $LOG_FILE"
log "=================================================="

# ==================================================
# Source ROS 2 Foxy EARLY (needed before require_cmd when launched from desktop icon)
# ==================================================
if [ -f /opt/ros/foxy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/foxy/setup.bash
fi

# ==================================================
# Basic validation
# ==================================================
log ""
log "[1] Check required commands and workspace"

require_cmd ros2
require_cmd colcon

if [ ! -d "$WS" ]; then
    log "[ERROR] Workspace does not exist: $WS"
    exit 1
fi

# ==================================================
# Go to workspace
# ==================================================
log ""
log "[2] Go to workspace"
cd "$WS"
pwd | tee -a "$LOG_FILE"

# ==================================================
# Source ROS 2 Foxy
# ==================================================
log ""
log "[3] Source ROS 2 Foxy"

if [ ! -f /opt/ros/foxy/setup.bash ]; then
    log "[ERROR] /opt/ros/foxy/setup.bash not found"
    exit 1
fi

# Already sourced early above; source again to ensure correct env after cd
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

# ==================================================
# Pre-flight checks (optional)
# ==================================================
if is_true "$PRE_CHECK"; then
    log ""
    log "[3b] Pre-flight checks (--pre-check enabled)"
    pre_run
else
    log ""
    log "[3b] Skip pre-flight checks (use --pre-check to enable)"
fi

# ==================================================
# Build package
# ==================================================
if is_true "$SKIP_BUILD"; then
    log ""
    log "[4] Skip build because SKIP_BUILD=true"
else
    log ""
    log "[4] Build package: $PKG"
    colcon build --packages-select "$PKG" 2>&1 | tee -a "$LOG_FILE"
fi

# ==================================================
# Source workspace
# ==================================================
log ""
log "[5] Source workspace install/setup.bash"

if [ ! -f "$WS/install/setup.bash" ]; then
    log "[ERROR] Workspace setup file not found: $WS/install/setup.bash"
    log "[ERROR] Build may have failed or package was not installed."
    exit 1
fi

# shellcheck disable=SC1090
source "$WS/install/setup.bash"

log ""
log "[5b] Write run metadata"
cat > "$META_FILE" << EOF
scenario=$SCENARIO
run_number=$RUN_NUMBER
speed_ms=$TELEOP_SPEED
speed_cm=v${SPEED_CM}
bag_name=$BAG_NAME
bag_path=$BAG_PATH
map_name=$MAP_NAME
map_path=$MAP_PATH
run_ts=$RUN_TS
record_bag=$RECORD_BAG
save_map=$SAVE_MAP
map_saved=false
result=PENDING
ate_rmse=
map_accuracy_pct=
notes=
EOF
log "Metadata → $META_FILE"

# ==================================================
# Compose launch command
# ==================================================
LAUNCH_CMD=(
    ros2 launch "$PKG" "$LAUNCH_FILE"
    "use_sim_time:=$USE_SIM_TIME"
    "us_serial_port:=$US_SERIAL_PORT"
    "us_baud_rate:=$US_BAUD_RATE"
    "scenario:=$SCENARIO"
    "speed:=$TELEOP_SPEED"
    "run_number:=$RUN_NUMBER"
    "record_bag:=$RECORD_BAG"
)

if [ -n "$EXTRA_LAUNCH_ARGS" ]; then
    # shellcheck disable=SC2206
    LAUNCH_CMD+=( $EXTRA_LAUNCH_ARGS )
fi

# ==================================================
# Launch SLAM mapping
# ==================================================
log ""
log "=================================================="
log "[6] Launch SLAM mapping"
log "Command: ${LAUNCH_CMD[*]}"
log "=================================================="

"${LAUNCH_CMD[@]}" 2>&1 | tee -a "$LOG_FILE"

# ==================================================
# Done — chờ xác nhận trước khi đóng terminal
# Dùng zenity (GUI dialog) nếu có, fallback sang read
# ==================================================
log ""
log "Navigation stopped. Press the button or Enter to close terminal."

if command -v zenity >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    zenity --question         --title="Navigation Stopped"         --text="Navigation đã dừng.\nBấm OK để đóng terminal."         --ok-label="Đóng terminal"         --cancel-label="Giữ terminal mở"         --width=300         2>/dev/null && exit 0 || true
fi

# Fallback: đọc từ stdin (hoạt động khi có physical keyboard)
read -r -p "Press Enter to close terminal... " || true
