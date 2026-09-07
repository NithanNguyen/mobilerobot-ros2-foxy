#!/usr/bin/env bash

set -eo pipefail

# ==================================================
# Experiment runner — AMR navigation with bag recording
# ROS 2 Foxy | mobile_robot package
# ==================================================
# Purpose:
#   Launch nav_v3 with experiment metadata (scenario, run number),
#   auto-record a bag, and save per-run metadata for Sheet 2.
#
# Usage:
#   ./run_nav.sh [OPTIONS]
#
# Required (or use defaults):
#   --scenario ID        Scenario ID: S1..S9         (default: S1)
#   --run NUM            Run number, zero-padded       (default: 01)
#
# Optional:
#   --floor FLOOR        Floor identifier              (default: e6)
#   --skip-build         Skip colcon build step
#   --no-skip-build      Force build
#   --cleanup-mode MODE  fast (default) or deep
#   --deep               Shorthand for --cleanup-mode deep
#   --timeout SECONDS    timeout_at_checkpoint         (default: 30.0)
#   --no-hard-kill       Skip force kill on exit
#   --no-bag             Disable bag recording for this run
#   --help, -h           Show this message and exit
#
# Examples:
#   ./run_nav.sh                              # run với bag (defaults: S1, run 01)
#   ./run_nav.sh --no-bag                     # run không ghi bag
#   ./run_nav.sh --scenario S1 --run 01
#   ./run_nav.sh --scenario S3 --run 02 --floor e6 --skip-build
#   ./run_nav.sh --scenario S6 --run 01 --no-bag
#
# Env-var overrides (CLI takes precedence):
#   SCENARIO=S3 RUN_NUMBER=02 ./run_nav.sh
# ==================================================

# ==================================================
# User-configurable variables
# ==================================================
WS="${WS:-$HOME/mbrobot_ws}"
PKG="${PKG:-mobile_robot}"
LAUNCH_FILE="${LAUNCH_FILE:-nav.launch.py}"

# Experiment identity
SCENARIO="${SCENARIO:-S1}"
RUN_NUMBER="${RUN_NUMBER:-01}"
RECORD_BAG="${RECORD_BAG:-true}"

# Nav parameters (inherited)
FLOOR="${FLOOR:-a1}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
TIMEOUT_AT_CHECKPOINT="${TIMEOUT_AT_CHECKPOINT:-30.0}"
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
# Log folder and log file
# ==================================================
BAGS_DIR="$WS/bags"
LOG_DIR="$WS/log_run"
RUN_TS="$(date +%Y%m%d_%H%M%S)"
BAG_NAME="s${SCENARIO}_run${RUN_NUMBER}_${RUN_TS}"
BAG_PATH="$BAGS_DIR/$BAG_NAME"
LOG_FILE="$LOG_DIR/exp_${BAG_NAME}.log"
META_FILE="$BAGS_DIR/${BAG_NAME}_meta.txt"

mkdir -p "$BAGS_DIR" "$LOG_DIR"

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
}

hard_stop_robot_nodes_deep()
{
    hard_stop_robot_nodes_fast
    kill_hard "behavior_server|smoother_server|velocity_smoother|collision_monitor"
    kill_hard "component_container|component_container_mt|component_container_isolated"
    kill_hard "ros2 bag record"
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

cleanup()
{
    local exit_code=$?

    # Cleanup must continue even if one command fails.
    set +e

    # Avoid repeated cleanup calls.
    trap - EXIT
    trap - INT
    trap - TERM

    log ""
    log "=================================================="
    log "[CLEANUP] Start real-robot cleanup"
    log "Cleanup mode: $CLEANUP_MODE"
    log "=================================================="

    case "$CLEANUP_MODE" in
        fast)
            cleanup_fast
            ;;
        deep)
            cleanup_deep
            ;;
        *)
            log "[CLEANUP][WARN] Unknown CLEANUP_MODE='$CLEANUP_MODE'. Falling back to fast."
            cleanup_fast
            ;;
    esac

    wait 2>/dev/null || true

    log "[CLEANUP] Done"
    log "=================================================="
    log "Exit code: $exit_code"
    log "Log file : $LOG_FILE"
    log "=================================================="

    # Update metadata file with final result
    if [ -f "$META_FILE" ]; then
        local result_str="UNKNOWN"
        if [ "$exit_code" -eq 0 ] || [ "$exit_code" -eq 130 ] || [ "$exit_code" -eq 143 ]; then
            result_str="STOPPED_OK"
        else
            result_str="ERROR_$exit_code"
        fi
        sed -i "s/^result=PENDING/result=$result_str/" "$META_FILE" 2>/dev/null || true
        log "Metadata updated   : result=$result_str"
        log "Metadata file      : $META_FILE"
    fi

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
            --scenario)     SCENARIO="$2";    shift ;;
            --run=*)        RUN_NUMBER="${1#--run=}" ;;
            --run)          RUN_NUMBER="$2";  shift ;;
            --no-bag)       RECORD_BAG=false ;;
            --floor=*)        FLOOR="${1#--floor=}" ;;
            --floor)          FLOOR="$2";                    shift ;;
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

# ==================================================
# Trap signals
# ==================================================
trap cleanup EXIT
trap 'log ""; log "[SIGNAL] Ctrl+C/SIGINT received. Stopping real robot launch..."; exit 130' INT
trap 'log ""; log "[SIGNAL] SIGTERM received. Stopping real robot launch..."; exit 143' TERM

# ==================================================
# Startup information
# ==================================================
log "=================================================="
log "AMR Experiment Runner"
log "--------------------------------------------------"
log "Scenario            : $SCENARIO"
log "Run number          : $RUN_NUMBER"
log "Floor               : $FLOOR"
log "Record bag          : $RECORD_BAG"
log "Bag path            : $BAG_PATH"
log "--------------------------------------------------"
log "Run timestamp       : $RUN_TS"
log "Workspace           : $WS"
log "Launch file         : $LAUNCH_FILE"
log "Timeout/checkpoint  : $TIMEOUT_AT_CHECKPOINT s"
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
floor=$FLOOR
bag_name=$BAG_NAME
bag_path=$BAG_PATH
run_ts=$RUN_TS
record_bag=$RECORD_BAG
timeout_at_checkpoint=$TIMEOUT_AT_CHECKPOINT
result=PENDING
fge_manual=
he_manual=
notes=
EOF
log "Metadata → $META_FILE"

# ==================================================
# Compose launch command
# ==================================================
LAUNCH_CMD=(
    ros2 launch "$PKG" "$LAUNCH_FILE"
    "floor:=$FLOOR"
    "use_sim_time:=$USE_SIM_TIME"
    "timeout_at_checkpoint:=$TIMEOUT_AT_CHECKPOINT"
    "us_serial_port:=$US_SERIAL_PORT"
    "us_baud_rate:=$US_BAUD_RATE"
    "scenario:=$SCENARIO"
    "run_number:=$RUN_NUMBER"
    "record_bag:=$RECORD_BAG"
)

if [ -n "$EXTRA_LAUNCH_ARGS" ]; then
    # shellcheck disable=SC2206
    LAUNCH_CMD+=( $EXTRA_LAUNCH_ARGS )
fi

# ==================================================
# Launch real robot navigation
# ==================================================
log ""
log "=================================================="
log "[6] Launch real robot navigation"
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
