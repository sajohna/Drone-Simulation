#!/bin/bash
# ============================================================
# PX4 All-in-One Launcher — WSL (Jazzy / Harmonic)
# Usage: bash px4_launch_with_keyboard.sh
# ============================================================

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
KB_SCRIPT="$HOME/Teleop_script.py"
DRONE_TRACKER="$HOME/drone_tracker.py"
IBVS_SCRIPT="$HOME/ibvs_mpc_controller.py"
ARUCO_SPAWNER="$HOME/aruco_spawner.py"

# Initialization delays
DELAY_PX4=10
DELAY_XRCE=6
DELAY_BRIDGE=5
DELAY_IMAGE=3
DELAY_ARUCO=5
DELAY_KB=5

# ── Preflight checks ─────────────────────────────────────────
if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "[ERROR] gnome-terminal not found."
  exit 1
fi
if [ ! -d "$PX4_DIR" ]; then
  echo "[ERROR] PX4-Autopilot not found at $PX4_DIR"
  echo "        Set PX4_DIR=/path/to/PX4-Autopilot and re-run."
  exit 1
fi
if [ ! -f "$ROS_SETUP" ]; then
  echo "[ERROR] ROS2 Jazzy not found at $ROS_SETUP"
  exit 1
fi

# ── Simulator display mode ───────────────────────────────────
echo ""
echo "============================================================"
echo "  PX4 Launcher — Simulator Display Mode"
echo "============================================================"
echo "  Run Gazebo with a visible GUI window, or headless"
echo "  (no window — lower CPU, required for SSH / no-display)."
echo ""
read -rp "Run Gazebo with GUI? [Y/n]: " HEADLESS_INPUT
HEADLESS_INPUT="${HEADLESS_INPUT,,}"
if [[ "$HEADLESS_INPUT" == "n" || "$HEADLESS_INPUT" == "no" ]]; then
  HEADLESS=1
  HEADLESS_FLAG="HEADLESS=1"
  echo "  ✓ Headless mode — Gazebo GUI will not open."
  echo "    Note: rqt_image_view, QGroundControl, and the trajectory"
  echo "    tracker will still work via ROS2 topics."
else
  HEADLESS=0
  HEADLESS_FLAG=""
  echo "  ✓ GUI mode — Gazebo window will open."
fi
echo ""

# ── Control mode selection ───────────────────────────────────
echo "============================================================"
echo "  PX4 Launcher — Select Control Mode"
echo "============================================================"
echo "  1) Manual keyboard control   (Teleop_script.py)"
echo "  2) IBVS-MPC autonomous mode  (ibvs_mpc_controller.py)"
echo ""
read -rp "Select mode [1/2]: " CONTROL_MODE
echo ""

if [[ "$CONTROL_MODE" == "2" ]]; then
  if [ ! -f "$IBVS_SCRIPT" ]; then
    echo "[ERROR] IBVS controller not found at $IBVS_SCRIPT"
    exit 1
  fi
  echo "  ✓ IBVS-MPC autonomous mode selected"
  echo ""

  # Random spawn config
  read -rp "Randomise drone + marker positions? [Y/n]: " DO_RANDOM
  DO_RANDOM="${DO_RANDOM,,}"
  if [[ "$DO_RANDOM" != "n" && "$DO_RANDOM" != "no" ]]; then
    python3 "$ARUCO_SPAWNER" --no-spawn 2>/dev/null || true

    # Read drone spawn position from config
    SPAWN_CONFIG="$HOME/ibvs_spawn_config.json"
    if [ -f "$SPAWN_CONFIG" ]; then
      DRONE_SPAWN_X=$(python3 -c "import json; d=json.load(open('$SPAWN_CONFIG')); print(d['drone']['x'])" 2>/dev/null || echo "0")
      DRONE_SPAWN_Y=$(python3 -c "import json; d=json.load(open('$SPAWN_CONFIG')); print(d['drone']['y'])" 2>/dev/null || echo "0")
      echo "  Drone will spawn at: ($DRONE_SPAWN_X, $DRONE_SPAWN_Y)"
    else
      DRONE_SPAWN_X="0"
      DRONE_SPAWN_Y="0"
    fi
  else
    DRONE_SPAWN_X="0"
    DRONE_SPAWN_Y="0"
    echo "  Using default positions (drone: 0,0  marker: 3,0)"
  fi
else
  if [ ! -f "$KB_SCRIPT" ]; then
    echo "[ERROR] Keyboard script not found at $KB_SCRIPT"
    exit 1
  fi
  echo "  ✓ Manual keyboard mode selected"
  DRONE_SPAWN_X="0"
  DRONE_SPAWN_Y="0"
fi

# ── Common optional components ───────────────────────────────
echo ""
# In headless mode, Gazebo GUI is suppressed but rqt tools still work
# (they connect via ROS2 topics, not Gazebo directly)
read -rp "Launch rqt_image_view (camera feed)? [y/N]: " LAUNCH_CAMERA
LAUNCH_CAMERA="${LAUNCH_CAMERA,,}"

if [[ "$CONTROL_MODE" == "2" ]]; then
  # IBVS mode: always spawn ArUco (handled after Gazebo is up)
  LAUNCH_ARUCO="y"
  echo "  [IBVS] ArUco marker will be spawned automatically."
else
  read -rp "Spawn ArUco board? [y/N]: " LAUNCH_ARUCO
  LAUNCH_ARUCO="${LAUNCH_ARUCO,,}"
fi

read -rp "Launch Drone Trajectory Tracker? [y/N]: " LAUNCH_TRACKER
LAUNCH_TRACKER="${LAUNCH_TRACKER,,}"
read -rp "Launch QGroundControl? [y/N]: " LAUNCH_QGC
LAUNCH_QGC="${LAUNCH_QGC,,}"
echo ""

# ── Kill stale processes ─────────────────────────────────────
echo "Killing any stale PX4/Gazebo processes..."
pkill -f "bin/px4"        2>/dev/null || true
pkill -f "px4_sitl"       2>/dev/null || true
pkill -f "gz sim"         2>/dev/null || true
pkill -f "ruby.*gz"       2>/dev/null || true
pkill -f "MicroXRCEAgent" 2>/dev/null || true
sleep 2

echo "Starting PX4 simulation stack..."
echo ""

# ── Display env for gnome-terminal children ──────────────────
DISP_ENV="export DISPLAY=$DISPLAY; export WAYLAND_DISPLAY=$WAYLAND_DISPLAY; export XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
ENV="$DISP_ENV; export LIBGL_ALWAYS_SOFTWARE=1; export GZ_VERSION=harmonic; export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models; source $ROS_SETUP 2>/dev/null || true"

# ── 1. PX4 SITL + Gazebo ─────────────────────────────────────
echo "[1/5] Launching PX4 SITL + Gazebo..."

PX4_FIFO="/tmp/px4_commands_$$"
mkfifo "$PX4_FIFO" 2>/dev/null || true

# Pass drone spawn position via environment if randomised
PX4_SPAWN_ARGS=""
if [[ "$DRONE_SPAWN_X" != "0" || "$DRONE_SPAWN_Y" != "0" ]]; then
  PX4_SPAWN_ARGS="PX4_HOME_LAT=0 PX4_HOME_LON=0 PX4_SIM_MODEL=x500_mono_cam_down"
  echo "      Drone spawn offset: X=$DRONE_SPAWN_X Y=$DRONE_SPAWN_Y"
fi

PX4_SH="/tmp/px4_sitl_$$.sh"
cat > "$PX4_SH" << SITLEOF
#!/bin/bash
export DISPLAY=$DISPLAY
export WAYLAND_DISPLAY=$WAYLAND_DISPLAY
export XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models"
source $ROS_SETUP 2>/dev/null || true
echo "[PX4] cd $PX4_DIR"
cd "$PX4_DIR" || { echo "ERROR: cannot cd to $PX4_DIR"; read; exit 1; }
make px4_sitl_default gz_x500_mono_cam_down $HEADLESS_FLAG < $PX4_FIFO
echo "[PX4] Exited. Press Enter to close."
read
SITLEOF
chmod +x "$PX4_SH"
xterm -title "PX4 SITL${HEADLESS:+ (headless)}" -fa "DejaVu Sans Mono" -fs 10 -e bash "$PX4_SH" &

PX4_PID=$!

{
  # Set EKF origin and heading — give EKF 40s to converge before arming
  echo "commander set_ekf_origin 0 0 0"
  sleep 2
  echo "commander set_heading 0"
  sleep 40
  # In IBVS mode, the controller handles arming — skip auto arm/takeoff
  if [[ "$CONTROL_MODE" != "2" ]]; then
    # Retry arm up to 3 times in case EKF needs a little longer
    for _attempt in 1 2 3; do
      echo "commander arm"
      sleep 3
    done
    echo "commander takeoff"
    sleep 2
  fi
  cat
} > "$PX4_FIFO" 2>/dev/null &

FIFO_PID=$!

# ── 2. MicroXRCE Agent ───────────────────────────────────────
echo "[2/5] Launching MicroXRCE Agent..."
SH_20="/tmp/px4_20_$$.sh"
cat > "$SH_20" << XEOF20
#!/bin/bash
export DISPLAY=$DISPLAY
export WAYLAND_DISPLAY=$WAYLAND_DISPLAY
export XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models"
source $ROS_SETUP 2>/dev/null || true
MicroXRCEAgent udp4 -p 8888
echo "Done. Press Enter to close."
read
XEOF20
chmod +x "$SH_20"
xterm -title "XRCE Agent" -fa "DejaVu Sans Mono" -fs 10 -e bash "$SH_20" &
sleep $DELAY_XRCE

# ── 2b. QGroundControl (optional) ────────────────────────────
if [[ "$LAUNCH_QGC" == "y" || "$LAUNCH_QGC" == "yes" ]]; then
  QGC_BIN=""
  for _candidate in \
      "$HOME/QGroundControl.AppImage" \
      "$HOME/Downloads/QGroundControl.AppImage" \
      QGroundControl qgroundcontrol \
      /opt/QGroundControl/QGroundControl; do
    if [[ -f "$_candidate" && -x "$_candidate" ]] || command -v "$_candidate" >/dev/null 2>&1; then
      QGC_BIN="$_candidate"
      break
    fi
  done
  if [[ -n "$QGC_BIN" ]]; then
    echo "[2b] Launching QGroundControl ($QGC_BIN)..."
    SH_QGC="/tmp/px4_qgc_$$.sh"
    cat > "$SH_QGC" << QGCEOF
#!/bin/bash
export DISPLAY=$DISPLAY
export WAYLAND_DISPLAY=$WAYLAND_DISPLAY
export XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
"$QGC_BIN" --appimage-extract-and-run
echo "QGC exited. Press Enter."
read
QGCEOF
    chmod +x "$SH_QGC"
    xterm -title "QGroundControl" -fa "DejaVu Sans Mono" -fs 10 -e bash "$SH_QGC" &
    sleep 3
  else
    echo "[2b] QGroundControl not found — skipping."
    echo "     Download: https://docs.qgroundcontrol.com"
  fi
else
  echo "[2b] Skipping QGroundControl."
fi

# ── 3. ROS-GZ Bridge ─────────────────────────────────────────
echo "[3/5] Launching ROS-GZ Bridge..."
SH_21="/tmp/px4_21_$$.sh"
cat > "$SH_21" << XEOF21
#!/bin/bash
export DISPLAY=$DISPLAY
export WAYLAND_DISPLAY=$WAYLAND_DISPLAY
export XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models"
source $ROS_SETUP 2>/dev/null || true
ros2 run ros_gz_bridge parameter_bridge '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' --ros-args -r /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image:=/camera
echo "Done. Press Enter to close."
read
XEOF21
chmod +x "$SH_21"
xterm -title "ROS-GZ Bridge" -fa "DejaVu Sans Mono" -fs 10 -e bash "$SH_21" &
sleep $DELAY_BRIDGE

# ── 4. rqt_image_view (optional) ─────────────────────────────
if [[ "$LAUNCH_CAMERA" == "y" || "$LAUNCH_CAMERA" == "yes" ]]; then
  echo "[4/5] Launching rqt_image_view..."
  if [[ "$CONTROL_MODE" == "2" ]]; then
    # IBVS mode: show debug image with ArUco overlay
    SH_22="/tmp/px4_22_$$.sh"
cat > "$SH_22" << XEOF22
#!/bin/bash
export DISPLAY=$DISPLAY
export WAYLAND_DISPLAY=$WAYLAND_DISPLAY
export XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models"
source $ROS_SETUP 2>/dev/null || true
echo 'Waiting for IBVS debug image...'
sleep 3
ros2 run rqt_image_view rqt_image_view /ibvs/debug_image
echo "Done. Press Enter to close."
read
XEOF22
chmod +x "$SH_22"
xterm -title "IBVS Debug View" -fa "DejaVu Sans Mono" -fs 10 -e bash "$SH_22" &
  else
    SH_23="/tmp/px4_23_$$.sh"
cat > "$SH_23" << XEOF23
#!/bin/bash
export DISPLAY=$DISPLAY
export WAYLAND_DISPLAY=$WAYLAND_DISPLAY
export XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models"
source $ROS_SETUP 2>/dev/null || true
ros2 run rqt_image_view rqt_image_view /camera
echo "Done. Press Enter to close."
read
XEOF23
chmod +x "$SH_23"
xterm -title "rqt Image View" -fa "DejaVu Sans Mono" -fs 10 -e bash "$SH_23" &
  fi
  sleep $DELAY_IMAGE
else
  echo "[4/5] Skipping rqt_image_view."
fi

# ── 5. ArUco spawn ───────────────────────────────────────────
if [[ "$LAUNCH_ARUCO" == "y" || "$LAUNCH_ARUCO" == "yes" ]]; then
  if [[ "$CONTROL_MODE" == "2" && -f "$ARUCO_SPAWNER" ]]; then
    # IBVS mode: use randomised spawner
    SPAWNER_ARGS=""
    if [[ -n "$RAND_SEED" ]]; then
      SPAWNER_ARGS="--seed $RAND_SEED"
    fi
    echo "[5/5] Spawning ArUco marker (random position) in ${DELAY_ARUCO}s..."
    sleep $DELAY_ARUCO
    SH_24="/tmp/px4_24_$$.sh"
cat > "$SH_24" << XEOF24
#!/bin/bash
export DISPLAY=$DISPLAY
export WAYLAND_DISPLAY=$WAYLAND_DISPLAY
export XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models"
source $ROS_SETUP 2>/dev/null || true
echo 'Running ArUco random spawner...'
python3 $HOME/aruco_spawner.py --remove-existing
echo "Done. Press Enter to close."
read
XEOF24
chmod +x "$SH_24"
xterm -title "ArUco Spawner" -fa "DejaVu Sans Mono" -fs 10 -e bash "$SH_24" &
  else
    # Manual mode: spawn at fixed position
    echo "[5/5] Spawning ArUco board in ${DELAY_ARUCO}s..."
    sleep $DELAY_ARUCO
    SH_25="/tmp/px4_25_$$.sh"
cat > "$SH_25" << XEOF25
#!/bin/bash
export DISPLAY=$DISPLAY
export WAYLAND_DISPLAY=$WAYLAND_DISPLAY
export XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PX4_DIR/Tools/simulation/gz/models"
source $ROS_SETUP 2>/dev/null || true
echo 'Spawning ArUco target...'
gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "/home/sajo/PX4-Autopilot/Tools/simulation/gz/models/visual_servoing_target/model.sdf", name: "aruco_target", pose: {position: {x: 2.0, y: 0.0, z: 0.05}}'
echo 'ArUco spawn command sent.'
echo "Done. Press Enter to close."
read
XEOF25
chmod +x "$SH_25"
xterm -title "ArUco Spawn" -fa "DejaVu Sans Mono" -fs 10 -e bash "$SH_25" &
  fi
else
  echo "[5/5] Skipping ArUco spawn."
fi

# ── 6. Drone Trajectory Tracker (optional) ───────────────────
if [[ "$LAUNCH_TRACKER" == "y" || "$LAUNCH_TRACKER" == "yes" ]]; then
  if [ ! -f "$DRONE_TRACKER" ]; then
    echo "[6/6] drone_tracker.py not found at $DRONE_TRACKER — skipping."
  else
    TRACKER_DELAY=$((DELAY_PX4 + DELAY_XRCE + DELAY_BRIDGE + DELAY_IMAGE + DELAY_ARUCO + 35))
    echo "[6/6] Scheduling Drone Trajectory Tracker in ${TRACKER_DELAY}s..."
    (
      sleep $TRACKER_DELAY
      SH_TRK="/tmp/px4_tracker_$$.sh"
      cat > "$SH_TRK" << TRKEOF
#!/bin/bash
export DISPLAY=$DISPLAY
export WAYLAND_DISPLAY=$WAYLAND_DISPLAY
export XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_VERSION=harmonic
source $ROS_SETUP 2>/dev/null || true
source "$HOME/px4_msgs_ws/install/setup.bash" 2>/dev/null || source "$HOME/colcon_ws/install/setup.bash" 2>/dev/null || true
python3 $DRONE_TRACKER
echo "Done. Press Enter to close."
read
TRKEOF
      chmod +x "$SH_TRK"
      xterm -title "Drone Tracker" -fa "DejaVu Sans Mono" -fs 10 -e bash "$SH_TRK" &
    ) &
  fi
else
  echo "[6/6] Skipping Drone Trajectory Tracker."
fi

# ── 7. Main controller ───────────────────────────────────────
echo ""
echo "============================================================"

if [[ "$CONTROL_MODE" == "2" ]]; then
  # ── IBVS-MPC autonomous mode ─────────────────────────────
  IBVS_DELAY=$((DELAY_XRCE + DELAY_BRIDGE + 5))
  echo "  Starting IBVS-MPC controller in ${IBVS_DELAY}s..."
  echo "============================================================"
  echo ""
  echo "  State machine:"
  echo "    IDLE → TAKEOFF → SEARCH → IBVS_APPROACH"
  echo "    → IBVS_DESCEND → HOVER"
  echo ""
  echo "  Monitor progress:"
  echo "    ros2 topic echo /ibvs/state"
  echo "    ros2 run rqt_image_view rqt_image_view /ibvs/debug_image"
  echo "============================================================"
  echo ""
  sleep $IBVS_DELAY

  export DISPLAY=$DISPLAY
  export WAYLAND_DISPLAY=$WAYLAND_DISPLAY
  export LIBGL_ALWAYS_SOFTWARE=1
  export GZ_VERSION=harmonic
  source "$ROS_SETUP" 2>/dev/null || true
  source "$HOME/px4_msgs_ws/install/setup.bash" 2>/dev/null || true
  source "$HOME/colcon_ws/install/setup.bash"  2>/dev/null || true

  echo "Launching IBVS-MPC controller..."
  python3 "$IBVS_SCRIPT"

else
  # ── Manual keyboard mode ─────────────────────────────────
  echo "  Starting offboard keyboard control in ${DELAY_KB}s..."
  echo "============================================================"
  echo ""
  echo "  w/s = forward/back   a/d = left/right"
  echo "  r/f = up/down        q/e = yaw"
  echo "  t   = takeoff        l   = land    x = exit"
  echo "============================================================"
  sleep $DELAY_KB

  export DISPLAY=$DISPLAY
  export WAYLAND_DISPLAY=$WAYLAND_DISPLAY
  export LIBGL_ALWAYS_SOFTWARE=1
  export GZ_VERSION=harmonic
  source "$ROS_SETUP" 2>/dev/null || true
  source "$HOME/px4_msgs_ws/install/setup.bash" 2>/dev/null || true
  source "$HOME/colcon_ws/install/setup.bash"  2>/dev/null || true

  echo "Launching keyboard control..."
  python3 "$KB_SCRIPT"
fi

# ── Cleanup ──────────────────────────────────────────────────
rm -f /tmp/px4_commands_* 2>/dev/null || true
kill $FIFO_PID 2>/dev/null || true