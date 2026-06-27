# Autonomous Drone Simulation — IBVS Control Framework

> **Image-Based Visual Servoing (IBVS) for fully autonomous ArUco marker tracking and hovering**
> Simulation validation of: *Gu & Shen, "Robust NMPC for Uncalibrated IBVS Control of AUVs", IEEE Control Systems Letters, Vol. 8, 2024* — adapted to quadrotor / NED frame.

---

## Overview

This project implements and validates an **Image-Based Visual Servoing (IBVS)** control framework for autonomous UAV navigation in simulation. The drone takes off, searches for a randomized **ArUco marker** on the ground using an expanding spiral, locks on via real-time camera feedback, descends, and hovers above the marker — **entirely without GPS**, relying only on visual and odometric feedback.

**State machine:**

```
IDLE → TAKEOFF → SEARCH → IBVS_APPROACH → IBVS_DESCEND → HOVER → LAND
```

The project is supervised research under **Prof. Chao**, aimed at reproducing academic simulation results and validating the IBVS control law in a realistic physics-based environment.
---

## Stack

| Component | Tool / Version |
|---|---|
| Flight controller | PX4 SITL v1.17.0-alpha |
| Physics simulation | Gazebo Harmonic |
| Middleware | ROS2 Jazzy |
| Computer vision | OpenCV + ArUco |
| Language | Python 3 |
| Environment | WSL2 + WSLg (Ubuntu 24.04) |

---

## Repository Structure

```
Drone-Simulation/
├── ibvs_mpc_controller.py     # Main ROS2 controller node (state machine + IBVS control law)
├── drone_tracker.py           # Real-time 3D trajectory visualizer (matplotlib FuncAnimation)
├── aruco_spawner.py           # Spawns a randomized ArUco marker into the Gazebo world
├── Teleop_script.py           # Manual keyboard teleoperation override
├── model.sdf                  # Gazebo SDF model for the ArUco marker
└── px4_launch_with_keyboard.sh # Master launch script — starts PX4, Gazebo, ROS2 bridge, and all nodes
```

---

## How It Works

### Control Architecture

The controller uses a **proportional IBVS law** with 4 corner points of the ArUco marker as visual features:

```
s ∈ ℝ⁸  →  normalised image coordinates of 4 marker corners
e = s - s*  →  visual error
v = -λ · L⁺ · e  →  velocity command [vx, vy, vz, yaw_rate]
```

Where `L` is the **image Jacobian** (analytically initialised, Broyden-updated online) and `λ` is a scalar gain. The Image Jacobian maps image-space errors to camera-frame velocities, bridging vision and control.

### State Descriptions

**TAKEOFF** — Climbs at 0.8 m/s until reaching the configured takeoff altitude (~3 m AGL).

**SEARCH** — Expands an outward spiral while ascending, running ArUco detection on every frame. Transitions the moment the marker is found.

**IBVS_APPROACH** — Drives the marker centroid to the image center using a proportional lateral law (`lam=1.5`), with an active P-controller altitude hold. Transitions to DESCEND once lateral error `< 0.10` (normalised).

**IBVS_DESCEND** — Simultaneously centers the marker and descends toward the hover altitude using the full IBVS velocity command.

**HOVER** — Holds position above the marker. Altitude is maintained via a P-controller; lateral hold engages if the marker disappears momentarily.

### Key Engineering Decisions

- **Stale-frame guard** — The camera (~10 Hz) can deliver the same frame across multiple 10 Hz control ticks. Reprocessing identical frames causes duration-multiplied velocity commands and oscillation. Fixed by tracking `img_count` and skipping IBVS updates on unchanged frames.

- **Dropout tolerance** — A single missed detection used to immediately reset back to SEARCH, discarding the learned Jacobian. Now requires **2.5 seconds** of consecutive dropout before abandoning the marker.

- **Gate reachability** — The HOVER transition gate (`s_star`) was originally hardcoded assuming a 2.5 m hover depth. At the actual 1 m hover altitude, `||s - s*||` could never satisfy the threshold. Fixed by deriving `s_star` dynamically from `hover_altitude` and `marker_size`.

- **Altitude dead-band consistency** — The altitude dead-band and the `at_hover_alt` threshold were mismatched, creating an unreachable gap. Narrowed to be consistent.

- **NMPC+Broyden abandoned** — An earlier NMPC path using scenario-based min-max optimisation with online Broyden Jacobian updates was found to be fundamentally broken: the Jacobian becomes singular at `v=0` after the first correction (`cond ≈ 1e15`). The proportional IBVS law is used instead for all active states.

---

## Prerequisites

- Ubuntu 22.04 / 24.04 (or WSL2 with WSLg for display)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- [PX4 Autopilot](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html) (built from source)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install)
- [px4_msgs](https://github.com/PX4/px4_msgs) built in a colcon workspace
- Python packages: `opencv-contrib-python`, `numpy`, `scipy`, `matplotlib`

```bash
pip install opencv-contrib-python numpy scipy matplotlib
```

---

## Setup

### 1. Build the px4_msgs workspace

```bash
mkdir -p ~/px4_msgs_ws/src && cd ~/px4_msgs_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/px4_msgs_ws
colcon build
source install/setup.bash
```

### 2. Clone this repository

```bash
git clone https://github.com/sajohna/Drone-Simulation.git
cd Drone-Simulation
```

### 3. Source your workspace

The controller will check for `px4_msgs` on `LD_LIBRARY_PATH` and print an error if not found:

```bash
source ~/px4_msgs_ws/install/setup.bash
```

---

## Running the Simulation

The launch script starts everything in coordinated `xterm` windows:

```bash
chmod +x px4_launch_with_keyboard.sh
./px4_launch_with_keyboard.sh
```

This will open separate terminals for:
- PX4 SITL (headless)
- Gazebo Harmonic
- XRCE-DDS Agent (PX4 ↔ ROS2 bridge)
- ROS2 ↔ Gazebo topic bridge
- `aruco_spawner.py`
- `ibvs_mpc_controller.py`
- `drone_tracker.py`

### Manual teleop override

```bash
python3 Teleop_script.py
```

Use keyboard arrow keys to override velocity commands during any state.

### Monitor progress

```bash
# Watch state machine transitions
ros2 topic echo /ibvs/state

# View camera with IBVS overlay
ros2 run rqt_image_view rqt_image_view /ibvs/debug_image

# Check controller logs
ros2 topic hz /fmu/out/vehicle_odometry
```

---

## Configuration

All tunable parameters are in the `Config` dataclass at the top of `ibvs_mpc_controller.py`:

```python
# Flight
takeoff_altitude: float = -3.0    # NED [m]
hover_altitude:   float = -3.0    # NED [m] (1 m above ground)
hover_threshold:  float = 0.12    # normalised image error

# Camera (x500_mono_cam_down defaults)
fx: float = 554.254690
fy: float = 554.254690
marker_size: float = 0.6          # ArUco marker side [m]

# Control
control_hz: float = 10.0
v_max: [1.5, 1.5, 1.0, 0.4]      # [vx, vy, vz, yaw] m/s
```

---

## ArUco Marker

The marker is spawned at a randomized position in the world by `aruco_spawner.py`. The controller uses `DICT_4X4_50` by default, with a fallback scan across `4×4_100`, `5×5_50`, `6×6_50`, and `ARUCO_ORIGINAL` if the primary detection fails.

The SDF model (`model.sdf`) can be customized for different marker sizes or textures.

---

## Known Limitations

- The fallback ArUco dictionary scan (4 extra detector passes on a 2× upscaled image) introduces latency spikes (~4–6 s per 10-tick interval) on primary detection failure. This is a known performance bottleneck and is being addressed.
- The simulation is validated in WSL2 with WSLg. Native Linux will have lower latency. Windows-native Gazebo is not supported.
- `VehicleLocalPosition` message type fails to resolve in some PX4 build configurations; position is read from `VehicleOdometry` instead.

---

## Acknowledgements

Based on: **Gu, S. & Shen, Y. (2024)**. *Robust NMPC for Uncalibrated Image-Based Visual Servoing Control of AUVs.* IEEE Control Systems Letters, Vol. 8. Adapted to quadrotor dynamics and NED coordinate frame.

Research supervised by **Prof. Chao**.
