#!/usr/bin/env python3
"""
Random ArUco Board + Drone Spawner
====================================
Randomises the starting position of both the ArUco target marker
and the drone within configurable bounds, then spawns them via
the Gazebo service API.

Usage:
  python3 aruco_spawner.py [--drone-random] [--seed N]

The drone position is written to a JSON file that the launch script
reads to pass --x --y to px4_sitl.
"""

import argparse
import json
import os
import subprocess
import sys
import time
import random
import tempfile


# ── Spawn area bounds ─────────────────────────────────────────
DRONE_X_RANGE = (-1.0,  1.0)
DRONE_Y_RANGE = (-1.0,  1.0)

MARKER_X_RANGE = (-4.0,  4.0)
MARKER_Y_RANGE = (-4.0,  4.0)
MARKER_Z       = 0.05

MIN_SEPARATION = 2.0

SPAWN_CONFIG_FILE = os.path.expanduser('~/ibvs_spawn_config.json')


def random_positions(seed=None):
    rng = random.Random(seed)
    for attempt in range(100):
        dx = rng.uniform(*DRONE_X_RANGE)
        dy = rng.uniform(*DRONE_Y_RANGE)
        mx = rng.uniform(*MARKER_X_RANGE)
        my = rng.uniform(*MARKER_Y_RANGE)
        sep = ((mx - dx)**2 + (my - dy)**2) ** 0.5
        if sep >= MIN_SEPARATION:
            return (dx, dy), (mx, my)
    return (0.0, 0.0), (3.0, 0.0)


def _build_inline_sdf(name):
    """
    Self-contained SDF for an ArUco-like marker.
    Uses stacked flat boxes with black/white materials —
    no external mesh or texture files needed.
    """
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">

      <!-- White outer border / base plate -->
      <visual name="base_white">
        <geometry>
          <box><size>0.7 0.7 0.005</size></box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
        </material>
      </visual>

      <!-- Black inner pattern (slightly raised so it renders on top) -->
      <visual name="pattern_black">
        <pose>0 0 0.003 0 0 0</pose>
        <geometry>
          <box><size>0.5 0.5 0.002</size></box>
        </geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.05 0.05 0.05 1</diffuse>
          <specular>0 0 0 1</specular>
        </material>
      </visual>

      <!-- White cells to create ArUco-like pattern -->
      <visual name="cell_tl">
        <pose>-0.15 0.15 0.006 0 0 0</pose>
        <geometry><box><size>0.14 0.14 0.002</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
      </visual>
      <visual name="cell_tr">
        <pose>0.15 0.15 0.006 0 0 0</pose>
        <geometry><box><size>0.14 0.14 0.002</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
      </visual>
      <visual name="cell_bl">
        <pose>-0.15 -0.15 0.006 0 0 0</pose>
        <geometry><box><size>0.14 0.14 0.002</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
      </visual>
      <visual name="cell_c">
        <pose>0 0 0.006 0 0 0</pose>
        <geometry><box><size>0.14 0.14 0.002</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
      </visual>

      <collision name="col">
        <geometry><box><size>0.7 0.7 0.01</size></box></geometry>
      </collision>

    </link>
  </model>
</sdf>"""


def _write_temp_sdf(name):
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', prefix='aruco_', delete=False)
    tmp.write(_build_inline_sdf(name))
    tmp.close()
    return tmp.name


def spawn_marker(x, y, z=MARKER_Z, world='default'):
    """
    Spawn the ArUco marker via gz service.

    Strategy:
      1. Use PX4 bundled model if found AND its parent dir is on
         GZ_SIM_RESOURCE_PATH (so Gazebo can find textures/meshes).
      2. Otherwise write an inline SDF to a temp file — always visible.
    """
    px4_model = os.path.expanduser(
        '~/PX4-Autopilot/Tools/simulation/gz/models/'
        'visual_servoing_target/model.sdf')
    px4_models_dir = os.path.expanduser(
        '~/PX4-Autopilot/Tools/simulation/gz/models')

    # Always inject the PX4 models dir into GZ_SIM_RESOURCE_PATH so
    # Gazebo can resolve the model's meshes/textures when using sdf_filename
    gz_resource = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if px4_models_dir not in gz_resource:
        os.environ['GZ_SIM_RESOURCE_PATH'] = (
            px4_models_dir + ':' + gz_resource).rstrip(':')
        gz_resource = os.environ['GZ_SIM_RESOURCE_PATH']

    use_px4 = os.path.isfile(px4_model)

    if use_px4:
        req = (
            f'sdf_filename: "{px4_model}", '
            f'name: "aruco_target", '
            f'pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}}}'
        )
        print(f'[SPAWN] Using PX4 bundled model.')
    else:
        sdf_path = _write_temp_sdf('aruco_target')
        req = (
            f'sdf_filename: "{sdf_path}", '
            f'name: "aruco_target", '
            f'pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}}}'
        )
        reason = ('GZ_SIM_RESOURCE_PATH missing model dir'
                  if os.path.isfile(px4_model) else 'PX4 model not found')
        print(f'[SPAWN] Using inline SDF ({reason}).')
        print(f'[SPAWN] Temp SDF: {sdf_path}')

    print(f'[SPAWN] ArUco marker at ({x:.2f}, {y:.2f}, {z:.2f})')

    cmd = [
        'gz', 'service',
        '-s', f'/world/{world}/create',
        '--reqtype',  'gz.msgs.EntityFactory',
        '--reptype',  'gz.msgs.Boolean',
        '--timeout',  '5000',
        '--req',      req,
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if 'true' in result.stdout.lower() or result.returncode == 0:
            print('[SPAWN] Marker spawned successfully.')
            return True
        else:
            print(f'[SPAWN] Spawn failed.')
            if result.stderr.strip():
                print(f'         {result.stderr.strip()}')
            return False
    except subprocess.TimeoutExpired:
        print('[SPAWN] gz service timed out — is Gazebo running?')
        return False
    except FileNotFoundError:
        print('[SPAWN] gz command not found. Is Gazebo Harmonic installed?')
        return False


def remove_existing_marker(world='default'):
    """Remove existing aruco_target — silently ignores 'not found' errors."""
    cmd = [
        'gz', 'service',
        '-s', f'/world/{world}/remove',
        '--reqtype',  'gz.msgs.Entity',
        '--reptype',  'gz.msgs.Boolean',
        '--timeout',  '3000',
        '--req',      'name: "aruco_target" type: MODEL',
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, timeout=5)
        # 'not found' is expected on first run — not an error
        if result.returncode != 0 and 'not found' not in result.stderr.lower():
            print(f'[SPAWN] Note: remove returned: {result.stderr.strip()[:80]}')
    except Exception:
        pass


def write_spawn_config(drone_pos, marker_pos):
    config = {
        'drone':  {'x': drone_pos[0],  'y': drone_pos[1]},
        'marker': {'x': marker_pos[0], 'y': marker_pos[1], 'z': MARKER_Z},
    }
    with open(SPAWN_CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=2)
    print(f'[CONFIG] Spawn config written to {SPAWN_CONFIG_FILE}')
    return config


def print_summary(drone_pos, marker_pos):
    dx, dy = drone_pos
    mx, my = marker_pos
    sep = ((mx - dx)**2 + (my - dy)**2) ** 0.5
    print()
    print('='*55)
    print('  Random Spawn Configuration')
    print('='*55)
    print(f'  Drone  start : ({dx:+.2f}, {dy:+.2f}) m')
    print(f'  Marker pos   : ({mx:+.2f}, {my:+.2f}) m')
    print(f'  Separation   : {sep:.2f} m')
    print('='*55)
    print()


def main():
    parser = argparse.ArgumentParser(
        description='Randomise drone and ArUco marker spawn positions')
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--marker-x', type=float, default=None)
    parser.add_argument('--marker-y', type=float, default=None)
    parser.add_argument('--drone-x',  type=float, default=None)
    parser.add_argument('--drone-y',  type=float, default=None)
    parser.add_argument('--world', type=str, default='default')
    parser.add_argument('--no-spawn', action='store_true',
                        help='Write config only, do not call gz service')
    parser.add_argument('--remove-existing', action='store_true',
                        help='Remove existing aruco_target before spawning')
    args = parser.parse_args()

    drone_pos, marker_pos = random_positions(seed=args.seed)

    if args.drone_x  is not None: drone_pos  = (args.drone_x,  drone_pos[1])
    if args.drone_y  is not None: drone_pos  = (drone_pos[0],  args.drone_y)
    if args.marker_x is not None: marker_pos = (args.marker_x, marker_pos[1])
    if args.marker_y is not None: marker_pos = (marker_pos[0], args.marker_y)

    print_summary(drone_pos, marker_pos)
    write_spawn_config(drone_pos, marker_pos)

    if args.no_spawn:
        return 0

    if args.remove_existing:
        print('[SPAWN] Removing existing aruco_target...')
        remove_existing_marker(world=args.world)
        time.sleep(1)

    ok = spawn_marker(marker_pos[0], marker_pos[1], world=args.world)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())