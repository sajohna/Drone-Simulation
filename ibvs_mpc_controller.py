#!/usr/bin/env python3
"""
IBVS-MPC Autonomous Drone Controller
=====================================
Implements Image-Based Visual Servoing with Model Predictive Control
for autonomous ArUco marker tracking and hovering.

State machine:
  IDLE → TAKEOFF → SEARCH → IBVS_DESCEND → HOVER

Based on: Gu & Shen, "Robust NMPC for Uncalibrated IBVS Control of AUVs",
IEEE Control Systems Letters, Vol. 8, 2024  (adapted for drone/NED frame)

ROS2 topics consumed:
  /fmu/out/vehicle_status          (VehicleStatus)
  /fmu/out/vehicle_odometry        (VehicleOdometry)   -- position + velocity
  /camera                          (sensor_msgs/Image)

ROS2 topics published:
  /fmu/in/offboard_control_mode    (OffboardControlMode)
  /fmu/in/trajectory_setpoint      (TrajectorySetpoint)
  /fmu/in/vehicle_command          (VehicleCommand)
  /ibvs/debug_image                (sensor_msgs/Image)
  /ibvs/state                      (std_msgs/String)
"""

import sys
import os

# ── Verify the workspace is properly sourced ──────────────────
# px4_msgs .so type-support files must be on LD_LIBRARY_PATH,
# which only happens after sourcing the workspace setup.bash.
_WS_CANDIDATES = [
    os.path.expanduser('~/px4_msgs_ws/install/setup.bash'),
    os.path.expanduser('~/colcon_ws/install/setup.bash'),
]

def _ld_has_px4_msgs():
    return 'px4_msgs' in os.environ.get('LD_LIBRARY_PATH', '')

if not _ld_has_px4_msgs():
    print("\n[ERROR] px4_msgs type support libraries not found in LD_LIBRARY_PATH.")
    print("You must source your workspace before running this script:\n")
    for ws in _WS_CANDIDATES:
        if os.path.isfile(ws):
            print(f"  source {ws}")
    print("  python3 ibvs_mpc_controller.py\n")
    print("The launch script sources this automatically — use it instead of")
    print("running ibvs_mpc_controller.py directly.")
    sys.exit(1)

try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLocalPosition,
        VehicleStatus,
        VehicleOdometry,
    )
except ModuleNotFoundError:
    print("\n[ERROR] px4_msgs Python package not found even though LD_LIBRARY_PATH looks correct.")
    print("Try re-sourcing your workspace and running again.")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.optimize import minimize, Bounds
import threading
import time
import math
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import warnings
warnings.filterwarnings('ignore')


# ══════════════════════════════════════════════════════════════
#  Configuration
# ══════════════════════════════════════════════════════════════

@dataclass
class Config:
    # ── Camera (x500_mono_cam_down, Gazebo defaults) ──────────
    img_width:   int   = 640
    img_height:  int   = 480
    fx:          float = 554.254690   # focal length x [px]
    fy:          float = 554.254690   # focal length y [px]
    cx:          float = 320.0        # principal point x
    cy:          float = 240.0        # principal point y
    marker_size: float = 0.6          # ArUco marker side length [m]

    # ── ArUco ─────────────────────────────────────────────────
    aruco_dict_id: int = aruco.DICT_4X4_50

    # ── Flight parameters ─────────────────────────────────────
    takeoff_altitude:  float = -3.0   # NED [m], negative = up
    search_altitude:   float = -10.0  # ascend to here if marker not seen
    hover_altitude:    float = -3.0   # final hover altitude NED [m] (1 m above ground/marker)
    hover_threshold:   float = 0.12   # image error [m] to declare hover
    search_spiral_r:   float = 0.8    # spiral search radius increment per loop [m]

    # ── MPC parameters ────────────────────────────────────────
    Tc:       float = 0.1    # sampling period [s]
    N:        int   = 5      # prediction horizon
    Nd:       int   = 5      # disturbance scenarios
    alpha:    float = 1.0    # Broyden update rate
    lam_max:  float = 8.0    # max lambda (< 1/Tc = 10)
    w_bound:  float = 0.003  # disturbance bound [m]

    # Velocity constraints [vx, vy, vz, vyaw] (body-ish NED)
    v_max: np.ndarray = field(
        default_factory=lambda: np.array([1.5, 1.5, 1.0, 0.4]))
    v_min: np.ndarray = field(
        default_factory=lambda: np.array([-1.5, -1.5, -1.0, -0.4]))

    # Visibility constraints on normalised image coords [m]
    s_max: float =  0.45
    s_min: float = -0.45

    # MPC weights
    Q_diag: float = 1e4
    R_diag: float = 1.0
    P_diag: float = 50.0

    # ── Control loop ──────────────────────────────────────────
    control_hz:   float = 10.0
    arm_after_ticks: int = 15   # stream setpoints before arming


CFG = Config()


# ══════════════════════════════════════════════════════════════
#  State machine
# ══════════════════════════════════════════════════════════════

class State(Enum):
    IDLE            = auto()   # waiting for /arm command
    TAKEOFF         = auto()   # climbing to takeoff_altitude
    SEARCH          = auto()   # ascending + spiral until marker visible
    IBVS_APPROACH   = auto()   # IBVS driving marker to image centre
    IBVS_DESCEND    = auto()   # descending while keeping marker centred
    HOVER           = auto()   # holding position above marker
    LAND            = auto()   # landing


STATE_NAMES = {s: s.name for s in State}


# ══════════════════════════════════════════════════════════════
#  Image Jacobian & MPC helpers  (single-λ uncalibrated NMPC)
# ══════════════════════════════════════════════════════════════

def image_jacobian(s_norm: np.ndarray, Z: float, f: float) -> np.ndarray:
    """
    Build 8×4 image Jacobian for 4 feature points.
    s_norm : (8,) normalised image coords [m] = (px - cx)/fx etc.
    Z      : estimated depth [m]
    f      : focal length [m] ≈ 1.0 for normalised coords

    We control only [vx, vy, vz, vyaw] (4-DOF), so L is (8×4).
    Full 6-DOF L columns are [vx, vy, vz, p, q, r]; we drop p, q.
    """
    f_n = 1.0   # normalised focal length
    L = np.zeros((8, 4))
    for i in range(4):
        m, n = s_norm[2*i], s_norm[2*i+1]
        # Row for m: [-f/Z, 0, m/Z, n]  → cols [vx, vy, vz, r]
        # Row for n: [0, -f/Z, n/Z, -m] → cols [vx, vy, vz, r]
        row_m = np.array([-f_n/Z,    0.0,  m/Z,  n])
        row_n = np.array([ 0.0,  -f_n/Z,  n/Z, -m])
        L[2*i,   :] = row_m
        L[2*i+1, :] = row_n
    return L


def broyden_update(L_hat: np.ndarray, ds: np.ndarray,
                   dv: np.ndarray, alpha: float = 1.0) -> np.ndarray:
    """Rank-1 Broyden update of Jacobian estimate (Eq. 7 in paper)."""
    norm2 = dv @ dv
    if norm2 < 1e-10:
        return L_hat.copy()
    residual = ds - L_hat @ dv
    return L_hat + alpha * np.outer(residual, dv) / norm2


class SingleLambdaMPC:
    """
    Single-λ uncalibrated min-max NMPC-IBVS (Section III-C-1 of paper).
    Optimises scalar λ ∈ [0, λ_max] per control step.
    Control law: v = -λ L_hat^+ e

    Adapted from AUV 6-DOF to drone 4-DOF [vx, vy, vz, vyaw].
    """

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.N   = cfg.N
        self.Nd  = cfg.Nd
        self.Tc  = cfg.Tc
        self.alpha = cfg.alpha

        self.Q = np.eye(8)  * cfg.Q_diag
        self.R = np.eye(4)  * cfg.R_diag
        self.P = np.eye(8)  * cfg.P_diag

        self.rng = np.random.default_rng(42)

        # Jacobian estimate — initialised on first detection
        self.L_hat: Optional[np.ndarray] = None
        self._prev_s: Optional[np.ndarray] = None
        self._prev_v: Optional[np.ndarray] = None

    # ── Jacobian initialisation & online update ───────────────

    def init_jacobian(self, s: np.ndarray, Z: float):
        """Initialise L_hat from analytical Jacobian (Sec III-B)."""
        self.L_hat = image_jacobian(s, Z, 1.0)
        self._prev_s = s.copy()
        self._prev_v = np.zeros(4)

    def update_jacobian(self, s_new: np.ndarray, v_applied: np.ndarray):
        """
        Broyden update after each control step.

        Guards against near-zero v_applied: when the commanded velocity is
        tiny, dv = Tc*v ≈ 0, and the rank-1 Broyden update degenerates to
        outer(ds, dv)/||dv||^2 → ∞, immediately making L_hat singular
        (cond ≈ 1e15). This was the primary cause of v→0 in APPROACH.
        Skip the update entirely if ||v|| is below a safe minimum.

        NOTE: dv intentionally uses the small nominal control period
        self.Tc, NOT the real elapsed wall-clock time (see earlier comments).
        """
        if self._prev_s is None or self.L_hat is None:
            return
        # Skip if velocity is too small — Broyden is meaningless and dangerous
        if np.linalg.norm(v_applied) < 0.05:
            self._prev_s = s_new.copy()
            return
        ds   = s_new - self._prev_s
        dv   = self.Tc * v_applied
        L_new = broyden_update(self.L_hat, ds, dv, self.alpha)
        # Reject update if it makes L_hat ill-conditioned
        if np.linalg.cond(L_new) < 1e8:
            self.L_hat = L_new
        self._prev_s = s_new.copy()
        self._prev_v = v_applied.copy()

    # ── Scenario-based min-max optimisation ──────────────────

    def _sample_scenarios(self) -> List[np.ndarray]:
        return [self.rng.uniform(-self.cfg.w_bound, self.cfg.w_bound, 8)
                for _ in range(self.Nd)]

    def _rollout_cost(self, lam: float, s0: np.ndarray,
                      L0: np.ndarray, w_seq: np.ndarray,
                      s_star: np.ndarray = None) -> float:
        """Cost for one disturbance scenario."""
        s = s0.copy()
        L = L0.copy()
        J = 0.0
        if s_star is None:
            s_star = np.zeros(8)

        for k in range(self.N):
            e = s - s_star
            L_pinv = np.linalg.pinv(L)
            v = -lam * L_pinv @ e
            v = np.clip(v, self.cfg.v_min, self.cfg.v_max)
            J += float(e @ self.Q @ e + v @ self.R @ v)
            ds = self.Tc * L @ v + w_seq
            dv = self.Tc * v
            s  = s + ds
            L  = broyden_update(L, ds, dv, self.alpha)

        e_N = s - s_star
        J  += float(e_N @ self.P @ e_N)
        return J

    def compute_velocity(self, s: np.ndarray, Z: float,
                         s_star: np.ndarray = None) -> Tuple[np.ndarray, float]:
        """
        Run single-λ min-max NMPC.
        Returns (v_cmd [vx,vy,vz,vyaw], lambda_opt).
        s_star: desired feature vector (8,). Defaults to np.zeros(8) if None.
        """
        if self.L_hat is None:
            self.init_jacobian(s, Z)

        if s_star is None:
            s_star = np.zeros(8)

        scenarios = self._sample_scenarios()
        L0 = self.L_hat.copy()

        def objective(lam_arr):
            lam = float(lam_arr[0])
            worst = max(self._rollout_cost(lam, s, L0, w, s_star) for w in scenarios)
            return worst

        bounds = Bounds([0.0], [self.cfg.lam_max])
        res = minimize(objective, [1.0], method='L-BFGS-B', bounds=bounds,
                       options={'maxiter': 80, 'ftol': 1e-5})
        lam_opt = float(res.x[0])

        e     = s - s_star
        L_pinv = np.linalg.pinv(L0)
        v_cmd = -lam_opt * L_pinv @ e
        v_cmd = np.clip(v_cmd, self.cfg.v_min, self.cfg.v_max)

        return v_cmd, lam_opt


# ══════════════════════════════════════════════════════════════
#  ArUco detector
# ══════════════════════════════════════════════════════════════

class ArucoDetector:
    """
    Detects ArUco markers and computes normalised image feature errors.
    Uses 4 corner points as IBVS features (s ∈ R^8).
    """

    def __init__(self, cfg: Config):
        self.cfg  = cfg
        self.dict = aruco.getPredefinedDictionary(cfg.aruco_dict_id)
        self.params = aruco.DetectorParameters()

        # Relax parameters for small/distant/low-contrast markers
        self.params.adaptiveThreshWinSizeMin    = 3
        self.params.adaptiveThreshWinSizeMax    = 53
        self.params.adaptiveThreshWinSizeStep   = 4
        self.params.adaptiveThreshConstant      = 7
        self.params.minMarkerPerimeterRate      = 0.01  # as small as 1% of image
        self.params.maxMarkerPerimeterRate      = 0.5
        self.params.polygonalApproxAccuracyRate = 0.08
        self.params.minCornerDistanceRate       = 0.01
        self.params.minDistanceToBorder         = 1
        self.params.cornerRefinementMethod      = aruco.CORNER_REFINE_SUBPIX

        self.detector = aruco.ArucoDetector(self.dict, self.params)

        self.K = np.array([
            [cfg.fx,    0.0, cfg.cx],
            [   0.0, cfg.fy, cfg.cy],
            [   0.0,    0.0,    1.0],
        ])
        self.dist = np.zeros((4, 1))

        # Desired feature positions: marker corners centred in image at the
        # ACTUAL hover depth → normalised half-extent = (marker_size/2) / hover_Z.
        #
        # Previously this was a hardcoded `half = 0.12`, which implies a hover
        # depth of marker_size/2/0.12 = 2.5 m — but CFG.hover_altitude targets
        # 1.0 m above ground. At the real hover altitude, even a PERFECTLY
        # centred marker has corners at half-extent (0.3/1.0)=0.30, giving
        # ||s_norm - s_star|| ≈ 0.31 against the old s_star — far above
        # hover_threshold (0.08), so the DESCEND→HOVER gate could never be
        # satisfied and the drone idled at alt_err≈0.25-0.3m until the
        # marker eventually left the FOV. Deriving half-extent from the
        # actual configured hover depth fixes this at the source.
        hover_Z = abs(cfg.hover_altitude)             # e.g. 1.0 m
        half = (cfg.marker_size / 2.0) / hover_Z       # = 0.30 at hover_Z=1.0
        self.s_star = np.array([
             half,  half,
             half, -half,
            -half, -half,
            -half,  half,
        ])

    def detect(self, img_bgr: np.ndarray
               ) -> Tuple[bool, np.ndarray, float, np.ndarray]:
        """
        Returns (found, s_norm, Z_est, debug_img).
        s_norm : (8,) normalised corner coords
        Z_est  : estimated depth from pose estimation [m]
        """
        debug = img_bgr.copy()

        # Upscale + CLAHE contrast boost so small markers are detectable
        h, w = img_bgr.shape[:2]
        scale = 2 if w <= 640 else 1
        if scale > 1:
            proc = cv2.resize(img_bgr, (w * scale, h * scale),
                              interpolation=cv2.INTER_LINEAR)
        else:
            proc = img_bgr.copy()

        gray = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        gray_eq = clahe.apply(gray)
        proc_eq = cv2.cvtColor(gray_eq, cv2.COLOR_GRAY2BGR)

        corners, ids, _ = self.detector.detectMarkers(proc_eq)

        # Scale corner coordinates back to original image space
        if ids is not None and scale > 1:
            corners = [c / scale for c in corners]

        # Try other common dicts if primary fails
        # (handles case where spawned model uses a different dict)
        if ids is None or len(ids) == 0:
            for fb_id in (aruco.DICT_4X4_100, aruco.DICT_5X5_50,
                          aruco.DICT_6X6_50, aruco.DICT_ARUCO_ORIGINAL):
                fb_det = aruco.ArucoDetector(
                    aruco.getPredefinedDictionary(fb_id), self.params)
                corners_fb, ids_fb, _ = fb_det.detectMarkers(proc_eq)
                if ids_fb is not None and len(ids_fb) > 0:
                    corners = [c / scale for c in corners_fb] if scale > 1 else corners_fb
                    ids = ids_fb
                    break

        if ids is None or len(ids) == 0:
            cv2.putText(debug, 'No marker', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return False, np.zeros(8), 1.0, debug

        # Use first detected marker
        idx    = 0
        corner = corners[idx][0]   # (4, 2) pixel coords

        # ── Pose estimation ──────────────────────────────────
        obj_pts = np.array([
            [-self.cfg.marker_size/2,  self.cfg.marker_size/2, 0],
            [ self.cfg.marker_size/2,  self.cfg.marker_size/2, 0],
            [ self.cfg.marker_size/2, -self.cfg.marker_size/2, 0],
            [-self.cfg.marker_size/2, -self.cfg.marker_size/2, 0],
        ], dtype=np.float32)

        ret, rvec, tvec = cv2.solvePnP(
            obj_pts, corner.astype(np.float32),
            self.K, self.dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        Z_est = float(tvec[2]) if ret else 2.0
        Z_est = max(Z_est, 0.3)   # safety clamp

        # ── Normalised feature coordinates ───────────────────
        s_norm = np.zeros(8)
        for i, (px, py) in enumerate(corner):
            s_norm[2*i]   = (px - self.cfg.cx) / self.cfg.fx
            s_norm[2*i+1] = (py - self.cfg.cy) / self.cfg.fy

        # ── Debug overlay ────────────────────────────────────
        aruco.drawDetectedMarkers(debug, corners, ids)
        cx_px = int(corner[:, 0].mean())
        cy_px = int(corner[:, 1].mean())
        cv2.circle(debug, (cx_px, cy_px), 5, (0, 255, 0), -1)
        cv2.circle(debug, (self.cfg.img_width//2, self.cfg.img_height//2),
                   5, (255, 0, 0), 2)
        cv2.line(debug,
                 (self.cfg.img_width//2, self.cfg.img_height//2),
                 (cx_px, cy_px), (0, 255, 255), 1)
        cv2.putText(debug, f'Z={Z_est:.2f}m  |e|={np.linalg.norm(s_norm):.3f}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return True, s_norm, Z_est, debug


# ══════════════════════════════════════════════════════════════
#  Main ROS2 Node
# ══════════════════════════════════════════════════════════════

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class IBVSMPCController(Node):

    def __init__(self):
        super().__init__('ibvs_mpc_controller')

        # ── ROS2 publishers ──────────────────────────────────
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)
        self.debug_img_pub = self.create_publisher(
            Image, '/ibvs/debug_image', 10)
        self.state_pub = self.create_publisher(
            String, '/ibvs/state', 10)

        # ── ROS2 subscriptions ───────────────────────────────
        # NOTE: VehicleLocalPosition subscription removed — that message
        # type fails to resolve in this environment ("message type
        # 'px4_msgs/msg/VehicleLocalPosition' is invalid"), which silently
        # starved self.pos of updates and caused TAKEOFF to never
        # complete. Position is now read from VehicleOdometry (_odom_cb
        # below), which is confirmed to work.
        self.create_subscription(VehicleStatus,
            '/fmu/out/vehicle_status', self._status_cb, PX4_QOS)
        self.create_subscription(VehicleOdometry,
            '/fmu/out/vehicle_odometry', self._odom_cb, PX4_QOS)
        self.create_subscription(Image,
            '/camera', self._image_cb, 10)

        # ── State ────────────────────────────────────────────
        self.state        = State.IDLE
        self.pos          = np.zeros(3)      # NED [m]
        self.vel          = np.zeros(3)
        self.yaw          = 0.0
        self.armed        = False
        self.nav_state    = -1
        self.latest_image: Optional[np.ndarray] = None
        self._img_lock    = threading.Lock()
        self._tick        = 0
        self._last_tick_s = 0.0   # wall-clock time of previous _control_loop call
        self._real_dt     = CFG.Tc  # actual measured inter-tick interval (updated each tick)
        self._pos_origin: Optional[np.ndarray] = None   # set on first pos msg
        self._img_count      = 0    # camera health counter (incremented per new frame)
        self._img_warn_tick  = 0    # last tick we warned about no camera
        self._last_processed_img_count = 0  # img_count at last IBVS computation

        # ── IBVS-MPC components ──────────────────────────────
        self.mpc      = SingleLambdaMPC(CFG)
        self.detector = ArucoDetector(CFG)

        # ── Search spiral ────────────────────────────────────
        self._search_angle = 0.0
        self._search_r     = 0.0
        self._search_origin: Optional[np.ndarray] = None

        # ── Hover bookkeeping ─────────────────────────────────
        self._hover_hold_count = 0

        # ── Detection dropout tolerance ───────────────────────
        # A single missed detection (motion blur, momentary occlusion, a
        # stale/late frame) used to immediately bounce the state machine
        # back to SEARCH, discarding the learned IBVS Jacobian and
        # re-triggering the spiral motion — producing a SEARCH<->APPROACH
        # thrash loop that itself causes more motion blur. Require several
        # consecutive misses before actually giving up on the marker.
        self.MISS_TOLERANCE_SECS = 2.5   # bail to SEARCH after this many real seconds of dropout
        self._miss_elapsed = 0.0         # accumulated dropout time (seconds)
        self._last_v = np.zeros(4)   # last MPC velocity command, held through brief dropouts
        self._last_vz = 0.0          # last commanded descend/climb rate
        self._approach_hold_z: Optional[float] = None  # NED z to hold during APPROACH

        # ── Control timer (10 Hz) ────────────────────────────
        period = 1.0 / CFG.control_hz
        self.create_timer(period, self._control_loop)

        self.get_logger().info('IBVS-MPC Controller initialised. '
                               'Publish to /ibvs/state to watch progress.')
        self.get_logger().info('Call node.arm() or send MAV_CMD 400 to start.')

    # ── Sensor callbacks ─────────────────────────────────────

    def _odom_cb(self, msg: VehicleOdometry):
        """
        Position + velocity source.

        NOTE: this controller originally read position from
        VehicleLocalPosition via /fmu/out/vehicle_local_position, but in
        this environment that message type fails to resolve at the DDS
        layer ("message type 'px4_msgs/msg/VehicleLocalPosition' is
        invalid"), so the subscription never receives data, self.pos
        stays at [0,0,0] forever, and TAKEOFF's altitude check never
        succeeds (the drone climbs forever without ever transitioning
        to SEARCH). VehicleOdometry carries the same NED position data
        and is confirmed working (drone_tracker.py uses it successfully),
        so we use it for both position and velocity.
        """
        raw = np.array([msg.position[0], msg.position[1], msg.position[2]])

        # Capture launch position on first message so altitude is relative
        # to the ground regardless of GPS/EKF origin offset
        if self._pos_origin is None:
            self._pos_origin = raw.copy()
            self.get_logger().info(
                f'Position origin set: ({raw[0]:.1f}, {raw[1]:.1f}, {raw[2]:.1f})')
        self.pos = raw - self._pos_origin

        self.vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]])

        # VehicleOdometry exposes orientation as a quaternion (w,x,y,z),
        # not a heading scalar — extract yaw (rotation about NED z-axis).
        if hasattr(msg, 'q') and len(msg.q) == 4:
            qw, qx, qy, qz = msg.q
            self.yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz))

    def _status_cb(self, msg: VehicleStatus):
        self.armed     = (msg.arming_state == 2)
        self.nav_state = msg.nav_state

    def _image_cb(self, msg: Image):
        """Convert any ROS Image encoding → OpenCV BGR."""
        try:
            h, w = msg.height, msg.width
            enc = msg.encoding.lower()
            data = np.frombuffer(msg.data, dtype=np.uint8)

            if enc in ('bgr8', '8uc3'):
                img = data.reshape(h, w, 3)

            elif enc in ('rgb8',):
                img = data.reshape(h, w, 3)[:, :, ::-1]

            elif enc in ('mono8', '8uc1', 'l8'):
                img = cv2.cvtColor(data.reshape(h, w), cv2.COLOR_GRAY2BGR)

            elif enc in ('mono16', '16uc1'):
                arr16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
                arr8  = (arr16 >> 8).astype(np.uint8)
                img   = cv2.cvtColor(arr8, cv2.COLOR_GRAY2BGR)

            elif enc in ('bayer_rggb8', 'bayer_bggr8', 'bayer_gbrg8', 'bayer_grbg8'):
                codes = {
                    'bayer_rggb8': cv2.COLOR_BayerBG2BGR,
                    'bayer_bggr8': cv2.COLOR_BayerRG2BGR,
                    'bayer_gbrg8': cv2.COLOR_BayerGR2BGR,
                    'bayer_grbg8': cv2.COLOR_BayerGB2BGR,
                }
                img = cv2.cvtColor(data.reshape(h, w), codes[enc])

            elif enc in ('rgba8', 'bgra8'):
                arr = data.reshape(h, w, 4)
                img = cv2.cvtColor(arr,
                    cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR)

            else:
                # Last resort: try treating as 3-channel BGR regardless of label
                self.get_logger().warn(
                    f'Unknown encoding "{msg.encoding}", trying raw BGR reshape.',
                    throttle_duration_sec=5.0)
                try:
                    img = data.reshape(h, w, 3)
                except Exception:
                    self.get_logger().error(
                        f'Cannot decode encoding "{msg.encoding}" '
                        f'(data len={len(data)}, expected {h*w*3})')
                    return

            with self._img_lock:
                self.latest_image = img.copy()
                self._img_count += 1

        except Exception as e:
            self.get_logger().warn(f'Image decode error: {e}')

    # ── Vehicle commands ─────────────────────────────────────

    def _send_cmd(self, command, p1=0., p2=0., p3=0.,
                  p4=0., p5=0., p6=0., p7=0.):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(p1)
        msg.param2           = float(p2)
        msg.param3           = float(p3)
        msg.param4           = float(p4)
        msg.param5           = float(p5)
        msg.param6           = float(p6)
        msg.param7           = float(p7)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def _set_offboard_mode(self):
        self._send_cmd(176, p1=1.0, p2=6.0)

    def _arm(self):
        self._send_cmd(400, p1=1.0)

    def _disarm(self):
        self._send_cmd(400, p1=0.0)

    def _land_cmd(self):
        self._send_cmd(21)

    # ── Setpoint publishing ──────────────────────────────────

    def _pub_offboard_velocity(self):
        msg = OffboardControlMode()
        msg.position     = False
        msg.velocity     = True
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def _pub_offboard_position(self):
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def _pub_velocity(self, vx=0., vy=0., vz=0., yawspeed=0.):
        """Publish NED velocity setpoint."""
        sp = TrajectorySetpoint()
        sp.position  = [float('nan')] * 3
        sp.velocity  = [float(vx), float(vy), float(vz)]
        sp.yawspeed  = float(yawspeed)
        sp.yaw       = float('nan')
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(sp)

    def _pub_position(self, x, y, z, yaw=0.):
        """Publish NED position setpoint."""
        sp = TrajectorySetpoint()
        sp.position  = [float(x), float(y), float(z)]
        sp.velocity  = [float('nan')] * 3
        sp.yaw       = float(yaw)
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(sp)

    def _publish_debug_image(self, img: np.ndarray):
        """Publish OpenCV image as ROS Image message."""
        try:
            h, w, c = img.shape
            msg = Image()
            msg.height   = h
            msg.width    = w
            msg.encoding = 'bgr8'
            msg.step     = w * 3
            msg.data     = img.tobytes()
            msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_img_pub.publish(msg)
        except Exception:
            pass

    def _publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    # ── State transitions ────────────────────────────────────

    def _transition(self, new_state: State):
        if new_state != self.state:
            self.get_logger().info(
                f'State: {self.state.name} → {new_state.name}')
            self.state = new_state
            # Always restore nominal velocity limits on state change;
            # IBVS handlers scale these down for rate-adaptive control,
            # but search/takeoff/hover should use the full configured limits.
            self.mpc.cfg.v_max = CFG.v_max.copy()
            self.mpc.cfg.v_min = CFG.v_min.copy()
            self._miss_elapsed = 0.0   # reset dropout timer on every transition

    # ── Main control loop (10 Hz nominal) ───────────────────

    def _control_loop(self):
        self._tick += 1
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        now_s  = now_us * 1e-6

        # Measure real elapsed time since last tick (varies under CPU load)
        real_dt = now_s - self._last_tick_s if self._last_tick_s > 0 else CFG.Tc
        real_dt = float(np.clip(real_dt, 0.02, 5.0))   # sanity bounds: 0.02–5s
        self._last_tick_s = now_s

        # Always stream offboard heartbeat
        self._pub_offboard_velocity()

        # Arm + switch to offboard after enough ticks
        if self._tick == CFG.arm_after_ticks:
            self._set_offboard_mode()
        if self._tick == CFG.arm_after_ticks + 3:
            self._arm()
        if self._tick == CFG.arm_after_ticks + 4 and self.state == State.IDLE:
            self._transition(State.TAKEOFF)

        self._real_dt = real_dt

        self._publish_state()

        # Grab latest camera frame
        with self._img_lock:
            frame = self.latest_image.copy() if self.latest_image is not None else None

        # ── State handlers ────────────────────────────────────
        if self.state == State.IDLE:
            self._pub_velocity(0., 0., 0.)

        elif self.state == State.TAKEOFF:
            self._handle_takeoff()

        elif self.state == State.SEARCH:
            self._handle_search(frame)

        elif self.state == State.IBVS_APPROACH:
            self._handle_ibvs_approach(frame, self._real_dt)

        elif self.state == State.IBVS_DESCEND:
            self._handle_ibvs_descend(frame, self._real_dt)

        elif self.state == State.HOVER:
            self._handle_hover(frame)

        elif self.state == State.LAND:
            self._pub_offboard_position()
            self._land_cmd()

    # ── TAKEOFF ──────────────────────────────────────────────

    def _handle_takeoff(self):
        """Climb at 1 m/s until takeoff altitude reached."""
        target_z = CFG.takeoff_altitude   # NED (negative = up)
        if self.pos[2] > target_z + 0.3:
            # Still climbing (NED: pos.z decreases as altitude increases)
            self._pub_velocity(vz=-0.8)
        else:
            self._pub_velocity(vz=0.)
            self.get_logger().info(
                f'Takeoff complete. Alt={-self.pos[2]:.1f}m. Starting search.')
            self._search_origin = self.pos[:2].copy()
            self._transition(State.SEARCH)

    # ── SEARCH ───────────────────────────────────────────────

    def _handle_search(self, frame: Optional[np.ndarray]):
        """
        Ascend while spiralling outward until ArUco marker is detected.
        """
        if frame is not None:
            found, s_norm, Z_est, debug = self.detector.detect(frame)
            self._publish_debug_image(debug)
            if found:
                self.get_logger().info(
                    f'Marker detected at Z={Z_est:.2f}m. '
                    f'Starting IBVS approach.')
                self.mpc.init_jacobian(s_norm, Z_est)
                self._transition(State.IBVS_APPROACH)
                return

        # Camera health check — warn if no images arriving
        if self._tick % 50 == 0:
            if self._img_count == 0:
                self.get_logger().warn(
                    '[SEARCH] No camera frames received! '
                    'Check ROS-GZ bridge: ros2 topic hz /camera')
            else:
                self.get_logger().info(
                    f'[SEARCH] Camera OK ({self._img_count} frames so far)')

        # Ascend toward search altitude (pos[2] is relative: negative = up)
        vz = 0.0
        if self.pos[2] > CFG.search_altitude + 0.2:
            vz = -0.5   # climb faster

        # Outward expanding spiral — no radius cap so we cover full arena
        self._search_angle += 0.12      # rad per tick (~69 deg/s at 10 Hz)
        if self._search_angle > 2 * np.pi:
            self._search_angle -= 2 * np.pi
            self._search_r += CFG.search_spiral_r

        r = self._search_r
        # Scale velocity with radius so outer loops move faster
        v_horiz = min(0.3 + r * 0.1, 1.0)
        vx = v_horiz * np.cos(self._search_angle)
        vy = v_horiz * np.sin(self._search_angle)

        self._pub_velocity(vx=vx, vy=vy, vz=vz)

        if self._tick % 20 == 0:
            self.get_logger().info(
                f'[SEARCH] Alt={-self.pos[2]:.1f}m (target {-CFG.search_altitude:.0f}m)  '
                f'spiral r={r:.1f}m  v={v_horiz:.2f}m/s  '
                f'cam_frames={self._img_count}')

    # ── IBVS APPROACH (lateral centering) ────────────────────

    def _handle_ibvs_approach(self, frame: Optional[np.ndarray], real_dt: float = 0.1):
        """
        Drive marker to image centre using a direct proportional centroid law,
        while actively holding the takeoff altitude.

        Three root causes fixed here:
          1. STALE FRAMES: the camera at ~10 Hz sometimes delivers the same
             frame across 2-3 control ticks (~10-30 Hz timer). Reprocessing
             an identical frame with a proportional controller causes the drone
             to continue applying the same large velocity for 2-3× longer than
             intended, overshooting and then oscillating. Fix: skip the IBVS
             update on ticks where img_count hasn't changed; hold last velocity.
          2. ALTITUDE SINK: PX4 in offboard-velocity mode accumulates a small
             downward drift even when vz=0 is commanded, because the velocity
             setpoint doesn't actively counteract gravity estimation errors. The
             drone drifted from 2.8m down to 1.6m, losing Z_est context and
             eventually causing the marker to fill the frame and be lost. Fix:
             active P-controller altitude hold using the NED z captured on
             APPROACH entry, publishing a small vz correction each tick.
          3. GAIN TOO HIGH: lam=3.0 * v_max=1.5 saturates immediately at any
             error > 0.5, giving bang-bang behaviour that overshoots at 10 Hz.
             Reduced to lam=1.5 so v ramps proportionally for e_lat < 1.0
             before saturating, giving smoother convergence.
        """
        if frame is None:
            self._pub_velocity(0., 0., 0.)
            return

        # Capture the altitude to hold on first entry
        if self._approach_hold_z is None:
            self._approach_hold_z = self.pos[2]

        # ── Stale-frame guard ─────────────────────────────────
        # Only run the IBVS update when a genuinely new camera frame arrived.
        current_img_count = self._img_count
        new_frame_available = (current_img_count != self._last_processed_img_count)

        if new_frame_available:
            found, s_norm, Z_est, debug = self.detector.detect(frame)
            self._publish_debug_image(debug)

            if not found:
                self._miss_elapsed += real_dt
                if self._miss_elapsed >= self.MISS_TOLERANCE_SECS:
                    self.get_logger().warn(
                        f'Marker lost during approach ({self._miss_elapsed:.1f}s dropout). '
                        f'Returning to SEARCH.')
                    self._miss_elapsed = 0.0
                    self.mpc.L_hat = None
                    self._approach_hold_z = None
                    self._transition(State.SEARCH)
                    self._pub_velocity(0., 0., 0.)
                    return
                # Within tolerance: hold last known lateral velocity
                vx, vy = self._last_v[0], self._last_v[1]
            else:
                self._miss_elapsed = 0.0
                self._last_processed_img_count = current_img_count

                # Centroid error in normalised image coords
                e_x = float(np.mean(s_norm[0::2]))
                e_y = float(np.mean(s_norm[1::2]))
                e_lateral = math.sqrt(e_x * e_x + e_y * e_y)

                # Transition to DESCEND once centred.
                # The original altitude gate prevented DESCEND/HOVER from
                # being reached because the vehicle often never satisfied
                # pos[2] < CFG.hover_altitude - 0.3 in practice.
                if e_lateral < 0.10:
                    self.mpc.init_jacobian(s_norm, Z_est)
                    self._approach_hold_z = None
                    self._transition(State.IBVS_DESCEND)
                    return

                # Proportional centroid control with moderate gain.
                # lam=1.5: saturates at v_max only when e_lat > 1.0 (marker
                # near image edge); ramps proportionally for smaller errors.
                lam_approach = 1.5
                v_max_lat = CFG.v_max[0]   # full 1.5 m/s cap (dt scaling removed —
                # the proportional gain itself limits the per-tick step to
                # lam * e * real_dt which is self-limiting without explicit scaling)

                vx = float(np.clip(-lam_approach * e_x, -v_max_lat, v_max_lat))
                vy = float(np.clip(-lam_approach * e_y, -v_max_lat, v_max_lat))
                self._last_v = np.array([vx, vy, 0., 0.])

                if self._tick % 10 == 0:
                    self.get_logger().info(
                        f'[APPROACH] |e_lat|={e_lateral:.3f}  Z={Z_est:.2f}m  '
                        f'alt={-self.pos[2]:.2f}m  v=({vx:.2f},{vy:.2f})')
        else:
            # No new frame: hold last lateral velocity, still correct altitude
            vx, vy = self._last_v[0], self._last_v[1]

        # ── Active altitude hold ──────────────────────────────
        # P-controller keeps the drone at the altitude it had when APPROACH
        # was entered. NED: pos[2] negative = above ground; more negative =
        # higher. alt_err > 0 means drone has sunk below hold target → climb.
        hold_z = self._approach_hold_z if self._approach_hold_z is not None else self.pos[2]
        alt_err = hold_z - self.pos[2]   # positive if drone is BELOW hold target
        vz = float(np.clip(-0.4 * alt_err, -0.3, 0.3))  # NED: negative = climb

        self._pub_offboard_velocity()
        self._pub_velocity(vx=vx, vy=vy, vz=vz, yawspeed=0.)

    # ── IBVS DESCEND (approach + descend simultaneously) ─────

    def _handle_ibvs_descend(self, frame: Optional[np.ndarray], real_dt: float = 0.1):
        """
        Drive marker to image centre while descending toward hover altitude.
        Transitions to HOVER when altitude and feature error are within bounds.

        Uses the same proportional centroid law and stale-frame guard as
        _handle_ibvs_approach (see that docstring for the full rationale).
        The previous NMPC+Broyden path here shared the same fragile-Jacobian
        failure mode that caused APPROACH to freeze at v=0, so it is replaced
        here as well even though DESCEND hadn't yet been reached in testing.
        """
        if frame is None:
            self._pub_velocity(0., 0., 0.)
            return

        # Stale-frame guard (see _handle_ibvs_approach for rationale)
        current_img_count = self._img_count
        new_frame_available = (current_img_count != self._last_processed_img_count)

        if new_frame_available:
            found, s_norm, Z_est, debug = self.detector.detect(frame)
            self._publish_debug_image(debug)

            if not found:
                self._miss_elapsed += real_dt
                if self._miss_elapsed >= self.MISS_TOLERANCE_SECS:
                    self.get_logger().warn(
                        f'Marker lost during descent ({self._miss_elapsed:.1f}s dropout). '
                        f'Returning to SEARCH.')
                    self._miss_elapsed = 0.0
                    self.mpc.L_hat = None
                    self._transition(State.SEARCH)
                    self._pub_velocity(0., 0., 0.)
                    return
                # Within tolerance: hold last known velocity
                vx, vy, vz = self._last_v[0], self._last_v[1], self._last_vz
            else:
                self._miss_elapsed = 0.0
                self._last_processed_img_count = current_img_count

                # Feature error norm relative to s_star (size/shape error,
                # used only as the hover-readiness gate)
                s_star = self.detector.s_star
                e_norm = np.linalg.norm(s_norm - s_star)

                # Check hover condition
                at_hover_alt   = self.pos[2] <= CFG.hover_altitude + 0.15
                features_good  = e_norm < CFG.hover_threshold

                if at_hover_alt and features_good:
                    self._hover_hold_count += 1
                    if self._hover_hold_count >= 5:   # hold for 5 ticks = 0.5s
                        self._transition(State.HOVER)
                        return
                else:
                    self._hover_hold_count = 0

                # Lateral control: same proportional centroid law as APPROACH.
                # Replaces the NMPC+Broyden path, which shares the singular-
                # Jacobian failure mode diagnosed in _handle_ibvs_approach —
                # DESCEND never ran long enough in testing to hit it, but the
                # same tiny-velocity-corrupts-L_hat mechanism applies here too.
                e_x = float(np.mean(s_norm[0::2]))
                e_y = float(np.mean(s_norm[1::2]))
                lam_descend = 1.2   # gentler than APPROACH; marker fills more of frame
                v_max_lat = 0.8
                vx = float(np.clip(-lam_descend * e_x, -v_max_lat, v_max_lat))
                vy = float(np.clip(-lam_descend * e_y, -v_max_lat, v_max_lat))
                self._last_v = np.array([vx, vy, 0., 0.])

                # vz: descend toward hover altitude.
                #
                # NED convention: pos[2] increases downward, so "higher" = more
                # negative. hover_altitude = -1.0 means 1 m up. search_altitude
                # = -10.0 means 10 m up. We want alt_err > 0 when the drone is
                # ABOVE the hover target (needs to descend) and alt_err < 0 when
                # it has overshot BELOW the target (needs to climb back up).
                # hover_altitude - pos[2]: e.g. at search alt, pos[2]=-10,
                # hover_altitude=-1 -> -1-(-10) = +9 (correctly "too high").
                #
                # Previous version had a 0.3 m dead-band (vz=0 whenever
                # |alt_err|<0.3), which left the drone permanently stalled
                # 0.25-0.3 m above the hover target — never close enough to
                # satisfy at_hover_alt (which requires alt_err<=0.15) but
                # with zero vz to close the remaining gap. Narrowed the
                # dead-band to match at_hover_alt's own 0.15 m tolerance, and
                # the proportional descent rate now reaches all the way down
                # to that point instead of stopping early.
                alt_err = CFG.hover_altitude - self.pos[2]

                if alt_err > 0.15:
                    vz = min(0.5, 0.3 * alt_err)    # positive NED = descend
                elif alt_err < -0.15:
                    vz = max(-0.5, 0.3 * alt_err)   # negative NED = climb
                else:
                    vz = 0.0
                self._last_vz = vz

                if self._tick % 10 == 0:
                    self.get_logger().info(
                        f'[DESCEND] |e|={e_norm:.3f}  Z={Z_est:.2f}m  '
                        f'alt={-self.pos[2]:.2f}m  alt_err={alt_err:.2f}  '
                        f'vz={vz:.2f}  v=({vx:.2f},{vy:.2f})')
        else:
            vx, vy, vz = self._last_v[0], self._last_v[1], self._last_vz

        self._pub_offboard_velocity()
        self._pub_velocity(vx=vx, vy=vy, vz=vz, yawspeed=0.)

    # ── HOVER ────────────────────────────────────────────────

    def _handle_hover(self, frame: Optional[np.ndarray]):
        """
        Hold position directly above marker using a proportional centroid
        correction, identical in spirit to IBVS_APPROACH/IBVS_DESCEND.

        Replaces the previous NMPC+Broyden path here, which shared the same
        fragile-Jacobian failure mode diagnosed in _handle_ibvs_approach:
        after the first small corrective velocity, the rank-1 Broyden update
        receives a near-zero dv and corrupts L_hat to near-singular, freezing
        all future corrections at v≈0. In HOVER specifically this is worse
        than in APPROACH, because a "frozen at zero" hover looks superficially
        correct (the drone just sits there) right up until natural drift
        pushes e_norm past 0.15 and the controller bails back to APPROACH —
        so the bug was silently undermining hover stability the whole time.

        Also fixes: the previous "no frame visible → hold zero velocity"
        branch dropped altitude hold entirely on any momentary dropout,
        which (per the same drift mechanism documented in IBVS_APPROACH)
        would let the drone sink. Brief dropouts now hold last known
        lateral correction and keep actively correcting altitude.
        """
        if frame is None:
            self._pub_offboard_velocity()
            self._pub_velocity(vx=self._last_v[0] * 0.3, vy=self._last_v[1] * 0.3,
                               vz=self._last_vz, yawspeed=0.)
            return

        current_img_count = self._img_count
        new_frame_available = (current_img_count != self._last_processed_img_count)

        if new_frame_available:
            found, s_norm, Z_est, debug = self.detector.detect(frame)
            self._publish_debug_image(debug)

            if not found:
                self._miss_elapsed += 0.1
                if self._miss_elapsed >= self.MISS_TOLERANCE_SECS:
                    self.get_logger().warn(
                        f'Marker lost during hover ({self._miss_elapsed:.1f}s dropout). '
                        f'Returning to SEARCH.')
                    self._miss_elapsed = 0.0
                    self.mpc.L_hat = None
                    self._transition(State.SEARCH)
                    self._pub_velocity(0., 0., 0.)
                    return
                # Within tolerance: hold last known correction + altitude
                vx, vy, vz = (self._last_v[0] * 0.3, self._last_v[1] * 0.3,
                             self._last_vz)
            else:
                self._miss_elapsed = 0.0
                self._last_processed_img_count = current_img_count

                s_star = self.detector.s_star
                e_norm = np.linalg.norm(s_norm - s_star)

                if e_norm > 0.20:
                    # Marker drifted significantly — go back to approach.
                    # (Threshold raised slightly from 0.15 to 0.20 to avoid
                    # bouncing out of HOVER on minor detection noise now
                    # that s_star/hover_threshold are both correctly scaled
                    # to the real hover depth.)
                    self.get_logger().warn(
                        f'Marker drift detected (|e|={e_norm:.3f}). '
                        'Returning to IBVS_APPROACH.')
                    self._transition(State.IBVS_APPROACH)
                    return

                # Gentle proportional centroid correction — same law as
                # APPROACH/DESCEND but with a smaller gain and tighter cap
                # since the drone should already be nearly stationary here.
                e_x = float(np.mean(s_norm[0::2]))
                e_y = float(np.mean(s_norm[1::2]))
                lam_hover = 0.8
                v_max_hover = 0.3
                vx = float(np.clip(-lam_hover * e_x, -v_max_hover, v_max_hover))
                vy = float(np.clip(-lam_hover * e_y, -v_max_hover, v_max_hover))
                self._last_v = np.array([vx, vy, 0., 0.])

                # Altitude hold: P-controller to stay at hover_altitude
                # (same NED polarity as IBVS_DESCEND: positive alt_err
                # means "too high, need to descend" → positive vz)
                alt_err = CFG.hover_altitude - self.pos[2]
                vz = float(np.clip(alt_err * 0.4, -0.3, 0.3))
                self._last_vz = vz

                if self._tick % 20 == 0:
                    self.get_logger().info(
                        f'[HOVER] |e|={e_norm:.4f}  Z={Z_est:.2f}m  '
                        f'alt={-self.pos[2]:.2f}m  ✓ HOVERING')
        else:
            vx, vy, vz = (self._last_v[0] * 0.3, self._last_v[1] * 0.3,
                         self._last_vz)

        self._pub_offboard_velocity()
        self._pub_velocity(vx=vx, vy=vy, vz=vz, yawspeed=0.)

    # ── Public API ───────────────────────────────────────────

    def request_land(self):
        """Trigger landing from external call."""
        self._transition(State.LAND)

    def get_state(self) -> str:
        return self.state.name


# ══════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = IBVSMPCController()

    print('\n' + '='*65)
    print('  IBVS-MPC Autonomous Drone Controller')
    print('='*65)
    print('\nState machine: IDLE → TAKEOFF → SEARCH → IBVS_APPROACH')
    print('               → IBVS_DESCEND → HOVER')
    print('\nTopics:')
    print('  Camera input  : /camera')
    print('  Debug image   : /ibvs/debug_image  (view with rqt_image_view)')
    print('  State monitor : /ibvs/state')
    print('\nThe drone will arm and take off automatically in ~2 seconds.')
    print('='*65 + '\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()