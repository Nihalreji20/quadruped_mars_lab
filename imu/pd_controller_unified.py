#!/usr/bin/env python3
"""
quadruped_unified_sim.py
========================
Unified Quadruped Simulator — Stage 9
Merges forward crawl gait (FREE_BASE_TWO_STEP_TEST.py) and
Stage 6B directional turning gait (turn_gait_real.py) into a single runtime
with a live PyBullet mode selector and IMU telemetry overlay.

Mode Slider (PyBullet GUI):
    0 = STOP
    1 = FORWARD WALK
    2 = LEFT TURN
    3 = RIGHT TURN

Switching modes:
    - Never respawns the robot
    - Never resets the simulation
    - Never resets gait state (GSM continues from current leg/phase)
    - Foot targets persist across mode transitions

IMU:
    SimulatedIMU reads ground-truth pose from PyBullet and adds
    configurable Gaussian noise + optional yaw drift.
    Swap to real hardware by changing create_imu(mode="bno055") — nothing
    else in this file needs to change.

Frozen (must not be modified):
    compute_ik(), LEG_CONFIG, GAIT_ORDER [1,2,4,3], COM stabilization (forward),
    forward foot target generation, Stage 6B turning logic (get_leg_params),
    6B stance sweep + swing trajectory.
"""

import sys
import pybullet as p
import pybullet_data
import time
import math

if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

# IMU module (Stage 9) — place imu_module.py alongside this file
from imu_module import create_imu

# ============================================================================
# SECTION 1 — CONFIGURATION
# ============================================================================

URDF_PATH         = "urdf/finalv6_pla.urdf"
BASE_SPAWN_HEIGHT = 0.38
SETTLE_STEPS      = 360

MG996R_MAX_TORQUE = 1.08   # Nm

# MG996R no-load speed at 6V ≈ 0.17 sec/60° ≈ 6.2 rad/s.
# Setting maxVelocity above the physical limit lets PyBullet's position
# controller run the joint as fast as torque allows, matching real servo
# behaviour. Without this, PyBullet uses a very low internal default that
# makes joints turn far slower than the real MG996R.
MG996R_MAX_VELOCITY = 8.0  # rad/s  (physical ≈ 6.2; headroom for sim dynamics)

STAND_PX =  158.0   # mm
STAND_PY =    0.0
STAND_PZ = -135.0   # mm

LIFT_HEIGHT =  35.0
APEX_PZ     = STAND_PZ + LIFT_HEIGHT

INTERP_STEPS  = 28          # was 40 — fewer points per swing phase = snappier steps
STEP_DELAY    = 0.004       # was 0.006 — shorter sleep per tick = faster cadence
SIM_SUBSTEPS  = 3

GAIT_ORDER    = [1, 2, 4, 3]   # frozen — do not change

USE_FIXED_BASE = False

# Forward gait amplitude
STEP_LENGTH   = 80.0   # mm

# ── Stage 6B turning subsystem constants (from turn_gait_real.py) ───────────
# Turning is a DIFFERENT mechanism from forward and needs its own geometry:
#   - wider stance (180mm) to keep the 3-leg support triangle stable under
#     the 1.08 Nm torque budget during yaw
#   - low lift (12mm) to minimise pendulum rocking while turning
#   - shorter stride tuned for real MG996R torque limits and explicitly reduced 
#     to limit coxa angle extent relative to forward gait.
STAND_PX_TURN    = 180.0    # mm  (6B wide turning stance)
LIFT_HEIGHT_TURN = 12.0     # mm  (6B low turning lift)
APEX_PZ_TURN     = STAND_PZ + LIFT_HEIGHT_TURN
STEP_LENGTH_TURN = 40.0     # mm  (Reduced to restrict coxa angle max extent)

# IMU telemetry print interval (steps) 
IMU_PRINT_INTERVAL = 48   # ~5 Hz at 240 Hz sim rate

# ============================================================================
# SECTION 1b — DYNAMIC POSTURE CONTROLLER CONFIG  (Roll + Pitch PD)
# ============================================================================
# Roll + pitch PD posture controller. Injects ONLY per-leg Δpz corrections into
# the locomotion-owned IK targets. Never touches px/py, gait sequencing, or
# trajectories.
#
# Pipeline:
#   IMU (roll, pitch, body gyro) → level-ref subtraction → EMA filter
#     → soft deadband → PD (D on Euler rate from body gyro) → clamp
#     → geometry mixing → per-leg authority fade (swing exclusion) → Δpz
#
# Conventions — VERIFIED numerically in PyBullet (see VERIFY notes below):
#   roll  = rotation about body +X.  +roll RAISES the +Y side (legs 2,3).
#   pitch = rotation about body +Y.  +pitch LOWERS the +X side (legs 2,4).
#   Δpz > 0 moves the foot UP relative to the hip → body LOWERS at that leg.
#   Gyro is BODY frame (imu_module rotates PyBullet's world-frame ω).
POSTURE_ENABLED = True          # master switch; False → controller outputs zeros

# Control timestep: simulation time advanced per outer tick (NOT wall-clock).
POSTURE_DT = SIM_SUBSTEPS / 240.0      # seconds per control tick (12.5 ms)

# EMA low-pass smoothing factors (0..1). Higher = less smoothing, faster.
POSTURE_EMA_ALPHA_ANGLE = 0.35  # roll/pitch angle filter
POSTURE_EMA_ALPHA_GYRO  = 0.15  # angular-rate (D-term) filter; heavier than the
                                # angle filter so foot-touchdown rate spikes
                                # don't pump Δpz (0.30 raised walking peaks 3x)

# Soft deadband half-width (deg). Output is continuous at the band edge:
#   e_db = sign(e) * max(|e| - db, 0)
POSTURE_DEADBAND_DEG = 0.5

# PD gains — mm of Δpz per deg of error (Kp) and per deg/s of Euler rate (Kd).
# Tuned in sim (see TUNING NOTES at the end of this section).
POSTURE_KP_ROLL  = 6.0
POSTURE_KD_ROLL  = 0.02
POSTURE_KP_PITCH = 4.5
POSTURE_KD_PITCH = 0.02

# Integral term (mm of Δpz per deg·s of error). Removes the steady-state tilt
# a PD leaves on a slope. Kept SLOW on purpose: integral time Ti = Kp/Ki is
# ~0.5 s, >10x slower than the ~30 ms P loop, so the I-term's phase lag sits
# far below the loop's crossover and does not erode its stability margin.
POSTURE_KI_ROLL  = 12.0          # Ti = 6.0/12  = 0.5 s
POSTURE_KI_PITCH = 9.0           # Ti = 4.5/9   = 0.5 s
# Anti-windup:
#   * clamp: |I contribution| ≤ POSTURE_I_MAX_MM
#   * conditional integration: frozen while the output is saturated in the
#     direction the error would push it, and on any tick an IK target was
#     infeasible (report_ik()).
#   * integration uses the DEADBANDED error, so it never hunts inside ±db.
POSTURE_I_MAX_MM = 20.0
# Where to integrate: "stand" = only while all four feet are down (STOP);
# while walking the I-state is HELD (not integrated, not reset), so a slope
# estimate learned while standing carries into walking.
# "always" = integrate during walking too.
POSTURE_I_MODE = "stand"
# Slow leak toward 0 (time constant, s) so a stale slope estimate fades.
POSTURE_I_LEAK_TAU = 20.0

# Pitch mixing mode:
#   "symmetric"    : front/back legs get ±u (body height preserved).
#   "retract_only" : only the HIGH side's feet are pulled up (Δpz ≥ 0).
# Why retract_only: with this URDF, a Δpz<0 (extend) command also moves the
# foot ~2.3 mm radially INWARD per mm (knee joint turns opposite to the IK's
# assumption; see notes). Legs sprawl along body X, so extending the
# downhill legs on a fore-aft slope shrinks the support polygon exactly on the
# downhill side → tip-over when a downhill leg swings. Retracting moves feet
# OUTWARD instead, so the polygon never shrinks. Roll is unaffected (radial
# motion is along X, the roll-critical edges run along X), so roll stays
# symmetric.
POSTURE_PITCH_MIX = "retract_only"

# Axis polarity: +1.0 = the geometry-derived direction (verified in sim).
# Only flip on REAL hardware if the IMU is mounted rotated relative to base_link.
POSTURE_ROLL_POLARITY  = +1.0
POSTURE_PITCH_POLARITY = +1.0

# Per-leg Δpz saturation (mm), mode dependent.
POSTURE_DPZ_MAX_FWD  = 25.0
POSTURE_DPZ_MAX_TURN = 16.7   # turning stance (px=180) has less Z authority

# Swing exclusion: each leg's authority weight w∈[0,1] ramps (not steps) to 0
# before lift-off and back to 1 after touchdown. Ticks for a full 0↔1 ramp.
# Must be ≤ 28 (turning LOWER length) so the fade completes before lift-off.
POSTURE_FADE_TICKS = 20

# TUNING NOTES (PyBullet, 1.42 kg model, 80 Hz control, IMU noise on):
#   Kp: forward-walk and slope metrics improve monotonically up to ~6 (roll)
#       and ~4.5 (pitch); pitch above 4.5 raised slope-walk tilt peaks.
#   Kd: closed-loop 3° setpoint steps — Kd=-0.02 raised overshoot (sign check
#       passes), 0.02–0.05 trims it, 0.1 chatters, 0.2 ≈ 100% overshoot.
#       0.02 keeps a 5–10x margin to that edge. The loop rises in ~30 ms
#       (2–3 control ticks), so D has little bandwidth to work with.
#   Kill switches: if parked on flat ground Δpz toggles continuously by more
#       than ±1 mm, or a 3° step overshoots more than 50%, halve Kp/Kd.
#       If the overlay shows SAT on most ticks while walking, the clamp
#       (not the gains) is the limit.

# Hold posture while parked (STOP). All four legs grounded, levels the body.
POSTURE_HOLD_WHEN_STOPPED = True

# Level reference capture (see main()). Legs are position-held at the stand
# pose BEFORE the gravity settle, then the reference is averaged. If the
# captured offset is larger than this, it is treated as a bad pose (not an
# IMU mounting offset) and the gravity-level reference (0,0) is used instead.
POSTURE_REF_SAMPLES = 48
POSTURE_REF_MAX_DEG = 1.5

# ============================================================================
# SECTION 2 — LEG CONFIGURATION TABLE  (frozen — do not change)
# ============================================================================

LEG_CONFIG = {
    1: {
        'label': 'Leg 1 -- FL',
        'hip_xyz_urdf': (-0.075, -0.093751, -0.01255),
        'coxa_idx': 0, 'femur_idx': 6, 'tibia_idx': 8,
        'coxa_joint_name':  'Revolute 1',
        'femur_joint_name': 'Revolute 8',
        'tibia_joint_name': 'Revolute 11',
        'coxa_sign': -1, 'femur_sign': +1, 'tibia_sign': +1,
    },
    2: {
        'label': 'Leg 2 -- BR',
        'hip_xyz_urdf': (+0.075, +0.093751, -0.01255),
        'coxa_idx': 1, 'femur_idx': 5, 'tibia_idx': 11,
        'coxa_joint_name':  'Revolute 2',
        'femur_joint_name': 'Revolute 7',
        'tibia_joint_name': 'Revolute 15',
        'coxa_sign': +1, 'femur_sign': +1, 'tibia_sign': -1,
    },
    3: {
        'label': 'Leg 3 -- FR',
        'hip_xyz_urdf': (-0.075, +0.093751, -0.01255),
        'coxa_idx': 2, 'femur_idx': 4, 'tibia_idx': 9,
        'coxa_joint_name':  'Revolute 3',
        'femur_joint_name': 'Revolute 6',
        'tibia_joint_name': 'Revolute 13',
        'coxa_sign': -1, 'femur_sign': +1, 'tibia_sign': +1,
    },
    4: {
        'label': 'Leg 4 -- BL',
        'hip_xyz_urdf': (+0.075, -0.093751, -0.01255),
        'coxa_idx': 3, 'femur_idx': 7, 'tibia_idx': 10,
        'coxa_joint_name':  'Revolute 4',
        'femur_joint_name': 'Revolute 10',
        'tibia_joint_name': 'Revolute 14',
        'coxa_sign': +1, 'femur_sign': +1, 'tibia_sign': +1,
    },
}

# ============================================================================
# SECTION 3 — CORE KINEMATICS  (frozen — do not change)
# ============================================================================

def resolve_joint_indices(robot_id):
    name_to_idx = {
        p.getJointInfo(robot_id, i)[1].decode('utf-8'): i
        for i in range(p.getNumJoints(robot_id))
    }
    for leg in LEG_CONFIG.values():
        leg['coxa_idx']  = name_to_idx.get(leg['coxa_joint_name'])
        leg['femur_idx'] = name_to_idx.get(leg['femur_joint_name'])
        leg['tibia_idx'] = name_to_idx.get(leg['tibia_joint_name'])


def compute_ik(px, py, pz):
    """Frozen analytical IK — Law of Cosines, mammalian knee."""
    a1 = 52.50;  a2 = 108.00;  a3 = 130.775
    T1_MIN, T1_MAX = -25,  25
    T2_MIN, T2_MAX = -90,  45
    T3_MIN, T3_MAX = -45,  90

    t1_rad = math.atan2(py, px)
    t1_deg = math.degrees(t1_rad)
    R      = math.sqrt(px**2 + py**2)
    r      = R - a1
    z      = pz

    L_squared = r**2 + z**2
    if L_squared == 0:
        return 0, 0, 0, False

    c3 = max(-1.0, min(1.0, (L_squared - a2**2 - a3**2) / (2 * a2 * a3)))
    s3 = -math.sqrt(1 - c3**2)
    t3_math = math.atan2(s3, c3)
    t2_math = math.atan2(z, r) - math.atan2(a3 * s3, a2 + a3 * c3)

    urdf_t2_deg = ((math.degrees(t2_math)           + 180) % 360) - 180
    urdf_t3_deg = ((-(math.degrees(t3_math) + 90.0) + 180) % 360) - 180

    tol = 0.5
    if (T1_MIN - tol <= t1_deg     <= T1_MAX + tol and
        T2_MIN - tol <= urdf_t2_deg <= T2_MAX + tol and
        T3_MIN - tol <= urdf_t3_deg <= T3_MAX + tol):
        return t1_deg, urdf_t2_deg, urdf_t3_deg, True
    return 0, 0, 0, False


def compute_leg_joints(leg_id, px, py, pz):
    cfg = LEG_CONFIG[leg_id]
    t1, t2, t3, ok = compute_ik(px, py, pz)
    if not ok:
        return 0, 0, 0, False
    jc = t1 * cfg['coxa_sign']
    jf = t2 * cfg['femur_sign']
    jt = -t3 if cfg['tibia_sign'] == -1 else t3
    return jc, jf, jt, True


def set_leg(robot_id, leg_id, px, py, pz):
    jc, jf, jt, ok = compute_leg_joints(leg_id, px, py, pz)
    if not ok:
        return False
    cfg = LEG_CONFIG[leg_id]
    kw = dict(controlMode=p.POSITION_CONTROL,
              force=MG996R_MAX_TORQUE,
              maxVelocity=MG996R_MAX_VELOCITY,
              positionGain=1.5)
    p.setJointMotorControl2(robot_id, cfg['coxa_idx'],  targetPosition=math.radians(jc), **kw)
    p.setJointMotorControl2(robot_id, cfg['femur_idx'], targetPosition=math.radians(jf), **kw)
    p.setJointMotorControl2(robot_id, cfg['tibia_idx'], targetPosition=math.radians(jt), **kw)
    return True


def settle(steps=SETTLE_STEPS):
    for _ in range(steps):
        update_camera()
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

# ============================================================================
# SECTION 4 — SWING TRAJECTORY & COM STABILIZATION  (frozen)
# ============================================================================

def generate_swing_trajectory(start_px, start_py, target_px, target_py,
                               num_points=INTERP_STEPS):
    """
    Unified swing trajectory generator.
    Interpolates both px AND py so it covers both gait modes:
      - Forward:  target_px == start_px == STAND_PX  (no lateral change)
      - Turning:  target_px may differ from start_px
    """
    traj = []
    for i in range(num_points):
        t  = i / max(num_points - 1, 1)
        pz = STAND_PZ + LIFT_HEIGHT * math.sin(math.pi / 2.0 * t)
        traj.append(("LIFT", start_px, start_py, pz, t))
    for i in range(num_points):
        t  = i / max(num_points - 1, 1)
        px = start_px + (target_px - start_px) * t
        py = start_py + (target_py - start_py) * t
        traj.append(("SWEEP", px, py, APEX_PZ, t))
    for i in range(num_points):
        t  = i / max(num_points - 1, 1)
        pz = APEX_PZ - LIFT_HEIGHT * math.sin(math.pi / 2.0 * t)
        traj.append(("LOWER", target_px, target_py, pz, t))
    return traj


def get_true_support_polygon(swing_leg, foot_targets):
    poly = {}
    for lid in range(1, 5):
        if lid != swing_leg:
            hx, hy, _ = LEG_CONFIG[lid]['hip_xyz_urdf']
            outward_sign = 1.0 if hy > 0 else -1.0
            base_x = (hx * 1000.0) - foot_targets[lid]['py']
            base_y = (hy * 1000.0) + (foot_targets[lid]['px'] * outward_sign)
            poly[lid] = (base_x, base_y)
    return poly


def project_true_com(robot_id, nominal_z_mm=STAND_PZ):
    _, orn = p.getBasePositionAndOrientation(robot_id)
    rot = p.getMatrixFromQuaternion(orn)
    gx, gy, gz = rot[6], rot[7], rot[8]
    if gz == 0:
        return 0.0, 0.0
    com_x = nominal_z_mm * (gx / gz)
    com_y = nominal_z_mm * (gy / gz)
    return com_x, com_y


def compute_temporary_shift(robot_id, swing_leg, foot_targets, max_shift=12.0):
    poly = get_true_support_polygon(swing_leg, foot_targets)
    com_x, com_y = project_true_com(robot_id)
    cx = sum(pt[0] for pt in poly.values()) / 3.0
    cy = sum(pt[1] for pt in poly.values()) / 3.0
    req_dx = cx - com_x
    req_dy = cy - com_y
    safe_dx = max(-max_shift, min(max_shift, req_dx))
    safe_dy = max(-max_shift, min(max_shift, req_dy))
    return safe_dx, safe_dy

# ============================================================================
# SECTION 5 — STAGE 6B TURNING SUBSYSTEM  (verbatim from turn_gait_real.py)
# ============================================================================

def get_leg_params(leg_id, mode):
    """
    Stage 6B directional parameters — THE turning behavioural contract.

    Returns (target_py, stance_dir).
      target_py : absolute swing AEP in py for this leg under `mode`
      stance_dir: +1 or -1, the direction the planted foot drags each tick

    Side grouping is 6B's:  right_side = [1, 3],  left_side = [2, 4].
    Inner legs reverse both their swing target and their stance direction,
    which is what produces the yaw — NOT an opposing-stride force couple.
    """
    S = STEP_LENGTH_TURN
    right_side = [1, 3]   # FR, BR
    left_side  = [2, 4]   # BL, FL

    if mode == 'FORWARD':
        return S / 2.0, 1.0
    elif mode == 'RIGHT':
        if leg_id in left_side:
            return S / 2.0, 1.0
        else:
            return -S / 2.0, -1.0
    elif mode == 'LEFT':
        if leg_id in right_side:
            return S / 2.0, 1.0
        else:
            return -S / 2.0, -1.0
    return S / 2.0, 1.0


def generate_turn_swing_trajectory(start_py, target_py, num_points=INTERP_STEPS):
    """
    Stage 6B swing trajectory — py-only, low lift, lateral px held at
    STAND_PX_TURN.  Verbatim shape from turn_gait_real.py (LIFT/SWEEP/LOWER),
    emitted in the unified 6-tuple (px,py,pz) form with px pinned.
    """
    traj = []
    for i in range(num_points):
        t  = i / max(num_points - 1, 1)
        pz = STAND_PZ + LIFT_HEIGHT_TURN * math.sin(math.pi / 2.0 * t)
        traj.append(("LIFT", STAND_PX_TURN, start_py, pz, t))
    for i in range(num_points):
        t  = i / max(num_points - 1, 1)
        py = start_py + (target_py - start_py) * t
        traj.append(("SWEEP", STAND_PX_TURN, py, APEX_PZ_TURN, t))
    for i in range(num_points):
        t  = i / max(num_points - 1, 1)
        pz = APEX_PZ_TURN - LIFT_HEIGHT_TURN * math.sin(math.pi / 2.0 * t)
        traj.append(("LOWER", STAND_PX_TURN, target_py, pz, t))
    return traj

# ============================================================================
# SECTION 6 — UNIFIED GAIT STATE MACHINE
# ============================================================================

class ContinuousGaitStateMachine:
    """
    Single GSM that drives both FORWARD and turning modes.

    Return signature of advance():
        (leg_id, phase, px_val, py_val, pz_val, t)

    This is the turn_gait.py signature (strict superset of the forward
    signature, which baked px == STAND_PX implicitly).
    """

    def __init__(self, gait_order, foot_targets):
        self.gait_order      = gait_order
        self.foot_targets    = foot_targets
        self.current_leg_idx = 0
        self.state           = "IDLE"
        self.trajectory      = []
        self.traj_point      = 0
        self.legs_completed  = 0
        self.shift_steps     = 40
        self.shift_point     = 0
        self.target_dx       = 0.0
        self.target_dy       = 0.0

        for lid in range(1, 5):
            self.foot_targets[lid]["temp_dx"] = 0.0
            self.foot_targets[lid]["temp_dy"] = 0.0

    @property
    def active_leg(self):
        return self.gait_order[self.current_leg_idx]

    @property
    def support_legs(self):
        return [lid for lid in range(1, 5) if lid != self.active_leg]

    def _start_stabilization(self, robot_id):
        self.state       = "STABILIZE"
        self.shift_point = 0
        dx, dy = compute_temporary_shift(robot_id, self.active_leg, self.foot_targets)
        self.target_dx   = dx
        self.target_dy   = dy

    # ── Stage 6B turning path ─────────────────────────────────────────────
    # 6B has NO STABILIZE and NO CoM shift. It runs a continuous
    # LIFT→SWEEP→LOWER cycle and gets yaw from per-leg stance reversal.
    # We keep this path completely separate from the forward STABILIZE path
    # so neither gait is corrupted (turn_gait_real.py is the contract).
    def _advance_turning(self, current_mode, robot_id):
        if self.traj_point >= len(self.trajectory):
            if len(self.trajectory) > 0:
                self.legs_completed += 1
                self.current_leg_idx = (self.current_leg_idx + 1) % len(self.gait_order)

            self.traj_point = 0
            # Per-leg mode memory: the stance sweep in the main loop reads this
            # so each planted leg keeps dragging in the direction it was last
            # commanded (exactly as 6B does via foot_targets[leg]["mode"]).
            self.foot_targets[self.active_leg]["mode"] = current_mode

            start_py = self.foot_targets[self.active_leg]["py"]
            target_py, _ = get_leg_params(self.active_leg, current_mode)
            self.trajectory = generate_turn_swing_trajectory(start_py, target_py)

        phase, px_val, py_val, pz_val, t = self.trajectory[self.traj_point]
        leg_id = self.active_leg
        self.traj_point += 1
        return leg_id, phase, px_val, py_val, pz_val, t

    def advance(self, current_mode, robot_id):
        """
        Advance one tick.  current_mode: 'FORWARD' | 'LEFT' | 'RIGHT'
        Returns: (leg_id, phase, px, py, pz, t)

        Dispatch:
          FORWARD      → STABILIZE + CoM-shift machine (FREE_BASE behaviour)
          LEFT / RIGHT → Stage 6B direct LIFT/SWEEP/LOWER (turn_gait_real)
        """
        # ── Turning: 6B path, no STABILIZE, no CoM ────────────────────────
        if current_mode in ('LEFT', 'RIGHT'):
            # If we were mid-STABILIZE when the mode flipped to turning,
            # drop straight into the turning cycle on the current leg.
            if self.state != "TURN":
                self.state      = "TURN"
                self.trajectory = []     # force fresh 6B trajectory build
                self.traj_point = 0
            return self._advance_turning(current_mode, robot_id)

        # ── Forward (and STOP-resume): STABILIZE/CoM machine ──────────────
        if self.state in ("IDLE", "TURN"):
            # Entering/!returning to forward — (re)enter stabilization cleanly.
            self._start_stabilization(robot_id)

        if self.state == "STABILIZE":
            if self.shift_point >= self.shift_steps:
                self.state = "SWING"

                start_px = self.foot_targets[self.active_leg]["px"]
                start_py = self.foot_targets[self.active_leg]["py"]

                # Forward: swing strictly to front of stride,
                # lateral (px) stays at STAND_PX (FREE_BASE behaviour).
                self.foot_targets[self.active_leg]["px"] = STAND_PX
                start_px   = STAND_PX
                target_px  = STAND_PX
                target_py  = STEP_LENGTH / 2.0

                self.trajectory = generate_swing_trajectory(
                    start_px, start_py, target_px, target_py)
                self.traj_point = 0
                return self.advance(current_mode, robot_id)

            t          = (self.shift_point + 1) / float(self.shift_steps)
            smooth_t   = 3 * t**2 - 2 * t**3
            current_dx = self.target_dx * smooth_t
            current_dy = self.target_dy * smooth_t
            self.shift_point += 1
            # pz_val = 0 during STABILIZE (pz not used in that branch)
            return self.active_leg, "STABILIZE", current_dx, current_dy, 0.0, t

        if self.traj_point >= len(self.trajectory):
            if len(self.trajectory) > 0:
                self.legs_completed  += 1
                self.current_leg_idx  = (self.current_leg_idx + 1) % len(self.gait_order)
            self._start_stabilization(robot_id)
            return self.advance(current_mode, robot_id)

        phase, px_val, py_val, pz_val, t = self.trajectory[self.traj_point]
        self.traj_point += 1
        return self.active_leg, phase, px_val, py_val, pz_val, t

# ============================================================================
# SECTION 7 — BUTTON-BASED MODE SELECTOR
# ============================================================================
# PyBullet buttons: addUserDebugParameter(name, rangeMin=1, rangeMax=0, start=0)
# renders a button. Each press increments the read value by 1. We track the
# last-seen value per button and fire on a rising edge (new_val > last_val).

_btn_ids   = {}      # button_name → parameter_id
_btn_last  = {}      # button_name → last read integer value
_current_mode_str   = None          # 'FORWARD' | 'LEFT' | 'RIGHT' | None
_current_mode_label = 'STOP'

_MODE_LABEL = {
    None:      'STOP',
    'FORWARD': 'FORWARD WALK',
    'LEFT':    'LEFT TURN',
    'RIGHT':   'RIGHT TURN',
}


def create_mode_buttons():
    """Register the four mode buttons in the PyBullet GUI panel.

    PyBullet renders a debug parameter as a BUTTON (not a slider) when
    rangeMin > rangeMax. Each click increments the value readUserDebugParameter
    returns, so a press is a change in that counter.
    """
    global _btn_ids, _btn_last
    buttons = [
        ("STOP",         "[  STOP  ]"),
        ("FORWARD",      "[ FORWARD WALK ]"),
        ("LEFT",         "[ LEFT TURN ]"),
        ("RIGHT",        "[ RIGHT TURN ]"),
    ]
    for key, label in buttons:
        # rangeMin=1, rangeMax=0 → BUTTON. startValue=0 → counter starts at 0.
        bid = p.addUserDebugParameter(label, 1, 0, 0)
        _btn_ids[key]  = bid
        _btn_last[key] = float(p.readUserDebugParameter(bid))


def poll_mode_buttons():
    """
    Check all four buttons for rising edges.
    Returns the new (mode_str, mode_label) if a button was pressed,
    otherwise returns the current unchanged state.
    """
    global _current_mode_str, _current_mode_label, _btn_last

    pressed = None
    for key, bid in _btn_ids.items():
        try:
            val = float(p.readUserDebugParameter(bid))
        except Exception:
            continue
        # A real PyBullet button increments its counter on each click.
        if val > _btn_last[key]:
            _btn_last[key] = val
            pressed = key

    if pressed is not None:
        new_str   = None if pressed == "STOP" else pressed
        new_label = _MODE_LABEL[new_str]
        _current_mode_str   = new_str
        _current_mode_label = new_label
        return new_str, new_label, True     # (mode_str, label, changed)

    return _current_mode_str, _current_mode_label, False

# ============================================================================
# SECTION 8 — VISUALIZATION
# ============================================================================

_text_ids = {}

def update_display(robot_id, leg_id, phase, px, py, pz, t,
                   current_mode_label, imu, posture=None):
    """
    Unified debug text overlay.
    Rows: title | mode | active leg | phase | foot pos | IMU roll/pitch/yaw | IMU ω | posture
    """
    global _text_ids

    cfg = LEG_CONFIG[leg_id]
    base_pos, _ = p.getBasePositionAndOrientation(robot_id)
    tx, ty, tz  = base_pos[0], base_pos[1], base_pos[2] + 0.18

    roll, pitch, yaw = imu.get_euler_deg()
    wx, wy, wz       = imu.get_angular_velocity()

    tilt_color = [1.0, 0.2, 0.2] if imu.is_tilting(20.0) else [0.2, 1.0, 0.6]

    lines = [
        ("title",  "UNIFIED SIM  |  Stage 9",
                   [tx, ty, tz+0.16], [1.0, 1.0, 0.3], 1.5),
        ("mode",   f"MODE: {current_mode_label}",
                   [tx, ty, tz+0.14], [1.0, 0.4, 0.4], 1.5),
        ("leg",    f"Active: {cfg['label']}",
                   [tx, ty, tz+0.12], [0.3, 1.0, 0.3], 1.2),
        ("phase",  f"Phase: {phase}  t={t:.2f}",
                   [tx, ty, tz+0.10], [0.3, 0.8, 1.0], 1.1),
        ("foot",   f"PX:{px:.1f}  PY:{py:+.1f}  PZ:{pz:.1f} mm",
                   [tx, ty, tz+0.08], [1.0, 0.6, 0.2], 1.1),
        ("imu1",   f"IMU  R:{roll:+6.2f}  P:{pitch:+6.2f}  Y:{yaw:+6.2f} deg",
                   [tx, ty, tz+0.06], tilt_color, 1.1),
        ("imu2",   f"     wx:{wx:+6.1f}  wy:{wy:+6.1f}  wz:{wz:+6.1f} deg/s",
                   [tx, ty, tz+0.04], tilt_color, 1.1),
    ]

    if posture is not None:
        d = posture.get_debug_state()
        dpz = d["dpz"]
        lines.append((
            "posture",
            f"PD  R:{d['roll_f']:+5.2f} uR:{d['u_roll']:+5.2f}  "
            f"P:{d['pitch_f']:+5.2f} uP:{d['u_pitch']:+5.2f}  "
            f"Δpz[{dpz[1]:+4.1f} {dpz[2]:+4.1f} {dpz[3]:+4.1f} {dpz[4]:+4.1f}]"
            f"{'  SAT' if d.get('sat') else ''}",
            [tx, ty, tz+0.02], [0.6, 0.9, 1.0], 1.1))

    for key, text, pos, color, size in lines:
        if key in _text_ids and _text_ids[key] >= 0:
            _text_ids[key] = p.addUserDebugText(
                text, pos,
                textColorRGB=color, textSize=size,
                replaceItemUniqueId=_text_ids[key])
        else:
            _text_ids[key] = p.addUserDebugText(
                text, pos, textColorRGB=color, textSize=size)


def update_camera():
    try:
        p.resetDebugVisualizerCamera(
            cameraDistance       = p.readUserDebugParameter(cam_dist_slider),
            cameraYaw            = p.readUserDebugParameter(cam_yaw_slider),
            cameraPitch          = p.readUserDebugParameter(cam_pitch_slider),
            cameraTargetPosition = [
                p.readUserDebugParameter(cam_target_x_slider),
                p.readUserDebugParameter(cam_target_y_slider),
                p.readUserDebugParameter(cam_target_z_slider),
            ]
        )
    except Exception:
        pass

# ============================================================================
# SECTION 8b — DYNAMIC POSTURE CONTROLLER  (Roll + Pitch PD)
# ============================================================================
# Closed-loop Z-admittance posture control. Reads IMU roll/pitch + body gyro,
# produces per-leg vertical corrections (Δpz) that are ADDED to the
# locomotion-owned pz before IK. Never touches px, py, gait scheduling,
# trajectories, or turning logic.
#
# Pure data-in/data-out object (no PyBullet handles), so a hardware / ROS2
# port is a wrapper exercise.
#
# Mixing (derived from LEG_CONFIG hip positions, read-only):
#   ROLL  (about body X): moment arm = foot Y.  Legs with hip_y > 0 (2,3) are
#         the +Y side. +roll raises +Y → push +Y legs' feet UP (Δpz>0) and
#         -Y legs' feet DOWN (Δpz<0):   mix_roll[leg]  = sign(hip_y)
#   PITCH (about body Y): moment arm = foot X.  +pitch lowers +X side (2,4)
#         → +X legs' feet DOWN (Δpz<0), -X legs' feet UP:
#                                        mix_pitch[leg] = -sign(hip_x)
#   Both verified numerically (roll/pitch sign test + closed-loop sim).
#
# D-term: PD acts on the ERROR and its derivative with the SAME sign
#   u = Kp*e + Kd*ė. ė is the Euler-angle rate computed from the BODY gyro:
#     φ̇ = p + (q sinφ + r cosφ) tanθ ,   θ̇ = q cosφ − r sinφ
#   This is heading-independent (the old world-frame wx was only correct at
#   yaw = π by accident, and became the PITCH rate at yaw = ±90°).


def _soft_deadband(e, db):
    """Continuous deadband: 0 inside ±db, then e shifted toward 0 by db."""
    if e > db:
        return e - db
    if e < -db:
        return e + db
    return 0.0


class PostureController:
    """Roll + pitch PD posture controller producing per-leg Δpz (mm)."""

    _MIX_ROLL  = {lid: (1.0 if cfg['hip_xyz_urdf'][1] > 0 else -1.0)
                  for lid, cfg in LEG_CONFIG.items()}
    _MIX_PITCH = {lid: (-1.0 if cfg['hip_xyz_urdf'][0] > 0 else 1.0)
                  for lid, cfg in LEG_CONFIG.items()}

    def __init__(self):
        self._roll_f       = 0.0
        self._pitch_f      = 0.0
        self._roll_rate_f  = 0.0
        self._pitch_rate_f = 0.0
        self._initialized  = False
        self._ref_roll     = 0.0
        self._ref_pitch    = 0.0
        # Per-leg authority weight (swing exclusion ramp), 1 = full authority
        self._w = {1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0}
        # Integral state (mm) and anti-windup flags
        self._i_roll  = 0.0
        self._i_pitch = 0.0
        self._sat_prev = {"roll": 0.0, "pitch": 0.0}   # sign of saturation
        self._ik_ok_prev = True
        self._dbg = {"u_roll": 0.0, "u_pitch": 0.0, "roll_f": 0.0,
                     "pitch_f": 0.0, "dpz": {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0},
                     "w": dict(self._w), "sat": False,
                     "i_roll": 0.0, "i_pitch": 0.0}

    # -- lifecycle ----------------------------------------------------------
    def reset(self):
        self.__init__()

    def set_level_reference(self, roll_deg, pitch_deg):
        """Set the 'flat' orientation (IMU mounting offset)."""
        self._ref_roll  = roll_deg
        self._ref_pitch = pitch_deg
        self._roll_f = self._pitch_f = 0.0
        self._roll_rate_f = self._pitch_rate_f = 0.0
        self._initialized = True

    def report_ik(self, all_ok):
        """Main loop reports whether every corrected leg target was IK-feasible
        this tick. Integration is frozen on the next tick if not."""
        self._ik_ok_prev = bool(all_ok)

    def _integrate(self, i_state, err_db, ki, sat_sign, allowed):
        dt = POSTURE_DT
        if POSTURE_I_LEAK_TAU > 0:
            i_state *= math.exp(-dt / POSTURE_I_LEAK_TAU)
        if not allowed or ki == 0.0:
            return i_state
        inc = ki * err_db * dt
        # conditional integration: don't push further into saturation
        if sat_sign != 0.0 and math.copysign(1.0, inc) == sat_sign:
            return i_state
        return max(-POSTURE_I_MAX_MM, min(POSTURE_I_MAX_MM, i_state + inc))

    # -- helpers -------------------------------------------------------------
    @staticmethod
    def _euler_rates(roll_deg, pitch_deg, wx, wy, wz):
        """Body rates (deg/s) → roll/pitch Euler rates (deg/s), ZYX Euler."""
        phi, th = math.radians(roll_deg), math.radians(pitch_deg)
        th = max(-1.4, min(1.4, th))          # keep tan() finite
        roll_dot  = wx + (wy * math.sin(phi) + wz * math.cos(phi)) * math.tan(th)
        pitch_dot = wy * math.cos(phi) - wz * math.sin(phi)
        return roll_dot, pitch_dot

    def _update_weights(self, gait_context):
        """Ramp each leg's authority toward 0 (excluded) or 1 (included)."""
        excluded = set(gait_context.get("exclude_legs", ()))
        step = 1.0 / max(1, POSTURE_FADE_TICKS)
        for lid in self._w:
            target = 0.0 if lid in excluded else 1.0
            w = self._w[lid]
            if w < target:
                w = min(target, w + step)
            elif w > target:
                w = max(target, w - step)
            self._w[lid] = w
        # A leg that is actually airborne gets exactly zero, never a residual.
        swing = gait_context.get("swing_leg")
        if swing in self._w and gait_context.get("airborne", False):
            self._w[swing] = 0.0

    # -- main entry point ---------------------------------------------------
    def update(self, imu_sample, gait_context):
        """
        imu_sample:   dict roll_deg, pitch_deg, wx, wy, wz (BODY-frame deg/s)
        gait_context: dict mode, swing_leg, airborne, exclude_legs, dpz_max
        Returns: dict {leg_id: Δpz_mm}
        """
        if not POSTURE_ENABLED:
            zero = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
            self._dbg.update(u_roll=0.0, u_pitch=0.0, roll_f=0.0,
                             pitch_f=0.0, dpz=dict(zero))
            return zero

        # 1) Error (level reference = IMU mounting offset)
        roll_err  = imu_sample["roll_deg"]  - self._ref_roll
        pitch_err = imu_sample["pitch_deg"] - self._ref_pitch
        # 2) Euler rates from body gyro (heading independent)
        roll_rate, pitch_rate = self._euler_rates(
            imu_sample["roll_deg"], imu_sample["pitch_deg"],
            imu_sample["wx"], imu_sample["wy"], imu_sample.get("wz", 0.0))

        # 3) EMA low-pass
        if not self._initialized:
            self._roll_f, self._pitch_f = roll_err, pitch_err
            self._roll_rate_f, self._pitch_rate_f = roll_rate, pitch_rate
            self._initialized = True
        else:
            a, g = POSTURE_EMA_ALPHA_ANGLE, POSTURE_EMA_ALPHA_GYRO
            self._roll_f       = a * roll_err   + (1 - a) * self._roll_f
            self._pitch_f      = a * pitch_err  + (1 - a) * self._pitch_f
            self._roll_rate_f  = g * roll_rate  + (1 - g) * self._roll_rate_f
            self._pitch_rate_f = g * pitch_rate + (1 - g) * self._pitch_rate_f

        # 4) Soft deadband on the angle error (continuous, no edge jump).
        #    The D-term is NOT deadbanded: damping stays active near level.
        db = POSTURE_DEADBAND_DEG
        roll_db  = _soft_deadband(self._roll_f,  db)
        pitch_db = _soft_deadband(self._pitch_f, db)

        # 5) Integral (anti-windup: clamp, conditional, IK-freeze, deadband)
        standing = gait_context.get("mode") == "STOP"
        allowed = self._ik_ok_prev and (POSTURE_I_MODE == "always" or standing)
        self._i_roll  = self._integrate(self._i_roll,  roll_db,  POSTURE_KI_ROLL,
                                        self._sat_prev["roll"],  allowed)
        self._i_pitch = self._integrate(self._i_pitch, pitch_db, POSTURE_KI_PITCH,
                                        self._sat_prev["pitch"], allowed)

        # 6) PID — P, I and D on the same side: u = Kp*e + I + Kd*ė
        u_roll  = POSTURE_ROLL_POLARITY  * (POSTURE_KP_ROLL  * roll_db
                                            + self._i_roll
                                            + POSTURE_KD_ROLL  * self._roll_rate_f)
        u_pitch = POSTURE_PITCH_POLARITY * (POSTURE_KP_PITCH * pitch_db
                                            + self._i_pitch
                                            + POSTURE_KD_PITCH * self._pitch_rate_f)

        # 6) Mixing + per-leg authority weights (smooth swing exclusion)
        self._update_weights(gait_context)
        dpz_max = gait_context.get("dpz_max", POSTURE_DPZ_MAX_FWD)
        dpz, sat = {}, False
        sat_r = sat_p = 0.0
        for lid in (1, 2, 3, 4):
            vp = self._MIX_PITCH[lid] * u_pitch
            if POSTURE_PITCH_MIX == "retract_only":
                # Same pitch moment, applied only by lifting the high side.
                vp = max(0.0, 2.0 * vp)
            vr = self._MIX_ROLL[lid] * u_roll
            v = vr + vp
            if abs(v) > dpz_max:
                sat = True
                v = math.copysign(dpz_max, v)
                # which axis is pushing this leg into the limit (in u units)
                if vr != 0.0 and math.copysign(1.0, vr) == math.copysign(1.0, v):
                    sat_r = math.copysign(1.0, u_roll)
                if vp != 0.0 and math.copysign(1.0, vp) == math.copysign(1.0, v):
                    sat_p = math.copysign(1.0, u_pitch)
            dpz[lid] = self._w[lid] * v
        # anti-windup memory, expressed in the I-state's sign convention
        self._sat_prev["roll"]  = sat_r * POSTURE_ROLL_POLARITY
        self._sat_prev["pitch"] = sat_p * POSTURE_PITCH_POLARITY

        self._dbg.update(u_roll=u_roll, u_pitch=u_pitch,
                         roll_f=self._roll_f, pitch_f=self._pitch_f,
                         dpz=dict(dpz), w=dict(self._w), sat=sat,
                         i_roll=self._i_roll, i_pitch=self._i_pitch)
        return dpz

    def get_debug_state(self):
        return self._dbg


def posture_gait_context(mode_str, leg_id, phase, gsm, turning):
    """Build the posture controller's view of the gait (read-only).

    exclude_legs: legs whose authority should be (or ramp to) zero:
      - the leg currently swinging (LIFT/SWEEP/LOWER)
      - forward STABILIZE: the leg about to lift (it lifts after 40 ticks)
      - turning LOWER: the NEXT leg in GAIT_ORDER (6B lifts it immediately
        after this LOWER ends, with no STABILIZE in between)
    """
    airborne = phase in ("LIFT", "SWEEP", "LOWER")
    exclude = [leg_id]
    if turning and phase == "LOWER":
        order = gsm.gait_order
        exclude.append(order[(order.index(leg_id) + 1) % len(order)])
    return {
        "mode": mode_str,
        "swing_leg": leg_id,
        "airborne": airborne,
        "exclude_legs": exclude,
        "dpz_max": POSTURE_DPZ_MAX_TURN if turning else POSTURE_DPZ_MAX_FWD,
    }


def set_leg_with_posture(robot_id, lid, px, py, pz_loco, dpz):
    """Command a leg with the posture correction; if that target is outside
    the IK envelope, fall back to the pure locomotion target (exactly what the
    gait would have commanded) so the leg never freezes. Returns
    (ok_with_correction, ok_final)."""
    if set_leg(robot_id, lid, px, py, pz_loco + dpz):
        return True, True
    return False, set_leg(robot_id, lid, px, py, pz_loco)

# ============================================================================
# SECTION 9 — UNIFIED MAIN LOOP
# ============================================================================

def run_unified_sim(robot_id, imu, posture):
    """
    Infinite gait loop driven by the mode slider.

    Mode changes take effect at the START of the next tick:
      - STOP (0):    stepSimulation still runs; servos hold; GSM idles.
      - FORWARD (1): STABILIZE + CoM-shift gait (FREE_BASE behaviour).
      - LEFT (2):    Stage 6B turning (per-leg stance reversal, no CoM).
      - RIGHT (3):   Stage 6B turning (per-leg stance reversal, no CoM).

    Forward and turning use different stance geometry (px/lift/stride);
    the turning path is reproduced from turn_gait_real.py exactly.
    """

    # Pre-compute stride timing constants.
    #   Forward stance sweep uses STEP_LENGTH spread over the full cycle.
    #   Turning (6B) uses its own delta: (STEP_LENGTH_TURN/3)/total_per_leg,
    #   exactly as turn_gait_real.py computes stance_delta_per_point.
    phase_len           = 40 + (INTERP_STEPS * 3)
    total_planted_ticks = (3 * phase_len) + 40
    stance_delta        = STEP_LENGTH / float(total_planted_ticks)

    total_per_leg_turn   = INTERP_STEPS * 3
    stance_delta_turn    = (STEP_LENGTH_TURN / 3.0) / float(total_per_leg_turn)

    # Initial staggered foot targets — forward's STEP_LENGTH spread (FREE_BASE).
    # Forward depends on this exact 80mm-based stagger for its support sweep.
    # Turning resets each leg's py to its 6B AEP on that leg's first swing, so
    # turning converges to 6B geometry within one gait cycle from here.
    initial_py = {
        1: -STEP_LENGTH / 2.0,
        2: -STEP_LENGTH / 4.0,
        4:  STEP_LENGTH / 4.0,
        3:  STEP_LENGTH / 2.0,
    }

    foot_targets = {
        lid: {
            "px": STAND_PX,
            "py": initial_py[lid],
            "pz": STAND_PZ,
            "temp_dx": 0.0,
            "temp_dy": 0.0,
            "mode": "FORWARD",   # 6B per-leg stance-direction memory
        }
        for lid in range(1, 5)
    }

    for lid in range(1, 5):
        set_leg(robot_id, lid,
                foot_targets[lid]["px"],
                foot_targets[lid]["py"],
                foot_targets[lid]["pz"])
    settle(240)

    gsm                = ContinuousGaitStateMachine(GAIT_ORDER, foot_targets)
    total_ik_failures  = 0
    current_active_leg = None
    step_count         = 0

    print("\n  ===== UNIFIED SIMULATOR RUNNING =====")
    print("  Press buttons in GUI:  STOP | FORWARD WALK | LEFT TURN | RIGHT TURN")
    print("  Ctrl-C to quit.\n")

    try:
        while True:
            # ── Poll mode buttons ─────────────────────────────────────────────
            mode_str, mode_label, changed = poll_mode_buttons()

            if changed:
                print(f"  [MODE] → {mode_label}")

            # ── STOP: idle tick — hold pose (optionally level via posture) ────
            if mode_str is None:
                imu.update()

                if POSTURE_HOLD_WHEN_STOPPED:
                    roll_deg, pitch_deg, _yaw = imu.get_euler_deg()
                    wx, wy, wz = imu.get_angular_velocity()
                    imu_sample = {"roll_deg": roll_deg, "pitch_deg": pitch_deg,
                                  "wx": wx, "wy": wy, "wz": wz}
                    gait_context = {
                        "mode": "STOP",
                        "swing_leg": None,          # all four grounded
                        "airborne": False,
                        "exclude_legs": (),
                        "dpz_max": POSTURE_DPZ_MAX_FWD,
                    }
                    corr = posture.update(imu_sample, gait_context)
                    all_ok = True
                    for lid in range(1, 5):
                        base_px = foot_targets[lid]["px"]
                        base_py = foot_targets[lid]["py"]
                        ok_c, _ = set_leg_with_posture(
                            robot_id, lid, base_px, base_py,
                            foot_targets[lid]["pz"], corr[lid])
                        if not ok_c:
                            total_ik_failures += 1
                            all_ok = False
                    posture.report_ik(all_ok)

                if step_count % IMU_PRINT_INTERVAL == 0:
                    print(imu.get_telemetry_string())
                for _ in range(SIM_SUBSTEPS):
                    update_camera()
                    p.stepSimulation()
                    time.sleep(STEP_DELAY / SIM_SUBSTEPS)
                step_count += 1
                continue

            # ── ACTIVE GAIT TICK ──────────────────────────────────────────────
            leg_id, phase, val1, val2, pz_val, t = gsm.advance(mode_str, robot_id)

            if leg_id != current_active_leg and phase != "STABILIZE":
                current_active_leg = leg_id
                print(f"    Swing Leg {leg_id}  |  {mode_label}  "
                      f"|  swings={gsm.legs_completed}")

            # ── Unpack phase values ───────────────────────────────────────────
            turning = mode_str in ('LEFT', 'RIGHT')

            if phase == "STABILIZE":
                # Forward-only phase. Update CoM shift targets.
                for lid in gsm.support_legs:
                    foot_targets[lid]["temp_dx"] = val1
                    foot_targets[lid]["temp_dy"] = val2
                # FREE_BASE sweeps ALL FOUR legs during STABILIZE (py -= stance_delta).
                sweep_legs = [1, 2, 3, 4]

            elif phase in ("LIFT", "SWEEP", "LOWER"):
                sweep_legs = gsm.support_legs
                # Active swing leg follows the trajectory (px,py,pz).
                foot_targets[leg_id]["px"] = val1
                foot_targets[leg_id]["py"] = val2
                foot_targets[leg_id]["pz"] = pz_val
                if not turning:
                    # Forward CoM transient relaxes toward zero each tick.
                    # Turning (6B) uses no CoM transient at all.
                    for lid in range(1, 5):
                        foot_targets[lid]["temp_dx"] *= 0.90
                        foot_targets[lid]["temp_dy"] *= 0.90

            else:
                sweep_legs = gsm.support_legs

            # ── Stance sweep — mode-specific ──────────────────────────────────
            for lid in sweep_legs:
                if turning:
                    # Stage 6B: drag each planted foot in py by its own stored
                    # stance direction.  Verbatim from turn_gait_real.py.
                    # Pin px to the wide turning stance so all four legs share
                    # the 180mm geometry 6B assumes.
                    foot_targets[lid]["px"] = STAND_PX_TURN
                    leg_mode = foot_targets[lid].get("mode", mode_str)
                    _, stance_dir = get_leg_params(lid, leg_mode)
                    foot_targets[lid]["py"] -= stance_delta_turn * stance_dir
                else:
                    # Forward: longitudinal stance sweep (FREE_BASE behaviour).
                    foot_targets[lid]["py"] -= stance_delta

            # ── IMU acquisition (single update/tick; read BEFORE IK) ──────────
            imu.update()
            roll_deg, pitch_deg, _yaw = imu.get_euler_deg()
            wx, wy, wz = imu.get_angular_velocity()

            # ── Posture controller → per-leg Δpz corrections ──────────────────
            imu_sample = {"roll_deg": roll_deg, "pitch_deg": pitch_deg,
                          "wx": wx, "wy": wy, "wz": wz}
            gait_context = posture_gait_context(mode_str, leg_id, phase,
                                                gsm, turning)
            corr = posture.update(imu_sample, gait_context)
            all_ok = True

            # ── CoM stabilization (forward) / raw apply (6B turning) + IK ─────
            for lid in range(1, 5):
                base_px = foot_targets[lid]["px"]
                base_py = foot_targets[lid]["py"]

                if turning:
                    # Stage 6B applies foot targets directly — no CoM shift.
                    transient_px = base_px
                    transient_py = base_py
                else:
                    outward_sign = (1.0 if LEG_CONFIG[lid]['hip_xyz_urdf'][1] > 0
                                    else -1.0)
                    transient_px = base_px - (foot_targets[lid]["temp_dy"] * outward_sign)
                    transient_py = base_py - foot_targets[lid]["temp_dx"]

                # Inject posture correction: final pz = locomotion pz + Δpz.
                # Counted for ALL legs; an infeasible corrected target falls
                # back to the pure locomotion target instead of freezing.
                ok_c, _ = set_leg_with_posture(robot_id, lid,
                                               transient_px, transient_py,
                                               foot_targets[lid]["pz"],
                                               corr[lid])
                if not ok_c:
                    total_ik_failures += 1
                    all_ok = False
            posture.report_ik(all_ok)

            # ── IMU telemetry print ───────────────────────────────────────────
            if step_count % IMU_PRINT_INTERVAL == 0:
                print(imu.get_telemetry_string())

            # ── Visualization ─────────────────────────────────────────────────
            active_px = foot_targets[leg_id]["px"]
            active_py = foot_targets[leg_id]["py"]
            active_pz = foot_targets[leg_id]["pz"]
            update_display(robot_id, leg_id, phase,
                           active_px, active_py, active_pz,
                           t, mode_label, imu, posture)

            # ── Simulation step ───────────────────────────────────────────────
            for _ in range(SIM_SUBSTEPS):
                update_camera()
                p.stepSimulation()
                time.sleep(STEP_DELAY / SIM_SUBSTEPS)

            step_count += 1

    except KeyboardInterrupt:
        r, pitch, yaw = imu.get_euler_deg()
        print(f"\n  Simulation stopped at step {step_count}.")
        print(f"  IK failures: {total_ik_failures}")
        print(f"  Final IMU — Roll:{r:+.2f}°  Pitch:{pitch:+.2f}°  Yaw:{yaw:+.2f}°")

# ============================================================================
# SECTION 10 — MAIN
# ============================================================================

# GUI slider globals
cam_dist_slider     = None
cam_yaw_slider      = None
cam_pitch_slider    = None
cam_target_x_slider = None
cam_target_y_slider = None
cam_target_z_slider = None


def main():
    global cam_dist_slider, cam_yaw_slider, cam_pitch_slider
    global cam_target_x_slider, cam_target_y_slider, cam_target_z_slider

    # ── PyBullet setup ────────────────────────────────────────────────────────
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=0.9)   # matches turn_gait.py
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

    # ── Mode buttons (replaces slider) ────────────────────────────────────────
    create_mode_buttons()

    # ── Camera sliders ────────────────────────────────────────────────────────
    cam_dist_slider     = p.addUserDebugParameter("Camera Distance",  0.2, 3.0,  0.65)
    cam_yaw_slider      = p.addUserDebugParameter("Camera Yaw",      -180, 180,  55)
    cam_pitch_slider    = p.addUserDebugParameter("Camera Pitch",      -89, 89,  -30)
    cam_target_x_slider = p.addUserDebugParameter("Target X",        -2.0, 2.0,  0.05)
    cam_target_y_slider = p.addUserDebugParameter("Target Y",        -2.0, 2.0,  0.05)
    cam_target_z_slider = p.addUserDebugParameter("Target Z",        -1.0, 2.0,  0.25)

    # ── Robot spawn ───────────────────────────────────────────────────────────
    robot_id = p.loadURDF(
        URDF_PATH,
        [0, 0, BASE_SPAWN_HEIGHT],
        p.getQuaternionFromEuler([0, 0, math.pi]),
        useFixedBase=USE_FIXED_BASE
    )
    resolve_joint_indices(robot_id)

    # Hard-reset joints to standing pose before gravity settle
    for lid in range(1, 5):
        jc, jf, jt, _ = compute_leg_joints(lid, STAND_PX, 0.0, STAND_PZ)
        cfg = LEG_CONFIG[lid]
        p.resetJointState(robot_id, cfg['coxa_idx'],  math.radians(jc))
        p.resetJointState(robot_id, cfg['femur_idx'], math.radians(jf))
        p.resetJointState(robot_id, cfg['tibia_idx'], math.radians(jt))

    for lid, cfg in LEG_CONFIG.items():
        p.changeDynamics(robot_id, cfg['tibia_idx'],
                         lateralFriction=0.9,
                         spinningFriction=0.1,
                         rollingFriction=0.01)

    # ── Stage 9 IMU — constructed once, never re-created on mode switch ───────
    imu = create_imu(
        mode="sim",                      # swap to "bno055" or "smartelex" for hardware
        pybullet_client=p,
        robot_id=robot_id,
        noise_std_deg=0.10,              # ±0.1° RMS orientation noise (BNO055-like)
        gyro_noise_std_deg_s=0.50,       # ±0.5°/s RMS gyro noise
        yaw_drift_rate_deg_s=0.00,       # set >0 to simulate heading drift
        update_rate_hz=240.0,
    )
    imu.reset_yaw_reference()
    print("[IMU] Simulated IMU initialised. Yaw reference zeroed at spawn.")

    # ── Hold the stand pose with the servos BEFORE the gravity settle ────────
    # resetJointState only teleports joints; without a position command the
    # legs are held only by PyBullet's default velocity motors and sag during
    # settle(). The level reference below would then be captured on a sagged
    # pose. Commanding the stand pose first makes the reference pose the one
    # the servos actually hold.
    for lid in range(1, 5):
        set_leg(robot_id, lid, STAND_PX, 0.0, STAND_PZ)

    # ── Gravity settle at standing pose ──────────────────────────────────────
    settle(SETTLE_STEPS)

    # ── Dynamic Posture Controller (Roll + Pitch PD) ──────────────────────────
    # Level reference = IMU mounting offset, averaged over a still window on
    # the servo-held stand pose. A large offset means a bad pose, not a mount
    # offset → fall back to the gravity-level reference (0, 0).
    posture = PostureController()
    n_ref = POSTURE_REF_SAMPLES
    sum_r = sum_p = 0.0
    for _ in range(n_ref):
        imu.update()
        r0, p0, _ = imu.get_euler_deg()
        sum_r += r0
        sum_p += p0
        p.stepSimulation()
    ref_r, ref_p = sum_r / n_ref, sum_p / n_ref
    if max(abs(ref_r), abs(ref_p)) > POSTURE_REF_MAX_DEG:
        print(f"[POSTURE] WARNING: captured level ref roll={ref_r:+.2f}° "
              f"pitch={ref_p:+.2f}° exceeds ±{POSTURE_REF_MAX_DEG}° — "
              f"using gravity level (0, 0) instead.")
        ref_r, ref_p = 0.0, 0.0
    posture.set_level_reference(ref_r, ref_p)
    print(f"[POSTURE] Roll+Pitch PD ready. Level ref "
          f"roll={ref_r:+.2f}°  pitch={ref_p:+.2f}°  "
          f"(enabled={POSTURE_ENABLED})")

    # ── Run infinite loop ─────────────────────────────────────────────────────
    run_unified_sim(robot_id, imu, posture)

    p.disconnect()


if __name__ == "__main__":
    main()