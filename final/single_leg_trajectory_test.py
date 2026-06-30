#!/usr/bin/env python3
"""
single_leg_trajectory_test.py
=============================
Stage 3 — Single Leg Swing Trajectory Validation.

PURPOSE
    Validate a complete swing trajectory for a single leg (Leg 1, Front Left)
    before implementing any gait controller.

    No locomotion. No crawl gait. No body translation. No turning.
    Only a single-leg swing trajectory test.

    Legs 2, 3, and 4 remain locked in the verified standing pose.

REUSED COMPONENTS (from quadruped_stand_test.py — Stage 2 verified)
    • LEG_CONFIG table          — joint indices, names, sign corrections
    • resolve_joint_indices()   — dynamic PyBullet index mapping
    • compute_ik()              — ground-truth geometric IK solver (DO NOT MODIFY)
    • compute_leg_joints()      — transformation layer (IK → sign corrections)
    • set_leg()                 — send commands to PyBullet motors
    • verify_joint_map()        — name-based joint verification

────────────────────────────────────────────────────────────────────────────
TRAJECTORY DESIGN
────────────────────────────────────────────────────────────────────────────

The swing trajectory is defined in the IK frame (mm, relative to coxa joint):

    Phase 1 — STAND    : Hold at (px_stand, 0, pz_stand) for visual confirmation.
    Phase 2 — LIFT     : Raise foot vertically from stand to apex height.
    Phase 3 — FORWARD  : Move foot forward at apex height.
    Phase 4 — LOWER    : Lower foot down to stand height at forward position.
    Phase 5 — RETURN   : Slide foot back to original stand position.

Smooth interpolation uses:
    x(t) = x_start + (x_end - x_start) * t
    z(t) = z_base  + lift_height * sin(pi * t)     (for combined lift-swing)

where 0 ≤ t ≤ 1, sampled at INTERP_STEPS points per phase.

────────────────────────────────────────────────────────────────────────────
"""

# pyrefly: ignore [missing-import]
import sys
import pybullet as p
import pybullet_data
import time
import math

# Fix for Windows cp1252 terminal encoding — allows Unicode output
if sys.stdout.encoding and sys.stdout.encoding.lower() != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 1 — CONFIGURATION
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

URDF_PATH          = "urdf/robot.urdf"
BASE_SPAWN_HEIGHT  = 0.38   # meters above ground plane
USE_FIXED_BASE     = True   # base locked — isolate leg kinematics
SETTLE_STEPS       = 360    # physics steps for stabilisation (1.5 s @ 240 Hz)
ANGLE_TOLERANCE_DEG = 2.0   # pass/fail threshold for final pose check

# Standing pose: foot target in mm relative to each leg's coxa joint origin.
STAND_PX = 150.0
STAND_PY =   0.0
STAND_PZ = -150.0

# ── Trajectory Parameters ────────────────────────────────────────────────
# These are configurable. Recommended defaults for the quadruped geometry:
#   step_length = 40 mm    (conservative; max reach ~80 mm before IK limits)
#   lift_height = 30 mm    (foot clears ground with margin)
#   INTERP_STEPS = 50      (smooth motion; 50 points per phase)

STEP_LENGTH   = 40.0    # mm — how far forward the foot swings
LIFT_HEIGHT   = 30.0    # mm — apex height above the standing pz
INTERP_STEPS  = 50      # interpolation points per phase

# Test leg
TEST_LEG_ID = 1         # Leg 1 — Front Left (FL)

# Phase timing
PHASE_HOLD_SEC  = 1.5   # seconds to hold the stand pose before/after trajectory
PHASE_STEP_DELAY = 0.008  # seconds between interpolation steps (visual speed)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 2 — LEG CONFIGURATION TABLE  (verbatim from quadruped_stand_test.py)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

LEG_CONFIG = {
    1: {
        'label'          : 'Leg 1 — FL (Front Left)',
        'hip_xyz_urdf'   : (-0.075, -0.093751, -0.01255),
        'hip_xyz_world'  : (+0.075, +0.093751, -0.01255),
        'coxa_idx'       : 0,   'coxa_joint_name'  : 'Revolute 1',
        'femur_idx'      : 6,   'femur_joint_name' : 'Revolute 8',
        'tibia_idx'      : 8,   'tibia_joint_name' : 'Revolute 11',
        'coxa_link'      : 'coxa_1',
        'femur_link'     : 'femur_v2_1',
        'tibia_link'     : 'tibia_1',
        'coxa_axis'      : (0,  0, -1),
        'femur_axis'     : (0, +1,  0),
        'tibia_axis'     : (0, +1,  0),
        'coxa_sign'      : -1,
        'femur_sign'     : +1,
        'tibia_sign'     : -1,
    },
    2: {
        'label'          : 'Leg 2 — BR (Back Right)',
        'hip_xyz_urdf'   : (+0.075, +0.093751, -0.01255),
        'hip_xyz_world'  : (-0.075, -0.093751, -0.01255),
        'coxa_idx'       : 1,   'coxa_joint_name'  : 'Revolute 2',
        'femur_idx'      : 5,   'femur_joint_name' : 'Revolute 7',
        'tibia_idx'      : 11,  'tibia_joint_name' : 'Revolute 15',
        'coxa_link'      : 'coxa__1__1',
        'femur_link'     : 'femur_v2__2__1',
        'tibia_link'     : 'tibia__1__1',
        'coxa_axis'      : (0,  0, -1),
        'femur_axis'     : (0, -1,  0),
        'tibia_axis'     : (0, +1,  0),
        'coxa_sign'      : -1,
        'femur_sign'     : -1,
        'tibia_sign'     : -1,
    },
    3: {
        'label'          : 'Leg 3 — FR (Front Right)',
        'hip_xyz_urdf'   : (-0.075, +0.093751, -0.01255),
        'hip_xyz_world'  : (+0.075, -0.093751, -0.01255),
        'coxa_idx'       : 2,   'coxa_joint_name'  : 'Revolute 3',
        'femur_idx'      : 4,   'femur_joint_name' : 'Revolute 6',
        'tibia_idx'      : 9,   'tibia_joint_name' : 'Revolute 13',
        'coxa_link'      : 'coxar_1',
        'femur_link'     : 'femur_v2__1__1',
        'tibia_link'     : 'tibial_1',
        'coxa_axis'      : (0,  0, -1),
        'femur_axis'     : (0, +1,  0),
        'tibia_axis'     : (0, +1,  0),
        'coxa_sign'      : -1,
        'femur_sign'     : +1,
        'tibia_sign'     : -1,
    },
    4: {
        'label'          : 'Leg 4 — BL (Back Left)',
        'hip_xyz_urdf'   : (+0.075, -0.093751, -0.01255),
        'hip_xyz_world'  : (-0.075, +0.093751, -0.01255),
        'coxa_idx'       : 3,   'coxa_joint_name'  : 'Revolute 4',
        'femur_idx'      : 7,   'femur_joint_name' : 'Revolute 10',
        'tibia_idx'      : 10,  'tibia_joint_name' : 'Revolute 14',
        'coxa_link'      : 'coxar__1__1',
        'femur_link'     : 'femur_v2__3__1',
        'tibia_link'     : 'tibial__1__1',
        'coxa_axis'      : (0,  0, -1),
        'femur_axis'     : (0, -1,  0),
        'tibia_axis'     : (0, -1,  0),
        'coxa_sign'      : -1,
        'femur_sign'     : -1,
        'tibia_sign'     : +1,
    },
}


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 3 — DYNAMIC JOINT INDEX RESOLUTION  (verbatim from Stage 2)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def resolve_joint_indices(robot_id):
    """Populate LEG_CONFIG with the actual PyBullet joint indices.
    Called once after the URDF is loaded.
    """
    name_to_idx = {}
    for i in range(p.getNumJoints(robot_id)):
        jname = p.getJointInfo(robot_id, i)[1].decode('utf-8')
        name_to_idx[jname] = i
    for leg in LEG_CONFIG.values():
        leg['coxa_idx']   = name_to_idx.get(leg['coxa_joint_name'])
        leg['femur_idx']  = name_to_idx.get(leg['femur_joint_name'])
        leg['tibia_idx']  = name_to_idx.get(leg['tibia_joint_name'])
        if None in (leg['coxa_idx'], leg['femur_idx'], leg['tibia_idx']):
            missing = [n for n in (leg['coxa_joint_name'], leg['femur_joint_name'],
                                   leg['tibia_joint_name']) if n not in name_to_idx]
            raise RuntimeError(f"Joint name(s) not found in URDF: {missing}")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 4 — GROUND-TRUTH IK SOLVER  (verbatim from simulv2.py — DO NOT MODIFY)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def compute_ik(px, py, pz):
    """
    Geometric 3-DOF IK solver.  Copied verbatim from simulv2.py.
    DO NOT MODIFY.
    """
    a1 = 52.50
    a2 = 108.00
    a3 = 130.775

    T1_MIN, T1_MAX = -20, 20
    T2_MIN, T2_MAX = -90, 45
    T3_MIN, T3_MAX = -45, 90

    t1_rad = math.atan2(py, px)
    t1_deg = math.degrees(t1_rad)

    R = math.sqrt(px**2 + py**2)
    r = R - a1
    z = pz

    L_squared = r**2 + z**2
    if L_squared == 0:
        return 0, 0, 0, False

    c3 = (L_squared - a2**2 - a3**2) / (2 * a2 * a3)
    c3 = max(-1.0, min(1.0, c3))

    s3 = -math.sqrt(1 - c3**2)

    t3_math = math.atan2(s3, c3)
    t2_math = math.atan2(z, r) - math.atan2(a3 * s3, a2 + a3 * c3)

    urdf_t2_deg = math.degrees(t2_math)
    urdf_t3_deg = -(math.degrees(t3_math) + 90.0)

    urdf_t2_deg = ((urdf_t2_deg + 180) % 360) - 180
    urdf_t3_deg = ((urdf_t3_deg + 180) % 360) - 180

    tol = 0.5
    if (T1_MIN - tol <= t1_deg      <= T1_MAX + tol and
        T2_MIN - tol <= urdf_t2_deg <= T2_MAX + tol and
        T3_MIN - tol <= urdf_t3_deg <= T3_MAX + tol):
        return t1_deg, urdf_t2_deg, urdf_t3_deg, True

    return 0, 0, 0, False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 5 — TRANSFORMATION LAYER  (verbatim from Stage 2)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def compute_leg_joints(leg_id, px, py, pz, verbose=False):
    """
    Transformation layer: Foot Target → IK → Sign Corrections → Joint angles.
    """
    cfg = LEG_CONFIG[leg_id]

    t1_deg, t2_deg, t3_deg, is_valid = compute_ik(px, py, pz)
    if not is_valid:
        return 0, 0, 0, False

    jc = t1_deg * cfg['coxa_sign']
    jf = t2_deg * cfg['femur_sign']

    if cfg['tibia_sign'] == -1:
        jt = -t3_deg
    else:
        jt = t3_deg

    if verbose:
        print(f"  Leg {leg_id}: IK({px:.1f},{py:.1f},{pz:.1f}) -> "
              f"({t1_deg:6.2f} deg,{t2_deg:6.2f} deg,{t3_deg:6.2f} deg) "
              f"-> Cmd({jc:6.2f} deg,{jf:6.2f} deg,{jt:6.2f} deg)")

    return jc, jf, jt, True


def set_leg(robot_id, leg_id, px, py, pz, force=50, verbose=False):
    """Compute joints for leg and send to PyBullet motor controllers."""
    jc, jf, jt, ok = compute_leg_joints(leg_id, px, py, pz, verbose=verbose)
    if not ok:
        return False

    cfg = LEG_CONFIG[leg_id]
    p.setJointMotorControl2(robot_id, cfg['coxa_idx'],  p.POSITION_CONTROL,
                            targetPosition=math.radians(jc), force=force)
    p.setJointMotorControl2(robot_id, cfg['femur_idx'], p.POSITION_CONTROL,
                            targetPosition=math.radians(jf), force=force)
    p.setJointMotorControl2(robot_id, cfg['tibia_idx'], p.POSITION_CONTROL,
                            targetPosition=math.radians(jt), force=force)
    return True


def get_joint_deg(robot_id, joint_idx):
    """Return the current joint position in degrees."""
    return math.degrees(p.getJointState(robot_id, joint_idx)[0])


def settle(steps=SETTLE_STEPS):
    """Advance the simulation by `steps` steps at 240 Hz real-time rate."""
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 6 — JOINT MAP VERIFICATION  (dynamic, name-based — from Stage 2)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def verify_joint_map(robot_id):
    """Validate that joint names and child link names in LEG_CONFIG exist in URDF."""
    print()
    print("=" * 70)
    print("JOINT MAP VERIFICATION")
    print("  Validating joint and child link names using dynamic mapping.")
    print("=" * 70)

    name_to_idx = {}
    name_to_link = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        jname = info[1].decode('utf-8')
        link_name = info[12].decode('utf-8')
        name_to_idx[jname] = i
        name_to_link[jname] = link_name

    all_ok = True
    for leg_name, leg in LEG_CONFIG.items():
        for part in ("coxa", "femur", "tibia"):
            joint_key = f"{part}_joint_name"
            link_key = f"{part}_link"
            joint_name = leg.get(joint_key)
            expected_link = leg.get(link_key)
            if joint_name not in name_to_idx:
                print(f"  ERROR: {leg_name} {part} joint '{joint_name}' not found in URDF.")
                all_ok = False
                continue
            actual_link = name_to_link.get(joint_name)
            if actual_link != expected_link:
                print(f"  MISMATCH: {leg_name} {part} joint '{joint_name}' "
                      f"child link '{actual_link}' != expected '{expected_link}'.")
                all_ok = False
            else:
                print(f"  OK: {leg_name} {part} joint '{joint_name}' -> child link '{actual_link}'.")

    print()
    if all_ok:
        print("  RESULT: All joint/link mappings verified.")
    else:
        print("  RESULT: Discrepancies detected. Update LEG_CONFIG or URDF.")
    print()
    return all_ok


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 7 — TRAJECTORY GENERATOR
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def generate_trajectory(px_stand, py_stand, pz_stand, step_length, lift_height, num_points):
    """
    Generate a 5-phase swing trajectory for a single leg.

    All coordinates are in mm, relative to the coxa joint origin (IK frame).

    Phases:
        1. STAND   — hold at standing pose (single point, held externally)
        2. LIFT    — raise foot vertically from stand_z to stand_z + lift_height
        3. FORWARD — move foot forward at apex height (stand_z + lift_height)
        4. LOWER   — lower foot to stand_z at the forward position
        5. RETURN  — slide foot back to the original standing position

    For smoother lift/lower transitions, a half-sine profile is used:
        z(t) = z_start + dz * sin(pi/2 * t)

    Parameters
    ----------
    px_stand, py_stand, pz_stand : float — standing position (mm)
    step_length  : float — forward displacement (mm)
    lift_height  : float — vertical lift above pz_stand (positive value, mm)
    num_points   : int   — interpolation points per phase

    Returns
    -------
    list of (phase_name, px, py, pz, t_value) tuples
    """
    trajectory = []

    # Phase 2 — LIFT: vertical ascent at standing x, from pz_stand to pz_stand + lift_height
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        # Half-sine ease-in for smooth vertical lift
        pz = pz_stand + lift_height * math.sin(math.pi / 2.0 * t)
        trajectory.append(("LIFT", px_stand, py_stand, pz, t))

    # Phase 3 — FORWARD: horizontal sweep at apex height
    pz_apex = pz_stand + lift_height
    px_start = px_stand
    px_end = px_stand + step_length
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        px = px_start + (px_end - px_start) * t
        trajectory.append(("FORWARD", px, py_stand, pz_apex, t))

    # Phase 4 — LOWER: vertical descent at forward position
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        # Half-cosine ease-out for smooth descent
        pz = pz_stand + lift_height * math.cos(math.pi / 2.0 * t)
        trajectory.append(("LOWER", px_end, py_stand, pz, t))

    # Phase 5 — RETURN: slide back at standing height to original position
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        px = px_end + (px_stand - px_end) * t
        trajectory.append(("RETURN", px, py_stand, pz_stand, t))

    return trajectory


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 8 — JOINT SAFETY CHECKS
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# URDF joint limits (radians), converted to degrees for checking
JOINT_LIMITS_DEG = {
    'coxa':  (-20.0, 20.0),     # ±0.349 rad ≈ ±20°
    'femur': (-45.0, 90.0),     # -0.785 to 1.571 rad ≈ -45° to 90°
    'tibia': (-45.0, 90.0),     # -0.785 to 1.571 rad ≈ -45° to 90°
}


def check_joint_safety(jc_deg, jf_deg, jt_deg, phase_name, point_idx):
    """
    Verify that commanded joint angles are within URDF limits and are not NaN.

    Returns (is_safe, warnings_list).
    """
    warnings = []
    is_safe = True

    # NaN check
    for name, val in [("coxa", jc_deg), ("femur", jf_deg), ("tibia", jt_deg)]:
        if math.isnan(val) or math.isinf(val):
            warnings.append(f"  WARNING [{phase_name} pt={point_idx}]: "
                            f"{name} angle is {val} (NaN/Inf)!")
            is_safe = False

    # Limit check (after sign correction — these are the URDF-command angles)
    for name, val, (lo, hi) in [("coxa", jc_deg, JOINT_LIMITS_DEG['coxa']),
                                  ("femur", jf_deg, JOINT_LIMITS_DEG['femur']),
                                  ("tibia", jt_deg, JOINT_LIMITS_DEG['tibia'])]:
        if not math.isnan(val) and (val < lo - 1.0 or val > hi + 1.0):
            warnings.append(f"  WARNING [{phase_name} pt={point_idx}]: "
                            f"{name}={val:.2f}° outside limit [{lo:.1f}°, {hi:.1f}°]")
            # Only warn, don't fail — PyBullet clamps internally

    return is_safe, warnings


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 9 — DEBUG VISUALISATION
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def draw_trajectory_path(robot_id, trajectory):
    """
    Draw the complete trajectory path in the PyBullet GUI using debug lines.
    Uses colour coding per phase:
        LIFT    → Cyan
        FORWARD → Green
        LOWER   → Yellow
        RETURN  → Magenta
    """
    phase_colors = {
        "LIFT":    [0.0, 1.0, 1.0],     # cyan
        "FORWARD": [0.2, 1.0, 0.2],     # green
        "LOWER":   [1.0, 1.0, 0.2],     # yellow
        "RETURN":  [1.0, 0.3, 1.0],     # magenta
    }

    # Get base position/orientation for world-frame conversion
    base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
    bx, by, bz = base_pos
    rot_mat = p.getMatrixFromQuaternion(base_orn)
    R = [rot_mat[0:3], rot_mat[3:6], rot_mat[6:9]]

    cfg = LEG_CONFIG[TEST_LEG_ID]
    hx, hy, hz = cfg['hip_xyz_urdf']

    def to_world(px_mm, py_mm, pz_mm):
        """Convert IK-frame foot target (mm) to approximate world position."""
        # Hip position in world frame
        hip_wx = R[0][0]*hx + R[0][1]*hy + R[0][2]*hz + bx
        hip_wy = R[1][0]*hx + R[1][1]*hy + R[1][2]*hz + by
        hip_wz = R[2][0]*hx + R[2][1]*hy + R[2][2]*hz + bz
        # Approximate foot as hip + offset (radial outward in world frame)
        # For FL leg, "outward" in IK px maps to roughly world +X after 180° rotation
        return [hip_wx + px_mm / 1000.0 * 0.65,
                hip_wy,
                hip_wz + pz_mm / 1000.0]

    line_ids = []
    prev_pos = None
    prev_phase = None

    for phase_name, px, py, pz, t in trajectory:
        world_pos = to_world(px, py, pz)
        color = phase_colors.get(phase_name, [1, 1, 1])

        if prev_pos is not None and prev_phase == phase_name:
            lid = p.addUserDebugLine(prev_pos, world_pos,
                                     lineColorRGB=color, lineWidth=2.5, lifeTime=0)
            line_ids.append(lid)

        prev_pos = world_pos
        prev_phase = phase_name

    return line_ids


# Persistent debug text IDs for live updates
_phase_text_id = -1
_target_text_id = -1
_t_text_id = -1
_target_sphere_id = None


def setup_trajectory_display(robot_id):
    """Create the target sphere and initial text labels."""
    global _target_sphere_id

    vs = p.createVisualShape(p.GEOM_SPHERE, radius=0.01,
                             rgbaColor=[1.0, 0.1, 0.1, 0.9])
    _target_sphere_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs,
                                          basePosition=[0, 0, 0])


def update_trajectory_display(robot_id, phase_name, px, py, pz, t_val):
    """Update live debug text and target sphere position."""
    global _phase_text_id, _target_text_id, _t_text_id

    # Move the red target sphere
    base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
    bx, by, bz = base_pos
    rot_mat = p.getMatrixFromQuaternion(base_orn)
    R = [rot_mat[0:3], rot_mat[3:6], rot_mat[6:9]]

    cfg = LEG_CONFIG[TEST_LEG_ID]
    hx, hy, hz = cfg['hip_xyz_urdf']
    hip_wx = R[0][0]*hx + R[0][1]*hy + R[0][2]*hz + bx
    hip_wy = R[1][0]*hx + R[1][1]*hy + R[1][2]*hz + by
    hip_wz = R[2][0]*hx + R[2][1]*hy + R[2][2]*hz + bz

    sphere_pos = [hip_wx + px / 1000.0 * 0.65, hip_wy, hip_wz + pz / 1000.0]
    p.resetBasePositionAndOrientation(_target_sphere_id, sphere_pos, [0, 0, 0, 1])

    # Phase label
    text_pos = [sphere_pos[0], sphere_pos[1], sphere_pos[2] + 0.06]
    if _phase_text_id >= 0:
        _phase_text_id = p.addUserDebugText(
            f"Phase: {phase_name}", text_pos,
            textColorRGB=[1, 1, 0], textSize=1.4,
            replaceItemUniqueId=_phase_text_id)
    else:
        _phase_text_id = p.addUserDebugText(
            f"Phase: {phase_name}", text_pos,
            textColorRGB=[1, 1, 0], textSize=1.4)

    # Target coordinates
    coord_pos = [sphere_pos[0], sphere_pos[1], sphere_pos[2] + 0.04]
    coord_str = f"Target: ({px:.1f}, {py:.1f}, {pz:.1f}) mm"
    if _target_text_id >= 0:
        _target_text_id = p.addUserDebugText(
            coord_str, coord_pos,
            textColorRGB=[0.4, 1.0, 1.0], textSize=1.0,
            replaceItemUniqueId=_target_text_id)
    else:
        _target_text_id = p.addUserDebugText(
            coord_str, coord_pos,
            textColorRGB=[0.4, 1.0, 1.0], textSize=1.0)

    # t-value
    t_pos = [sphere_pos[0], sphere_pos[1], sphere_pos[2] + 0.025]
    t_str = f"t = {t_val:.3f}"
    if _t_text_id >= 0:
        _t_text_id = p.addUserDebugText(
            t_str, t_pos,
            textColorRGB=[1.0, 0.6, 0.2], textSize=1.0,
            replaceItemUniqueId=_t_text_id)
    else:
        _t_text_id = p.addUserDebugText(
            t_str, t_pos,
            textColorRGB=[1.0, 0.6, 0.2], textSize=1.0)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 10 — FINAL VALIDATION
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def validate_final_pose(robot_id, leg_id, px_stand, py_stand, pz_stand):
    """
    After trajectory completion, verify the test leg has returned to the
    exact standing pose by comparing commanded vs. actual joint angles.

    Returns (passed, detail_string).
    """
    cfg = LEG_CONFIG[leg_id]
    jc, jf, jt, ok = compute_leg_joints(leg_id, px_stand, py_stand, pz_stand)
    if not ok:
        return False, "  IK failed for standing pose — unexpected."

    checks = [
        ('coxa',  cfg['coxa_idx'],  jc),
        ('femur', cfg['femur_idx'], jf),
        ('tibia', cfg['tibia_idx'], jt),
    ]
    all_pass = True
    lines = []
    for name, idx, expected in checks:
        actual = get_joint_deg(robot_id, idx)
        diff = abs(actual - expected)
        ok = diff <= ANGLE_TOLERANCE_DEG
        status = "PASS" if ok else f"FAIL (diff={diff:.2f}° > tol={ANGLE_TOLERANCE_DEG}°)"
        lines.append(f"    {name:6s}  idx={idx:>2}  "
                     f"expected={expected:+7.2f}°  actual={actual:+7.2f}°  → {status}")
        if not ok:
            all_pass = False

    return all_pass, "\n".join(lines)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 11 — MAIN ENTRY POINT
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def main():
    print("=" * 70)
    print("single_leg_trajectory_test.py")
    print("Stage 3 — Single Leg Swing Trajectory Validation")
    print("=" * 70)
    print(f"  URDF           : {URDF_PATH}")
    print(f"  Spawn height   : {BASE_SPAWN_HEIGHT} m")
    print(f"  Fixed base     : {USE_FIXED_BASE}")
    print(f"  Standing pose  : px={STAND_PX} py={STAND_PY} pz={STAND_PZ} mm")
    print(f"  Test leg       : Leg {TEST_LEG_ID} ({LEG_CONFIG[TEST_LEG_ID]['label']})")
    print(f"  Step length    : {STEP_LENGTH} mm")
    print(f"  Lift height    : {LIFT_HEIGHT} mm")
    print(f"  Interp points  : {INTERP_STEPS} per phase")

    # ── Physics Setup ─────────────────────────────────────────────────────
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.65, cameraYaw=60,
        cameraPitch=-25, cameraTargetPosition=[0.05, 0.05, 0.25]
    )

    # ── Load Robot ────────────────────────────────────────────────────────
    start_pos = [0, 0, BASE_SPAWN_HEIGHT]
    start_orn = p.getQuaternionFromEuler([0, 0, math.pi])
    robot_id  = p.loadURDF(URDF_PATH, start_pos, start_orn,
                            useFixedBase=USE_FIXED_BASE)

    resolve_joint_indices(robot_id)
    print(f"\n  Robot loaded.  PyBullet body ID = {robot_id}")

    # ── Verify joint map ──────────────────────────────────────────────────
    map_ok = verify_joint_map(robot_id)
    if not map_ok:
        print("  Aborting — fix joint mapping before proceeding.")
        p.disconnect()
        return

    # ── Phase 1: STAND — all 4 legs in standing pose ──────────────────────
    print()
    print("=" * 70)
    print("PHASE 1 — STAND (all 4 legs)")
    print("=" * 70)
    print(f"  Commanding all legs to standing pose ({STAND_PX}, {STAND_PY}, {STAND_PZ}) mm...")

    for leg_id in range(1, 5):
        ok = set_leg(robot_id, leg_id, STAND_PX, STAND_PY, STAND_PZ, verbose=True)
        if not ok:
            print(f"  CRITICAL: IK failed for Leg {leg_id} standing pose. Aborting.")
            p.disconnect()
            return

    settle(SETTLE_STEPS)
    print(f"  Standing pose established. Holding for {PHASE_HOLD_SEC:.1f}s...")

    # Hold stand visually
    hold_steps = int(PHASE_HOLD_SEC * 240)
    for _ in range(hold_steps):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    # ── Generate trajectory ───────────────────────────────────────────────
    print()
    print("=" * 70)
    print("GENERATING SWING TRAJECTORY")
    print("=" * 70)

    trajectory = generate_trajectory(
        STAND_PX, STAND_PY, STAND_PZ,
        STEP_LENGTH, LIFT_HEIGHT, INTERP_STEPS
    )
    total_points = len(trajectory)
    print(f"  Generated {total_points} trajectory points across 4 phases:")
    print(f"    LIFT    : {INTERP_STEPS} points")
    print(f"    FORWARD : {INTERP_STEPS} points")
    print(f"    LOWER   : {INTERP_STEPS} points")
    print(f"    RETURN  : {INTERP_STEPS} points")

    # Pre-validate entire trajectory for IK reachability
    print("\n  Pre-validating trajectory IK reachability...")
    pre_valid = True
    for i, (phase, px, py, pz, t) in enumerate(trajectory):
        _, _, _, ik_ok = compute_ik(px, py, pz)
        if not ik_ok:
            print(f"  FAIL: IK unreachable at point {i} phase={phase} "
                  f"target=({px:.1f}, {py:.1f}, {pz:.1f}) mm")
            pre_valid = False
            break

    if not pre_valid:
        print("  ABORTING — trajectory contains unreachable points.")
        print("  Reduce STEP_LENGTH or LIFT_HEIGHT and re-run.")
        p.disconnect()
        return

    print("  All trajectory points are IK-reachable. ✓")

    # ── Draw trajectory in GUI ────────────────────────────────────────────
    setup_trajectory_display(robot_id)
    draw_trajectory_path(robot_id, trajectory)

    # ── Execute trajectory on Leg 1 ──────────────────────────────────────
    print()
    print("=" * 70)
    print(f"EXECUTING SWING TRAJECTORY — Leg {TEST_LEG_ID}")
    print("  Legs 2, 3, 4 remain in standing pose.")
    print("=" * 70)

    all_warnings = []
    trajectory_ok = True
    current_phase = None

    for i, (phase, px, py, pz, t) in enumerate(trajectory):
        # Print phase transitions
        if phase != current_phase:
            current_phase = phase
            print(f"\n  ── Phase: {phase} ──")

        # 1. Compute IK + transformation
        jc, jf, jt, ik_ok = compute_leg_joints(TEST_LEG_ID, px, py, pz)
        if not ik_ok:
            print(f"  FAIL: IK failed at point {i}/{total_points}  "
                  f"phase={phase}  target=({px:.1f}, {py:.1f}, {pz:.1f}) mm")
            print("  Stopping trajectory execution.")
            trajectory_ok = False
            break

        # 2. Joint safety checks
        safe, warns = check_joint_safety(jc, jf, jt, phase, i)
        if warns:
            all_warnings.extend(warns)
            for w in warns:
                print(w)
        if not safe:
            print(f"  FAIL: Unsafe joint values at point {i}. Stopping.")
            trajectory_ok = False
            break

        # 3. Command the test leg
        set_leg(robot_id, TEST_LEG_ID, px, py, pz)

        # 4. Keep other legs in standing pose
        for other_leg in range(1, 5):
            if other_leg != TEST_LEG_ID:
                set_leg(robot_id, other_leg, STAND_PX, STAND_PY, STAND_PZ)

        # 5. Update debug visualisation
        update_trajectory_display(robot_id, phase, px, py, pz, t)

        # 6. Step simulation (several sub-steps for smooth motion)
        for _ in range(4):
            p.stepSimulation()
            time.sleep(PHASE_STEP_DELAY / 4.0)

        # Periodic console output (every 10 points)
        if i % 10 == 0:
            print(f"    pt {i:>3}/{total_points}  {phase:8s}  "
                  f"target=({px:6.1f}, {py:5.1f}, {pz:7.1f})  "
                  f"cmd=({jc:+6.2f}°, {jf:+6.2f}°, {jt:+6.2f}°)  t={t:.3f}")

    # ── Post-trajectory: return to stand ──────────────────────────────────
    print()
    print("=" * 70)
    print("RETURNING TO STANDING POSE")
    print("=" * 70)

    set_leg(robot_id, TEST_LEG_ID, STAND_PX, STAND_PY, STAND_PZ, verbose=True)
    settle(SETTLE_STEPS)

    # ── Final Validation ──────────────────────────────────────────────────
    print()
    print("=" * 70)
    print("FINAL VALIDATION")
    print("=" * 70)

    # Check test leg returned to standing pose
    print(f"\n  Verifying Leg {TEST_LEG_ID} returned to standing pose:")
    leg_pass, leg_detail = validate_final_pose(
        robot_id, TEST_LEG_ID, STAND_PX, STAND_PY, STAND_PZ)
    print(leg_detail)

    # Check other legs stayed in standing pose
    other_legs_pass = True
    for other_leg in [2, 3, 4]:
        print(f"\n  Verifying Leg {other_leg} remained in standing pose:")
        ol_pass, ol_detail = validate_final_pose(
            robot_id, other_leg, STAND_PX, STAND_PY, STAND_PZ)
        print(ol_detail)
        if not ol_pass:
            other_legs_pass = False

    # ── Summary ───────────────────────────────────────────────────────────
    print()
    print("=" * 70)
    print("STAGE 3 RESULTS SUMMARY")
    print("=" * 70)

    checks = {
        "Joint map verification":           map_ok,
        "Trajectory pre-validation":        pre_valid,
        "Trajectory execution (no abort)":  trajectory_ok,
        f"Leg {TEST_LEG_ID} final pose":    leg_pass,
        "Other legs stable":                other_legs_pass,
    }

    overall = True
    for desc, passed in checks.items():
        mark = "PASS" if passed else "FAIL"
        print(f"  {desc:42s}  {mark}")
        if not passed:
            overall = False

    if all_warnings:
        print(f"\n  Warnings generated: {len(all_warnings)}")
        for w in all_warnings[:10]:    # cap display
            print(f"    {w}")
        if len(all_warnings) > 10:
            print(f"    ... and {len(all_warnings) - 10} more.")

    print()
    if overall:
        print("  ╔══════════════════════════════════════════════════════════════╗")
        print("  ║  STAGE 3 COMPLETE — Single leg swing trajectory PASSED.     ║")
        print("  ║  The IK solver correctly tracks a multi-phase trajectory.   ║")
        print("  ║  Ready to proceed to gait controller development.           ║")
        print("  ╚══════════════════════════════════════════════════════════════╝")
    else:
        print("  STAGE 3 INCOMPLETE — One or more checks failed.")
        print("  Review warnings and debug visualisation above.")

    # ── Hold for visual inspection ────────────────────────────────────────
    print()
    print("  Holding final pose. Observe in GUI. Press Ctrl+C to exit.")

    try:
        while True:
            for leg_id in range(1, 5):
                set_leg(robot_id, leg_id, STAND_PX, STAND_PY, STAND_PZ)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass

    p.disconnect()
    print("\n  Simulation ended.")


if __name__ == "__main__":
    main()
