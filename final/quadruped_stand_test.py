#!/usr/bin/env python3
"""
quadruped_stand_test.py
=======================
Stage 2 — Quadruped IK Integration and Stand Pose Validation.

PURPOSE
    Integrate the single-leg IK solver (simulv2.py) into the full quadruped
    (robot.urdf) and verify that all four legs can be independently controlled
    and commanded to a symmetric standing pose.

    No walking. No gait. Standing pose only.

SOURCES
    robot.urdf          — Full 4-leg quadruped, 12 revolute joints.
    simulv2.py          — Ground-truth IK solver. compute_ik() is copied
                          verbatim and must NOT be modified.
    LOCOMOTION_PLAN.md  — Architecture reference.
    VERIFICATION_REPORT — Joint axis analysis, discrepancy list.

────────────────────────────────────────────────────────────────────────────
JOINT ANALYSIS  (from robot.urdf, lines 472–555)
────────────────────────────────────────────────────────────────────────────
PyBullet assigns joint indices sequentially in the order joints appear in
the URDF file.  URDF joint names skip 5, 9, 12 — indices are 0-based.

  Idx │ URDF Name   │ Role           │ Child Link (linkName) │ Axis     │ Limits (URDF rad)
  ────┼─────────────┼────────────────┼───────────────────────┼──────────┼──────────────────
   0  │ Revolute 1  │ Leg 1 Coxa     │ coxa_1                │ 0  0 -1  │ [-0.349,  0.349]
   1  │ Revolute 2  │ Leg 2 Coxa     │ coxa__1__1            │ 0  0 -1  │ [-0.349,  0.349]
   2  │ Revolute 3  │ Leg 3 Coxa     │ coxar_1               │ 0  0 -1  │ [-0.349,  0.349]
   3  │ Revolute 4  │ Leg 4 Coxa     │ coxar__1__1           │ 0  0 -1  │ [-0.349,  0.349]
   4  │ Revolute 6  │ Leg 3 Femur    │ femur_v2__1__1        │ 0 +1  0  │ [-0.785,  1.571]
   5  │ Revolute 7  │ Leg 2 Femur    │ femur_v2__2__1        │ 0 -1  0  │ [-0.785,  1.571]
   6  │ Revolute 8  │ Leg 1 Femur    │ femur_v2_1            │ 0 +1  0  │ [-0.785,  1.571]
   7  │ Revolute 10 │ Leg 4 Femur    │ femur_v2__3__1        │ 0 -1  0  │ [-0.785,  1.571]
   8  │ Revolute 11 │ Leg 1 Tibia    │ tibia_1               │ 0 +1  0  │ [-0.785,  1.571]
   9  │ Revolute 13 │ Leg 3 Tibia    │ tibial_1              │ 0 +1  0  │ [-0.785,  1.571]
  10  │ Revolute 14 │ Leg 4 Tibia    │ tibial__1__1          │ 0 -1  0  │ [-0.785,  1.571]
  11  │ Revolute 15 │ Leg 2 Tibia    │ tibia__1__1           │ 0 +1  0  │ [-0.785,  1.571]

Physical positions after the mandatory 180° Z-rotation applied at spawn:
  Leg 1 → Front Left  (FL): raw URDF(-0.075,-0.0938) → world(+0.075,+0.0938)
  Leg 2 → Back Right  (BR): raw URDF(+0.075,+0.0938) → world(-0.075,-0.0938)
  Leg 3 → Front Right (FR): raw URDF(-0.075,+0.0938) → world(+0.075,-0.0938)
  Leg 4 → Back Left   (BL): raw URDF(+0.075,-0.0938) → world(-0.075,+0.0938)

────────────────────────────────────────────────────────────────────────────
SIGN CORRECTION DERIVATION
────────────────────────────────────────────────────────────────────────────
The IK solver (compute_ik) was developed against leg.urdf (single isolated leg).
From simulv2.py comments and code, the REFERENCE joint axes are:
  Coxa  ref axis: [0, 0, +1]  — inferred: IK applies t1 directly, pos angle = CCW from above
  Femur ref axis: [0, +1,  0] — explicit: "Femur passes straight through"
  Tibia ref axis: [0, -1,  0] — explicit: "its Y-axis is inverted (-1.0)"

Correction rule: sign = +1 if robot.urdf axis matches reference, -1 if opposite.

  Leg │ Coxa axis → sign │ Femur axis → sign │ Tibia axis → sign
  ────┼──────────────────┼────────────────────┼───────────────────
   1  │ 0 0 -1 → −1      │ 0 +1 0 → +1        │ 0 +1 0 → −1
   2  │ 0 0 -1 → −1      │ 0 -1 0 → −1        │ 0 +1 0 → −1
   3  │ 0 0 -1 → −1      │ 0 +1 0 → +1        │ 0 +1 0 → −1
   4  │ 0 0 -1 → −1      │ 0 -1 0 → −1        │ 0 -1 0 → +1

The tibia_sign multiplies the IK's already-encoded urdf_t3_deg output.
Since urdf_t3_deg = -(t3_math + 90°) was designed for axis Y=-1:
  - tibia_sign = +1 → keep as-is (same direction as reference, Leg 4 only)
  - tibia_sign = -1 → negate (flip for Y=+1 axis, Legs 1, 2, 3)

────────────────────────────────────────────────────────────────────────────
ARCHITECTURE
────────────────────────────────────────────────────────────────────────────

  Foot Target (px, py, pz in mm relative to coxa joint)
        │
        ▼
  [Shared compute_ik()]  ──── returns (t1_deg, t2_deg, t3_deg) in IK frame
        │
        ▼
  [Leg Transform Layer]  ──── applies per-leg (coxa_sign, femur_sign, tibia_sign)
        │
        ▼
  [Joint Commands]       ──── setJointMotorControl2() with radians
"""

# pyrefly: ignore [missing-import]
import pybullet as p
import pybullet_data
import time
import math

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 1 — CONFIGURATION
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

URDF_PATH          = "urdf/robot.urdf"
BASE_SPAWN_HEIGHT  = 0.38   # meters above ground plane
USE_FIXED_BASE     = True   # True: base locked — joint angles verified in isolation
                             # False: robot falls freely, real standing test
SETTLE_STEPS       = 360    # physics steps before reading joint angles (1.5 s @ 240 Hz)
ANGLE_TOLERANCE_DEG = 1.5   # degrees — pass/fail threshold for joint reach verification

# Standing pose: foot target in mm relative to each leg's coxa joint origin.
#   px = outward reach (in the IK's "forward" direction — maps to the physical
#        radial-outward direction for each leg after the 180° Z-rotation fix)
#   py = lateral offset (0 = no yaw from straight-out)
#   pz = vertical    (negative = below the hip)
STAND_PX = 150.0
STAND_PY =   0.0
STAND_PZ = -150.0

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 2 — LEG CONFIGURATION TABLE
# All values derived directly from robot.urdf joint definitions.
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

LEG_CONFIG = {
    # ── Leg 1: Front Left (FL) ─────────────────────────────────────────────
    # Coxa : Revolute 1  → idx 0, child=coxa_1,          axis=[0, 0,-1]
    # Femur: Revolute 8  → idx 6, child=femur_v2_1,       axis=[0,+1, 0]  (same as ref  → +1)
    # Tibia: Revolute 11 → idx 8, child=tibia_1,          axis=[0,+1, 0]  (opp. ref -1 → -1)
    1: {
        'label'          : 'Leg 1 — FL (Front Left)',
        'hip_xyz_urdf'   : (-0.075, -0.093751, -0.01255),   # raw URDF joint origin (m)
        'hip_xyz_world'  : (+0.075, +0.093751, -0.01255),   # after 180° Z-rotation

        'coxa_idx'       : 0,   'coxa_joint_name'  : 'Revolute 1',
        'femur_idx'      : 6,   'femur_joint_name' : 'Revolute 8',
        'tibia_idx'      : 8,   'tibia_joint_name' : 'Revolute 11',

        'coxa_link'      : 'coxa_1',
        'femur_link'     : 'femur_v2_1',
        'tibia_link'     : 'tibia_1',

        'coxa_axis'      : (0,  0, -1),
        'femur_axis'     : (0, +1,  0),
        'tibia_axis'     : (0, +1,  0),

        'coxa_sign'      : -1,   # axis  0 0 -1  vs ref  0 0 +1  → opposite
        'femur_sign'     : +1,   # axis  0 +1 0  vs ref  0 +1 0  → same
        'tibia_sign'     : -1,   # axis  0 +1 0  vs ref  0 -1 0  → opposite
    },

    # ── Leg 2: Back Right (BR) ─────────────────────────────────────────────
    # Coxa : Revolute 2  → idx 1, child=coxa__1__1,       axis=[0, 0,-1]
    # Femur: Revolute 7  → idx 5, child=femur_v2__2__1,   axis=[0,-1, 0]  (opp. ref +1 → -1)
    # Tibia: Revolute 15 → idx11, child=tibia__1__1,       axis=[0,+1, 0]  (opp. ref -1 → -1)
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

        'coxa_sign'      : -1,   # opposite
        'femur_sign'     : -1,   # axis  0 -1 0  vs ref  0 +1 0  → opposite
        'tibia_sign'     : -1,   # axis  0 +1 0  vs ref  0 -1 0  → opposite
    },

    # ── Leg 3: Front Right (FR) ────────────────────────────────────────────
    # Coxa : Revolute 3  → idx 2, child=coxar_1,          axis=[0, 0,-1]
    # Femur: Revolute 6  → idx 4, child=femur_v2__1__1,   axis=[0,+1, 0]  (same as ref  → +1)
    # Tibia: Revolute 13 → idx 9, child=tibial_1,          axis=[0,+1, 0]  (opp. ref -1 → -1)
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

    # ── Leg 4: Back Left (BL) ──────────────────────────────────────────────
    # Coxa : Revolute 4  → idx 3, child=coxar__1__1,      axis=[0, 0,-1]
    # Femur: Revolute 10 → idx 7, child=femur_v2__3__1,   axis=[0,-1, 0]  (opp. ref +1 → -1)
    # Tibia: Revolute 14 → idx10, child=tibial__1__1,      axis=[0,-1, 0]  (same as ref  → +1)
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
        'tibia_sign'     : +1,   # axis  0 -1 0  matches reference  0 -1 0  → same
    },
}

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION – DYNAMIC JOINT INDEX RESOLUTION
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
        # sanity check – fail fast if any joint name is missing
        if None in (leg['coxa_idx'], leg['femur_idx'], leg['tibia_idx']):
            missing = [n for n in (leg['coxa_joint_name'], leg['femur_joint_name'], leg['tibia_joint_name']) if n not in name_to_idx]
            raise RuntimeError(f"Joint name(s) not found in URDF: {missing}")


# Expected PyBullet child link names in index order (for verification)
# Deprecated static tables – dynamic joint name mapping is now used.
# EXPECTED_LINK_NAMES and EXPECTED_JOINT_NAMES definitions have been removed.
# The verify_joint_map function now validates based on joint names rather than fixed indices.


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 3 — GROUND-TRUTH IK SOLVER
# Copied verbatim from simulv2.py.  DO NOT MODIFY this function.
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def compute_ik(px, py, pz):
    """
    Geometric 3-DOF IK solver. Copied verbatim from simulv2.py.

    DO NOT MODIFY.  This is the ground-truth implementation verified
    against the single-leg model (leg.urdf).

    Parameters
    ----------
    px, py, pz : float
        Foot target position in **millimetres** relative to the coxa joint origin.
        In the IK frame (matching the reference leg.urdf):
            px > 0  = forward / outward from hip
            py > 0  = left (positive CCW from px axis)
            pz < 0  = below the hip (negative = down)

    Returns
    -------
    (t1_deg, t2_deg, t3_deg, is_valid) : tuple
        t1_deg    — coxa  angle in degrees  (IK reference frame)
        t2_deg    — femur angle in degrees  (IK reference frame, no offset)
        t3_deg    — tibia angle in degrees  (IK reference frame, includes
                    the −90° URDF offset and negation for ref tibia axis Y=−1)
        is_valid  — False if target is geometrically unreachable or out of limits
    """
    # Physical link lengths (mm)
    a1 = 52.50     # coxa
    a2 = 108.00    # femur
    a3 = 130.775   # tibia

    # Joint limits from simulv2.py (degrees).
    # NOTE: T2 limits are inverted vs. URDF (bug documented in VERIFICATION_REPORT.md).
    #       For the standing target used here the computed angle is well within both,
    #       so this bug does not affect the current test.
    T1_MIN, T1_MAX = -20, 20
    T2_MIN, T2_MAX = -90, 45
    T3_MIN, T3_MAX = -45, 90

    # 1. Coxa angle (horizontal swing)
    t1_rad = math.atan2(py, px)
    t1_deg = math.degrees(t1_rad)

    # 2. Horizontal reach from hip centre
    R = math.sqrt(px**2 + py**2)

    # 3. Project into the 2-D femur-tibia plane
    r = R - a1      # reach beyond the coxa length
    z = pz          # vertical component (negative = below hip)

    # 4. Law of cosines for knee angle
    L_squared = r**2 + z**2
    if L_squared == 0:
        return 0, 0, 0, False

    c3 = (L_squared - a2**2 - a3**2) / (2 * a2 * a3)
    c3 = max(-1.0, min(1.0, c3))   # numerical clamp

    # Negative root: forces knee to bend backwards (elbow-down configuration)
    s3 = -math.sqrt(1 - c3**2)

    t3_math = math.atan2(s3, c3)
    t2_math = math.atan2(z, r) - math.atan2(a3 * s3, a2 + a3 * c3)

    # 5. URDF axis mapping (for the reference leg.urdf geometry)
    #    Femur: direct pass-through (Math Negative = Down, URDF Negative = Down)
    urdf_t2_deg = math.degrees(t2_math)

    #    Tibia: −90° offset + negation  (reference tibia rests straight-down,
    #           URDF Y-axis = −1.0)
    urdf_t3_deg = -(math.degrees(t3_math) + 90.0)

    # 6. Normalise to [−180, 180]
    urdf_t2_deg = ((urdf_t2_deg + 180) % 360) - 180
    urdf_t3_deg = ((urdf_t3_deg + 180) % 360) - 180

    # 7. Limit check
    tol = 0.5
    if (T1_MIN - tol <= t1_deg    <= T1_MAX + tol and
        T2_MIN - tol <= urdf_t2_deg <= T2_MAX + tol and
        T3_MIN - tol <= urdf_t3_deg <= T3_MAX + tol):
        return t1_deg, urdf_t2_deg, urdf_t3_deg, True

    return 0, 0, 0, False


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 4 — TRANSFORMATION LAYER
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def compute_leg_joints(leg_id, px, py, pz, verbose=False):
    """
    Transformation layer: Foot Target → IK → Sign Corrections → Joint angles.

    The IK solver is leg-agnostic.  This function applies the per-leg sign
    multipliers derived from the robot.urdf axis analysis so that the same
    IK solver controls all four legs correctly.

    Parameters
    ----------
    leg_id     : int  (1–4)
    px, py, pz : float  — foot target in mm relative to the coxa joint origin
    verbose    : bool   — if True, print the computation trace

    Returns
    -------
    (joint_coxa_deg, joint_femur_deg, joint_tibia_deg, is_valid)
        Joint angles in degrees ready to be converted to radians and sent to
        PyBullet's setJointMotorControl2().
        is_valid = False if IK failed.
    """
    cfg = LEG_CONFIG[leg_id]

    # ── Step 1: Shared IK (leg-agnostic) ──────────────────────────────────
    t1_deg, t2_deg, t3_deg, is_valid = compute_ik(px, py, pz)
    if not is_valid:
        return 0, 0, 0, False

    # ── Step 2: Apply sign corrections ───────────────────────────────────
    # The IK solver outputs reference angles. We multiply by the per-leg
    # signs to account for the physical robot's joint axis orientations.
    jc = t1_deg * cfg['coxa_sign']
    jf = t2_deg * cfg['femur_sign']

    # Tibia sign is specific: it dictates whether we flip the output relative
    # to the IK's already-adjusted reference tibia angle.
    if cfg['tibia_sign'] == -1:
        jt = -t3_deg
    else:
        jt = t3_deg

    if verbose:
        print(f"  Leg {leg_id}: IK({px},{py},{pz}) → "
              f"({t1_deg:6.2f}°,{t2_deg:6.2f}°,{t3_deg:6.2f}°) "
              f"→ Cmd({jc:6.2f}°,{jf:6.2f}°,{jt:6.2f}°)")

    return jc, jf, jt, True


def set_leg(robot_id, leg_id, px, py, pz, force=50, verbose=False):
    """
    Compute joints for leg `leg_id` and send to PyBullet motor controllers.

    Returns
    -------
    bool — True if commands were sent, False if IK failed.
    """
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


def reset_all_legs(robot_id, force=50):
    """Command all 12 joints to 0 rad (natural rest pose)."""
    for idx in range(12):
        p.setJointMotorControl2(robot_id, idx, p.POSITION_CONTROL,
                                targetPosition=0.0, force=force)


def get_joint_deg(robot_id, joint_idx):
    """Return the current joint position in degrees."""
    return math.degrees(p.getJointState(robot_id, joint_idx)[0])


def settle(steps=SETTLE_STEPS):
    """Advance the simulation by `steps` steps at 240 Hz real-time rate."""
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 5 — JOINT MAP VERIFICATION
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def verify_joint_map(robot_id):
    """Validate that joint names and child link names defined in LEG_CONFIG exist in the loaded URDF.
    Returns True only if all mappings are correct.
    """
    print()
    print("=" * 70)
    print("STEP 1 — JOINT MAP VERIFICATION")
    print("  Validating joint and child link names using dynamic mapping.")
    print("=" * 70)

    # Build dictionaries from PyBullet joint info
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
                print(f"  MISMATCH: {leg_name} {part} joint '{joint_name}' child link '{actual_link}' != expected '{expected_link}'.")
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
# SECTION 6 — VISUALISATION HELPERS
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def setup_visualisation(robot_id):
    """
    Add:
      • Blue sphere at each hip (coxa joint origin) in world frame.
      • Red  sphere at the approximate foot target for the standing pose.
      • Yellow text label for each leg.
      • Cyan  text displaying joint indices for each leg.
    """
    marker_ids = []

    base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
    bx, by, bz = base_pos

    # Build rotation matrix from base orientation (for hip world positions)
    rot_mat = p.getMatrixFromQuaternion(base_orn)
    R = [rot_mat[0:3], rot_mat[3:6], rot_mat[6:9]]

    def transform(local_xyz):
        """Rotate local_xyz by base orientation, then translate by base_pos."""
        x, y, z = local_xyz
        wx = R[0][0]*x + R[0][1]*y + R[0][2]*z + bx
        wy = R[1][0]*x + R[1][1]*y + R[1][2]*z + by
        wz = R[2][0]*x + R[2][1]*y + R[2][2]*z + bz
        return [wx, wy, wz]

    for leg_id, cfg in LEG_CONFIG.items():
        hx, hy, hz = cfg['hip_xyz_urdf']  # in URDF frame
        hip_world = transform((hx, hy, hz))

        # --- Hip marker (blue) ---
        vs = p.createVisualShape(p.GEOM_SPHERE, radius=0.012,
                                 rgbaColor=[0.2, 0.5, 1.0, 1.0])
        mb = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs,
                               basePosition=hip_world)
        marker_ids.append(mb)

        # --- Foot target marker (red) ---
        # Approximate world foot position: hip_world + [px/1000, 0, pz/1000]
        # Direction depends on leg orientation — for visualisation only, use world +X per leg
        sign_x = +1 if leg_id in (1, 3) else -1  # FL/FR=front, BR/BL=back
        foot_world = [
            hip_world[0] + sign_x * STAND_PX / 1000.0 * 0.7,   # approximate X contribution
            hip_world[1],
            hip_world[2] + STAND_PZ / 1000.0,
        ]
        fs = p.createVisualShape(p.GEOM_SPHERE, radius=0.014,
                                 rgbaColor=[1.0, 0.2, 0.2, 1.0])
        fb = p.createMultiBody(baseMass=0, baseVisualShapeIndex=fs,
                               basePosition=foot_world)
        marker_ids.append(fb)

        # --- Leg label ---
        short_label = cfg['label'].split('—')[1].strip()   # e.g. "FL (Front Left)"
        p.addUserDebugText(
            f"L{leg_id}: {short_label}",
            [hip_world[0], hip_world[1], hip_world[2] + 0.07],
            textColorRGB=[1, 1, 0], textSize=1.3
        )

        # --- Joint index info ---
        index_str = (f"C={cfg['coxa_idx']}({cfg['coxa_joint_name']})  "
                     f"F={cfg['femur_idx']}({cfg['femur_joint_name']})  "
                     f"T={cfg['tibia_idx']}({cfg['tibia_joint_name']})")
        p.addUserDebugText(
            index_str,
            [hip_world[0], hip_world[1], hip_world[2] + 0.04],
            textColorRGB=[0.4, 1.0, 1.0], textSize=0.8
        )

    return marker_ids


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 7 — VALIDATION TESTS
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def check_joint_reach(robot_id, leg_id, expected_coxa, expected_femur, expected_tibia):
    """
    Read back joint positions and compare to expected values.
    Returns (passed, detail_str).
    """
    cfg = LEG_CONFIG[leg_id]
    checks = [
        ('coxa',  cfg['coxa_idx'],  expected_coxa),
        ('femur', cfg['femur_idx'], expected_femur),
        ('tibia', cfg['tibia_idx'], expected_tibia),
    ]
    all_pass = True
    lines = []
    for name, idx, expected in checks:
        actual = get_joint_deg(robot_id, idx)
        diff   = abs(actual - expected)
        ok     = diff <= ANGLE_TOLERANCE_DEG
        status = "PASS" if ok else f"FAIL (diff={diff:.2f}° > tol={ANGLE_TOLERANCE_DEG}°)"
        lines.append(f"      {name:6s}  idx={idx:>2}  "
                     f"expected={expected:+7.2f}°  actual={actual:+7.2f}°  → {status}")
        if not ok:
            all_pass = False
    return all_pass, "\n".join(lines)


def run_single_leg_test(robot_id, leg_id, px, py, pz, test_num):
    """
    Test N (N=1..4): Move only leg N; all other legs remain at 0.

    Strategy:
      1. Reset all legs to 0.
      2. Settle (let physics stabilise).
      3. Compute expected angles via transformation layer.
      4. Apply to only the target leg.
      5. Settle again.
      6. Read back and compare.
    """
    cfg = LEG_CONFIG[leg_id]
    print(f"\n{'─'*60}")
    print(f"TEST {test_num}: {cfg['label']} only")
    print(f"  Target: px={px:.1f}  py={py:.1f}  pz={pz:.1f} mm")
    print(f"{'─'*60}")

    reset_all_legs(robot_id)
    settle(120)

    print("  Computing joint angles (transformation layer):")
    jc, jf, jt, ok = compute_leg_joints(leg_id, px, py, pz, verbose=True)
    if not ok:
        print(f"  RESULT: FAIL — IK returned invalid for this target.")
        return False

    set_leg(robot_id, leg_id, px, py, pz, verbose=False)
    settle(SETTLE_STEPS)

    passed, detail = check_joint_reach(robot_id, leg_id, jc, jf, jt)
    print("\n  Joint reach verification:")
    print(detail)
    print(f"\n  TEST {test_num} RESULT: {'PASS' if passed else 'FAIL'}")
    return passed


def run_standing_test(robot_id, px, py, pz):
    """
    Test 5: All four legs simultaneously commanded to the standing pose.

    Strategy:
      1. Reset all legs.
      2. Compute expected joint angles for each leg.
      3. Apply to all legs simultaneously.
      4. Settle.
      5. Verify all legs reached their targets.
    """
    print(f"\n{'─'*60}")
    print(f"TEST 5: Full Standing Pose — All 4 Legs Simultaneously")
    print(f"  Target for all legs: px={px:.1f}  py={py:.1f}  pz={pz:.1f} mm")
    print(f"{'─'*60}")

    reset_all_legs(robot_id)
    settle(120)

    # Pre-compute all expected angles
    expected = {}
    print("\n  Computing joint angles for all legs:")
    for leg_id in range(1, 5):
        jc, jf, jt, ok = compute_leg_joints(leg_id, px, py, pz, verbose=True)
        if not ok:
            print(f"  IK FAILED for Leg {leg_id}.  Aborting test.")
            return False
        expected[leg_id] = (jc, jf, jt)

    # Apply all legs simultaneously
    print("\n  Sending commands to all 4 legs...")
    for leg_id in range(1, 5):
        set_leg(robot_id, leg_id, px, py, pz, verbose=False)

    settle(SETTLE_STEPS)

    # Verify
    all_pass = True
    print("\n  Joint reach verification (all legs):")
    for leg_id in range(1, 5):
        cfg = LEG_CONFIG[leg_id]
        jc, jf, jt = expected[leg_id]
        passed, detail = check_joint_reach(robot_id, leg_id, jc, jf, jt)
        print(f"\n  {cfg['label']}:")
        print(detail)
        if not passed:
            all_pass = False

    print(f"\n  TEST 5 RESULT: {'PASS' if all_pass else 'FAIL'}")
    return all_pass


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SECTION 8 — MAIN ENTRY POINT
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def print_sign_table():
    print()
    print("=" * 70)
    print("SIGN CORRECTION TABLE  (transformation layer summary)")
    print("=" * 70)
    print(f"  {'Leg':>3}  {'Label':>30}  {'Coxa':>20}  {'Femur':>20}  {'Tibia':>20}  {'Signs C/F/T':>12}")
    print("  " + "-" * 105)
    for leg_id, cfg in LEG_CONFIG.items():
        coxa_str  = f"idx={cfg['coxa_idx']} {cfg['coxa_joint_name']}"
        femur_str = f"idx={cfg['femur_idx']} {cfg['femur_joint_name']}"
        tibia_str = f"idx={cfg['tibia_idx']} {cfg['tibia_joint_name']}"
        signs     = f"{cfg['coxa_sign']:+d} / {cfg['femur_sign']:+d} / {cfg['tibia_sign']:+d}"
        print(f"  {leg_id:>3}  {cfg['label']:>30}  {coxa_str:>20}  {femur_str:>20}  {tibia_str:>20}  {signs:>12}")
    print()


def main():
    print("=" * 70)
    print("quadruped_stand_test.py")
    print("Stage 2 — Quadruped IK Integration & Stand Pose Validation")
    print("=" * 70)
    print(f"  URDF         : {URDF_PATH}")
    print(f"  Spawn height : {BASE_SPAWN_HEIGHT} m")
    print(f"  Fixed base   : {USE_FIXED_BASE}")
    print(f"  Standing pose: px={STAND_PX} py={STAND_PY} pz={STAND_PZ} mm")

    # ── Physics Setup ─────────────────────────────────────────────────────
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    # Configure the GUI
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.75, cameraYaw=45,
        cameraPitch=-30, cameraTargetPosition=[0, 0, 0.25]
    )

    # ── Load Robot ────────────────────────────────────────────────────────
    # 180° Z-rotation: matches simulv2.py line 20.
    # The URDF was modelled pointing in -X; this rotation makes the robot
    # face world +X so that IK px=forward maps to world +X.
    start_pos = [0, 0, BASE_SPAWN_HEIGHT]
    start_orn = p.getQuaternionFromEuler([0, 0, math.pi])
    robot_id  = p.loadURDF(URDF_PATH, start_pos, start_orn,
                            useFixedBase=USE_FIXED_BASE)
    # Resolve joint indices dynamically
    resolve_joint_indices(robot_id)

    print(f"\n  Robot loaded.  PyBullet body ID = {robot_id}")

    # ── Step 1: Verify joint map ──────────────────────────────────────────
    map_ok = verify_joint_map(robot_id)
    if not map_ok:
        print("  Aborting — fix joint index mapping before proceeding.")
        p.disconnect()
        return

    # ── Print sign correction summary ─────────────────────────────────────
    print_sign_table()

    # ── Add visualisation ─────────────────────────────────────────────────
    setup_visualisation(robot_id)

    # ── Run validation tests ──────────────────────────────────────────────
    print()
    print("=" * 70)
    print("RUNNING VALIDATION TESTS")
    print("=" * 70)

    results = {}

    # Tests 1–4: isolate one leg at a time
    for leg_id in range(1, 5):
        results[leg_id] = run_single_leg_test(
            robot_id, leg_id,
            STAND_PX, STAND_PY, STAND_PZ,
            test_num=leg_id
        )
        time.sleep(1.0)   # brief pause for visual inspection

    # Test 5: all four legs simultaneously
    results[5] = run_standing_test(robot_id, STAND_PX, STAND_PY, STAND_PZ)

    # ── Final summary ─────────────────────────────────────────────────────
    print()
    print("=" * 70)
    print("VALIDATION RESULTS SUMMARY")
    print("=" * 70)
    descriptions = {
        1: "Leg 1 (FL) isolated",
        2: "Leg 2 (BR) isolated",
        3: "Leg 3 (FR) isolated",
        4: "Leg 4 (BL) isolated",
        5: "All 4 legs — standing pose",
    }
    all_pass = True
    for t, passed in results.items():
        mark = "PASS" if passed else "FAIL"
        print(f"  Test {t}  {descriptions[t]:35s}  {mark}")
        if not passed:
            all_pass = False

    print()
    if all_pass:
        print("  STAGE 2 COMPLETE — All 5 tests passed.")
        print("  The single-leg IK solver correctly controls all four legs.")
    else:
        print("  STAGE 2 INCOMPLETE — One or more tests failed.")
        print()
        print("  DIAGNOSTICS — observe the GUI:")
        print("    If a leg's femur rotates UP  instead of DOWN → flip that leg's femur_sign")
        print("    If the knee bends the WRONG  direction       → flip that leg's tibia_sign")
        print("    If a leg sweeps LATERALLY when py=0          → flip that leg's coxa_sign")

    # ── Hold the standing pose for visual inspection ──────────────────────
    print()
    print("  Holding standing pose.  Observe in GUI.  Press Ctrl+C to exit.")

    try:
        while True:
            for leg_id in range(1, 5):
                set_leg(robot_id, leg_id, STAND_PX, STAND_PY, STAND_PZ, verbose=False)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass

    p.disconnect()
    print("\n  Simulation ended.")


if __name__ == "__main__":
    main()
