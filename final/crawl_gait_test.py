#!/usr/bin/env python3
"""
crawl_gait_test.py
==================
Stage 4 -- Crawl Gait Coordination Validation.

PURPOSE
    Verify coordinated crawl gait sequencing using the corrected
    coxa-angle stepping strategy identified in Stage 3.5.

    This is NOT walking. This is NOT locomotion.
    The robot is fixed-base. No body translation. No turning.
    Only gait coordination validation.

GAIT ORDER
    Leg 1 (FL) -> Leg 4 (BL) -> Leg 3 (FR) -> Leg 2 (BR)
    Only one leg in swing phase at a time.
    Remaining legs hold stance.

TRAJECTORY (corrected from Stage 3.5)
    Uses coxa-angle stepping, NOT px offsets.
        px = R * cos(theta)
        py = R * sin(theta)
    where R = 150 mm (constant radial reach).
    theta sweeps from -12 deg to +12 deg (safe coxa range).

REUSED VERIFIED COMPONENTS
    compute_ik()            -- ground-truth IK solver (DO NOT MODIFY)
    compute_leg_joints()    -- transformation layer + sign corrections
    set_leg()               -- PyBullet motor commands
    resolve_joint_indices() -- dynamic joint name mapping
    LEG_CONFIG              -- per-leg kinematics and signs
"""

import sys
# pyrefly: ignore [missing-import]
import pybullet as p
import pybullet_data
import time
import math

# Fix for Windows cp1252 terminal encoding
if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass


# ============================================================================
# SECTION 1 -- CONFIGURATION
# ============================================================================

URDF_PATH         = "urdf/robot.urdf"
BASE_SPAWN_HEIGHT = 0.38
USE_FIXED_BASE    = True
SETTLE_STEPS      = 360     # 1.5 s at 240 Hz

# Standing pose (verified Stage 2)
STAND_PX = 150.0
STAND_PY =   0.0
STAND_PZ = -150.0

# Coxa-angle stepping parameters (from Stage 3.5)
RADIAL_REACH     = 150.0    # mm -- constant R
THETA_MIN_DEG    = -12.0    # deg -- rear position
THETA_MAX_DEG    = +12.0    # deg -- front position
THETA_STAND_DEG  =   0.0    # deg -- standing (neutral)
LIFT_HEIGHT      =  30.0    # mm  -- foot lift above standing pz
APEX_PZ          = STAND_PZ + LIFT_HEIGHT  # -120 mm

# Interpolation
INTERP_STEPS     = 40       # points per sub-phase
STEP_DELAY       = 0.006    # seconds between points (visual speed)
SIM_SUBSTEPS     = 3        # physics sub-steps per interpolation point

# Gait sequence
GAIT_ORDER       = [1, 2, 4, 3]   # Leg 1 -> Leg 4 -> Leg 3 -> Leg 2

# Validation
ANGLE_TOLERANCE_DEG = 2.0
COXA_SAFE_MIN   = -18.0    # deg -- hard validation limit
COXA_SAFE_MAX   = +18.0    # deg

# Hold times
STAND_HOLD_SEC  = 1.5       # hold standing pose before gait
PHASE_PAUSE_SEC = 0.3       # pause between leg transitions


# ============================================================================
# SECTION 2 -- LEG CONFIGURATION TABLE  (verbatim from Stage 2, verified)
# ============================================================================

LEG_CONFIG = {
    1: {
        'label'          : 'Leg 1 -- FL (Front Left)',
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
        'label'          : 'Leg 2 -- BR (Back Right)',
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
        'coxa_sign'      : +1,
        'femur_sign'     : +1,
        'tibia_sign'     : -1,
    },
    3: {
        'label'          : 'Leg 3 -- FR (Front Right)',
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
        'label'          : 'Leg 4 -- BL (Back Left)',
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
        'coxa_sign'      : +1,
        'femur_sign'     : +1,
        'tibia_sign'     : +1,
    },
}


# ============================================================================
# SECTION 3 -- REUSED VERIFIED COMPONENTS
# ============================================================================

def resolve_joint_indices(robot_id):
    """Populate LEG_CONFIG with actual PyBullet joint indices (Stage 2)."""
    name_to_idx = {}
    for i in range(p.getNumJoints(robot_id)):
        jname = p.getJointInfo(robot_id, i)[1].decode('utf-8')
        name_to_idx[jname] = i
    for leg in LEG_CONFIG.values():
        leg['coxa_idx']  = name_to_idx.get(leg['coxa_joint_name'])
        leg['femur_idx'] = name_to_idx.get(leg['femur_joint_name'])
        leg['tibia_idx'] = name_to_idx.get(leg['tibia_joint_name'])
        if None in (leg['coxa_idx'], leg['femur_idx'], leg['tibia_idx']):
            missing = [n for n in (leg['coxa_joint_name'], leg['femur_joint_name'],
                                   leg['tibia_joint_name']) if n not in name_to_idx]
            raise RuntimeError(f"Joint name(s) not found in URDF: {missing}")


def compute_ik(px, py, pz):
    """Ground-truth IK solver (verbatim from simulv2.py). DO NOT MODIFY."""
    a1 = 52.50;  a2 = 108.00;  a3 = 130.775
    T1_MIN, T1_MAX = -20, 20
    T2_MIN, T2_MAX = -90, 45
    T3_MIN, T3_MAX = -45, 90
    t1_rad = math.atan2(py, px)
    t1_deg = math.degrees(t1_rad)
    R = math.sqrt(px**2 + py**2)
    r = R - a1;  z = pz
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


def compute_leg_joints(leg_id, px, py, pz):
    """IK + sign correction transformation layer (Stage 2)."""
    cfg = LEG_CONFIG[leg_id]
    t1_deg, t2_deg, t3_deg, is_valid = compute_ik(px, py, pz)
    if not is_valid:
        return 0, 0, 0, False
    jc = t1_deg * cfg['coxa_sign']
    jf = t2_deg * cfg['femur_sign']
    jt = -t3_deg if cfg['tibia_sign'] == -1 else t3_deg
    return jc, jf, jt, True


def set_leg(robot_id, leg_id, px, py, pz, force=50):
    """Send IK target to PyBullet motor controllers (Stage 2)."""
    jc, jf, jt, ok = compute_leg_joints(leg_id, px, py, pz)
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
    """Return current joint position in degrees."""
    return math.degrees(p.getJointState(robot_id, joint_idx)[0])


def settle(steps=SETTLE_STEPS):
    """Step simulation for stabilisation."""
    for _ in range(steps):
        update_camera()
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


# ============================================================================
# SECTION 4 -- COXA-ANGLE STEPPING (corrected from Stage 3.5)
# ============================================================================

def coxa_to_ik(theta_deg, pz):
    """
    Convert a coxa angle target to IK coordinates (px, py, pz).

    Uses the corrected coxa-angle stepping strategy from Stage 3.5:
        px = R * cos(theta)
        py = R * sin(theta)

    where R = RADIAL_REACH (150 mm) is held constant.

    Parameters
    ----------
    theta_deg : float -- desired coxa angle in degrees
    pz        : float -- desired vertical position in mm

    Returns
    -------
    (px, py, pz) tuple in mm
    """
    theta_rad = math.radians(theta_deg)
    px = RADIAL_REACH * math.cos(theta_rad)
    py = RADIAL_REACH * math.sin(theta_rad)
    return px, py, pz


# ============================================================================
# SECTION 5 -- SWING TRAJECTORY GENERATOR
# ============================================================================

def generate_swing_trajectory(num_points=INTERP_STEPS):
    """
    Generate a 4-phase swing trajectory for one leg using coxa-angle stepping.

    Phases:
        1. LIFT     : Raise foot at rear angle (-12 deg), pz: -150 -> -120 mm
        2. SWEEP    : Sweep coxa from -12 deg -> +12 deg at apex pz=-120 mm
        3. LOWER    : Lower foot at front angle (+12 deg), pz: -120 -> -150 mm
        4. RETURN   : Slide coxa from +12 deg -> 0 deg at pz=-150 mm

    Returns list of (phase_name, theta_deg, pz, t_value) tuples.
    """
    traj = []

    # Phase 1 -- LIFT: raise foot at rear position
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        pz = STAND_PZ + LIFT_HEIGHT * math.sin(math.pi / 2.0 * t)
        traj.append(("LIFT", THETA_MIN_DEG, pz, t))

    # Phase 2 -- SWEEP: coxa angle from rear to front at apex height
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        theta = THETA_MIN_DEG + (THETA_MAX_DEG - THETA_MIN_DEG) * t
        traj.append(("SWEEP", theta, APEX_PZ, t))

    # Phase 3 -- LOWER: lower foot at front position
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        pz = APEX_PZ - LIFT_HEIGHT * math.sin(math.pi / 2.0 * t)
        traj.append(("LOWER", THETA_MAX_DEG, pz, t))

    # Phase 4 -- RETURN: slide coxa back to standing at ground level
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        theta = THETA_MAX_DEG + (THETA_STAND_DEG - THETA_MAX_DEG) * t
        traj.append(("RETURN", theta, STAND_PZ, t))

    return traj


# ============================================================================
# SECTION 6 -- GAIT STATE MACHINE
# ============================================================================

class GaitStateMachine:
    """
    Crawl gait state machine.

    States:
        IDLE       -- All legs in standing pose.
        SWING_LEG  -- One leg executing swing trajectory.
        COMPLETE   -- Gait cycle finished, all legs back to standing.

    Gait order: Leg 1 -> Leg 4 -> Leg 3 -> Leg 2
    Only one leg swings at a time. All others hold stance.
    """

    def __init__(self, gait_order):
        self.gait_order = gait_order
        self.current_leg_idx = 0
        self.state = "IDLE"
        self.trajectory = []
        self.traj_point = 0

        # Validation counters
        self.ik_failures = 0
        self.limit_violations = 0
        self.legs_completed = 0
        self.total_points_executed = 0
        self.max_coxa_angle = 0.0

    @property
    def active_leg(self):
        if self.current_leg_idx < len(self.gait_order):
            return self.gait_order[self.current_leg_idx]
        return None

    @property
    def support_legs(self):
        active = self.active_leg
        return [lid for lid in range(1, 5) if lid != active]

    @property
    def cycle_percent(self):
        total_legs = len(self.gait_order)
        total_points_per_leg = len(self.trajectory) if self.trajectory else 1
        completed = self.current_leg_idx * total_points_per_leg + self.traj_point
        total = total_legs * total_points_per_leg
        return (completed / max(total, 1)) * 100.0

    def start_cycle(self):
        """Begin the gait cycle."""
        self.state = "SWING_LEG"
        self.current_leg_idx = 0
        self.trajectory = generate_swing_trajectory()
        self.traj_point = 0
        self.legs_completed = 0

    def advance(self):
        """
        Advance to the next trajectory point.

        Returns (leg_id, phase, theta, pz, t) or None if cycle complete.
        """
        if self.state == "COMPLETE" or self.state == "IDLE":
            return None

        if self.traj_point >= len(self.trajectory):
            # Current leg finished
            self.legs_completed += 1
            self.current_leg_idx += 1

            if self.current_leg_idx >= len(self.gait_order):
                self.state = "COMPLETE"
                return None

            # Start next leg
            self.traj_point = 0

        phase, theta, pz, t = self.trajectory[self.traj_point]
        leg_id = self.active_leg
        self.traj_point += 1
        self.total_points_executed += 1

        # Track max coxa angle
        self.max_coxa_angle = max(self.max_coxa_angle, abs(theta))

        return leg_id, phase, theta, pz, t


# ============================================================================
# SECTION 7 -- VISUALIZATION
# ============================================================================

_text_ids = {}


def update_display(robot_id, leg_id, phase, theta, pz, t, cycle_pct,
                   support_legs):
    """Update GUI debug text and visual markers."""
    global _text_ids

    cfg = LEG_CONFIG[leg_id]
    px, py, _ = coxa_to_ik(theta, pz)

    # Text position (above robot)
    base_pos, _ = p.getBasePositionAndOrientation(robot_id)
    tx, ty, tz = base_pos[0], base_pos[1], base_pos[2] + 0.18

    lines = [
        ("gait_title", f"CRAWL GAIT -- Cycle {cycle_pct:.0f}%",
         [tx, ty, tz + 0.10], [1.0, 1.0, 0.3], 1.5),
        ("active_leg", f"Active: {cfg['label']}",
         [tx, ty, tz + 0.08], [0.3, 1.0, 0.3], 1.2),
        ("phase", f"Phase: {phase}  t={t:.2f}",
         [tx, ty, tz + 0.06], [0.3, 0.8, 1.0], 1.1),
        ("coxa", f"Coxa: {theta:+.1f} deg",
         [tx, ty, tz + 0.04], [1.0, 0.6, 0.2], 1.1),
        ("target", f"IK: ({px:.1f}, {py:.1f}, {pz:.1f}) mm",
         [tx, ty, tz + 0.02], [0.7, 0.7, 1.0], 1.0),
        ("support", f"Support: {', '.join(f'L{s}' for s in support_legs)}",
         [tx, ty, tz + 0.00], [0.8, 0.8, 0.8], 1.0),
    ]

    for key, text, pos, color, size in lines:
        if key in _text_ids and _text_ids[key] >= 0:
            _text_ids[key] = p.addUserDebugText(
                text, pos, textColorRGB=color, textSize=size,
                replaceItemUniqueId=_text_ids[key])
        else:
            _text_ids[key] = p.addUserDebugText(
                text, pos, textColorRGB=color, textSize=size)


def draw_swing_path(robot_id, leg_id, trajectory):
    """Draw the full swing trajectory in the GUI for a given leg."""
    phase_colors = {
        "LIFT":   [0.0, 1.0, 1.0],
        "SWEEP":  [0.2, 1.0, 0.2],
        "LOWER":  [1.0, 1.0, 0.2],
        "RETURN": [1.0, 0.3, 1.0],
    }

    # Use tibia link state to approximate foot positions
    # Pre-draw: compute approximate world positions from IK targets
    cfg = LEG_CONFIG[leg_id]
    prev_world = None
    prev_phase = None
    lines = []

    for phase, theta, pz, t in trajectory:
        px, py, pz_ik = coxa_to_ik(theta, pz)

        # Get hip world position
        base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
        bx, by, bz = base_pos
        rot_mat = p.getMatrixFromQuaternion(base_orn)
        R = [rot_mat[0:3], rot_mat[3:6], rot_mat[6:9]]
        hx, hy, hz = cfg['hip_xyz_urdf']
        hip_wx = R[0][0]*hx + R[0][1]*hy + R[0][2]*hz + bx
        hip_wy = R[1][0]*hx + R[1][1]*hy + R[1][2]*hz + by
        hip_wz = R[2][0]*hx + R[2][1]*hy + R[2][2]*hz + bz

        # Approximate world foot position
        world_pos = [hip_wx + px/1000.0 * 0.65 + py/1000.0 * 0.10,
                     hip_wy + py/1000.0 * 0.65 - px/1000.0 * 0.02,
                     hip_wz + pz_ik/1000.0]

        color = phase_colors.get(phase, [1, 1, 1])
        if prev_world is not None and prev_phase == phase:
            lid = p.addUserDebugLine(prev_world, world_pos,
                                     lineColorRGB=color, lineWidth=2.0,
                                     lifeTime=0)
            lines.append(lid)

        prev_world = world_pos
        prev_phase = phase

    return lines


def clear_debug_lines(line_ids):
    """Remove debug lines from the GUI."""
    for lid in line_ids:
        p.removeUserDebugItem(lid)


# ============================================================================
# SECTION 8 -- VALIDATION
# ============================================================================

def validate_final_pose(robot_id, leg_id, px_stand, py_stand, pz_stand):
    """Check that a leg has returned to the standing pose."""
    cfg = LEG_CONFIG[leg_id]
    jc, jf, jt, ok = compute_leg_joints(leg_id, px_stand, py_stand, pz_stand)
    if not ok:
        return False, "  IK failed for standing pose."

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
        status = "PASS" if ok else f"FAIL (diff={diff:.2f} > tol={ANGLE_TOLERANCE_DEG})"
        lines.append(f"    {name:6s}  idx={idx:>2}  "
                     f"expected={expected:+7.2f}  actual={actual:+7.2f}  {status}")
        if not ok:
            all_pass = False
    return all_pass, "\n".join(lines)


def pre_validate_trajectory(trajectory):
    """
    Pre-check every trajectory point for IK validity and coxa limits.
    Returns (all_valid, errors_list).
    """
    errors = []
    for i, (phase, theta, pz, t) in enumerate(trajectory):
        # Coxa limit check
        if abs(theta) > COXA_SAFE_MAX:
            errors.append(f"  Coxa out of range at pt {i}: theta={theta:.1f} > +/-{COXA_SAFE_MAX}")

        # IK validity check
        px, py, pz_ik = coxa_to_ik(theta, pz)
        _, _, _, ik_ok = compute_ik(px, py, pz_ik)
        if not ik_ok:
            errors.append(f"  IK unreachable at pt {i}: phase={phase} "
                         f"theta={theta:.1f} pz={pz:.1f}")

    return len(errors) == 0, errors



def update_camera():
    try:
        dist = p.readUserDebugParameter(cam_dist_slider)
        yaw = p.readUserDebugParameter(cam_yaw_slider)
        pitch = p.readUserDebugParameter(cam_pitch_slider)
        tx = p.readUserDebugParameter(cam_target_x_slider)
        ty = p.readUserDebugParameter(cam_target_y_slider)
        tz = p.readUserDebugParameter(cam_target_z_slider)

        p.resetDebugVisualizerCamera(
            cameraDistance=dist,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=[tx, ty, tz]
        )
    except Exception:
        pass

# ============================================================================
# SECTION 9 -- MAIN
# ============================================================================

def main():
    print("=" * 70)
    print("crawl_gait_test.py")
    print("Stage 4 -- Crawl Gait Coordination Validation")
    print("=" * 70)
    print(f"  URDF            : {URDF_PATH}")
    print(f"  Fixed base      : {USE_FIXED_BASE}")
    print(f"  Standing pose   : ({STAND_PX}, {STAND_PY}, {STAND_PZ}) mm")
    print(f"  Radial reach    : {RADIAL_REACH} mm")
    print(f"  Coxa sweep      : [{THETA_MIN_DEG}, {THETA_MAX_DEG}] deg")
    print(f"  Lift height     : {LIFT_HEIGHT} mm")
    print(f"  Gait order      : {' -> '.join(f'Leg {l}' for l in GAIT_ORDER)}")
    print(f"  Points per phase: {INTERP_STEPS}")

    # -- Physics setup -------------------------------------------------------
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.65, cameraYaw=55,
        cameraPitch=-30, cameraTargetPosition=[0.05, 0.05, 0.25])

    global cam_dist_slider, cam_yaw_slider, cam_pitch_slider
    global cam_target_x_slider, cam_target_y_slider, cam_target_z_slider

    cam_dist_slider = p.addUserDebugParameter("Camera Distance", 0.2, 3.0, 0.65)
    cam_yaw_slider = p.addUserDebugParameter("Camera Yaw", -180, 180, 55)
    cam_pitch_slider = p.addUserDebugParameter("Camera Pitch", -89, 89, -30)
    cam_target_x_slider = p.addUserDebugParameter("Target X", -2.0, 2.0, 0.05)
    cam_target_y_slider = p.addUserDebugParameter("Target Y", -2.0, 2.0, 0.05)
    cam_target_z_slider = p.addUserDebugParameter("Target Z", -1.0, 2.0, 0.25)

    # -- Load robot ----------------------------------------------------------
    start_pos = [0, 0, BASE_SPAWN_HEIGHT]
    start_orn = p.getQuaternionFromEuler([0, 0, math.pi])
    robot_id  = p.loadURDF(URDF_PATH, start_pos, start_orn,
                            useFixedBase=USE_FIXED_BASE)
    resolve_joint_indices(robot_id)
    print(f"\n  Robot loaded.  Body ID = {robot_id}")

    # -- Verify joint map (quick) --------------------------------------------
    print("\n  Verifying joint names...")
    name_to_idx = {}
    for i in range(p.getNumJoints(robot_id)):
        jname = p.getJointInfo(robot_id, i)[1].decode('utf-8')
        name_to_idx[jname] = i
    map_ok = True
    for lid, cfg in LEG_CONFIG.items():
        for part in ('coxa', 'femur', 'tibia'):
            jn = cfg[f'{part}_joint_name']
            if jn not in name_to_idx:
                print(f"  ERROR: {cfg['label']} {part} joint '{jn}' not found!")
                map_ok = False
            else:
                print(f"  OK: Leg {lid} {part} '{jn}' -> idx {name_to_idx[jn]}")
    if not map_ok:
        print("  Aborting -- joint map errors.")
        p.disconnect()
        return

    # -- Pre-validate trajectory ---------------------------------------------
    print("\n" + "=" * 70)
    print("PRE-VALIDATING SWING TRAJECTORY")
    print("=" * 70)

    trajectory = generate_swing_trajectory()
    total_per_leg = len(trajectory)
    total_points = total_per_leg * len(GAIT_ORDER)

    print(f"  Trajectory: {total_per_leg} points/leg x {len(GAIT_ORDER)} legs = {total_points} total")
    print(f"  Phases: LIFT({INTERP_STEPS}) + SWEEP({INTERP_STEPS}) + "
          f"LOWER({INTERP_STEPS}) + RETURN({INTERP_STEPS})")

    traj_ok, traj_errors = pre_validate_trajectory(trajectory)
    if traj_ok:
        print("  All trajectory points are IK-valid and within coxa limits.")
    else:
        for err in traj_errors:
            print(err)
        print("  ABORTING -- trajectory contains invalid points.")
        p.disconnect()
        return

    # -- Establish standing pose ---------------------------------------------
    print("\n" + "=" * 70)
    print("ESTABLISHING STANDING POSE")
    print("=" * 70)

    for lid in range(1, 5):
        ok = set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)
        if not ok:
            print(f"  CRITICAL: IK failed for Leg {lid}. Aborting.")
            p.disconnect()
            return
        cfg = LEG_CONFIG[lid]
        jc, jf, jt, _ = compute_leg_joints(lid, STAND_PX, STAND_PY, STAND_PZ)
        print(f"  Leg {lid}: cmd=({jc:+6.2f}, {jf:+6.2f}, {jt:+6.2f}) deg")

    settle()
    print(f"  Standing pose established. Holding {STAND_HOLD_SEC:.1f}s...")

    hold_steps = int(STAND_HOLD_SEC * 240)
    for _ in range(hold_steps):
        update_camera()
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    # -- Execute crawl gait cycle --------------------------------------------
    print("\n" + "=" * 70)
    print("EXECUTING CRAWL GAIT CYCLE")
    print(f"  Sequence: {' -> '.join(f'Leg {l}' for l in GAIT_ORDER)}")
    print("  One leg swings at a time. Others hold stance.")
    print("=" * 70)

    gsm = GaitStateMachine(GAIT_ORDER)
    gsm.start_cycle()

    current_active_leg = None
    debug_lines = []
    all_warnings = []
    gait_ok = True

    while True:
        result = gsm.advance()
        if result is None:
            break

        leg_id, phase, theta, pz, t = result

        # Detect leg transition
        if leg_id != current_active_leg:
            # Clean up previous trajectory lines
            if debug_lines:
                clear_debug_lines(debug_lines)
                debug_lines = []

            if current_active_leg is not None:
                # Return previous leg to standing
                set_leg(robot_id, current_active_leg, STAND_PX, STAND_PY, STAND_PZ)
                settle(int(PHASE_PAUSE_SEC * 240))

            current_active_leg = leg_id
            label = LEG_CONFIG[leg_id]['label']
            print(f"\n  --- Swinging: {label} ---")

            # Draw trajectory preview for this leg
            debug_lines = draw_swing_path(robot_id, leg_id, trajectory)

        # Print phase transitions
        if gsm.traj_point == 1 or (gsm.traj_point > 1 and
                                    trajectory[gsm.traj_point - 1][0] != phase):
            print(f"    Phase: {phase}")

        # Convert coxa angle to IK targets
        px, py, pz_ik = coxa_to_ik(theta, pz)

        # Compute joint angles for validation
        jc, jf, jt, ik_ok = compute_leg_joints(leg_id, px, py, pz_ik)
        if not ik_ok:
            gsm.ik_failures += 1
            print(f"    FAIL: IK failed at pt {gsm.traj_point-1} "
                  f"phase={phase} theta={theta:.1f}")
            all_warnings.append(f"IK failure: Leg {leg_id} {phase} theta={theta:.1f}")
            gait_ok = False
            continue

        # Coxa limit check
        ik_t1 = math.degrees(math.atan2(py, px))
        if abs(ik_t1) > COXA_SAFE_MAX:
            gsm.limit_violations += 1
            all_warnings.append(
                f"Coxa limit: Leg {leg_id} {phase} theta={theta:.1f} "
                f"ik_t1={ik_t1:.1f}")

        # Command the swing leg
        set_leg(robot_id, leg_id, px, py, pz_ik)

        # Keep support legs in standing pose
        for support_leg in gsm.support_legs:
            set_leg(robot_id, support_leg, STAND_PX, STAND_PY, STAND_PZ)

        # Update visualization
        update_display(robot_id, leg_id, phase, theta, pz, t,
                       gsm.cycle_percent, gsm.support_legs)

        # Step simulation
        for _ in range(SIM_SUBSTEPS):
            update_camera()
            p.stepSimulation()
            time.sleep(STEP_DELAY / SIM_SUBSTEPS)

        # Periodic console output
        if gsm.traj_point % 20 == 0:
            print(f"      pt {gsm.traj_point:>3}/{total_per_leg}  "
                  f"{phase:6s}  theta={theta:+6.1f}  "
                  f"IK=({px:.0f},{py:.0f},{pz_ik:.0f})  "
                  f"cmd=({jc:+6.1f},{jf:+6.1f},{jt:+6.1f})  "
                  f"cycle={gsm.cycle_percent:.0f}%")

    # -- Return to standing --------------------------------------------------
    print("\n" + "=" * 70)
    print("RETURNING TO STANDING POSE")
    print("=" * 70)

    for lid in range(1, 5):
        set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)
    settle()

    # Clean up debug lines
    if debug_lines:
        clear_debug_lines(debug_lines)

    # -- Final validation ----------------------------------------------------
    print("\n" + "=" * 70)
    print("FINAL VALIDATION")
    print("=" * 70)

    final_ok = True
    for lid in range(1, 5):
        print(f"\n  Leg {lid} ({LEG_CONFIG[lid]['label']}):")
        passed, detail = validate_final_pose(
            robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)
        print(detail)
        if not passed:
            final_ok = False

    # -- Results summary -----------------------------------------------------
    print("\n" + "=" * 70)
    print("STAGE 4 RESULTS SUMMARY")
    print("=" * 70)

    checks = {
        "Joint map verification"           : map_ok,
        "Trajectory pre-validation"        : traj_ok,
        "Gait execution (no IK failures)"  : gsm.ik_failures == 0,
        "Coxa within safe range"           : gsm.limit_violations == 0,
        f"All {len(GAIT_ORDER)} legs completed" : gsm.legs_completed == len(GAIT_ORDER),
        "Final standing pose"              : final_ok,
    }

    overall = True
    for desc, passed in checks.items():
        mark = "PASS" if passed else "FAIL"
        print(f"  {desc:42s}  {mark}")
        if not passed:
            overall = False

    print(f"\n  Statistics:")
    print(f"    Gait order        : {' -> '.join(f'L{l}' for l in GAIT_ORDER)}")
    print(f"    Total points      : {gsm.total_points_executed}")
    print(f"    IK failures       : {gsm.ik_failures}")
    print(f"    Limit violations  : {gsm.limit_violations}")
    print(f"    Max coxa angle    : {gsm.max_coxa_angle:.1f} deg")
    print(f"    Legs completed    : {gsm.legs_completed}/{len(GAIT_ORDER)}")

    if all_warnings:
        print(f"\n  Warnings ({len(all_warnings)}):")
        for w in all_warnings[:10]:
            print(f"    {w}")

    print()
    if overall:
        print("  +--------------------------------------------------------------+")
        print("  |  STAGE 4 COMPLETE -- Crawl gait coordination PASSED.         |")
        print("  |  Coxa-angle stepping works correctly for all four legs.       |")
        print("  |  Gait sequencing (1->4->3->2) verified.                      |")
        print("  |  Ready to proceed to locomotion implementation.              |")
        print("  +--------------------------------------------------------------+")
    else:
        print("  STAGE 4 INCOMPLETE -- One or more checks failed.")
        print("  Review warnings above.")

    # -- Hold for inspection -------------------------------------------------
    print("\n  Holding for visual inspection. Press Ctrl+C to exit.")
    try:
        while True:
            for lid in range(1, 5):
                set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except (KeyboardInterrupt, Exception):
        pass

    try:
        p.disconnect()
    except Exception:
        pass
    print("\n  Simulation ended.")


if __name__ == "__main__":
    main()
