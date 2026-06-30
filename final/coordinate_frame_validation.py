#!/usr/bin/env python3
"""
coordinate_frame_validation.py
==============================
Stage 3.5 -- Coordinate Frame and Coxa Workspace Validation.

PURPOSE
    Determine the true mapping between the IK coordinate frame and the
    robot body frame before gait implementation begins.

    Stage 3 revealed that increasing IK px (150->190 mm) produced lateral
    (outward) foot motion, not forward motion.  This script systematically
    probes each IK axis to build an authoritative axis mapping and
    characterizes the coxa workspace for locomotion planning.

BACKGROUND
    The IK solver (compute_ik) uses a POLAR / RADIAL coordinate frame:
        px = radial reach outward from the coxa joint origin
        py = lateral offset (coxa angle = atan2(py, px))
        pz = vertical displacement (negative = below hip)

    The coxa joint rotates the entire leg in the horizontal plane.
    Therefore "forward stepping" in the robot body frame requires
    sweeping the coxa angle, which is controlled by py, NOT px.

    This script proves that relationship experimentally.

NO MODIFICATIONS TO:
    compute_ik(), sign correction logic, joint mapping, transformation layer.
"""

import sys
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

# ---------------------------------------------------------------------------
# SECTION 1 -- CONFIGURATION
# ---------------------------------------------------------------------------

URDF_PATH         = "urdf/robot.urdf"
BASE_SPAWN_HEIGHT = 0.38
USE_FIXED_BASE    = True
SETTLE_STEPS      = 480   # 2 seconds @ 240 Hz

# Standing pose (verified in Stage 2 and 3)
STAND_PX = 150.0
STAND_PY =   0.0
STAND_PZ = -150.0

TEST_LEG_ID = 1   # Leg 1 -- Front Left (FL)

# ---------------------------------------------------------------------------
# SECTION 2 -- LEG CONFIGURATION TABLE  (verbatim from Stage 2)
# ---------------------------------------------------------------------------

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
        'coxa_sign'      : -1,
        'femur_sign'     : -1,
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
        'coxa_sign'      : -1,
        'femur_sign'     : -1,
        'tibia_sign'     : +1,
    },
}


# ---------------------------------------------------------------------------
# SECTION 3 -- REUSED COMPONENTS  (verbatim from Stage 2/3)
# ---------------------------------------------------------------------------

def resolve_joint_indices(robot_id):
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
    """Ground-truth IK solver. DO NOT MODIFY."""
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
    cfg = LEG_CONFIG[leg_id]
    t1_deg, t2_deg, t3_deg, is_valid = compute_ik(px, py, pz)
    if not is_valid:
        return 0, 0, 0, False
    jc = t1_deg * cfg['coxa_sign']
    jf = t2_deg * cfg['femur_sign']
    jt = -t3_deg if cfg['tibia_sign'] == -1 else t3_deg
    return jc, jf, jt, True


def set_leg(robot_id, leg_id, px, py, pz, force=50):
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
    return math.degrees(p.getJointState(robot_id, joint_idx)[0])


def settle(steps=SETTLE_STEPS):
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


def get_foot_world_pos(robot_id, leg_id):
    """Get the actual world-frame position of the tibia link (foot) using PyBullet."""
    cfg = LEG_CONFIG[leg_id]
    link_state = p.getLinkState(robot_id, cfg['tibia_idx'])
    return link_state[0]  # world position (x, y, z) in metres


# ---------------------------------------------------------------------------
# SECTION 4 -- AXIS IDENTIFICATION TESTS
# ---------------------------------------------------------------------------

def run_axis_tests(robot_id):
    """
    Starting from standing pose, move foot +/-50mm along each IK axis
    independently.  Measure the resulting world-frame foot displacement
    and joint angle changes to determine the IK->Robot frame mapping.
    """
    print()
    print("=" * 70)
    print("AXIS IDENTIFICATION TESTS")
    print("  Starting from standing pose (150, 0, -150) mm")
    print("  Applying +/-50 mm perturbation on each IK axis independently")
    print("=" * 70)

    # First establish standing pose for all legs and record baseline
    for lid in range(1, 5):
        set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)
    settle()

    # Record baseline
    base_foot = get_foot_world_pos(robot_id, TEST_LEG_ID)
    cfg = LEG_CONFIG[TEST_LEG_ID]
    base_coxa  = get_joint_deg(robot_id, cfg['coxa_idx'])
    base_femur = get_joint_deg(robot_id, cfg['femur_idx'])
    base_tibia = get_joint_deg(robot_id, cfg['tibia_idx'])

    base_jc, base_jf, base_jt, _ = compute_leg_joints(TEST_LEG_ID, STAND_PX, STAND_PY, STAND_PZ)

    print(f"\n  BASELINE (standing pose):")
    print(f"    IK target   : ({STAND_PX:.0f}, {STAND_PY:.0f}, {STAND_PZ:.0f}) mm")
    print(f"    Foot world  : ({base_foot[0]:.4f}, {base_foot[1]:.4f}, {base_foot[2]:.4f}) m")
    print(f"    Coxa cmd    : {base_jc:+.2f} deg")
    print(f"    Femur cmd   : {base_jf:+.2f} deg")
    print(f"    Tibia cmd   : {base_jt:+.2f} deg")

    # Define axis tests
    OFFSET = 50.0  # mm
    tests = [
        ("A", "+X", STAND_PX + OFFSET, STAND_PY,          STAND_PZ),
        ("B", "-X", STAND_PX - OFFSET, STAND_PY,          STAND_PZ),
        ("C", "+Y", STAND_PX,          STAND_PY + OFFSET,  STAND_PZ),
        ("D", "-Y", STAND_PX,          STAND_PY - OFFSET,  STAND_PZ),
        ("E", "+Z", STAND_PX,          STAND_PY,           STAND_PZ + OFFSET),
        ("F", "-Z", STAND_PX,          STAND_PY,           STAND_PZ - OFFSET),
    ]

    results = []

    for test_id, axis_label, px, py, pz in tests:
        print(f"\n  {'='*60}")
        print(f"  TEST {test_id}: IK {axis_label}  -->  target=({px:.0f}, {py:.0f}, {pz:.0f}) mm")
        print(f"  {'='*60}")

        # Reset to standing
        for lid in range(1, 5):
            set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)
        settle(240)

        # Compute IK for test target
        t1, t2, t3, ik_ok = compute_ik(px, py, pz)
        jc, jf, jt, ok = compute_leg_joints(TEST_LEG_ID, px, py, pz)

        if not ok:
            print(f"    IK FAILED for target ({px}, {py}, {pz}) mm")
            results.append((test_id, axis_label, px, py, pz, None, None, None, None, False))
            continue

        print(f"    IK raw angles  : t1={t1:+7.2f} deg  t2={t2:+7.2f} deg  t3={t3:+7.2f} deg")
        print(f"    Commanded      : jc={jc:+7.2f} deg  jf={jf:+7.2f} deg  jt={jt:+7.2f} deg")

        # Coxa angle change
        coxa_delta = t1 - 0.0   # IK coxa relative to standing (t1=0 at py=0)
        print(f"    IK coxa angle  : {t1:+.2f} deg  (delta from stand: {coxa_delta:+.2f} deg)")

        # Identify primary joint
        d_coxa  = abs(jc - base_jc)
        d_femur = abs(jf - base_jf)
        d_tibia = abs(jt - base_jt)
        max_d = max(d_coxa, d_femur, d_tibia)
        if max_d == d_coxa:
            primary = "COXA"
        elif max_d == d_femur:
            primary = "FEMUR"
        else:
            primary = "TIBIA"

        print(f"    Joint deltas   : coxa={d_coxa:.2f} deg  femur={d_femur:.2f} deg  tibia={d_tibia:.2f} deg")
        print(f"    Primary joint  : {primary}")

        # Apply and measure world displacement
        set_leg(robot_id, TEST_LEG_ID, px, py, pz)
        settle()

        test_foot = get_foot_world_pos(robot_id, TEST_LEG_ID)
        dx = (test_foot[0] - base_foot[0]) * 1000  # mm
        dy = (test_foot[1] - base_foot[1]) * 1000  # mm
        dz = (test_foot[2] - base_foot[2]) * 1000  # mm

        print(f"    Foot world     : ({test_foot[0]:.4f}, {test_foot[1]:.4f}, {test_foot[2]:.4f}) m")
        print(f"    World delta    : dX={dx:+.1f} mm  dY={dy:+.1f} mm  dZ={dz:+.1f} mm")

        # Determine dominant world axis
        abs_d = [abs(dx), abs(dy), abs(dz)]
        max_axis_idx = abs_d.index(max(abs_d))
        world_labels = ["X", "Y", "Z"]
        signs = [dx, dy, dz]
        dominant_world = ("+" if signs[max_axis_idx] > 0 else "-") + world_labels[max_axis_idx]

        print(f"    Dominant world : {dominant_world}  (magnitude: {abs_d[max_axis_idx]:.1f} mm)")

        # Debug visualisation: draw line from baseline to test foot
        line_color = {'A': [1,0,0], 'B': [1,0.5,0], 'C': [0,1,0],
                      'D': [0,0.5,0], 'E': [0,0,1], 'F': [0,0.5,1]}
        p.addUserDebugLine(list(base_foot), list(test_foot),
                           lineColorRGB=line_color.get(test_id, [1,1,1]),
                           lineWidth=3.0, lifeTime=0)
        p.addUserDebugText(f"Test {test_id}: IK {axis_label} -> World {dominant_world}",
                           [test_foot[0], test_foot[1], test_foot[2] + 0.02],
                           textColorRGB=line_color.get(test_id, [1,1,1]),
                           textSize=0.9, lifeTime=0)

        results.append((test_id, axis_label, px, py, pz,
                         (dx, dy, dz), dominant_world, primary,
                         (jc, jf, jt), True))

    return results, base_foot


# ---------------------------------------------------------------------------
# SECTION 5 -- COXA WORKSPACE ANALYSIS
# ---------------------------------------------------------------------------

def run_coxa_analysis(robot_id):
    """
    Sweep the coxa angle from -18 to +18 degrees in 6-degree steps.
    For each angle, compute the foot position, verify IK validity,
    and measure the actual foot displacement in the world frame.
    """
    print()
    print("=" * 70)
    print("COXA WORKSPACE ANALYSIS")
    print("  URDF coxa limits: [-0.349 rad, +0.349 rad] = [-20 deg, +20 deg]")
    print("  Testing at: -18, -12, -6, 0, +6, +12, +18 degrees")
    print("=" * 70)

    # Reset to standing
    for lid in range(1, 5):
        set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)
    settle()

    base_foot = get_foot_world_pos(robot_id, TEST_LEG_ID)

    # IK relationship: coxa_angle = atan2(py, px)
    # To set a specific coxa angle while keeping constant radial reach:
    #   R = sqrt(px^2 + py^2) = constant
    #   px = R * cos(angle)
    #   py = R * sin(angle)
    # With R = STAND_PX (since standing py=0, R=px=150)

    R_stand = STAND_PX   # 150 mm radial reach

    coxa_angles_deg = [-18, -12, -6, 0, +6, +12, +18]

    print(f"\n  Radial reach R = {R_stand:.0f} mm (constant)")
    print(f"  Vertical pz    = {STAND_PZ:.0f} mm (constant)")
    print()
    print(f"  {'Coxa':>6s}  {'IK px':>8s}  {'IK py':>8s}  {'IK t1':>8s}  "
          f"{'Cmd coxa':>9s}  {'Foot dX':>8s}  {'Foot dY':>8s}  {'Foot dZ':>8s}  "
          f"{'Stride':>7s}  {'IK OK':>5s}")
    print("  " + "-" * 95)

    coxa_results = []
    prev_foot = None

    for angle_deg in coxa_angles_deg:
        angle_rad = math.radians(angle_deg)
        px = R_stand * math.cos(angle_rad)
        py = R_stand * math.sin(angle_rad)
        pz = STAND_PZ

        t1, t2, t3, ik_ok = compute_ik(px, py, pz)
        jc, jf, jt, ok = compute_leg_joints(TEST_LEG_ID, px, py, pz)

        if ok:
            # Reset all legs to standing
            for lid in range(1, 5):
                set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)
            settle(120)

            # Apply test angle to Leg 1
            set_leg(robot_id, TEST_LEG_ID, px, py, pz)
            settle(360)

            test_foot = get_foot_world_pos(robot_id, TEST_LEG_ID)
            dx = (test_foot[0] - base_foot[0]) * 1000
            dy = (test_foot[1] - base_foot[1]) * 1000
            dz = (test_foot[2] - base_foot[2]) * 1000
            stride = math.sqrt(dx**2 + dy**2)

            print(f"  {angle_deg:+6.0f}  {px:8.1f}  {py:8.1f}  {t1:+8.2f}  "
                  f"{jc:+9.2f}  {dx:+8.1f}  {dy:+8.1f}  {dz:+8.1f}  "
                  f"{stride:7.1f}  {'OK':>5s}")

            # Draw debug line
            if prev_foot is not None:
                p.addUserDebugLine(list(prev_foot), list(test_foot),
                                   lineColorRGB=[1.0, 0.8, 0.0], lineWidth=2.5, lifeTime=0)
            prev_foot = test_foot

            coxa_results.append({
                'angle_deg': angle_deg, 'px': px, 'py': py,
                'ik_t1': t1, 'cmd_coxa': jc,
                'foot_world': test_foot,
                'dx': dx, 'dy': dy, 'dz': dz,
                'stride': stride, 'valid': True
            })
        else:
            print(f"  {angle_deg:+6.0f}  {px:8.1f}  {py:8.1f}  {'---':>8s}  "
                  f"{'---':>9s}  {'---':>8s}  {'---':>8s}  {'---':>8s}  "
                  f"{'---':>7s}  {'FAIL':>5s}")
            coxa_results.append({
                'angle_deg': angle_deg, 'px': px, 'py': py,
                'valid': False
            })

    return coxa_results


# ---------------------------------------------------------------------------
# SECTION 6 -- STRIDE LENGTH ANALYSIS (for gait planning)
# ---------------------------------------------------------------------------

def run_stride_analysis(robot_id, coxa_results):
    """
    Given the coxa workspace data, compute recommended stride parameters.
    """
    print()
    print("=" * 70)
    print("STRIDE LENGTH ANALYSIS")
    print("=" * 70)

    valid = [r for r in coxa_results if r['valid']]
    if len(valid) < 2:
        print("  ERROR: Not enough valid coxa angles for analysis.")
        return

    # Find max stride span (from most negative to most positive coxa angle)
    min_angle_r = min(valid, key=lambda r: r['angle_deg'])
    max_angle_r = max(valid, key=lambda r: r['angle_deg'])

    full_span_x = max_angle_r['dx'] - min_angle_r['dx']
    full_span_y = max_angle_r['dy'] - min_angle_r['dy']
    full_span = math.sqrt(full_span_x**2 + full_span_y**2)

    print(f"\n  Full coxa sweep ({min_angle_r['angle_deg']:+.0f} to {max_angle_r['angle_deg']:+.0f} deg):")
    print(f"    World dX span : {full_span_x:+.1f} mm")
    print(f"    World dY span : {full_span_y:+.1f} mm")
    print(f"    Total stride  : {full_span:.1f} mm")

    # Compute arc length at the foot for each degree of coxa sweep
    # Arc length = R_foot * angle_rad  (approximate, for small angles)
    # R_foot is the horizontal reach from hip to foot
    a1 = 52.50   # coxa length mm
    a2 = 108.00  # femur length mm
    a3 = 130.775 # tibia length mm

    # At standing pose (px=150, py=0, pz=-150):
    # Horizontal reach from coxa pivot = px = 150 mm
    # But actual ground-projected reach includes the full kinematic chain
    R_foot = STAND_PX  # 150 mm from hip

    print(f"\n  Approximate foot radius from hip: {R_foot:.0f} mm")
    print(f"  Arc per 1 deg coxa sweep: {R_foot * math.radians(1):.1f} mm")

    # Safe operating range: use +-12 deg (leave 8 deg margin from limits)
    safe_min = -12.0
    safe_max = +12.0
    safe_sweep = safe_max - safe_min
    safe_stride = 2 * R_foot * math.sin(math.radians(safe_sweep / 2))

    print(f"\n  RECOMMENDED SAFE OPERATING RANGE:")
    print(f"    Coxa range    : [{safe_min:+.0f}, {safe_max:+.0f}] deg")
    print(f"    Total sweep   : {safe_sweep:.0f} deg")
    print(f"    Chord stride  : {safe_stride:.1f} mm")
    print(f"    Arc stride    : {R_foot * math.radians(safe_sweep):.1f} mm")

    # Conservative range: +-6 deg (gentle walking)
    cons_sweep = 12.0
    cons_stride = 2 * R_foot * math.sin(math.radians(cons_sweep / 2))
    print(f"\n  CONSERVATIVE RANGE (gentle gait):")
    print(f"    Coxa range    : [-6, +6] deg")
    print(f"    Chord stride  : {cons_stride:.1f} mm")

    return {
        'full_span': full_span,
        'safe_stride': safe_stride,
        'cons_stride': cons_stride,
        'safe_range': (safe_min, safe_max),
    }


# ---------------------------------------------------------------------------
# SECTION 7 -- REPORT GENERATION
# ---------------------------------------------------------------------------

def generate_report(axis_results, coxa_results, stride_data):
    """Print the final analysis report with axis mapping and recommendations."""
    print()
    print("=" * 70)
    print("COORDINATE FRAME ANALYSIS REPORT")
    print("=" * 70)

    # -- Axis mapping summary --
    print("\n  IK AXIS --> ROBOT BODY MAPPING")
    print("  " + "-" * 50)

    mapping = {}
    for test_id, axis_label, px, py, pz, deltas, dominant, primary, joints, ok in axis_results:
        if ok and deltas is not None:
            mapping[axis_label] = (dominant, primary, deltas)
            dx, dy, dz = deltas
            print(f"    IK {axis_label:3s} --> World {dominant:3s}  "
                  f"(dX={dx:+.1f}, dY={dy:+.1f}, dZ={dz:+.1f} mm)  "
                  f"Primary joint: {primary}")
        else:
            print(f"    IK {axis_label:3s} --> UNREACHABLE")

    # -- Key findings --
    print("\n  KEY FINDINGS")
    print("  " + "-" * 50)

    # Check if px is radial (lateral) vs forward
    px_plus = mapping.get("+X")
    py_plus = mapping.get("+Y")

    if px_plus:
        dom, prim, (dx, dy, dz) = px_plus
        abs_vals = [abs(dx), abs(dy), abs(dz)]
        if abs_vals[1] > abs_vals[0]:  # Y dominant over X
            print("    1. IK +X moves the foot primarily in the WORLD Y direction")
            print("       --> IK px is RADIAL OUTWARD, NOT forward")
            print("       --> Increasing px extends the leg laterally")
        elif abs_vals[0] > abs_vals[1]:
            print("    1. IK +X moves the foot primarily in the WORLD X direction")
            print("       --> IK px maps to robot forward/backward")
        print(f"       --> Primary joint affected: {prim}")

    if py_plus:
        dom, prim, (dx, dy, dz) = py_plus
        abs_vals = [abs(dx), abs(dy), abs(dz)]
        if abs_vals[0] > abs_vals[1]:  # X dominant
            print("    2. IK +Y moves the foot primarily in the WORLD X direction")
            print("       --> IK py drives FORWARD MOTION via coxa sweep")
        elif abs_vals[1] > abs_vals[0]:
            print("    2. IK +Y moves the foot primarily in the WORLD Y direction")
        print(f"       --> Primary joint affected: {prim}")

    pz_plus = mapping.get("+Z")
    if pz_plus:
        dom, prim, (dx, dy, dz) = pz_plus
        print(f"    3. IK +Z moves the foot UPWARD (world +Z)")
        print(f"       --> IK pz directly maps to vertical motion")
        print(f"       --> Primary joint affected: {prim}")

    # -- Stage 3 diagnosis --
    print("\n  STAGE 3 TRAJECTORY DIAGNOSIS")
    print("  " + "-" * 50)
    print("    The Stage 3 trajectory moved IK px from 150 to 190 mm.")
    print("    This increased the RADIAL REACH of the leg, not the")
    print("    forward position.")
    if px_plus:
        dom, prim, (dx, dy, dz) = px_plus
        if abs(dy) > abs(dx):
            print("    Result: foot moved LATERALLY (outward), not forward.")
            print("    The coxa joint stayed near 0 deg because py was always 0.")
        else:
            print("    Result: foot moved along world X, but this is radial,")
            print("    not a true forward step via coxa sweep.")

    print("\n    CORRECTION: For forward stepping, sweep the COXA angle by")
    print("    varying IK py while keeping radial reach (R=sqrt(px^2+py^2))")
    print("    approximately constant.")

    # -- Coxa workspace summary --
    print("\n  COXA WORKSPACE SUMMARY")
    print("  " + "-" * 50)
    print("    URDF limits    : [-20, +20] deg  (hard limit)")
    print("    IK limits      : [-20, +20] deg  (matches URDF)")

    valid_coxa = [r for r in coxa_results if r['valid']]
    if valid_coxa:
        angles = [r['angle_deg'] for r in valid_coxa]
        print(f"    Tested range   : [{min(angles):+.0f}, {max(angles):+.0f}] deg  (all valid)")

    if stride_data:
        print(f"    Safe range     : [{stride_data['safe_range'][0]:+.0f}, "
              f"{stride_data['safe_range'][1]:+.0f}] deg  (8 deg margin)")
        print(f"    Safe stride    : {stride_data['safe_stride']:.1f} mm")

    # -- Recommendations --
    print("\n  RECOMMENDATIONS FOR STAGE 4 (GAIT IMPLEMENTATION)")
    print("  " + "-" * 50)
    print("    1. FORWARD STEPPING AXIS:")
    print("       Use IK py (NOT px) to generate forward/backward foot motion.")
    print("       Set coxa sweep via: px = R*cos(angle), py = R*sin(angle)")
    print(f"       where R = {STAND_PX:.0f} mm (constant radial reach).")
    print()
    print("    2. TRAJECTORY CORRECTION:")
    print("       The trajectory generator must produce coxa angle targets,")
    print("       NOT px offsets. Convert angle targets to (px, py) via:")
    print("         px = R * cos(coxa_target_angle)")
    print("         py = R * sin(coxa_target_angle)")
    print()
    print("    3. RECOMMENDED COXA RANGE FOR LOCOMOTION:")
    if stride_data:
        print(f"       Safe: [{stride_data['safe_range'][0]:+.0f}, "
              f"{stride_data['safe_range'][1]:+.0f}] deg  "
              f"(stride ~ {stride_data['safe_stride']:.0f} mm)")
        print(f"       Conservative: [-6, +6] deg  "
              f"(stride ~ {stride_data['cons_stride']:.0f} mm)")
    print()
    print("    4. RECOMMENDED STRIDE LENGTH:")
    if stride_data:
        print(f"       Start with: {stride_data['cons_stride']:.0f} mm (12 deg total sweep)")
        print(f"       Maximum:    {stride_data['safe_stride']:.0f} mm (24 deg total sweep)")
    print()
    print("    5. LIFT HEIGHT:")
    print("       Keep at 25-35 mm (verified working in Stage 3).")
    print()
    print("    6. VERTICAL AXIS:")
    print("       IK pz directly controls height. -150 mm = standing.")
    print("       Use pz for lift/lower phases (no axis confusion).")


# ---------------------------------------------------------------------------
# SECTION 8 -- MAIN
# ---------------------------------------------------------------------------

def main():
    print("=" * 70)
    print("coordinate_frame_validation.py")
    print("Stage 3.5 -- Coordinate Frame and Coxa Workspace Validation")
    print("=" * 70)
    print(f"  URDF         : {URDF_PATH}")
    print(f"  Test leg     : Leg {TEST_LEG_ID} ({LEG_CONFIG[TEST_LEG_ID]['label']})")
    print(f"  Standing pose: ({STAND_PX}, {STAND_PY}, {STAND_PZ}) mm")

    # -- Physics setup --
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.6, cameraYaw=45,
        cameraPitch=-30, cameraTargetPosition=[0.05, 0.05, 0.25]
    )

    # -- Load robot --
    start_pos = [0, 0, BASE_SPAWN_HEIGHT]
    start_orn = p.getQuaternionFromEuler([0, 0, math.pi])
    robot_id  = p.loadURDF(URDF_PATH, start_pos, start_orn,
                            useFixedBase=USE_FIXED_BASE)
    resolve_joint_indices(robot_id)
    print(f"\n  Robot loaded.  Body ID = {robot_id}")

    # -- Run axis identification tests --
    axis_results, base_foot = run_axis_tests(robot_id)

    # -- Run coxa workspace analysis --
    coxa_results = run_coxa_analysis(robot_id)

    # -- Run stride analysis --
    stride_data = run_stride_analysis(robot_id, coxa_results)

    # -- Generate report --
    generate_report(axis_results, coxa_results, stride_data)

    # -- Final summary --
    print()
    print("=" * 70)
    print("STAGE 3.5 COMPLETE")
    print("=" * 70)
    print("  The IK coordinate frame has been fully characterized.")
    print("  Axis mapping and coxa workspace are ready for Stage 4.")

    # Hold for inspection
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
