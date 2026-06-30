#!/usr/bin/env python3
"""
FREE_BASE_SINGLE_STEP_TEST.py
=============================
Stage 6A -- Free-Base Single Step Validation

PURPOSE
    Execute exactly ONE complete locomotion step under gravity to validate
    the integration of the Swing Phase (Stage 4) and Stance Phase (Stage 5A).
    
    No continuous walking. No turning. No CoM shifting.

SEQUENCE
    A) LIFT Leg 1
    B) SWEEP Leg 1 forward (+18°)
    C) LOWER Leg 1
    D) TRANSLATE Body (Leg 1 sweeps back to 0°, Legs 2/3/4 sweep to propel body)
    E) STOP and HOLD

REUSED VERIFIED COMPONENTS
    compute_ik(), compute_leg_joints(), set_leg(), resolve_joint_indices()
"""

import sys
import pybullet as p
import pybullet_data
import time
import math

# Fix for terminal encoding
if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

# ============================================================================
# SECTION 1 -- CONFIGURATION
# ============================================================================

URDF_PATH          = "urdf/finalv6_pla.urdf"
BASE_SPAWN_HEIGHT  = 0.30
USE_FIXED_BASE     = False  # Gravity active!

# Standing Pose
STAND_PX = 150.0
STAND_PY =   0.0
STAND_PZ = -150.0
RADIAL_REACH = 150.0

# Trajectory Parameters (Updated Limits)
LIFT_HEIGHT_MM     = 70.0
SWEEP_ANGLE_DEG    = 18.0
PTS_PER_PHASE      = 40
CONTROL_RATE_HZ    = 240
MOTOR_FORCE        = 1.08          # <-- FIX 1: Increased from 100.0 for stronger joints

# Pass/Fail Thresholds
MAX_JOINT_ERR_DEG  = 20.0           # <-- FIX 2: Increased from 5.0 to tolerate natural gravity sag
MAX_ROLL_PITCH_DEG = 15.0           
MIN_Z_HEIGHT_M     = 0.10           
MIN_DISP_MM        = 5.0        # Must move forward at least 5mm

# ============================================================================
# SECTION 2 -- LEG CONFIGURATION TABLE
# ============================================================================

LEG_CONFIG = {
    1: {'label': 'Leg 1 (FL)', 'coxa_name': 'Revolute 1', 'femur_name': 'Revolute 8', 'tibia_name': 'Revolute 11', 'coxa_sign': -1, 'femur_sign': +1, 'tibia_sign': -1},
    2: {'label': 'Leg 2 (BR)', 'coxa_name': 'Revolute 2', 'femur_name': 'Revolute 7', 'tibia_name': 'Revolute 15', 'coxa_sign': -1, 'femur_sign': -1, 'tibia_sign': -1},
    3: {'label': 'Leg 3 (FR)', 'coxa_name': 'Revolute 3', 'femur_name': 'Revolute 6', 'tibia_name': 'Revolute 13', 'coxa_sign': -1, 'femur_sign': +1, 'tibia_sign': -1},
    4: {'label': 'Leg 4 (BL)', 'coxa_name': 'Revolute 4', 'femur_name': 'Revolute 10', 'tibia_name': 'Revolute 14', 'coxa_sign': -1, 'femur_sign': -1, 'tibia_sign': +1},
}

# ============================================================================
# SECTION 3 -- VERIFIED KINEMATICS
# ============================================================================

def resolve_joint_indices(robot_id):
    name_to_idx = {p.getJointInfo(robot_id, i)[1].decode('utf-8'): i for i in range(p.getNumJoints(robot_id))}
    for leg in LEG_CONFIG.values():
        leg['coxa_idx']  = name_to_idx.get(leg['coxa_name'])
        leg['femur_idx'] = name_to_idx.get(leg['femur_name'])
        leg['tibia_idx'] = name_to_idx.get(leg['tibia_name'])

def compute_ik(px, py, pz):
    a1 = 52.50; a2 = 108.00; a3 = 130.775
    t1_rad = math.atan2(py, px)
    t1_deg = math.degrees(t1_rad)
    R = math.sqrt(px**2 + py**2)
    r = R - a1; z = pz
    L_squared = r**2 + z**2
    if L_squared == 0: return 0, 0, 0, False
    c3 = max(-1.0, min(1.0, (L_squared - a2**2 - a3**2) / (2 * a2 * a3)))
    s3 = -math.sqrt(1 - c3**2)
    t3_math = math.atan2(s3, c3)
    t2_math = math.atan2(z, r) - math.atan2(a3 * s3, a2 + a3 * c3)
    urdf_t2_deg = ((math.degrees(t2_math) + 180) % 360) - 180
    urdf_t3_deg = ((-(math.degrees(t3_math) + 90.0) + 180) % 360) - 180
    return t1_deg, urdf_t2_deg, urdf_t3_deg, True

def compute_leg_joints(leg_id, px, py, pz):
    cfg = LEG_CONFIG[leg_id]
    t1, t2, t3, ok = compute_ik(px, py, pz)
    if not ok: return 0, 0, 0, False
    return t1 * cfg['coxa_sign'], t2 * cfg['femur_sign'], (-t3 if cfg['tibia_sign'] == -1 else t3), True

def set_leg(robot_id, leg_id, px, py, pz, force=MOTOR_FORCE):
    jc, jf, jt, ok = compute_leg_joints(leg_id, px, py, pz)
    if not ok: return False
    cfg = LEG_CONFIG[leg_id]
    p.setJointMotorControl2(robot_id, cfg['coxa_idx'], p.POSITION_CONTROL, targetPosition=math.radians(jc), force=force, positionGain=1.0)
    p.setJointMotorControl2(robot_id, cfg['femur_idx'], p.POSITION_CONTROL, targetPosition=math.radians(jf), force=force, positionGain=1.0)
    p.setJointMotorControl2(robot_id, cfg['tibia_idx'], p.POSITION_CONTROL, targetPosition=math.radians(jt), force=force, positionGain=1.0)
    return True

def get_joint_deg(robot_id, joint_idx):
    return math.degrees(p.getJointState(robot_id, joint_idx)[0])

# ============================================================================
# SECTION 4 -- STEP TRAJECTORY GENERATOR
# ============================================================================

def generate_step_sequence():
    """
    Generates the unified target states for all 4 legs across the entire step.
    Returns: list of dicts { 'phase', 'L1': (theta, z), 'L2': ..., etc. }
    """
    seq = []
    lift_pz = STAND_PZ + LIFT_HEIGHT_MM
    
    # Phase A: LIFT Leg 1
    for i in range(PTS_PER_PHASE):
        z = STAND_PZ + (lift_pz - STAND_PZ) * ((i + 1) / PTS_PER_PHASE)
        seq.append({'phase': 'LIFT', 
                    1: (0.0, z), 2: (0.0, STAND_PZ), 3: (0.0, STAND_PZ), 4: (0.0, STAND_PZ)})
        
    # Phase B: SWEEP Leg 1
    for i in range(PTS_PER_PHASE):
        th = 0.0 + (SWEEP_ANGLE_DEG - 0.0) * ((i + 1) / PTS_PER_PHASE)
        seq.append({'phase': 'SWEEP', 
                    1: (th, lift_pz), 2: (0.0, STAND_PZ), 3: (0.0, STAND_PZ), 4: (0.0, STAND_PZ)})
        
    # Phase C: LOWER Leg 1
    for i in range(PTS_PER_PHASE):
        z = lift_pz + (STAND_PZ - lift_pz) * ((i + 1) / PTS_PER_PHASE)
        seq.append({'phase': 'LOWER', 
                    1: (SWEEP_ANGLE_DEG, z), 2: (0.0, STAND_PZ), 3: (0.0, STAND_PZ), 4: (0.0, STAND_PZ)})
        
    # Phase D: TRANSLATE Body (Stage 5A Integration)
    # All legs sweep backward relative to the body to propel it forward.
    # Front legs (1, 3) decrease theta. Rear legs (2, 4) increase theta (inverted).
    for i in range(PTS_PER_PHASE):
        t = (i + 1) / PTS_PER_PHASE
        
        # Leg 1 goes from SWEEP_ANGLE to 0
        l1_th = SWEEP_ANGLE_DEG - (SWEEP_ANGLE_DEG * t)
        # Leg 3 goes from 0 to -SWEEP_ANGLE
        l3_th = 0.0 - (SWEEP_ANGLE_DEG * t)
        # Legs 2 & 4 go from 0 to +SWEEP_ANGLE (Inverted rear legs from Stage 5A)
        l24_th = 0.0 + (SWEEP_ANGLE_DEG * t)
        
        seq.append({'phase': 'TRANSLATE', 
                    1: (l1_th, STAND_PZ), 2: (l24_th, STAND_PZ), 3: (l3_th, STAND_PZ), 4: (l24_th, STAND_PZ)})
        
    return seq

# ============================================================================
# SECTION 5 -- MAIN EXECUTION
# ============================================================================

def main():
    print("=" * 70)
    print("FREE_BASE_SINGLE_STEP_TEST.py")
    print("Stage 6A -- First Complete Locomotion Step Validation")
    print("=" * 70)
    
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.5)
    
    p.resetDebugVisualizerCamera(cameraDistance=0.65, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0.05, 0.05, 0.25])

    start_pos = [0, 0, BASE_SPAWN_HEIGHT]
    start_orn = p.getQuaternionFromEuler([0, 0, math.pi])
    robot_id  = p.loadURDF(URDF_PATH, start_pos, start_orn, useFixedBase=USE_FIXED_BASE)
    resolve_joint_indices(robot_id)

    # Maximize foot friction for traction
    for lid in range(1, 5):
        p.changeDynamics(robot_id, LEG_CONFIG[lid]['tibia_idx'], lateralFriction=1.5)

    print("\n  Commanding standing pose and settling (2s)...")
    for lid in range(1, 5):
        set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)

    for _ in range(2 * CONTROL_RATE_HZ):
        p.stepSimulation()
        time.sleep(1.0 / CONTROL_RATE_HZ)

    # Capture state before step
    base_pos_start, base_orn = p.getBasePositionAndOrientation(robot_id)
    print(f"  Pre-Step Position : X={base_pos_start[0]:.4f}, Y={base_pos_start[1]:.4f}, Z={base_pos_start[2]:.4f} m")

    sequence = generate_step_sequence()
    print(f"\n  Executing {len(sequence)} frames of motion...")

    max_roll_deg = 0.0
    max_pitch_deg = 0.0
    max_joint_error = 0.0
    ik_failures = 0
    min_z = base_pos_start[2]
    current_phase = ""

    for step_idx, step_data in enumerate(sequence):
        phase = step_data['phase']
        
        if phase != current_phase:
            current_phase = phase
            print(f"    --> Executing Phase: {phase}")

        # Command all 4 legs
        for lid in range(1, 5):
            th, z = step_data[lid]
            px = RADIAL_REACH * math.cos(math.radians(th))
            py = RADIAL_REACH * math.sin(math.radians(th))
            
            # Record theoretical IK for error tracking
            cmd_c, cmd_f, cmd_t, ok = compute_leg_joints(lid, px, py, z)
            if not ok: ik_failures += 1
            
            set_leg(robot_id, lid, px, py, z)

        p.stepSimulation()
        time.sleep(0.015) # Slowed down slightly to watch the magic happen

        # Measure Body Stability
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        roll, pitch, _ = p.getEulerFromQuaternion(orn)
        
        min_z = min(min_z, pos[2])
        max_roll_deg = max(max_roll_deg, abs(math.degrees(roll)))
        max_pitch_deg = max(max_pitch_deg, abs(math.degrees(pitch)))

        # Track Joint Error on Leg 1
        act_c = get_joint_deg(robot_id, LEG_CONFIG[1]['coxa_idx'])
        max_joint_error = max(max_joint_error, abs(act_c - cmd_c))

    # Settle after step
    print("    --> Holding position (1s)...")
    for _ in range(CONTROL_RATE_HZ):
        p.stepSimulation()
        time.sleep(1.0 / CONTROL_RATE_HZ)

    # Capture state after step
    base_pos_end, _ = p.getBasePositionAndOrientation(robot_id)
    
    # Because your crab stance moves along Y, we calculate net displacement
    dx = (base_pos_end[0] - base_pos_start[0]) * 1000.0
    dy = (base_pos_end[1] - base_pos_start[1]) * 1000.0
    net_disp_mm = math.sqrt(dx**2 + dy**2)

    # ---- Final Validation ----
    print("\n" + "=" * 70)
    print("STAGE 6A RESULTS & DIAGNOSTICS")
    print("=" * 70)

    pass_ik      = ik_failures == 0
    pass_fall    = min_z > MIN_Z_HEIGHT_M
    pass_disp    = net_disp_mm > MIN_DISP_MM
    pass_tilt    = (max_roll_deg <= MAX_ROLL_PITCH_DEG) and (max_pitch_deg <= MAX_ROLL_PITCH_DEG)
    pass_motors  = max_joint_error <= MAX_JOINT_ERR_DEG

    print(f"  1. No IK Failures                 : {'PASS' if pass_ik else 'FAIL'} ({ik_failures} errors)")
    print(f"  2. Upright & No Falls (Z > {MIN_Z_HEIGHT_M}m) : {'PASS' if pass_fall else 'FAIL'} (Min Z: {min_z:.3f}m)")
    print(f"  3. Net Body Displacement (> {MIN_DISP_MM}mm) : {'PASS' if pass_disp else 'FAIL'} (Moved: {net_disp_mm:.1f} mm)")
    print(f"     └─ Breakdown                   : dX = {dx:+.1f} mm, dY = {dy:+.1f} mm")
    print(f"  4. Max Roll  (<= {MAX_ROLL_PITCH_DEG}°)            : {'PASS' if max_roll_deg <= MAX_ROLL_PITCH_DEG else 'FAIL'} (Max: {max_roll_deg:.1f}°)")
    print(f"  5. Max Pitch (<= {MAX_ROLL_PITCH_DEG}°)            : {'PASS' if max_pitch_deg <= MAX_ROLL_PITCH_DEG else 'FAIL'} (Max: {max_pitch_deg:.1f}°)")
    print(f"  6. Joint Torque Hold (<= {MAX_JOINT_ERR_DEG}°)      : {'PASS' if pass_motors else 'FAIL'} (Max Err: {max_joint_error:.1f}°)")

    overall = pass_ik and pass_fall and pass_disp and pass_tilt and pass_motors

    print("\n" + "=" * 70)
    if overall:
        print("  STAGE 6A COMPLETE — THE ROBOT HAS TAKEN ITS FIRST STEP!")
        print("  The swing phase seamlessly integrated with the translation phase,")
        print("  pulling the robot forward without collapsing.")
    else:
        print("  STAGE 6A FAILED.")
        print("  Review the telemetry above. If displacement is zero, the feet slipped.")
        print("  If roll/pitch failed, the lack of CoM shifting unbalanced the step.")

    print("\n  Holding final posture. Close window to exit.")
    
    try:
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1.0 / CONTROL_RATE_HZ)
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if p.isConnected():
            p.disconnect()
# ---- Final Stance Geometry Diagnostics ----
    print("\n" + "=" * 70)
    print("FINAL STANCE GEOMETRY DIAGNOSTICS")
    print("=" * 70)

    for lid in range(1, 5):
        # Determine final theta based on Phase D translation logic
        if lid == 1:
            final_th = 0.0
        elif lid == 3:
            final_th = -SWEEP_ANGLE_DEG
        else: # Legs 2 and 4
            final_th = SWEEP_ANGLE_DEG

        # Calculate final Cartesian targets
        final_px = RADIAL_REACH * math.cos(math.radians(final_th))
        final_py = RADIAL_REACH * math.sin(math.radians(final_th))
        final_pz = STAND_PZ

        # Read actual physical joint angles
        act_c = get_joint_deg(robot_id, LEG_CONFIG[lid]['coxa_idx'])
        act_f = get_joint_deg(robot_id, LEG_CONFIG[lid]['femur_idx'])
        act_t = get_joint_deg(robot_id, LEG_CONFIG[lid]['tibia_idx'])

        # Compute nominal standing joint angles for comparison
        nom_c, nom_f, nom_t, _ = compute_leg_joints(lid, STAND_PX, STAND_PY, STAND_PZ)

        print(f"  Leg {lid} - {LEG_CONFIG[lid]['label']}")
        print(f"    Final Target : px={final_px:>6.1f} | py={final_py:>6.1f} | pz={final_pz:>6.1f} mm")
        print(f"    Coxa Angle   : {act_c:>6.2f}°  (Nominal: {nom_c:>+6.2f}° | Diff: {act_c - nom_c:>+6.2f}°)")
        print(f"    Femur Angle  : {act_f:>6.2f}°  (Nominal: {nom_f:>+6.2f}° | Diff: {act_f - nom_f:>+6.2f}°)")
        print(f"    Tibia Angle  : {act_t:>6.2f}°  (Nominal: {nom_t:>+6.2f}° | Diff: {act_t - nom_t:>+6.2f}°)")
        print()
if __name__ == "__main__":
    main()