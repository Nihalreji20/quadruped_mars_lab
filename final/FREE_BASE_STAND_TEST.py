#!/usr/bin/env python3
"""
FREE_BASE_STAND_TEST.py
=======================
Stage 5B -- Free-Base Static Standing Validation (Diagnostic Version)

PURPOSE
    Determine whether the robot can statically support its own mass under gravity.

DIAGNOSTIC UPDATES APPLIED:
    - TEST A: positionGain=1.0 applied to motors, lateralFriction=1.5 applied 
              to the floor and tibia links to stop sliding and yielding.
    - TEST B: STAND_PZ reduced to -120.0 to lower the center of gravity and 
              reduce the physical lever-arm on the joints.
    - TEST C: Configurable DROP_TOWER_TEST boolean added to lock the robot to 
              a Z-axis rail for isolating pure vertical motor strength.

REUSED VERIFIED COMPONENTS (DO NOT MODIFY)
    compute_ik()            -- ground-truth IK solver
    compute_leg_joints()    -- sign correction transformation layer
"""

import sys
import pybullet as p
import pybullet_data
import time
import math

# ============================================================================
# SECTION 1 -- CONFIGURATION
# ============================================================================

URDF_PATH          = "urdf/finalv6_pla.urdf"
BASE_SPAWN_HEIGHT  = 0.30   # Dropping from 30cm
USE_FIXED_BASE     = False  # CRITICAL: Let gravity take over

# DIAGNOSTIC TOGGLE: TEST C (The Drop Tower)
# Set to True to lock the robot on a vertical rail (no rolling or pitching allowed)
RUN_DROP_TOWER_TEST = False 

# Standing pose (TEST B: Squat lowered from -150 to -120 mm)
STAND_PX = 150.0                    # mm
STAND_PY =   0.0                    # mm
STAND_PZ = -120.0                   # mm

# Physics & Test Parameters
SIM_TIME_SEC       = 15.0
SETTLE_TIME_SEC    = 2.0
CONTROL_RATE_HZ    = 240
MOTOR_FORCE        = 1.08    # N·m

# Pass/Fail Thresholds
MAX_JOINT_ERR_DEG  = 5.0            # Motors must hold within 5 degrees
MAX_ROLL_PITCH_DEG = 10.0           # Body must not tip over
MIN_BASE_Z_M       = 0.08           # Adjusted for lower squat stance
MAX_Z_DRIFT_M      = 0.02           # Max allowed sag after settling (2 cm)

# Default Camera Target
CAMERA_TARGET      = [0, 0, 0.15]   # [X, Y, Z] focus point

# ============================================================================
# SECTION 2 -- LEG CONFIGURATION TABLE (Verified)
# ============================================================================

LEG_CONFIG = {
    1: {
        'label'         : 'Leg 1 -- FL (Front Left)',
        'coxa_joint_name'  : 'Revolute 1',
        'femur_joint_name' : 'Revolute 8',
        'tibia_joint_name' : 'Revolute 11',
        'coxa_sign'     : -1,
        'femur_sign'    : -1,
        'tibia_sign'    : +1,
    },
    2: {
        'label'         : 'Leg 2 -- BR (Back Right)',
        'coxa_joint_name'  : 'Revolute 2',
        'femur_joint_name' : 'Revolute 7',
        'tibia_joint_name' : 'Revolute 15',
        'coxa_sign'     : -1,
        'femur_sign'    : -1,
        'tibia_sign'    : -1,
    },
    3: {
        'label'         : 'Leg 3 -- FR (Front Right)',
        'coxa_joint_name'  : 'Revolute 3',
        'femur_joint_name' : 'Revolute 6',
        'tibia_joint_name' : 'Revolute 13',
        'coxa_sign'     : -1,
        'femur_sign'    : -1,
        'tibia_sign'    : +1,
    },
    4: {
        'label'         : 'Leg 4 -- BL (Back Left)',
        'coxa_joint_name'  : 'Revolute 4',
        'femur_joint_name' : 'Revolute 10',
        'tibia_joint_name' : 'Revolute 14',
        'coxa_sign'     : -1,
        'femur_sign'    : -1,
        'tibia_sign'    : +1,
    },
}

# ============================================================================
# SECTION 3 -- REUSED VERIFIED COMPONENTS (DO NOT MODIFY)
# ============================================================================

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
            raise RuntimeError(f"Joint names not found.")

def compute_ik(px, py, pz):
    a1 = 52.50;  a2 = 108.00;  a3 = 130.775
    t1_rad = math.atan2(py, px)
    t1_deg = math.degrees(t1_rad)
    R = math.sqrt(px**2 + py**2)
    r = R - a1;  z = pz
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
    jc = t1 * cfg['coxa_sign']
    jf = t2 * cfg['femur_sign']
    jt = -t3 if cfg['tibia_sign'] == -1 else t3
    return jc, jf, jt, True

def set_leg(robot_id, leg_id, px, py, pz, force=MOTOR_FORCE):
    jc, jf, jt, ok = compute_leg_joints(leg_id, px, py, pz)
    if not ok: return False
    cfg = LEG_CONFIG[leg_id]
    
    # TEST A: positionGain=1.0 added to enforce rigid motor tracking
    p.setJointMotorControl2(robot_id, cfg['coxa_idx'], p.POSITION_CONTROL, 
                            targetPosition=math.radians(jc), force=force, positionGain=1.0)
    p.setJointMotorControl2(robot_id, cfg['femur_idx'], p.POSITION_CONTROL, 
                            targetPosition=math.radians(jf), force=force, positionGain=1.0)
    p.setJointMotorControl2(robot_id, cfg['tibia_idx'], p.POSITION_CONTROL, 
                            targetPosition=math.radians(jt), force=force, positionGain=1.0)
    return True

def get_joint_deg(robot_id, joint_idx):
    return math.degrees(p.getJointState(robot_id, joint_idx)[0])

def update_camera(dist_slider, yaw_slider, pitch_slider):
    """Reads slider values and updates the PyBullet camera."""
    dist = p.readUserDebugParameter(dist_slider)
    yaw = p.readUserDebugParameter(yaw_slider)
    pitch = p.readUserDebugParameter(pitch_slider)
    p.resetDebugVisualizerCamera(
        cameraDistance=dist,
        cameraYaw=yaw,
        cameraPitch=pitch,
        cameraTargetPosition=CAMERA_TARGET
    )

# ============================================================================
# SECTION 4 -- MAIN EXECUTION
# ============================================================================

def main():
    print("=" * 70)
    print("FREE_BASE_STAND_TEST.py (Diagnostic Version)")
    print("Stage 5B -- Free-Base Static Standing Validation")
    print("=" * 70)
    print(f"  URDF         : {URDF_PATH}")
    print(f"  Fixed base   : {USE_FIXED_BASE} (Testing Gravity Support)")
    print(f"  Target PZ    : {STAND_PZ} mm (Test B: Squat Posture)")
    print(f"  Drop Tower   : {'ENABLED' if RUN_DROP_TOWER_TEST else 'DISABLED'}")
    print(f"  Motor Force  : {MOTOR_FORCE} N·m (Test A: positionGain=1.0)\n")

    # ---- Physics Setup ----
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # TEST A: High floor friction
    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.5)
    
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

    # --- NEW: Setup Camera Sliders ---
    # format: name, min_val, max_val, start_val
    dist_slider = p.addUserDebugParameter("Camera Distance", 0.1, 5.0, 1.0)
    yaw_slider = p.addUserDebugParameter("Camera Yaw", -180, 180, 45)
    pitch_slider = p.addUserDebugParameter("Camera Pitch", -89, 89, -30)
    
    # Spawn Robot
    start_pos = [0, 0, BASE_SPAWN_HEIGHT]
    start_orn = p.getQuaternionFromEuler([0, 0, math.pi])
    robot_id  = p.loadURDF(URDF_PATH, start_pos, start_orn, useFixedBase=USE_FIXED_BASE)
    resolve_joint_indices(robot_id)

    # TEST A: High foot friction
    for lid in range(1, 5):
        p.changeDynamics(robot_id, LEG_CONFIG[lid]['tibia_idx'], lateralFriction=1.5)

    # TEST C: Drop Tower Rail Constraint
    if RUN_DROP_TOWER_TEST:
        print("  [DIAGNOSTIC] Drop Tower enabled. Robot is locked to Z-axis rail.")
        p.createConstraint(robot_id, -1, -1, -1, p.JOINT_PRISMATIC, [0, 0, 1], [0, 0, 0], [0, 0, 0])

    # Calculate expected joint angles for the error tracker
    expected_joints = {}
    for lid in range(1, 5):
        jc, jf, jt, _ = compute_leg_joints(lid, STAND_PX, STAND_PY, STAND_PZ)
        cfg = LEG_CONFIG[lid]
        expected_joints[cfg['coxa_idx']] = jc
        expected_joints[cfg['femur_idx']] = jf
        expected_joints[cfg['tibia_idx']] = jt

    # Command rigid standing pose immediately so it braces for impact
    print("  Commanding standing pose and dropping robot...")
    for lid in range(1, 5):
        set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)

    # ---- Phase 1: Settle ----
    print(f"  Settling for {SETTLE_TIME_SEC} seconds to dissipate impact...")
    for _ in range(int(SETTLE_TIME_SEC * CONTROL_RATE_HZ)):
        update_camera(dist_slider, yaw_slider, pitch_slider) # Allow camera movement while settling
        p.stepSimulation()
        time.sleep(1.0 / CONTROL_RATE_HZ)

    # Capture Baseline
    base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
    baseline_z = base_pos[2]
    roll_base, pitch_base, _ = p.getEulerFromQuaternion(base_orn)

    print(f"  Baseline Z established at : {baseline_z:.4f} m")

    # ---- Phase 2: Hold & Measure ----
    print(f"  Holding under gravity for {SIM_TIME_SEC} seconds...")
    
    max_z_drift = 0.0
    max_roll_deg = 0.0
    max_pitch_deg = 0.0
    max_joint_error = 0.0
    final_z = 0.0

    steps = int(SIM_TIME_SEC * CONTROL_RATE_HZ)
    for step in range(steps):
        # Update camera based on slider positions
        update_camera(dist_slider, yaw_slider, pitch_slider)

        # Refresh motor commands
        for lid in range(1, 5):
            set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)

        p.stepSimulation()
        time.sleep(1.0 / CONTROL_RATE_HZ)

        # Measure Tracking Error
        for j_idx, expected_deg in expected_joints.items():
            actual_deg = get_joint_deg(robot_id, j_idx)
            err = abs(actual_deg - expected_deg)
            if err > max_joint_error:
                max_joint_error = err

        # Measure Body state
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        roll, pitch, yaw = p.getEulerFromQuaternion(orn)
        
        z_drift = abs(pos[2] - baseline_z)
        if z_drift > max_z_drift: max_z_drift = z_drift
        
        roll_deg = abs(math.degrees(roll))
        if roll_deg > max_roll_deg: max_roll_deg = roll_deg
        
        pitch_deg = abs(math.degrees(pitch))
        if pitch_deg > max_pitch_deg: max_pitch_deg = pitch_deg
        
        final_z = pos[2]

        if step % CONTROL_RATE_HZ == 0:
            print(f"    Elapsed {step/CONTROL_RATE_HZ:2.0f}s | Z: {pos[2]:.4f}m | Max Err: {max_joint_error:.1f}°")
    dyn = p.getDynamicsInfo(robot_id, -1)

    print("Mass:", dyn[0], "kg")
    print("Local inertia diagonal:", dyn[2])
    print("COM offset from base frame:", dyn[3])

    # ---- Phase 3: Validation & Reporting ----
    print("\n" + "=" * 70)
    print("STAGE 5B RESULTS & DIAGNOSTICS")
    print("=" * 70)

    pass_z       = final_z > MIN_BASE_Z_M
    pass_drift   = max_z_drift <= MAX_Z_DRIFT_M
    pass_tilt    = (max_roll_deg <= MAX_ROLL_PITCH_DEG) and (max_pitch_deg <= MAX_ROLL_PITCH_DEG)
    pass_motors  = max_joint_error <= MAX_JOINT_ERR_DEG

    print(f"  1. Ground Clearance (Z > {MIN_BASE_Z_M}m)       : {final_z:.4f} m  ({'PASS' if pass_z else 'FAIL'})")
    print(f"  2. Sag/Drift post-settle (<= {MAX_Z_DRIFT_M}m)  : {max_z_drift:.4f} m  ({'PASS' if pass_drift else 'FAIL'})")
    print(f"  3. Max Roll  (<= {MAX_ROLL_PITCH_DEG}°)            : {max_roll_deg:.2f}°   ({'PASS' if pass_tilt and max_roll_deg <= MAX_ROLL_PITCH_DEG else 'FAIL'})")
    print(f"  4. Max Pitch (<= {MAX_ROLL_PITCH_DEG}°)            : {max_pitch_deg:.2f}°   ({'PASS' if pass_tilt and max_pitch_deg <= MAX_ROLL_PITCH_DEG else 'FAIL'})")
    print(f"  5. Max Joint Error (<= {MAX_JOINT_ERR_DEG}°)        : {max_joint_error:.2f}°   ({'PASS' if pass_motors else 'FAIL'})")

    overall = pass_z and pass_drift and pass_tilt and pass_motors

    print("\n" + "=" * 70)
    if overall:
        print("  STAGE 5B COMPLETE — ROBOT SUPPORTS ITS OWN WEIGHT.")
        print("  The physical center of mass and motor strengths are balanced.")
        print("  You are fully cleared for Stage 6: Free-Base Locomotion.")
    else:
        print("  STAGE 5B FAILED — STRUCTURAL OR MOTOR COLLAPSE.")
        print("\n  NEXT STEPS:")
        if not RUN_DROP_TOWER_TEST:
            print("  Change `RUN_DROP_TOWER_TEST = True` at the top of the script and re-run.")
            print("  This will lock the robot vertically to tell us if the COM is unbalancing the robot,")
            print("  or if the motors/geometry are simply too weak.")

    print("\n  Holding for visual inspection. Press Ctrl+C to exit.")
    try:
        while True:
            # Allow camera movement during the final visual hold
            update_camera(dist_slider, yaw_slider, pitch_slider)
            
            for lid in range(1, 5):
                set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)
            p.stepSimulation()
            time.sleep(1.0 / CONTROL_RATE_HZ)
    except KeyboardInterrupt:
        p.disconnect()
        print("\n  Simulation ended.")


if __name__ == "__main__":
    main()