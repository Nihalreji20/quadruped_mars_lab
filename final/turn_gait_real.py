#!/usr/bin/env python3
"""
free_base_crawl_stage6b.py
===========================
Stage 6B -- Continuous Directional Crawl Locomotion.
(Pure Emergent Physics tuned strictly for real MG996R 1.08 Nm Torque limits)
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

# ============================================================================
# SECTION 1 -- REAL MOTOR & PHYSICAL CONFIGURATION
# ============================================================================

URDF_PATH          = "urdf/finalv6_pla.urdf"
BASE_SPAWN_HEIGHT = 0.38
SETTLE_STEPS      = 360

# ### REAL MOTOR SPECIFICATION ###
# MG996R at 6.0V outputs ~11 kg-cm of torque, which equals exactly 1.078 Nm.
MG996R_MAX_TORQUE = 1.078 

# ### STABILITY OPTIMIZATIONS FOR 1.08 Nm LIMIT ###
STAND_PX = 180.0    # Widens the footprint area of the 3-legged support triangle
STAND_PZ = -135.0   # Lowers the center of gravity so the 1.08Nm servos aren't overwhelmed

LIFT_HEIGHT      =  12.0    # Keep leg lift low to minimize pendulum rocking forces
APEX_PZ          = STAND_PZ + LIFT_HEIGHT  

INTERP_STEPS     = 40       
STEP_DELAY       = 0.006    
SIM_SUBSTEPS     = 3        

GAIT_ORDER       = [1, 2, 4, 3]

ANGLE_TOLERANCE_DEG = 2.0
COXA_SAFE_MAX   = +20    

STAND_HOLD_SEC  = 1.5       
PHASE_PAUSE_SEC = 0.3       
USE_FIXED_BASE_6B = False    

STEP_LENGTH = 62.4
TRACK_WIDTH = 187.502
TURN_CYCLES = 5

# ============================================================================
# SECTION 2 -- LEG CONFIGURATION TABLE
# ============================================================================

LEG_CONFIG = {
    # ### KNEE SIGN FIX: Legs 1, 2, and 3 use +1 to match your inward-bending simulv2.py math ###
    1: {'label': 'Leg 1 -- FL', 'hip_xyz_urdf': (-0.075, -0.093751, -0.01255), 'coxa_idx': 0, 'femur_idx': 6, 'tibia_idx': 8, 'coxa_joint_name': 'Revolute 1', 'femur_joint_name': 'Revolute 8', 'tibia_joint_name': 'Revolute 11', 'coxa_sign': -1, 'femur_sign': +1, 'tibia_sign': +1},
    2: {'label': 'Leg 2 -- BR', 'hip_xyz_urdf': (+0.075, +0.093751, -0.01255), 'coxa_idx': 1, 'femur_idx': 5, 'tibia_idx': 11, 'coxa_joint_name': 'Revolute 2', 'femur_joint_name': 'Revolute 7', 'tibia_joint_name': 'Revolute 15', 'coxa_sign': +1, 'femur_sign': +1, 'tibia_sign': +1},
    3: {'label': 'Leg 3 -- FR', 'hip_xyz_urdf': (-0.075, +0.093751, -0.01255), 'coxa_idx': 2, 'femur_idx': 4, 'tibia_idx': 9, 'coxa_joint_name': 'Revolute 3', 'femur_joint_name': 'Revolute 6', 'tibia_joint_name': 'Revolute 13', 'coxa_sign': -1, 'femur_sign': +1, 'tibia_sign': +1},
    
    # Leg 4 is physically mirrored in your URDF (xyz="0 -1 0"), so it retains -1 to handle hardware inversion
    4: {'label': 'Leg 4 -- BL', 'hip_xyz_urdf': (+0.075, -0.093751, -0.01255), 'coxa_idx': 3, 'femur_idx': 7, 'tibia_idx': 10, 'coxa_joint_name': 'Revolute 4', 'femur_joint_name': 'Revolute 10', 'tibia_joint_name': 'Revolute 14', 'coxa_sign': +1, 'femur_sign': +1, 'tibia_sign': -1},
}

# ============================================================================
# SECTION 3 -- CORE COMPONENTS
# ============================================================================

def resolve_joint_indices(robot_id):
    name_to_idx = {p.getJointInfo(robot_id, i)[1].decode('utf-8'): i for i in range(p.getNumJoints(robot_id))}
    for leg in LEG_CONFIG.values():
        leg['coxa_idx']  = name_to_idx.get(leg['coxa_joint_name'])
        leg['femur_idx'] = name_to_idx.get(leg['femur_joint_name'])
        leg['tibia_idx'] = name_to_idx.get(leg['tibia_joint_name'])

def compute_ik(px, py, pz):
    a1 = 52.50;  a2 = 108.00;  a3 = 130.775
    T1_MIN, T1_MAX = -20, 20
    T2_MIN, T2_MAX = -90, 45
    T3_MIN, T3_MAX = -45, 90
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
    if (T1_MIN - 0.5 <= t1_deg <= T1_MAX + 0.5 and T2_MIN - 0.5 <= urdf_t2_deg <= T2_MAX + 0.5 and T3_MIN - 0.5 <= urdf_t3_deg <= T3_MAX + 0.5):
        return t1_deg, urdf_t2_deg, urdf_t3_deg, True
    return 0, 0, 0, False

def compute_leg_joints(leg_id, px, py, pz):
    cfg = LEG_CONFIG[leg_id]
    t1_deg, t2_deg, t3_deg, is_valid = compute_ik(px, py, pz)
    if not is_valid: return 0, 0, 0, False
    return t1_deg * cfg['coxa_sign'], t2_deg * cfg['femur_sign'], (-t3_deg if cfg['tibia_sign'] == -1 else t3_deg), True

def set_leg(robot_id, leg_id, px, py, pz):
    jc, jf, jt, ok = compute_leg_joints(leg_id, px, py, pz)
    if not ok: return False
    cfg = LEG_CONFIG[leg_id]
    
    # ### REAL TORQUE CONSTRAINT APPLIED HERE ###
    p.setJointMotorControl2(robot_id, cfg['coxa_idx'], p.POSITION_CONTROL, targetPosition=math.radians(jc), force=MG996R_MAX_TORQUE)
    p.setJointMotorControl2(robot_id, cfg['femur_idx'], p.POSITION_CONTROL, targetPosition=math.radians(jf), force=MG996R_MAX_TORQUE)
    p.setJointMotorControl2(robot_id, cfg['tibia_idx'], p.POSITION_CONTROL, targetPosition=math.radians(jt), force=MG996R_MAX_TORQUE)
    return True

def settle(steps=SETTLE_STEPS):
    for _ in range(steps):
        update_camera()
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

# ============================================================================
# SECTION 4 -- DIRECTIONAL PARAMETERS
# ============================================================================

def get_leg_params(leg_id, mode):
    S = STEP_LENGTH
    right_side = [1, 3] # FR, BR
    left_side =  [2, 4] # BL, FL

    if mode == 'FORWARD':
        return S/2.0, 1.0
    elif mode == 'RIGHT':
        if leg_id in left_side:
            return S/2.0, 1.0
        else:
            return -S/2.0, -1.0
    elif mode == 'LEFT':
        if leg_id in right_side:
            return S/2.0, 1.0
        else:
            return -S/2.0, -1.0
    return S/2.0, 1.0

# ============================================================================
# SECTION 5 -- SWING TRAJECTORY GENERATOR
# ============================================================================

def generate_swing_trajectory(start_py, target_py, num_points=INTERP_STEPS):
    traj = []
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        pz = STAND_PZ + LIFT_HEIGHT * math.sin(math.pi / 2.0 * t)
        traj.append(("LIFT", start_py, pz, t))
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        py = start_py + (target_py - start_py) * t
        traj.append(("SWEEP", py, APEX_PZ, t))
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        pz = APEX_PZ - LIFT_HEIGHT * math.sin(math.pi / 2.0 * t)
        traj.append(("LOWER", target_py, pz, t))
    return traj

# ============================================================================
# SECTION 6 -- CONTINUOUS GAIT STATE MACHINE
# ============================================================================

class ContinuousGaitStateMachine:
    def __init__(self, gait_order, foot_targets):
        self.gait_order = gait_order
        self.foot_targets = foot_targets
        self.current_leg_idx = 0
        self.trajectory = []
        self.traj_point = 0
        self.legs_completed = 0

    @property
    def active_leg(self):
        return self.gait_order[self.current_leg_idx]

    @property
    def support_legs(self):
        return [lid for lid in range(1, 5) if lid != self.active_leg]

    def advance(self, current_global_mode):
        if self.traj_point >= len(self.trajectory):
            if len(self.trajectory) > 0:
                self.legs_completed += 1
                self.current_leg_idx = (self.current_leg_idx + 1) % len(self.gait_order)

            self.traj_point = 0
            self.foot_targets[self.active_leg]["mode"] = current_global_mode
            
            start_py = self.foot_targets[self.active_leg]["py"]
            target_py, _ = get_leg_params(self.active_leg, current_global_mode)
            self.trajectory = generate_swing_trajectory(start_py, target_py)

        phase, py_val, pz_val, t = self.trajectory[self.traj_point]
        leg_id = self.active_leg
        self.traj_point += 1
        return leg_id, phase, py_val, pz_val, t

# ============================================================================
# SECTION 7 -- VISUALIZATION
# ============================================================================

_text_ids = {}

def update_display(robot_id, leg_id, phase, py, pz, t, current_mode, support_legs):
    global _text_ids
    cfg = LEG_CONFIG[leg_id]
    px = STAND_PX 
    base_pos, _ = p.getBasePositionAndOrientation(robot_id)
    tx, ty, tz = base_pos[0], base_pos[1], base_pos[2] + 0.18

    lines = [
        ("gait_title", f"MG996R EMERGENT DIRECTIONAL CRAWL", [tx, ty, tz + 0.12], [1.0, 1.0, 0.3], 1.5),
        ("mode", f"GLOBAL MODE: {current_mode}", [tx, ty, tz + 0.10], [1.0, 0.4, 0.4], 1.5),
        ("active_leg", f"Active: {cfg['label']}", [tx, ty, tz + 0.08], [0.3, 1.0, 0.3], 1.2),
        ("phase", f"Phase: {phase}  t={t:.2f}", [tx, ty, tz + 0.06], [0.3, 0.8, 1.0], 1.1),
        ("coxa", f"PY Offset: {py:+.1f} mm", [tx, ty, tz + 0.04], [1.0, 0.6, 0.2], 1.1),
    ]

    for key, text, pos, color, size in lines:
        if key in _text_ids and _text_ids[key] >= 0:
            _text_ids[key] = p.addUserDebugText(text, pos, textColorRGB=color, textSize=size, replaceItemUniqueId=_text_ids[key])
        else:
            _text_ids[key] = p.addUserDebugText(text, pos, textColorRGB=color, textSize=size)

def update_camera():
    try:
        p.resetDebugVisualizerCamera(
            cameraDistance=p.readUserDebugParameter(cam_dist_slider), cameraYaw=p.readUserDebugParameter(cam_yaw_slider),
            cameraPitch=p.readUserDebugParameter(cam_pitch_slider), cameraTargetPosition=[p.readUserDebugParameter(cam_target_x_slider), p.readUserDebugParameter(cam_target_y_slider), p.readUserDebugParameter(cam_target_z_slider)]
        )
    except Exception: pass

# ============================================================================
# SECTION 11 -- MAIN GAIT SEQUENCE LOOP
# ============================================================================

def run_directional_demo(robot_id):
    total_per_leg = INTERP_STEPS * 3  
    
    SEQUENCE = [
        ('FORWARD', 0),
        ('RIGHT', 5),
        ('FORWARD', 0),
        ('LEFT', 5),
        ('FORWARD', 0)
    ]
    
    commands = []
    for mode, cycles in SEQUENCE:
        for _ in range(cycles * 4):
            commands.append(mode)
            
    total_swings_target = len(commands)

    stance_delta_per_point = (STEP_LENGTH / 3.0) / total_per_leg

    # ### BALANCED SYMMETRICAL SPREAD FOR TRUE TORQUE LIMITS ###
    initial_py = {
        1: -STEP_LENGTH / 2.0,            
        2: -STEP_LENGTH / 4.0,          
        4: +STEP_LENGTH / 4.0,         
        3: +STEP_LENGTH / 2.0             
    }

    foot_targets = {lid: {"px": STAND_PX, "py": initial_py[lid], "pz": STAND_PZ, "mode": "FORWARD"} for lid in range(1, 5)}
    gsm = ContinuousGaitStateMachine(GAIT_ORDER, foot_targets)
    
    total_ik_failures = 0
    current_active_leg = None

    print(f"\n  ===== STARTING EMERGENT DIRECTIONAL DEMO =====")

    # Move legs into optimized pre-pose positions
    for lid in range(1, 5):
        set_leg(robot_id, lid, foot_targets[lid]["px"], foot_targets[lid]["py"], foot_targets[lid]["pz"])
    settle(240)

    while gsm.legs_completed < total_swings_target:
        current_global_mode = commands[gsm.legs_completed]

        leg_id, phase, target_py, pz, t = gsm.advance(current_global_mode)

        if leg_id != current_active_leg:
            current_active_leg = leg_id
            print(f"    --- Swing: Leg {leg_id} | Global Mode: {current_global_mode} | Swings: {gsm.legs_completed}/{total_swings_target} ---")

        foot_targets[leg_id]["py"] = target_py
        foot_targets[leg_id]["pz"] = pz

        for support_leg in gsm.support_legs:
            leg_mode = foot_targets[support_leg]["mode"]
            _, stance_dir = get_leg_params(support_leg, leg_mode)
            foot_targets[support_leg]["py"] -= stance_delta_per_point * stance_dir

        for lid in range(1, 5):
            px = foot_targets[lid]["px"]
            py = foot_targets[lid]["py"]
            pz_ik = foot_targets[lid]["pz"]
            jc, jf, jt, ik_ok = compute_leg_joints(lid, px, py, pz_ik)

            if not ik_ok and lid == leg_id:
                total_ik_failures += 1
            set_leg(robot_id, lid, px, py, pz_ik)

        update_display(robot_id, leg_id, phase, target_py, pz, t, current_global_mode, gsm.support_legs)

        for _ in range(SIM_SUBSTEPS):
            update_camera()
            p.stepSimulation()
            time.sleep(STEP_DELAY / SIM_SUBSTEPS)

    return total_ik_failures

# ============================================================================
# SECTION 12 -- MAIN
# ============================================================================

def main():
    global cam_dist_slider, cam_yaw_slider, cam_pitch_slider
    global cam_target_x_slider, cam_target_y_slider, cam_target_z_slider

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.0)
    
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

    cam_dist_slider = p.addUserDebugParameter("Camera Distance", 0.2, 3.0, 0.65)
    cam_yaw_slider = p.addUserDebugParameter("Camera Yaw", -180, 180, 55)
    cam_pitch_slider = p.addUserDebugParameter("Camera Pitch", -89, 89, -30)
    cam_target_x_slider = p.addUserDebugParameter("Target X", -2.0, 2.0, 0.05)
    cam_target_y_slider = p.addUserDebugParameter("Target Y", -2.0, 2.0, 0.05)
    cam_target_z_slider = p.addUserDebugParameter("Target Z", -1.0, 2.0, 0.25)

    robot_id = p.loadURDF(URDF_PATH, [0, 0, BASE_SPAWN_HEIGHT], p.getQuaternionFromEuler([0, 0, math.pi]), useFixedBase=USE_FIXED_BASE_6B)
    resolve_joint_indices(robot_id)

    # Highly critical traction adjustment to handle turning forces within the 1.08 Nm physical constraint limit
    for lid, cfg in LEG_CONFIG.items():
        p.changeDynamics(robot_id, cfg['tibia_idx'], lateralFriction=0.9, spinningFriction=0.1, rollingFriction=0.01)

    failures = run_directional_demo(robot_id)
    print(f"\n  DEMO COMPLETE. Total IK Failures: {failures}")

    try:
        while True:
            update_camera()
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except Exception: pass
    p.disconnect()

if __name__ == "__main__":
    main()