#!/usr/bin/env python3
"""
free_base_crawl_stage8.py
===============================
Stage 8 -- True Emergent Physics Locomotion.
(Corrected Affine Rotations, True CoM Projection, Fluid Stance)
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
# SECTION 1 -- CONFIGURATION
# ============================================================================

URDF_PATH         = "urdf/finalv6_pla.urdf"
BASE_SPAWN_HEIGHT = 0.38
SETTLE_STEPS      = 360

MG996R_MAX_TORQUE = 1.08   # Nm

STAND_PX =  158.0   # mm
STAND_PY =    0.0
STAND_PZ = -135.0   # mm

LIFT_HEIGHT =  35.0                    
APEX_PZ     = STAND_PZ + LIFT_HEIGHT  

INTERP_STEPS  = 40
STEP_DELAY    = 0.006
SIM_SUBSTEPS  = 3

GAIT_ORDER    = [1, 2, 4, 3] 

USE_FIXED_BASE    = False

STEP_LENGTH = 80.0  # mm 

# ============================================================================
# SECTION 2 -- LEG CONFIGURATION TABLE
# ============================================================================

LEG_CONFIG = {
    1: {'label': 'Leg 1 -- FL', 'hip_xyz_urdf': (-0.075, -0.093751, -0.01255), 'coxa_idx': 0, 'femur_idx': 6, 'tibia_idx': 8, 'coxa_joint_name': 'Revolute 1', 'femur_joint_name': 'Revolute 8', 'tibia_joint_name': 'Revolute 11', 'coxa_sign': -1, 'femur_sign': +1, 'tibia_sign': +1},
    2: {'label': 'Leg 2 -- BR', 'hip_xyz_urdf': (+0.075, +0.093751, -0.01255), 'coxa_idx': 1, 'femur_idx': 5, 'tibia_idx': 11, 'coxa_joint_name': 'Revolute 2', 'femur_joint_name': 'Revolute 7', 'tibia_joint_name': 'Revolute 15', 'coxa_sign': +1, 'femur_sign': +1, 'tibia_sign': -1},
    3: {'label': 'Leg 3 -- FR', 'hip_xyz_urdf': (-0.075, +0.093751, -0.01255), 'coxa_idx': 2, 'femur_idx': 4, 'tibia_idx': 9, 'coxa_joint_name': 'Revolute 3', 'femur_joint_name': 'Revolute 6', 'tibia_joint_name': 'Revolute 13', 'coxa_sign': -1, 'femur_sign': +1, 'tibia_sign': +1},
    4: {'label': 'Leg 4 -- BL', 'hip_xyz_urdf': (+0.075, -0.093751, -0.01255), 'coxa_idx': 3, 'femur_idx': 7, 'tibia_idx': 10, 'coxa_joint_name': 'Revolute 4', 'femur_joint_name': 'Revolute 10', 'tibia_joint_name': 'Revolute 14', 'coxa_sign': +1, 'femur_sign': +1, 'tibia_sign': +1},
}

# ============================================================================
# SECTION 3 -- CORE COMPONENTS
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
    a1 = 52.50;  a2 = 108.00;  a3 = 130.775
    T1_MIN, T1_MAX = -20,  20
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
              positionGain=1.0)
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
# SECTION 4 -- SWING TRAJECTORY GENERATOR & COM SHIFT
# ============================================================================

def generate_swing_trajectory(start_py, target_py, num_points=INTERP_STEPS):
    traj = []
    for i in range(num_points):
        t  = i / max(num_points - 1, 1)
        pz = STAND_PZ + LIFT_HEIGHT * math.sin(math.pi / 2.0 * t)
        traj.append(("LIFT", start_py, pz, t))
    for i in range(num_points):
        t  = i / max(num_points - 1, 1)
        py = start_py + (target_py - start_py) * t
        traj.append(("SWEEP", py, APEX_PZ, t))
    for i in range(num_points):
        t  = i / max(num_points - 1, 1)
        pz = APEX_PZ - LIFT_HEIGHT * math.sin(math.pi / 2.0 * t)
        traj.append(("LOWER", target_py, pz, t))
    return traj

def get_true_support_polygon(swing_leg, foot_targets):
    """FIXED: Accurately maps IK coordinates back to true chassis frame."""
    poly = {}
    for lid in range(1, 5):
        if lid != swing_leg:
            hx, hy, _ = LEG_CONFIG[lid]['hip_xyz_urdf']
            outward_sign = 1.0 if hy > 0 else -1.0
            
            # Local Body X (-X is Forward in URDF, so +py makes foot more negative)
            base_x = (hx * 1000.0) - foot_targets[lid]['py']
            # Local Body Y (+Y is Right in URDF)
            base_y = (hy * 1000.0) + (foot_targets[lid]['px'] * outward_sign)
            
            poly[lid] = (base_x, base_y)
    return poly

def project_true_com(robot_id, nominal_z_mm=STAND_PZ):
    """FIXED: Uses indices [6,7,8] to get the World Z vector in the Local Frame."""
    _, orn = p.getBasePositionAndOrientation(robot_id)
    rot = p.getMatrixFromQuaternion(orn)
    local_grav_x, local_grav_y, local_grav_z = rot[6], rot[7], rot[8]
    if local_grav_z == 0: return 0.0, 0.0
    shift_x = nominal_z_mm * (local_grav_x / local_grav_z)
    shift_y = nominal_z_mm * (local_grav_y / local_grav_z)
    return shift_x, shift_y

def compute_temporary_shift(robot_id, swing_leg, foot_targets, max_shift=12.0):
    """Increased authority (max_shift) to guarantee centroid capture for 0.44kg mass."""
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
# SECTION 5 -- CONTINUOUS GAIT STATE MACHINE
# ============================================================================

class ContinuousGaitStateMachine:
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

    def advance(self, current_mode, robot_id):
        if self.state == "IDLE":
            self._start_stabilization(robot_id)

        if self.state == "STABILIZE":
            if self.shift_point >= self.shift_steps:
                self.state = "SWING"
                
                # Enforce lateral stance reset before swing trajectory generation
                self.foot_targets[self.active_leg]["px"] = STAND_PX
                
                start_py        = self.foot_targets[self.active_leg]["py"]
                target_py       = STEP_LENGTH / 2.0  # Swing strictly forward (rotation is stance-driven)
                self.trajectory = generate_swing_trajectory(start_py, target_py)
                self.traj_point = 0
                return self.advance(current_mode, robot_id)

            t         = (self.shift_point + 1) / float(self.shift_steps)
            smooth_t  = 3 * t**2 - 2 * t**3 
            current_dx = self.target_dx * smooth_t
            current_dy = self.target_dy * smooth_t
            self.shift_point += 1
            return self.active_leg, "STABILIZE", current_dx, current_dy, t

        if self.traj_point >= len(self.trajectory):
            if len(self.trajectory) > 0:
                self.legs_completed  += 1
                self.current_leg_idx  = (self.current_leg_idx + 1) % len(self.gait_order)
            self._start_stabilization(robot_id)
            return self.advance(current_mode, robot_id)

        phase, py_val, pz_val, t = self.trajectory[self.traj_point]
        self.traj_point += 1
        return self.active_leg, phase, py_val, pz_val, t

# ============================================================================
# SECTION 6 -- VISUALIZATION
# ============================================================================

_text_ids = {}

def update_display(robot_id, leg_id, phase, py, pz, t, current_mode):
    global _text_ids
    cfg      = LEG_CONFIG[leg_id]
    base_pos, _ = p.getBasePositionAndOrientation(robot_id)
    tx, ty, tz  = base_pos[0], base_pos[1], base_pos[2] + 0.18

    lines = [
        ("title", "STAGE 8: AFFINE ROTATION & COM FIX",  [tx, ty, tz+0.12], [1.0,1.0,0.3], 1.5),
        ("mode",  f"GLOBAL MODE: {current_mode}",        [tx, ty, tz+0.10], [1.0,0.4,0.4], 1.5),
        ("leg",   f"Active: {cfg['label']}",             [tx, ty, tz+0.08], [0.3,1.0,0.3], 1.2),
        ("phase", f"Phase: {phase}  t={t:.2f}",          [tx, ty, tz+0.06], [0.3,0.8,1.0], 1.1),
        ("py",    f"PY Offset: {py:+.1f} mm",            [tx, ty, tz+0.04], [1.0,0.6,0.2], 1.1),
    ]
    for key, text, pos, color, size in lines:
        if key in _text_ids and _text_ids[key] >= 0:
            _text_ids[key] = p.addUserDebugText(
                text, pos, textColorRGB=color, textSize=size, replaceItemUniqueId=_text_ids[key])
        else:
            _text_ids[key] = p.addUserDebugText(text, pos, textColorRGB=color, textSize=size)

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
# SECTION 7 -- MAIN GAIT SEQUENCE LOOP
# ============================================================================

def run_directional_demo(robot_id):
    SEQUENCE = [
        ('FORWARD', 3),
        ('RIGHT',   0),
        ('FORWARD', 3),
        ('LEFT',    0),
        ('FORWARD', 3),
    ]

    commands = []
    for mode, cycles in SEQUENCE:
        for _ in range(cycles * 4):
            commands.append(mode)
    total_swings_target = len(commands)

    # Fluid Stance Sweep Mathematics
    phase_len = 40 + (INTERP_STEPS * 3)
    total_planted_ticks = (3 * phase_len) + 40
    stance_delta = STEP_LENGTH / float(total_planted_ticks)

    initial_py = {
        1: -STEP_LENGTH / 2.0, 
        2: -STEP_LENGTH / 4.0, 
        4:  STEP_LENGTH / 4.0, 
        3:  STEP_LENGTH / 2.0, 
    }

    foot_targets = {
        lid: {"px": STAND_PX, "py": initial_py[lid], "pz": STAND_PZ, "temp_dx": 0.0, "temp_dy": 0.0}
        for lid in range(1, 5)
    }

    for lid in range(1, 5):
        set_leg(robot_id, lid, foot_targets[lid]["px"], foot_targets[lid]["py"], foot_targets[lid]["pz"])
    settle(240)

    gsm                = ContinuousGaitStateMachine(GAIT_ORDER, foot_targets)
    total_ik_failures  = 0
    current_active_leg = None

    print(f"\n  ===== STARTING PHYSICS-EMERGENT DIRECTIONAL DEMO =====")

    while gsm.legs_completed < total_swings_target:
        current_global_mode = commands[gsm.legs_completed]
        leg_id, phase, val1, val2, t = gsm.advance(current_global_mode, robot_id)

        if leg_id != current_active_leg and phase != "STABILIZE":
            current_active_leg = leg_id
            print(f"    --- Swing: Leg {leg_id} | Mode: {current_global_mode} | "
                  f"Swings: {gsm.legs_completed}/{total_swings_target} ---")

        if phase == "STABILIZE":
            sweep_legs = [1, 2, 3, 4]
            for lid in gsm.support_legs:
                foot_targets[lid]["temp_dx"] = val1
                foot_targets[lid]["temp_dy"] = val2
        elif phase in ("LIFT", "SWEEP", "LOWER"):
            sweep_legs = gsm.support_legs
            foot_targets[leg_id]["py"] = val1
            foot_targets[leg_id]["pz"] = val2
            for lid in range(1, 5):
                foot_targets[lid]["temp_dx"] *= 0.90
                foot_targets[lid]["temp_dy"] *= 0.90

        # --- TRUE EMERGENT AFFINE ROTATION FOR STANCE ---
        for lid in sweep_legs:
            if current_global_mode == 'FORWARD':
                foot_targets[lid]["py"] -= stance_delta
            elif current_global_mode in ['LEFT', 'RIGHT']:
                # Radians per tick (approximated turning arc based on stance width)
                d_theta = stance_delta / STAND_PX
                
                # CCW (+ yaw) chassis turn requires CW (- yaw) foot drag relative to chassis
                if current_global_mode == 'LEFT': 
                    d_theta = -d_theta 
                
                hx, hy, _ = LEG_CONFIG[lid]['hip_xyz_urdf']
                outward_sign = 1.0 if hy > 0 else -1.0
                
                # Transform to 2D true-body cartesian coordinates
                curr_x = (hx * 1000.0) - foot_targets[lid]["py"]
                curr_y = (hy * 1000.0) + (foot_targets[lid]["px"] * outward_sign)
                
                # Standard 2D Rotation Matrix
                cos_t = math.cos(d_theta)
                sin_t = math.sin(d_theta)
                new_x = curr_x * cos_t - curr_y * sin_t
                new_y = curr_x * sin_t + curr_y * cos_t
                
                # Inverse Map back to IK space
                foot_targets[lid]["py"] = (hx * 1000.0) - new_x
                foot_targets[lid]["px"] = (new_y - (hy * 1000.0)) * outward_sign

        # Apply shifts and compute IK
        for lid in range(1, 5):
            base_px = foot_targets[lid]["px"]
            base_py = foot_targets[lid]["py"]
            pz_ik   = foot_targets[lid]["pz"]

            outward_sign = 1.0 if LEG_CONFIG[lid]['hip_xyz_urdf'][1] > 0 else -1.0
            
            transient_px = base_px - (foot_targets[lid]["temp_dy"] * outward_sign)
            transient_py = base_py - foot_targets[lid]["temp_dx"]

            _, _, _, ik_ok = compute_ik(transient_px, transient_py, pz_ik)
            if not ik_ok and lid == leg_id:
                total_ik_failures += 1
            set_leg(robot_id, lid, transient_px, transient_py, pz_ik)

        update_display(robot_id, leg_id, phase, foot_targets[leg_id]["py"], foot_targets[leg_id]["pz"], t, current_global_mode)

        for _ in range(SIM_SUBSTEPS):
            update_camera()
            p.stepSimulation()
            time.sleep(STEP_DELAY / SIM_SUBSTEPS)

    return total_ik_failures

# ============================================================================
# SECTION 8 -- MAIN
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

    cam_dist_slider     = p.addUserDebugParameter("Camera Distance",  0.2, 3.0,  0.65)
    cam_yaw_slider      = p.addUserDebugParameter("Camera Yaw",      -180, 180,  55)
    cam_pitch_slider    = p.addUserDebugParameter("Camera Pitch",      -89, 89,  -30)
    cam_target_x_slider = p.addUserDebugParameter("Target X",        -2.0, 2.0,  0.05)
    cam_target_y_slider = p.addUserDebugParameter("Target Y",        -2.0, 2.0,  0.05)
    cam_target_z_slider = p.addUserDebugParameter("Target Z",        -1.0, 2.0,  0.25)

    robot_id = p.loadURDF(
        URDF_PATH,
        [0, 0, BASE_SPAWN_HEIGHT],
        p.getQuaternionFromEuler([0, 0, math.pi]),
        useFixedBase=USE_FIXED_BASE
    )
    resolve_joint_indices(robot_id)

    for lid in range(1, 5):
        jc, jf, jt, _ = compute_leg_joints(lid, STAND_PX, 0.0, STAND_PZ)
        cfg = LEG_CONFIG[lid]
        p.resetJointState(robot_id, cfg['coxa_idx'],  math.radians(jc))
        p.resetJointState(robot_id, cfg['femur_idx'], math.radians(jf))
        p.resetJointState(robot_id, cfg['tibia_idx'], math.radians(jt))

    for lid, cfg in LEG_CONFIG.items():
        p.changeDynamics(robot_id, cfg['tibia_idx'], lateralFriction=0.9, spinningFriction=0.1, rollingFriction=0.01)

    failures = run_directional_demo(robot_id)
    print(f"\n  DEMO COMPLETE. Total IK Failures: {failures}")

    try:
        while True:
            update_camera()
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except Exception:
        pass
    p.disconnect()

if __name__ == "__main__":
    main()