#!/usr/bin/env python3
"""
free_base_crawl_stage6b.py
===========================
Stage 6B -- Free-Base Continuous Crawl Locomotion (Surgically Patched).
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

URDF_PATH         = "urdf/robot.urdf"
BASE_SPAWN_HEIGHT = 0.38
SETTLE_STEPS      = 360

STAND_PX = 150.0
STAND_PY =   0.0
STAND_PZ = -120.0

RADIAL_REACH     = 150.0   
THETA_MIN_DEG    = -12.0    
THETA_MAX_DEG    = +12.0    
THETA_STAND_DEG  =   0.0    
LIFT_HEIGHT      =  30.0    
APEX_PZ          = STAND_PZ + LIFT_HEIGHT  

INTERP_STEPS     = 40       
STEP_DELAY       = 0.006    
SIM_SUBSTEPS     = 3        

GAIT_ORDER       = [1, 2, 4, 3]

ANGLE_TOLERANCE_DEG = 2.0
COXA_SAFE_MAX   = +18.0    

STAND_HOLD_SEC  = 1.5       
PHASE_PAUSE_SEC = 0.3       

USE_FIXED_BASE_6B = False    
NUM_CYCLES        = 20       

STEP_LENGTH = 2.0 * RADIAL_REACH * math.sin(math.radians(THETA_MAX_DEG))  

# ### CHANGED ###: Defined AEP/PEP cleanly based on step length.
AEP_PY = STEP_LENGTH / 2.0
PEP_PY = -STEP_LENGTH / 2.0


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

def set_leg(robot_id, leg_id, px, py, pz, force=80):
    jc, jf, jt, ok = compute_leg_joints(leg_id, px, py, pz)
    if not ok: return False
    cfg = LEG_CONFIG[leg_id]
    p.setJointMotorControl2(robot_id, cfg['coxa_idx'], p.POSITION_CONTROL, targetPosition=math.radians(jc), force=force)
    p.setJointMotorControl2(robot_id, cfg['femur_idx'], p.POSITION_CONTROL, targetPosition=math.radians(jf), force=force)
    p.setJointMotorControl2(robot_id, cfg['tibia_idx'], p.POSITION_CONTROL, targetPosition=math.radians(jt), force=force)
    return True

def get_joint_deg(robot_id, joint_idx):
    return math.degrees(p.getJointState(robot_id, joint_idx)[0])

def settle(steps=SETTLE_STEPS):
    for _ in range(steps):
        update_camera()
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


# ============================================================================
# SECTION 5 -- SWING TRAJECTORY GENERATOR (PATCHED)
# ============================================================================

# ### CHANGED ###: Removed RETURN phase entirely. Replaced theta with Cartesian PY mapping.
def generate_swing_trajectory(start_py, target_py, num_points=INTERP_STEPS):
    """
    Dynamically generate a 3-phase Cartesian swing trajectory from current PEP to target AEP.
    Returns list of (phase_name, py, pz, t_value).
    """
    traj = []

    # Phase 1 -- LIFT
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        pz = STAND_PZ + LIFT_HEIGHT * math.sin(math.pi / 2.0 * t)
        traj.append(("LIFT", start_py, pz, t))

    # Phase 2 -- SWEEP (Forward in Air)
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        py = start_py + (target_py - start_py) * t
        traj.append(("SWEEP", py, APEX_PZ, t))

    # Phase 3 -- LOWER
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        pz = APEX_PZ - LIFT_HEIGHT * math.sin(math.pi / 2.0 * t)
        traj.append(("LOWER", target_py, pz, t))

    # RETURN PHASE DELETED to preserve stance continuity.
    return traj


# ============================================================================
# SECTION 6 -- GAIT STATE MACHINE (PATCHED)
# ============================================================================

# ### CHANGED ###: GSM now holds foot targets and generates trajectories dynamically.
class GaitStateMachine:
    def __init__(self, gait_order, foot_targets, aep_py):
        self.gait_order = gait_order
        self.foot_targets = foot_targets
        self.aep_py = aep_py
        self.current_leg_idx = 0
        self.state = "IDLE"
        self.trajectory = []
        self.traj_point = 0
        self.legs_completed = 0
        self.total_points_executed = 0

    @property
    def active_leg(self):
        return self.gait_order[self.current_leg_idx] if self.current_leg_idx < len(self.gait_order) else None

    @property
    def support_legs(self):
        return [lid for lid in range(1, 5) if lid != self.active_leg]

    @property
    def cycle_percent(self):
        total_legs = len(self.gait_order)
        total_points_per_leg = len(self.trajectory) if self.trajectory else 1
        return ((self.current_leg_idx * total_points_per_leg + self.traj_point) / max(total_legs * total_points_per_leg, 1)) * 100.0

    def start_cycle(self):
        self.state = "SWING_LEG"
        self.current_leg_idx = 0
        self.traj_point = 0
        self.legs_completed = 0
        self._generate_current_trajectory()

    def _generate_current_trajectory(self):
        if self.active_leg is not None:
            # ### CHANGED ###: Generate trajectory from leg's CURRENT py to the AEP
            start_py = self.foot_targets[self.active_leg]["py"]
            self.trajectory = generate_swing_trajectory(start_py, self.aep_py)
            self.traj_point = 0

    def advance(self):
        if self.state in ("COMPLETE", "IDLE"): return None

        if self.traj_point >= len(self.trajectory):
            self.legs_completed += 1
            self.current_leg_idx += 1

            if self.current_leg_idx >= len(self.gait_order):
                self.state = "COMPLETE"
                return None

            # Generate new dynamic trajectory for the next leg
            self._generate_current_trajectory()

        phase, py_val, pz_val, t = self.trajectory[self.traj_point]
        leg_id = self.active_leg
        self.traj_point += 1
        self.total_points_executed += 1
        return leg_id, phase, py_val, pz_val, t


# ============================================================================
# SECTION 7 -- VISUALIZATION (PATCHED)
# ============================================================================

_text_ids = {}

# ### CHANGED ###: Converted theta arguments back to py to reflect Cartesian tracking.
def update_display(robot_id, leg_id, phase, py, pz, t, cycle_pct, support_legs):
    global _text_ids
    cfg = LEG_CONFIG[leg_id]
    px = STAND_PX 
    base_pos, _ = p.getBasePositionAndOrientation(robot_id)
    tx, ty, tz = base_pos[0], base_pos[1], base_pos[2] + 0.18

    lines = [
        ("gait_title", f"FREE-BASE CRAWL -- Cycle {cycle_pct:.0f}%", [tx, ty, tz + 0.10], [1.0, 1.0, 0.3], 1.5),
        ("active_leg", f"Active: {cfg['label']}", [tx, ty, tz + 0.08], [0.3, 1.0, 0.3], 1.2),
        ("phase", f"Phase: {phase}  t={t:.2f}", [tx, ty, tz + 0.06], [0.3, 0.8, 1.0], 1.1),
        ("coxa", f"PY Offset: {py:+.1f} mm", [tx, ty, tz + 0.04], [1.0, 0.6, 0.2], 1.1),
        ("target", f"IK: ({px:.1f}, {py:.1f}, {pz:.1f}) mm", [tx, ty, tz + 0.02], [0.7, 0.7, 1.0], 1.0),
        ("support", f"Support: {', '.join(f'L{s}' for s in support_legs)}", [tx, ty, tz + 0.00], [0.8, 0.8, 0.8], 1.0),
    ]

    for key, text, pos, color, size in lines:
        if key in _text_ids and _text_ids[key] >= 0:
            _text_ids[key] = p.addUserDebugText(text, pos, textColorRGB=color, textSize=size, replaceItemUniqueId=_text_ids[key])
        else:
            _text_ids[key] = p.addUserDebugText(text, pos, textColorRGB=color, textSize=size)


def draw_swing_path(robot_id, leg_id, trajectory):
    phase_colors = {"LIFT": [0.0, 1.0, 1.0], "SWEEP": [0.2, 1.0, 0.2], "LOWER": [1.0, 1.0, 0.2]}
    cfg = LEG_CONFIG[leg_id]
    prev_world = None; prev_phase = None; lines = []

    for phase, py, pz_ik, t in trajectory:
        px = STAND_PX 
        base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
        bx, by, bz = base_pos
        rot_mat = p.getMatrixFromQuaternion(base_orn)
        R = [rot_mat[0:3], rot_mat[3:6], rot_mat[6:9]]
        hx, hy, hz = cfg['hip_xyz_urdf']
        hip_wx = R[0][0]*hx + R[0][1]*hy + R[0][2]*hz + bx
        hip_wy = R[1][0]*hx + R[1][1]*hy + R[1][2]*hz + by
        hip_wz = R[2][0]*hx + R[2][1]*hy + R[2][2]*hz + bz

        world_pos = [hip_wx + px/1000.0 * 0.65 + py/1000.0 * 0.10,
                     hip_wy + py/1000.0 * 0.65 - px/1000.0 * 0.02,
                     hip_wz + pz_ik/1000.0]

        if prev_world is not None and prev_phase == phase:
            lines.append(p.addUserDebugLine(prev_world, world_pos, lineColorRGB=phase_colors.get(phase, [1,1,1]), lineWidth=2.0, lifeTime=0))
        prev_world = world_pos; prev_phase = phase
    return lines

def clear_debug_lines(line_ids):
    for lid in line_ids: p.removeUserDebugItem(lid)


# ============================================================================
# SECTION 8 -- VALIDATION (PATCHED)
# ============================================================================

def pre_validate_trajectory(trajectory):
    errors = []
    for i, (phase, py, pz, t) in enumerate(trajectory):
        theta = math.degrees(math.atan2(py, STAND_PX))
        if abs(theta) > COXA_SAFE_MAX: errors.append(f"  Coxa out of range at pt {i}: theta={theta:.1f}")
        _, _, _, ik_ok = compute_ik(STAND_PX, py, pz)
        if not ik_ok: errors.append(f"  IK unreachable at pt {i}: phase={phase} py={py:.1f} pz={pz:.1f}")
    return len(errors) == 0, errors

def update_camera():
    try:
        p.resetDebugVisualizerCamera(
            cameraDistance=p.readUserDebugParameter(cam_dist_slider),
            cameraYaw=p.readUserDebugParameter(cam_yaw_slider),
            cameraPitch=p.readUserDebugParameter(cam_pitch_slider),
            cameraTargetPosition=[p.readUserDebugParameter(cam_target_x_slider), p.readUserDebugParameter(cam_target_y_slider), p.readUserDebugParameter(cam_target_z_slider)]
        )
    except Exception: pass


# ============================================================================
# SECTION 10 -- STAGE 6B: FREE-BASE BODY ADVANCEMENT
# ============================================================================

def advance_body_forward(robot_id, delta_mm):
    """Physics-based version: the body is moved only by contact forces and joint actuation."""
    return


# ============================================================================
# SECTION 11 -- MAIN GAIT LOOP (PATCHED)
# ============================================================================

def run_stage6b(robot_id):
    total_per_leg = INTERP_STEPS * 3  # LIFT, SWEEP, LOWER (No Return)
    
    # ### CHANGED ###: Body advancement calculation.
    # To sweep from AEP to PEP exactly, a leg planted for 3 support phases must move backward
    # by exactly the STEP_LENGTH total. Thus, the body advances by STEP_LENGTH/3 per leg phase.
    body_advance_per_leg = STEP_LENGTH / 3.0
    delta_per_point = body_advance_per_leg / total_per_leg

    total_ik_failures = 0
    total_points = 0
    total_legs_completed = 0
    debug_lines = []

    # ### CHANGED ###: The Persistent State Dictionary!
    # Support legs will decrement their Py offset safely from this persistent state.
    foot_targets = {lid: {"px": STAND_PX, "py": STAND_PY, "pz": STAND_PZ} for lid in range(1, 5)}

    for cycle in range(NUM_CYCLES):
        gsm = GaitStateMachine(GAIT_ORDER, foot_targets, AEP_PY)
        gsm.start_cycle()
        current_active_leg = None

        print(f"\n  ===== CYCLE {cycle + 1}/{NUM_CYCLES} =====")

        while True:
            result = gsm.advance()
            if result is None: break

            leg_id, phase, target_py, pz, t = result

            if leg_id != current_active_leg:
                if debug_lines: clear_debug_lines(debug_lines)
                current_active_leg = leg_id
                print(f"    --- Swinging: Leg {leg_id} ---")
                debug_lines = draw_swing_path(robot_id, leg_id, gsm.trajectory)

            if gsm.traj_point == 1 or (gsm.traj_point > 1 and gsm.trajectory[gsm.traj_point - 1][0] != phase):
                print(f"      Phase: {phase}")

            # ### CHANGED ###: Update Swing Leg Target
            foot_targets[leg_id]["py"] = target_py
            foot_targets[leg_id]["pz"] = pz

            # ### CHANGED ###: Incrementally update Stance Legs (Propulsion without snapping)
            for support_leg in gsm.support_legs:
                foot_targets[support_leg]["py"] -= delta_per_point

            # Update IK/Physics
            for lid in range(1, 5):
                px = foot_targets[lid]["px"]
                py = foot_targets[lid]["py"]
                pz_ik = foot_targets[lid]["pz"]
                jc, jf, jt, ik_ok = compute_leg_joints(lid, px, py, pz_ik)

                if not ik_ok and lid == leg_id:
                    total_ik_failures += 1
                set_leg(robot_id, lid, px, py, pz_ik)

            update_display(robot_id, leg_id, phase, target_py, pz, t, gsm.cycle_percent, gsm.support_legs)

            for _ in range(SIM_SUBSTEPS):
                update_camera()
                p.stepSimulation()
                time.sleep(STEP_DELAY / SIM_SUBSTEPS)

            total_points += 1

        total_legs_completed += gsm.legs_completed

        # ### CHANGED ###: All hard resets previously found here have been DELETED. 
        # The transition into the next cycle is now completely smooth.
        for _ in range(int(PHASE_PAUSE_SEC * 240)):
            update_camera()
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    if debug_lines: clear_debug_lines(debug_lines)
    return total_ik_failures, total_points, total_legs_completed


# ============================================================================
# SECTION 12 -- MAIN
# ============================================================================

def main():
    global cam_dist_slider, cam_yaw_slider, cam_pitch_slider
    global cam_target_x_slider, cam_target_y_slider, cam_target_z_slider

    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

    cam_dist_slider = p.addUserDebugParameter("Camera Distance", 0.2, 3.0, 0.65)
    cam_yaw_slider = p.addUserDebugParameter("Camera Yaw", -180, 180, 55)
    cam_pitch_slider = p.addUserDebugParameter("Camera Pitch", -89, 89, -30)
    cam_target_x_slider = p.addUserDebugParameter("Target X", -2.0, 2.0, 0.05)
    cam_target_y_slider = p.addUserDebugParameter("Target Y", -2.0, 2.0, 0.05)
    cam_target_z_slider = p.addUserDebugParameter("Target Z", -1.0, 2.0, 0.25)

    robot_id = p.loadURDF(URDF_PATH, [0, 0, BASE_SPAWN_HEIGHT], p.getQuaternionFromEuler([0, 0, math.pi]), useFixedBase=USE_FIXED_BASE_6B)
    resolve_joint_indices(robot_id)

    # Pre-validate with a mock trajectory spanning PEP to AEP
    mock_traj = generate_swing_trajectory(PEP_PY, AEP_PY)
    traj_ok, traj_errors = pre_validate_trajectory(mock_traj)
    if not traj_ok:
        for err in traj_errors: print(err)
        p.disconnect()
        return

    # Establish standing pose
    for lid in range(1, 5): set_leg(robot_id, lid, STAND_PX, STAND_PY, STAND_PZ)
    settle()
    for _ in range(int(STAND_HOLD_SEC * 240)):
        update_camera()
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    # Run Patched Gait
    ik_failures, total_points_executed, legs_completed = run_stage6b(robot_id)

    try:
        while True:
            update_camera()
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    except Exception: pass
    p.disconnect()

if __name__ == "__main__":
    main()