#!/usr/bin/env python3
"""
tilt_surface_test.py
====================
Interactive tilted-surface test for the unified quadruped simulator.

Runs pd_controller_unified.py UNCHANGED, but swaps its flat ground plane for a
large tiltable platform driven by two GUI sliders (Surface Roll / Surface
Pitch). Move the sliders while the sim is running and watch whether the
posture controller keeps the BODY level while the GROUND tilts underneath it.

Nothing in pd_controller_unified.py is edited: compute_ik(), LEG_CONFIG, the
gait state machine, trajectories, turning logic and the PD controller are all
used exactly as they are. This file only:
  * replaces the "plane.urdf" load with a tiltable platform (at load time),
  * hooks the existing per-substep update_camera() call to drive the tilt,
  * adds its own sliders and a HUD line.

How the platform works
----------------------
The platform is a fixed-base 2-joint gimbal:
    world ─(revolute, X axis = ROLL)─ link ─(revolute, Y axis = PITCH)─ deck
The pivot is at the world origin, on the deck's top surface (robot spawn
point). Tilting is done with position-controlled joints, so the deck moves
with real velocity and friction carries the feet along (no teleporting).
Motion is rate-limited by the "Tilt Rate" slider.

Run:
    python tilt_surface_test.py
(from the same folder as pd_controller_unified.py, imu_module.py and urdf/)
"""

import math
import pybullet as p
import pybullet_data

import pd_controller_unified as Q      # the simulator under test (unchanged)

# ============================================================================
# TEST CONFIG
# ============================================================================
SURFACE_HALF_SIZE    = 2.0      # m, half-width of the square deck (4 m x 4 m)
SURFACE_THICKNESS    = 0.05     # m
SURFACE_FRICTION     = 0.9      # matches the plane friction set in Q.main()
SURFACE_MAX_TILT_DEG = 20.0     # slider range ±
TILT_RATE_DEFAULT    = 5.0      # deg/s, default rate the deck follows sliders
TILT_RATE_MAX        = 30.0     # deg/s, slider max
MOTOR_FORCE          = 1.0e5    # N·m, gimbal joints (effectively rigid)

# Contact-model fixes (anti-creep). Bullet's default contact model lets a
# resting foot creep a few mm/s under a sideways load (e.g. on a slope) even
# when friction is more than enough to hold it. Friction anchors pin each
# contact to the point where it started sticking; more solver iterations let
# the 4-foot contact problem converge. Measured on this robot, controller
# OFF, standing on 10-15 deg slopes: creep 0.6-3.4 mm/s -> 0.0-0.1 mm/s.
USE_FRICTION_ANCHORS = True
SOLVER_ITERATIONS    = 150      # PyBullet default is 50
FOOT_LOAD_THRESHOLD  = 1.0      # N, a foot counts as 'loaded' above this

# Foot material model (applied to the tibia contact, overrides the 0.9 that
# pd_controller_unified.main() sets). Bullet multiplies the two frictions,
# so effective mu = foot_mu * SURFACE_FRICTION.
#   "rubber" : rubber tip, mu ~1.2 + slight compliance (pad squashes ~0.2 mm
#              under body weight) which helps all four feet share load
#   "pla"    : bare printed PLA tip, mu ~0.35 — what the real robot has now
#   "sim"    : the simulator's original setting (mu 0.9, rigid)
FOOT_MODEL = "rubber"
FOOT_MODELS = {
    "rubber": dict(lateralFriction=1.2, spinningFriction=0.1, rollingFriction=0.01,
                   contactStiffness=2.0e4, contactDamping=120.0),
    "pla":    dict(lateralFriction=0.35, spinningFriction=0.05, rollingFriction=0.01),
    "sim":    dict(lateralFriction=0.9, spinningFriction=0.1, rollingFriction=0.01),
}

# ============================================================================
# INTERNAL STATE
# ============================================================================
_surface_id   = None
_roll_joint   = 0
_pitch_joint  = 1
_sl_roll      = None
_sl_pitch     = None
_sl_rate      = None
_sl_posture   = None
_btn_level    = None
_btn_level_last = 0.0
_tilt_enabled = False           # held level until the level reference is taken
_cmd_roll     = 0.0             # rate-limited commanded angles (deg)
_cmd_pitch    = 0.0
_substep      = 0
_hud_id       = -1


def _create_tilt_surface():
    """Build the gimbal platform. Returns its body id."""
    deck_col = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[SURFACE_HALF_SIZE, SURFACE_HALF_SIZE, SURFACE_THICKNESS / 2],
        collisionFramePosition=[0, 0, -SURFACE_THICKNESS / 2])
    deck_vis = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[SURFACE_HALF_SIZE, SURFACE_HALF_SIZE, SURFACE_THICKNESS / 2],
        visualFramePosition=[0, 0, -SURFACE_THICKNESS / 2],
        rgbaColor=[0.85, 0.85, 0.9, 1.0])
    # Intermediate roll link: tiny sphere (needs a shape for sane inertia);
    # its collisions are disabled below.
    mid_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.01)

    body = p.createMultiBody(
        baseMass=0.0,                               # fixed base
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=-1,
        basePosition=[0, 0, 0],
        linkMasses=[1.0, 20.0],
        linkCollisionShapeIndices=[mid_col, deck_col],
        linkVisualShapeIndices=[-1, deck_vis],
        linkPositions=[[0, 0, 0], [0, 0, 0]],
        linkOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
        linkInertialFramePositions=[[0, 0, 0], [0, 0, -SURFACE_THICKNESS / 2]],
        linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
        linkParentIndices=[0, 1],
        linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE],
        linkJointAxis=[[1, 0, 0], [0, 1, 0]])       # roll about X, pitch about Y

    p.setCollisionFilterGroupMask(body, _roll_joint, 0, 0)   # mid link: no contact
    p.changeDynamics(body, _pitch_joint,
                     lateralFriction=SURFACE_FRICTION,
                     spinningFriction=0.1, rollingFriction=0.01)
    for j in (_roll_joint, _pitch_joint):
        p.changeDynamics(body, j, linearDamping=0.0, angularDamping=0.0)
        p.setJointMotorControl2(body, j, p.POSITION_CONTROL,
                                targetPosition=0.0, force=MOTOR_FORCE)
    if USE_FRICTION_ANCHORS:
        p.changeDynamics(body, _pitch_joint, frictionAnchor=1)
    p.setPhysicsEngineParameter(numSolverIterations=SOLVER_ITERATIONS)
    try:
        tex = p.loadTexture("checker_blue.png")
        p.changeVisualShape(body, _pitch_joint, textureUniqueId=tex)
    except Exception:
        pass
    return body


def _create_test_sliders():
    global _sl_roll, _sl_pitch, _sl_rate, _sl_posture, _btn_level, _btn_level_last
    m = SURFACE_MAX_TILT_DEG
    _sl_roll    = p.addUserDebugParameter("Surface Roll (deg)",  -m, m, 0.0)
    _sl_pitch   = p.addUserDebugParameter("Surface Pitch (deg)", -m, m, 0.0)
    _sl_rate    = p.addUserDebugParameter("Tilt Rate (deg/s)", 0.5, TILT_RATE_MAX,
                                          TILT_RATE_DEFAULT)
    _sl_posture = p.addUserDebugParameter("Posture PD  (0=OFF 1=ON)", 0, 1,
                                          1 if Q.POSTURE_ENABLED else 0)
    _btn_level  = p.addUserDebugParameter("[ LEVEL SURFACE ]", 1, 0, 0)
    _btn_level_last = _read(_btn_level, 0.0)


def _read(pid, default):
    try:
        return float(p.readUserDebugParameter(pid))
    except Exception:
        return default


# ── Hook 1: swap the ground plane for the tiltable platform ─────────────────
_real_loadURDF = p.loadURDF

def _loadURDF_hook(fileName, *args, **kwargs):
    global _surface_id
    if isinstance(fileName, str) and fileName.endswith("plane.urdf") \
            and _surface_id is None:
        _surface_id = _create_tilt_surface()
        _create_test_sliders()
        return _surface_id
    return _real_loadURDF(fileName, *args, **kwargs)


# ── Hook 2: per-substep surface update (piggybacks on update_camera) ────────
_real_update_camera = Q.update_camera

def _update_surface():
    global _cmd_roll, _cmd_pitch, _btn_level_last, _substep, _hud_id
    if _surface_id is None:
        return
    _substep += 1

    tgt_r = _read(_sl_roll, 0.0)
    tgt_p = _read(_sl_pitch, 0.0)
    rate  = _read(_sl_rate, TILT_RATE_DEFAULT)

    # Level-surface button: snap sliders' effect back to 0 (sliders themselves
    # can't be moved from code, so the button overrides until sliders move).
    b = _read(_btn_level, _btn_level_last)
    if b > _btn_level_last:
        _btn_level_last = b
        _update_surface.override = (tgt_r, tgt_p)
    ov = getattr(_update_surface, "override", None)
    if ov is not None:
        if abs(tgt_r - ov[0]) > 1e-6 or abs(tgt_p - ov[1]) > 1e-6:
            _update_surface.override = None        # slider moved → resume
        else:
            tgt_r = tgt_p = 0.0

    if not _tilt_enabled:                           # hold level until ref taken
        tgt_r = tgt_p = 0.0

    # Rate limit (one substep = 1/240 s)
    step = rate / 240.0
    _cmd_roll  += max(-step, min(step, tgt_r - _cmd_roll))
    _cmd_pitch += max(-step, min(step, tgt_p - _cmd_pitch))
    p.setJointMotorControl2(_surface_id, _roll_joint, p.POSITION_CONTROL,
                            targetPosition=math.radians(_cmd_roll),
                            force=MOTOR_FORCE, maxVelocity=math.radians(rate) * 2)
    p.setJointMotorControl2(_surface_id, _pitch_joint, p.POSITION_CONTROL,
                            targetPosition=math.radians(_cmd_pitch),
                            force=MOTOR_FORCE, maxVelocity=math.radians(rate) * 2)

    # Posture controller on/off (reads the controller's own master switch;
    # the controller code itself is untouched).
    Q.POSTURE_ENABLED = _read(_sl_posture, 1.0) >= 0.5

    # HUD, ~10 Hz
    if _substep % 24 == 0 and _robot_id is not None:
        act_r = math.degrees(p.getJointState(_surface_id, _roll_joint)[0])
        act_p = math.degrees(p.getJointState(_surface_id, _pitch_joint)[0])
        pos, orn = p.getBasePositionAndOrientation(_robot_id)
        # Body tilt relative to gravity (what the controller tries to zero):
        R = p.getMatrixFromQuaternion(orn)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[8]))))
        br, bp, _ = (math.degrees(a) for a in p.getEulerFromQuaternion(orn))
        # Which feet actually carry load (a posture/IK problem shows up here
        # as only 2-3 loaded feet while standing still).
        loaded = []
        for lid, cfg in Q.LEG_CONFIG.items():
            f = sum(c[9] for c in p.getContactPoints(_robot_id, _surface_id,
                                                     cfg['tibia_idx'], _pitch_joint))
            if f > FOOT_LOAD_THRESHOLD:
                loaded.append(str(lid))
        txt = (f"SURFACE R:{act_r:+5.1f} P:{act_p:+5.1f}  |  "
               f"BODY R:{br:+5.1f} P:{bp:+5.1f}  tilt:{tilt:4.1f} deg  |  "
               f"feet loaded:{len(loaded)}/4 [{','.join(loaded)}]  |  "
               f"PD:{'ON' if Q.POSTURE_ENABLED else 'OFF'}"
               f"{'' if _tilt_enabled else '  (surface locked: settling)'}")
        col = [0.2, 1.0, 0.4] if tilt < 3.0 else ([1.0, 0.8, 0.2] if tilt < 8.0
                                                   else [1.0, 0.25, 0.25])
        at = [pos[0], pos[1], pos[2] + 0.40]
        if _hud_id >= 0:
            _hud_id = p.addUserDebugText(txt, at, textColorRGB=col, textSize=1.2,
                                         replaceItemUniqueId=_hud_id)
        else:
            _hud_id = p.addUserDebugText(txt, at, textColorRGB=col, textSize=1.2)


def _update_camera_hook():
    _real_update_camera()
    _update_surface()


# ── Hook 3: capture robot id; unlock tilting once the sim loop starts ───────
_robot_id = None
_real_run = Q.run_unified_sim

def _run_hook(robot_id, imu, posture):
    global _robot_id, _tilt_enabled
    _robot_id = robot_id
    for cfg in Q.LEG_CONFIG.values():   # feet side of the contact pair
        p.changeDynamics(robot_id, cfg['tibia_idx'], **FOOT_MODELS[FOOT_MODEL])
        if USE_FRICTION_ANCHORS:
            p.changeDynamics(robot_id, cfg['tibia_idx'], frictionAnchor=1)
    print(f"  [TILT TEST] Foot model: {FOOT_MODEL} {FOOT_MODELS[FOOT_MODEL]}")
    _tilt_enabled = True     # level reference was captured on a level surface
    print("\n  [TILT TEST] Surface unlocked. Use 'Surface Roll' / 'Surface Pitch'"
          " sliders. HUD: SURFACE = ground angle, BODY = robot angle"
          " (goal: tilt ~0).\n")
    return _real_run(robot_id, imu, posture)


def main():
    p.loadURDF = _loadURDF_hook
    Q.p.loadURDF = _loadURDF_hook          # same module object; explicit anyway
    Q.update_camera = _update_camera_hook
    Q.run_unified_sim = _run_hook
    try:
        Q.main()
    finally:
        p.loadURDF = _real_loadURDF


if __name__ == "__main__":
    main()