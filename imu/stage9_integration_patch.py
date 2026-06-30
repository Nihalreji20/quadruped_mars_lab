"""
stage9_integration_patch.py
============================
This file documents the EXACT surgical changes needed to integrate the IMU
into FREE_BASE_TWO_STEP_TEST.py (forward crawl) and turn_gait.py (turning).

Nothing in the locomotion stack is modified.  Only three things are added:
  1.  Import the IMU module at the top of the file.
  2.  Construct the IMU once, after the robot is spawned.
  3.  Call imu.update() and print imu.get_telemetry_string() inside the
      existing simulation loop.

----------------------------------------------------------------------
PATCH A — FREE_BASE_TWO_STEP_TEST.py  (forward crawl)
----------------------------------------------------------------------

LOCATION 1 — Add import after existing imports (near the top of the file):

    # ── Stage 9: IMU ───────────────────────────────────────────────────
    from imu_module import create_imu
    # ──────────────────────────────────────────────────────────────────

LOCATION 2 — Add IMU construction AFTER `robot = p.loadURDF(...)` and AFTER
`p.resetBasePositionAndOrientation(...)`, but BEFORE the main loop:

    # ── Stage 9: Construct simulated IMU ──────────────────────────────
    imu = create_imu(
        mode="sim",
        pybullet_client=p,
        robot_id=robot,
        noise_std_deg=0.10,          # ±0.1 deg RMS orientation noise
        gyro_noise_std_deg_s=0.5,    # ±0.5 deg/s RMS gyro noise
        yaw_drift_rate_deg_s=0.0,    # set >0 to simulate gyro drift
        update_rate_hz=240.0,        # match your p.setTimeStep
    )
    imu.reset_yaw_reference()        # zero yaw at spawn pose
    # ──────────────────────────────────────────────────────────────────

LOCATION 3 — Inside the main simulation while-loop, at the END of each
iteration (after p.stepSimulation() and before time.sleep()):

    # ── Stage 9: IMU update + telemetry ───────────────────────────────
    imu.update()
    if step_count % 48 == 0:         # print ~5 times/sec at 240 Hz
        print(imu.get_telemetry_string())
    # ──────────────────────────────────────────────────────────────────

    Where `step_count` is a counter you already have, or add one:

    step_count = 0                   # initialise before the loop
    # ...  inside the loop:
    step_count += 1

----------------------------------------------------------------------
PATCH B — turn_gait.py  (turning gait)
----------------------------------------------------------------------

Identical three-location patch.  The only difference: after initialising
the IMU, also stash the starting yaw so the turn controller can compare
against it later (Stage 10 / closed-loop turning — not needed for Stage 9):

    imu = create_imu(
        mode="sim",
        pybullet_client=p,
        robot_id=robot,
        noise_std_deg=0.10,
        gyro_noise_std_deg_s=0.5,
        yaw_drift_rate_deg_s=0.0,
        update_rate_hz=240.0,
    )
    imu.reset_yaw_reference()

The telemetry line inside the loop is identical to Patch A.
During a turn, Yaw in the telemetry will show the angle accumulated
from the starting heading — useful for validating the turn angle visually.

----------------------------------------------------------------------
WHAT IS NOT CHANGED
----------------------------------------------------------------------
- compute_ik()               ← frozen
- LEG_CONFIG                 ← frozen
- Gait order 1→2→4→3        ← frozen
- COM stabilization logic    ← frozen
- Foot target generation     ← frozen
- Turning logic              ← frozen
- p.setTimeStep / physics    ← unchanged
- URDF loading / joint map   ← unchanged

----------------------------------------------------------------------
FULL EXAMPLE (minimal forward crawl skeleton with IMU, for reference)
----------------------------------------------------------------------

See  stage9_forward_crawl_with_imu.py  in this same directory.
"""
