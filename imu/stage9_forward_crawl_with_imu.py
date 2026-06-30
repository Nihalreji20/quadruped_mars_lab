"""
stage9_forward_crawl_with_imu.py
=================================
Reference integration of Stage 9 IMU into the forward crawl gait.

THIS FILE IS A TEMPLATE — copy the three IMU blocks (marked with ── Stage 9 ──)
into your real FREE_BASE_TWO_STEP_TEST.py.  The locomotion code between the
markers is left as a placeholder comment so the structure is clear.

Nothing in the locomotion logic is modified.
"""

import pybullet as p
import pybullet_data
import time
import math

# ── Stage 9: IMU import ───────────────────────────────────────────────────────
from imu_module import create_imu
# ─────────────────────────────────────────────────────────────────────────────

# ── Your existing imports (IK, gait, COM helpers, etc.) ──────────────────────
# from your_locomotion_file import compute_ik, LEG_CONFIG, ...
# ─────────────────────────────────────────────────────────────────────────────


def main():
    # ── PyBullet setup (unchanged from your file) ─────────────────────────────
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)

    plane = p.loadURDF("plane.urdf")

    # Spawn robot (use your actual URDF path and spawn pose)
    URDF_PATH = "urdf/finalv6_pla.urdf"
    START_POS = [0, 0, 0.18]
    START_ORN = p.getQuaternionFromEuler([0, 0, math.pi])   # 180° yaw spawn

    robot = p.loadURDF(URDF_PATH, START_POS, START_ORN, useFixedBase=False)
    p.resetBasePositionAndOrientation(robot, START_POS, START_ORN)

    # ── Stage 9: Construct simulated IMU (right after robot spawn) ────────────
    imu = create_imu(
        mode="sim",
        pybullet_client=p,
        robot_id=robot,
        noise_std_deg=0.10,           # ±0.10 deg RMS (BNO055-like)
        gyro_noise_std_deg_s=0.50,    # ±0.50 deg/s RMS gyro noise
        yaw_drift_rate_deg_s=0.00,    # 0 = no drift; try 0.05 to simulate bias
        update_rate_hz=240.0,
    )
    imu.reset_yaw_reference()         # zero yaw at the current spawn heading
    print("[IMU] Simulated IMU initialised.  Yaw reference zeroed.")
    # ─────────────────────────────────────────────────────────────────────────

    # ── Your existing initialisation (joint map, gait state, COM, etc.) ───────
    # joint_map = { ... }
    # gait_phase = ...
    # ─────────────────────────────────────────────────────────────────────────

    # ── Main simulation loop ──────────────────────────────────────────────────
    step_count = 0
    TELEMETRY_INTERVAL = 48          # print every 48 steps ≈ 5 Hz at 240 Hz

    try:
        while True:
            # ── Your existing gait + COM code (FROZEN, not touched) ───────────
            # compute_swing_targets(...)
            # apply_ik_to_legs(...)
            # stabilize_com(...)
            # ─────────────────────────────────────────────────────────────────

            p.stepSimulation()

            # ── Stage 9: IMU update + telemetry output ────────────────────────
            imu.update()
            if step_count % TELEMETRY_INTERVAL == 0:
                print(imu.get_telemetry_string())

                # Optional: fall-detection guard (does not affect gait)
                if imu.is_tilting(threshold_deg=20.0):
                    print("[IMU] WARNING — excessive tilt detected!")
            # ─────────────────────────────────────────────────────────────────

            step_count += 1
            time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        print(f"\n[IMU] Simulation ended after {step_count} steps.")
        r, pitch, yaw = imu.get_euler_deg()
        print(f"[IMU] Final orientation — Roll:{r:+.2f}°  Pitch:{pitch:+.2f}°  Yaw:{yaw:+.2f}°")
        p.disconnect()


if __name__ == "__main__":
    main()
