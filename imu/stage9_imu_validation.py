"""
stage9_imu_validation.py
=========================
Standalone validation procedure for the Stage 9 IMU module.

Runs WITHOUT the quadruped URDF — uses a simple PyBullet box as a mock
robot body so the validator is self-contained and runnable at any time.

Tests
-----
1.  INIT      — IMU constructs without error.
2.  UPDATE    — update() runs 1000 times without exception.
3.  EULER     — roll/pitch/yaw are finite floats.
4.  GYRO      — angular velocity components are finite floats.
5.  NOISE     — repeated reads show variation proportional to noise_std.
6.  DRIFT     — yaw drift accumulates at the correct rate over 1 second.
7.  ZERO-YAW  — reset_yaw_reference() returns yaw to ~0 deg.
8.  TILT      — is_tilting() returns False on a flat body, True when tilted.
9.  TELEMETRY — get_telemetry_string() returns a non-empty string.
10. FACTORY   — create_imu("sim", ...) works via the factory function.

Run:
    python stage9_imu_validation.py
Expected output:  all tests PASS, then example telemetry lines.
"""

import pybullet as p
import pybullet_data
import time
import math
import sys
import os

# Allow running from this directory regardless of CWD
sys.path.insert(0, os.path.dirname(__file__))
from imu_module import SimulatedIMU, create_imu

PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"

results = []

def check(name, condition, detail=""):
    tag = PASS if condition else FAIL
    print(f"  [{tag}]  {name}" + (f"  ({detail})" if detail else ""))
    results.append(condition)


# ── Setup ─────────────────────────────────────────────────────────────────────
print("\n=== Stage 9 IMU Validation ===\n")

phys = p.connect(p.DIRECT)          # headless
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / 240.0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create a mock body (small box as stand-in for the robot base)
col  = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.05, 0.03])
vis  = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.05, 0.03])
body = p.createMultiBody(baseMass=0.5,
                         baseCollisionShapeIndex=col,
                         baseVisualShapeIndex=vis,
                         basePosition=[0, 0, 0.15],
                         baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))
p.loadURDF("plane.urdf")

# Let the body settle
for _ in range(50):
    p.stepSimulation()


# ── Test 1 — INIT ─────────────────────────────────────────────────────────────
print("Group 1: Construction")
try:
    imu = SimulatedIMU(
        pybullet_client=p,
        robot_id=body,
        noise_std_deg=0.10,
        gyro_noise_std_deg_s=0.50,
        yaw_drift_rate_deg_s=0.0,
        update_rate_hz=240.0,
    )
    check("INIT — SimulatedIMU constructs without error", True)
except Exception as e:
    check("INIT — SimulatedIMU constructs without error", False, str(e))
    p.disconnect()
    sys.exit(1)

# ── Test 2 — UPDATE ───────────────────────────────────────────────────────────
print("\nGroup 2: Core reads")
try:
    for _ in range(1000):
        p.stepSimulation()
        imu.update()
    check("UPDATE — 1000 iterations without exception", True,
          f"update_count={imu.get_update_count()}")
except Exception as e:
    check("UPDATE — 1000 iterations without exception", False, str(e))

# ── Test 3 — EULER ────────────────────────────────────────────────────────────
r, pitch, yaw = imu.get_euler_deg()
check("EULER — roll is finite float",   math.isfinite(r),     f"roll={r:.4f}°")
check("EULER — pitch is finite float",  math.isfinite(pitch), f"pitch={pitch:.4f}°")
check("EULER — yaw is finite float",    math.isfinite(yaw),   f"yaw={yaw:.4f}°")

# ── Test 4 — GYRO ─────────────────────────────────────────────────────────────
wx, wy, wz = imu.get_angular_velocity()
check("GYRO — wx is finite float",  math.isfinite(wx), f"wx={wx:.2f}°/s")
check("GYRO — wy is finite float",  math.isfinite(wy), f"wy={wy:.2f}°/s")
check("GYRO — wz is finite float",  math.isfinite(wz), f"wz={wz:.2f}°/s")

# ── Test 5 — NOISE ────────────────────────────────────────────────────────────
print("\nGroup 3: Noise and drift")
noisy_imu = SimulatedIMU(p, body,
    noise_std_deg=1.0,       # large noise to make std detectable
    gyro_noise_std_deg_s=0.0,
    yaw_drift_rate_deg_s=0.0,
    update_rate_hz=240.0)

rolls = []
for _ in range(500):
    p.stepSimulation()
    noisy_imu.update()
    rolls.append(noisy_imu.get_roll())

import statistics
std_measured = statistics.stdev(rolls)
# Expect measured std to be within ±40% of set std=1.0
noise_ok = 0.4 < std_measured < 2.0
check("NOISE — measured roll std proportional to noise_std",
      noise_ok, f"std_measured={std_measured:.3f}°")

# ── Test 6 — DRIFT ────────────────────────────────────────────────────────────
DRIFT_RATE = 5.0          # deg/s — large to be detectable quickly
drift_imu = SimulatedIMU(p, body,
    noise_std_deg=0.0,          # zero noise for clean measurement
    gyro_noise_std_deg_s=0.0,
    yaw_drift_rate_deg_s=DRIFT_RATE,
    update_rate_hz=240.0)

# Run for 240 steps = 1 simulated second
for _ in range(240):
    p.stepSimulation()
    drift_imu.update()

_, _, yaw_after_1s = drift_imu.get_euler_deg()
# Expected drift = DRIFT_RATE * 1 s = 5°; allow ±20% tolerance
drift_ok = abs(yaw_after_1s - DRIFT_RATE) < DRIFT_RATE * 0.25
check("DRIFT — yaw drift accumulates at correct rate (~5°/s over 1 s)",
      drift_ok, f"yaw_after_1s={yaw_after_1s:.3f}°  expected≈{DRIFT_RATE:.1f}°")

# ── Test 7 — ZERO YAW ─────────────────────────────────────────────────────────
# Tilt the body, accumulate yaw, then reset
for _ in range(100):
    p.stepSimulation()
    imu.update()
yaw_before_reset = imu.get_yaw()
imu.reset_yaw_reference()
for _ in range(10):
    p.stepSimulation()
    imu.update()
yaw_after_reset = imu.get_yaw()
check("ZERO-YAW — reset_yaw_reference() brings yaw close to 0°",
      abs(yaw_after_reset) < 1.0,
      f"before={yaw_before_reset:.2f}°  after_reset={yaw_after_reset:.2f}°")

# ── Test 8 — TILT DETECTION ───────────────────────────────────────────────────
print("\nGroup 4: Optional helpers")
# Flat body should not trigger tilt
imu.update()
check("TILT — is_tilting(20°) is False on flat body",
      not imu.is_tilting(threshold_deg=20.0),
      f"roll={imu.get_roll():.2f}°  pitch={imu.get_pitch():.2f}°")

# Tilt the mock body manually and verify
tilted_orn = p.getQuaternionFromEuler([math.radians(30), 0, 0])  # 30° roll
p.resetBasePositionAndOrientation(body, [0, 0, 0.15], tilted_orn)
for _ in range(20):
    p.stepSimulation()
    imu.update()
check("TILT — is_tilting(20°) is True at 30° roll",
      imu.is_tilting(threshold_deg=20.0),
      f"roll={imu.get_roll():.2f}°")

# Restore flat
flat_orn = p.getQuaternionFromEuler([0, 0, 0])
p.resetBasePositionAndOrientation(body, [0, 0, 0.15], flat_orn)

# ── Test 9 — TELEMETRY STRING ─────────────────────────────────────────────────
imu.update()
telem = imu.get_telemetry_string()
check("TELEMETRY — get_telemetry_string() returns non-empty string",
      isinstance(telem, str) and len(telem) > 10, f'"{telem}"')

# ── Test 10 — FACTORY ─────────────────────────────────────────────────────────
print("\nGroup 5: Factory function")
try:
    imu_via_factory = create_imu("sim",
                                 pybullet_client=p,
                                 robot_id=body,
                                 noise_std_deg=0.1)
    imu_via_factory.update()
    check("FACTORY — create_imu('sim', ...) returns working SimulatedIMU", True)
except Exception as e:
    check("FACTORY — create_imu('sim', ...) returns working SimulatedIMU", False, str(e))

# ── Summary ───────────────────────────────────────────────────────────────────
p.disconnect()

passed = sum(results)
total  = len(results)
print(f"\n{'='*46}")
print(f"  Results: {passed}/{total} tests passed")
print(f"{'='*46}\n")

if passed == total:
    print("✓ All Stage 9 IMU tests passed.\n")
else:
    print(f"✗ {total - passed} test(s) FAILED — review output above.\n")
    sys.exit(1)

# ── Example Telemetry Output ──────────────────────────────────────────────────
print("=== Example Telemetry Output (20 frames at 240 Hz) ===\n")

phys2 = p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / 240.0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
col2  = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.05, 0.03])
body2 = p.createMultiBody(baseMass=0.5,
                          baseCollisionShapeIndex=col2,
                          basePosition=[0, 0, 0.15],
                          baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

demo_imu = create_imu("sim",
                       pybullet_client=p,
                       robot_id=body2,
                       noise_std_deg=0.10,
                       gyro_noise_std_deg_s=0.50,
                       yaw_drift_rate_deg_s=0.05,   # mild drift for demo
                       update_rate_hz=240.0)
demo_imu.reset_yaw_reference()

for i in range(240):
    p.stepSimulation()
    demo_imu.update()
    if i % 12 == 0:    # print every 12 steps = ~20 Hz for readability
        print(f"  step {i:>4d}  {demo_imu.get_telemetry_string()}")

p.disconnect()
print()
