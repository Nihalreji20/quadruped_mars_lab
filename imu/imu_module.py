"""
imu_module.py  —  Stage 9: Simulated IMU Integration
=====================================================
Provides a hardware-compatible IMU interface for the quadruped locomotion stack.

Simulated backend reads ground-truth orientation and angular velocity directly
from PyBullet.  The public API is designed so the entire class can be swapped
for a real BNO055 / SmartElex 9-DoF driver without touching any other file.

Hardware-swap contract
----------------------
Any real-sensor replacement must expose the same three methods:
    update()                 — called once per simulation / control tick
    get_euler_deg()          — returns (roll, pitch, yaw) in degrees
    get_angular_velocity()   — returns (wx, wy, wz) in deg/s
    get_telemetry_string()   — returns a one-line human-readable string
    reset_yaw_reference()    — zeros the yaw reference

Author:  Nihal's Quadruped Project
Stage:   9 — Simulated IMU Integration
"""

import math
import time
import numpy as np


# ---------------------------------------------------------------------------
# Base class — defines the hardware-swap contract
# ---------------------------------------------------------------------------

class IMUBase:
    """
    Abstract interface that every IMU backend must satisfy.
    Do NOT modify this class when swapping to a real sensor;
    write a new subclass instead.
    """

    def update(self):
        """Poll the sensor / PyBullet.  Call once per control tick."""
        raise NotImplementedError

    def get_euler_deg(self):
        """Return (roll, pitch, yaw) in degrees (floats)."""
        raise NotImplementedError

    def get_angular_velocity(self):
        """Return (wx, wy, wz) angular velocity in deg/s."""
        raise NotImplementedError

    def get_telemetry_string(self):
        """Return a single formatted telemetry line for console output."""
        raise NotImplementedError

    def reset_yaw_reference(self):
        """Zero the yaw reference (useful when robot is placed / re-homed)."""
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Simulated backend — reads from PyBullet
# ---------------------------------------------------------------------------

class SimulatedIMU(IMUBase):
    """
    Simulates a BNO055 / SmartElex 9-DoF IMU by reading PyBullet state.

    Parameters
    ----------
    pybullet_client : pybullet module (already connected)
        The `p` object from the caller — passed in, never imported here.
    robot_id : int
        PyBullet body unique ID of the robot base link.
    noise_std_deg : float
        Gaussian noise standard deviation applied to roll/pitch/yaw (degrees).
        Set 0.0 to disable.  BNO055 datasheet typical: ~0.1 deg.
    gyro_noise_std_deg_s : float
        Gaussian noise std-dev for angular velocity readings (deg/s).
        Set 0.0 to disable.
    yaw_drift_rate_deg_s : float
        Constant yaw drift injected per second (simulates gyro bias).
        Set 0.0 to disable.  Positive = clockwise drift.
    update_rate_hz : float
        Nominal sensor update rate.  Used only for drift accumulation.
        Set to match your simulation step (default 240 Hz).
    """

    def __init__(
        self,
        pybullet_client,
        robot_id,
        noise_std_deg=0.1,
        gyro_noise_std_deg_s=0.5,
        yaw_drift_rate_deg_s=0.0,
        update_rate_hz=240.0,
    ):
        self._p = pybullet_client
        self._robot_id = robot_id

        # Noise / drift configuration
        self._noise_std = noise_std_deg
        self._gyro_noise_std = gyro_noise_std_deg_s
        self._drift_rate = yaw_drift_rate_deg_s
        self._dt = 1.0 / update_rate_hz

        # Internal state
        self._roll = 0.0          # deg
        self._pitch = 0.0         # deg
        self._yaw_raw = 0.0       # deg  (noisy, no drift)
        self._yaw = 0.0           # deg  (with drift applied)
        self._wx = 0.0            # deg/s
        self._wy = 0.0            # deg/s
        self._wz = 0.0            # deg/s

        self._yaw_drift_accum = 0.0      # accumulated drift (deg)
        self._yaw_reference = None       # set on first update or manual reset

        self._last_update_time = None
        self._update_count = 0

    # ------------------------------------------------------------------
    # Public API (hardware-swap contract)
    # ------------------------------------------------------------------

    def update(self):
        """
        Read PyBullet base state, apply noise and drift, update internal cache.
        Call once per simulation step.
        """
        now = time.monotonic()
        if self._last_update_time is not None:
            dt = now - self._last_update_time
            # Clamp dt: if a step took too long, don't accumulate huge drift
            dt = min(dt, 0.1)
        else:
            dt = self._dt
        self._last_update_time = now

        # --- Ground-truth from PyBullet ---
        pos, orn = self._p.getBasePositionAndOrientation(self._robot_id)
        euler_rad = self._p.getEulerFromQuaternion(orn)  # (roll, pitch, yaw) radians
        vel_lin, vel_ang = self._p.getBaseVelocity(self._robot_id)
        # vel_ang is world-frame rad/s; convert to deg/s
        wx_gt = math.degrees(vel_ang[0])
        wy_gt = math.degrees(vel_ang[1])
        wz_gt = math.degrees(vel_ang[2])

        roll_gt  = math.degrees(euler_rad[0])
        pitch_gt = math.degrees(euler_rad[1])
        yaw_gt   = math.degrees(euler_rad[2])

        # --- Gaussian noise ---
        if self._noise_std > 0.0:
            roll_gt  += np.random.normal(0.0, self._noise_std)
            pitch_gt += np.random.normal(0.0, self._noise_std)
            yaw_gt   += np.random.normal(0.0, self._noise_std)

        if self._gyro_noise_std > 0.0:
            wx_gt += np.random.normal(0.0, self._gyro_noise_std)
            wy_gt += np.random.normal(0.0, self._gyro_noise_std)
            wz_gt += np.random.normal(0.0, self._gyro_noise_std)

        # --- Yaw reference zeroing (initialise on first tick) ---
        if self._yaw_reference is None:
            self._yaw_reference = yaw_gt

        # --- Yaw drift accumulation ---
        self._yaw_drift_accum += self._drift_rate * dt

        # --- Store ---
        self._roll  = roll_gt
        self._pitch = pitch_gt
        self._yaw_raw = yaw_gt - self._yaw_reference
        self._yaw = self._yaw_raw + self._yaw_drift_accum
        self._wx = wx_gt
        self._wy = wy_gt
        self._wz = wz_gt

        self._update_count += 1

    def get_euler_deg(self):
        """Return (roll, pitch, yaw) in degrees."""
        return (self._roll, self._pitch, self._yaw)

    def get_angular_velocity(self):
        """Return (wx, wy, wz) in deg/s  (world frame)."""
        return (self._wx, self._wy, self._wz)

    def reset_yaw_reference(self):
        """Zero yaw to the current heading.  Also resets drift accumulator."""
        self._yaw_reference = None   # will re-initialise on next update()
        self._yaw_drift_accum = 0.0

    def get_telemetry_string(self):
        """Return a compact one-liner for console display."""
        r, p, y = self._roll, self._pitch, self._yaw
        wx, wy, wz = self._wx, self._wy, self._wz
        return (
            f"[IMU] R:{r:+7.2f}°  P:{p:+7.2f}°  Y:{y:+7.2f}°  "
            f"| ω  wx:{wx:+7.1f}  wy:{wy:+7.1f}  wz:{wz:+7.1f}  °/s"
        )

    # ------------------------------------------------------------------
    # Extended accessors (optional, not part of base contract)
    # ------------------------------------------------------------------

    def get_roll(self):   return self._roll
    def get_pitch(self):  return self._pitch
    def get_yaw(self):    return self._yaw

    def get_update_count(self): return self._update_count

    def is_tilting(self, threshold_deg=15.0):
        """
        Returns True if |roll| or |pitch| exceeds the threshold.
        Useful for a simple fall-detection trigger.
        """
        return abs(self._roll) > threshold_deg or abs(self._pitch) > threshold_deg


# ---------------------------------------------------------------------------
# Real-sensor stub — fill in later when hardware arrives
# ---------------------------------------------------------------------------

class BNO055_IMU(IMUBase):
    """
    Stub for the real BNO055 sensor.
    Replace the NotImplementedError bodies with actual I²C / UART reads.

    Wiring note (typical Raspberry Pi):
        SDA → GPIO 2 (pin 3)
        SCL → GPIO 3 (pin 5)
        VIN → 3.3 V
        GND → GND

    Library: `pip install adafruit-circuitpython-bno055`
    """

    def __init__(self, i2c_address=0x28):
        # import board, busio, adafruit_bno055
        # self._sensor = adafruit_bno055.BNO055_I2C(busio.I2C(board.SCL, board.SDA))
        self._address = i2c_address
        raise NotImplementedError(
            "BNO055_IMU: replace this stub with real I²C reads. "
            "See comments in the class body."
        )

    def update(self):             raise NotImplementedError
    def get_euler_deg(self):      raise NotImplementedError
    def get_angular_velocity(self): raise NotImplementedError
    def get_telemetry_string(self): raise NotImplementedError
    def reset_yaw_reference(self):  raise NotImplementedError


class SmartElex9DOF_IMU(IMUBase):
    """
    Stub for the SmartElex 9-DoF sensor (ICM-20948 or similar).
    Fill in with the appropriate serial / I²C protocol.
    """

    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        self._port = port
        self._baud = baud
        raise NotImplementedError(
            "SmartElex9DOF_IMU: replace this stub with actual serial reads."
        )

    def update(self):             raise NotImplementedError
    def get_euler_deg(self):      raise NotImplementedError
    def get_angular_velocity(self): raise NotImplementedError
    def get_telemetry_string(self): raise NotImplementedError
    def reset_yaw_reference(self):  raise NotImplementedError


# ---------------------------------------------------------------------------
# Factory function — the single place to toggle sim vs. real hardware
# ---------------------------------------------------------------------------

def create_imu(mode="sim", **kwargs):
    """
    Factory that returns the correct IMU backend.

    Parameters
    ----------
    mode : str
        "sim"       → SimulatedIMU  (default, needs pybullet_client + robot_id)
        "bno055"    → BNO055_IMU    (real hardware stub)
        "smartelex" → SmartElex9DOF_IMU (real hardware stub)

    kwargs are forwarded to the chosen class constructor.

    Example
    -------
    # Simulation
    imu = create_imu("sim", pybullet_client=p, robot_id=robot,
                     noise_std_deg=0.15, yaw_drift_rate_deg_s=0.05)

    # Real hardware (future)
    imu = create_imu("bno055", i2c_address=0x28)
    """
    if mode == "sim":
        return SimulatedIMU(**kwargs)
    elif mode == "bno055":
        return BNO055_IMU(**kwargs)
    elif mode == "smartelex":
        return SmartElex9DOF_IMU(**kwargs)
    else:
        raise ValueError(f"Unknown IMU mode: '{mode}'.  Choose 'sim', 'bno055', or 'smartelex'.")
