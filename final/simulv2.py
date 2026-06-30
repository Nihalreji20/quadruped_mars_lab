import pybullet as p
import pybullet_data
import time
import math

# ==========================================
# 1. PHYSICS ENGINE & WORLD SETUP
# ==========================================
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")

# THE FIX #1: Rotate the base by 180 degrees (math.pi) around the Z-axis.
# Because the URDF was modeled pointing backwards (-X), this spins it 
# around so it naturally faces forward (+X) to match your math.
base_spawn_height = 0.4 
startPos = [0, 0, base_spawn_height]
startOrientation = p.getQuaternionFromEuler([0, 0, math.pi])

legId = p.loadURDF("urdf/leg.urdf", startPos, startOrientation, useFixedBase=True)

# ==========================================
# 2. USER INTERFACE (SLIDERS)
# ==========================================
p.addUserDebugText("IK Target Controls (mm)", [0, 0, 0.1], textColorRGB=[1, 0.2, 0.2])
sld_x = p.addUserDebugParameter("Target X", 50, 250, 150)
sld_y = p.addUserDebugParameter("Target Y", -100, 100, 0)
sld_z = p.addUserDebugParameter("Target Z", -250, 20, -150)

p.addUserDebugText("Camera Controls", [0, 0, 0.1], textColorRGB=[0.2, 0.6, 1])
sld_cam_dist = p.addUserDebugParameter("Cam Distance", 0.1, 2.0, 0.38)
sld_cam_yaw = p.addUserDebugParameter("Cam Yaw", -180, 180, 45)
sld_cam_pitch = p.addUserDebugParameter("Cam Pitch", -90, 90, -30)

# Create a visual red sphere to track exactly where the IK target is
target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.015, rgbaColor=[1, 0, 0, 1])
target_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=target_visual, basePosition=[0,0,0])

# ==========================================
# 3. PURE GEOMETRIC INVERSE KINEMATICS
# ==========================================
def compute_ik(px, py, pz):
    # Physical Constants (mm)
    a1 = 52.50
    a2 = 108.00
    a3 = 130.775
    
    # Motor Limits (Degrees)
    T1_MIN, T1_MAX = -20, 20
    T2_MIN, T2_MAX = -90, 45
    T3_MIN, T3_MAX = -45, 90
    
    # 1. Coxa Angle
    t1_rad = math.atan2(py, px)
    t1_deg = math.degrees(t1_rad)
    
    # 2. Horizontal distance from hip to target
    R = math.sqrt(px**2 + py**2)
    
    # 3. Distances for the Femur-Tibia 2D Plane
    r = R - a1
    z = pz
    
    # 4. Law of Cosines for the Knee Angle
    L_squared = r**2 + z**2
    if L_squared == 0:
        return 0, 0, 0, False
        
    c3 = (L_squared - a2**2 - a3**2) / (2 * a2 * a3)
    c3 = max(-1.0, min(1.0, c3)) # Precision Clamping
    
    # Negative root forces the knee to bend backwards
    s3 = -math.sqrt(1 - c3**2) 
    
    t3_math = math.atan2(s3, c3)
    t2_math = math.atan2(z, r) - math.atan2(a3 * s3, a2 + a3 * c3)
    
    # ---------------------------------------------------------
    # 5. THE FIX #2: URDF AXIS MAPPING
    # ---------------------------------------------------------
    # Femur passes straight through (Math Negative = Down, URDF Negative = Down)
    urdf_t2_deg = math.degrees(t2_math)
    
    # Tibia requires an inverted 90-degree offset because its URDF 
    # resting pose is straight down AND its Y-axis is inverted (-1.0).
    urdf_t3_deg = -math.degrees(t3_math) 
    
    urdf_t2_deg = ((urdf_t2_deg + 180) % 360) - 180
    urdf_t3_deg = ((urdf_t3_deg + 180) % 360) - 180
    
    # 6. Validate
    tol = 0.5
    if (T1_MIN - tol <= t1_deg <= T1_MAX + tol and 
        T2_MIN - tol <= urdf_t2_deg <= T2_MAX + tol and 
        T3_MIN - tol <= urdf_t3_deg <= T3_MAX + tol):
        
        return t1_deg, urdf_t2_deg, urdf_t3_deg, True
    
    return 0, 0, 0, False

# ==========================================
# 4. MAIN SIMULATION LOOP
# ==========================================
print("Simulation Started. Move the Target X/Y/Z sliders to control the leg.")

joint_ids = []
for i in range(p.getNumJoints(legId)):
    if p.getJointInfo(legId, i)[2] == p.JOINT_REVOLUTE:
        joint_ids.append(i)

if len(joint_ids) < 3:
    joint_ids = [0, 1, 2] 

while True:
    # --- Update Camera ---
    cam_dist = p.readUserDebugParameter(sld_cam_dist)
    cam_yaw = p.readUserDebugParameter(sld_cam_yaw)
    cam_pitch = p.readUserDebugParameter(sld_cam_pitch)
    
    p.resetDebugVisualizerCamera(cameraDistance=cam_dist, 
                                 cameraYaw=cam_yaw, 
                                 cameraPitch=cam_pitch, 
                                 cameraTargetPosition=[0, 0, base_spawn_height])
    
    # --- Read Target Sliders ---
    target_x = p.readUserDebugParameter(sld_x)
    target_y = p.readUserDebugParameter(sld_y)
    target_z = p.readUserDebugParameter(sld_z)
    
    # Update visual red sphere (PyBullet uses meters, so divide mm by 1000)
    p.resetBasePositionAndOrientation(target_body, 
                                      [target_x/1000.0, target_y/1000.0, base_spawn_height + (target_z/1000.0)], 
                                      [0, 0, 0, 1])
    
    # --- Compute IK ---
    t1_deg, t2_deg, t3_deg, is_valid = compute_ik(target_x, target_y, target_z)
    
    if is_valid:
        p.setJointMotorControl2(legId, joint_ids[0], p.POSITION_CONTROL, targetPosition=math.radians(t1_deg))
        p.setJointMotorControl2(legId, joint_ids[1], p.POSITION_CONTROL, targetPosition=math.radians(t2_deg))
        p.setJointMotorControl2(legId, joint_ids[2], p.POSITION_CONTROL, targetPosition=math.radians(t3_deg))
    
    p.stepSimulation()
    time.sleep(1./240.)