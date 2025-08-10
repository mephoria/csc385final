import pybullet as p
import pybullet_data
import time
import math
import serial

# Start PyBullet in GUI mode
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.81)

quat = p.getQuaternionFromEuler([math.pi/2, 0, 0])

# Load your URDF
robot_id = p.loadURDF("ASSEMBLY/ASSEMBLY.urdf",
                      useFixedBase=True,
                      flags=p.URDF_USE_SELF_COLLISION,
                      baseOrientation=quat)

# Analyze joints and find controllable ones
num_joints = p.getNumJoints(robot_id)
print(f"Robot has {num_joints} joints")

controllable_joints = []
joint_names = []

for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    joint_name = info[1].decode()
    joint_type = info[2]
    
    print(f"Joint {i}: {joint_name}, Type: {joint_type}")
    
    # Only consider revolute (0) and prismatic (1) joints for control
    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        controllable_joints.append(i)
        joint_names.append(joint_name)
        print(f"  -> Controllable joint added")

print(f"\nControllable joints: {controllable_joints}")
print(f"Joint names: {joint_names}")

# Initialize serial connection
try:
    # Windows: 'COM3', 'COM4', etc.
    # Linux/Mac: '/dev/ttyUSB0', '/dev/ttyACM0', etc.
    ser = serial.Serial('COM4', 115200, timeout=1)  # Change COM port as needed
    print("Connected to STM32")
except:
    print("Could not connect to STM32 - using demo mode")
    ser = None

# Robot control parameters
current_positions = [0.0] * len(controllable_joints)  # Current target positions
move_speed = 0.02  # How fast joints move

def map_imu_to_joints(tilt_x, tilt_y):
    """
    Map IMU tilt to joint positions
    Adjust this function based on your robot's kinematics
    """
    if len(controllable_joints) == 0:
        return []
    
    # Example mapping for a typical 6-DOF arm
    # Adjust these mappings based on your robot's joint configuration
    joint_targets = [0.0] * len(controllable_joints)
    
    if len(controllable_joints) >= 2:
        # Map tilt_x to first joint (base rotation or shoulder)
        joint_targets[0] = math.radians(tilt_y *4)  # Scale factor of 2
        
        # Map tilt_y to second joint (shoulder or elbow)
        joint_targets[1] = math.radians(tilt_x *4)
    
    if len(controllable_joints) >= 3:
        # Third joint - could be elbow
        joint_targets[2] = math.radians(tilt_x * 3)
    
    if len(controllable_joints) >= 4:
        # Fourth joint - could be wrist
        joint_targets[3] = math.radians(-tilt_y * 1.0)
    
    # Keep remaining joints at default positions or add more mappings
    
    return joint_targets

def smooth_move_joints(target_positions):
    """
    Smoothly move joints to target positions
    """
    global current_positions
    
    for i, target in enumerate(target_positions):
        if i < len(current_positions):
            # Smooth interpolation
            diff = target - current_positions[i]
            current_positions[i] += diff * move_speed
            
            # Apply to robot
            if i < len(controllable_joints):
                p.setJointMotorControl2(robot_id,
                                        controllable_joints[i],
                                        p.POSITION_CONTROL,
                                        targetPosition=current_positions[i],
                                        maxVelocity=1.0)

def demo_movement():
    """
    Demo function to test joint movement without IMU
    """
    t = time.time()
    
    if len(controllable_joints) >= 2:
        # Sinusoidal movement for demo
        angle1 = math.sin(t * 0.5) * 0.8  # Slow oscillation
        angle2 = math.cos(t * 0.3) * 0.6
        
        target_positions = [angle1, angle2] + [0.0] * (len(controllable_joints) - 2)
        smooth_move_joints(target_positions)

def parse_imu_data(line):
    """
    Parse IMU data from STM32
    Expected format: "TILT_X:angle,TILT_Y:angle,..."
    """
    line = line.strip()
    if line.startswith("TILT_X:"):
        try:
            parts = line.split(',')
            tilt_x = float(parts[0].split(':')[1])
            tilt_y = float(parts[1].split(':')[1])
            return tilt_x, tilt_y
        except:
            return None, None
    return None, None

# Main control loop
print("\nStarting robot control...")
print("Demo mode: Robot will move in a pattern")
print("Uncomment serial code to use IMU control")

demo_mode = False  # Set to True if you want demo even with IMU connected

while True:
    imu_connected = False
    
    # IMU control when serial data is available
    if ser and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8')
            tilt_x, tilt_y = parse_imu_data(line)
            
            if tilt_x is not None and tilt_y is not None:
                target_positions = map_imu_to_joints(tilt_x, tilt_y)
                smooth_move_joints(target_positions)
                imu_connected = True
                
                print(f"IMU: X={tilt_x:6.2f}°, Y={tilt_y:6.2f}° -> Joints: {[f'{p:6.2f}' for p in current_positions[:3]]}")
        except Exception as e:
            print(f"Serial Error: {e}")
    
    # Demo mode when no IMU data
    if demo_mode and not imu_connected:
        demo_movement()
    
    # Step simulation
    p.stepSimulation()
    time.sleep(1/240)

# Cleanup
# if ser:
#     ser.close()
p.disconnect()