from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Direction, Port, Stop, Color
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# -------------------------
# Hardware setup
# -------------------------
hub = TechnicHub()
light = hub.light
motor_left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
motor_right = Motor(Port.B, Direction.CLOCKWISE)
drive = DriveBase(motor_left, motor_right, 56, 127)

# -------------------------
# 1 Wait for stability
# -------------------------
print("Step 1: Waiting for stability...")
light.on(Color.ORANGE)
wait(10000)  # Increased from 5000 for better settling
light.on(Color.WHITE)
print("Stable. Starting calibration...\n")

# -------------------------
# 2 Calibration
# -------------------------
print("Step 2: Calibrating gyro...")

# --- Gyro bias ---
samples = 5000  # Increased from 2000 for better precision
sum_x = sum_y = sum_z = 0

for i in range(samples):
    gx = hub.imu.angular_velocity(Axis.X)
    gy = hub.imu.angular_velocity(Axis.Y)
    gz = hub.imu.angular_velocity(Axis.Z)
    sum_x += gx
    sum_y += gy
    sum_z += gz
    wait(5)

bias_x = sum_x / samples
bias_y = sum_y / samples
bias_z = sum_z / samples
angular_velocity_bias = (round(bias_x, 4), round(bias_y, 4), round(bias_z, 4))
print("Gyro bias (deg/s):", angular_velocity_bias)

# --- Heading correction / scale ---
print("\nRotate one full revolution (360°) for heading correction...")
wait(2000)
light.on(Color.YELLOW)
drive.use_gyro(True)
drive.settings(turn_rate=25)

start_heading = hub.imu.heading()
drive.arc(250, 360)
end_heading = hub.imu.heading()

light.on(Color.WHITE)
reported_rotation = end_heading - start_heading
if reported_rotation <= 0:
    reported_rotation += 360

heading_correction = round(360 / reported_rotation * 360, 4)
angular_velocity_scale = (heading_correction, heading_correction, heading_correction)
print("Heading correction:", heading_correction)
print("Angular velocity scale:", angular_velocity_scale)

# -------------------------
# 3️⃣ Heading tracking with EMA smoothing
# -------------------------
print("\nStep 3: Tracking gyro heading (1 per second for 60s)")

heading = 0.0
watch = StopWatch()
last_time = 0
light.on(Color.GREEN)

# Exponential Moving Average parameters
alpha = 0.2  # Smoothing factor (lower = smoother)
gz_smooth = 0.0  # Initialize EMA

# Deadband threshold
deadband = 0.15  # deg/s - ignore readings below this

while watch.time() < 60000:  # 60 seconds
    current_time = watch.time()
    dt = (current_time - last_time) / 1000  # seconds
    last_time = current_time
    
    # Raw gyro Z (yaw)
    gz = hub.imu.angular_velocity(Axis.Z)
    
    # Apply bias and scale correction
    gz_corr = (gz - angular_velocity_bias[2]) * (360 / angular_velocity_scale[2])
    
    # Apply Exponential Moving Average
    gz_smooth = alpha * gz_corr + (1 - alpha) * gz_smooth
    
    # Apply deadband to ignore noise
    if abs(gz_smooth) < deadband:
        gz_smooth = 0.0
    
    # Integrate heading
    heading += gz_smooth * dt
    heading %= 360
    
    # Print once per second
    if current_time % 1000 < 50:
        print(f"{current_time/1000:5.1f}s → Heading: {heading:.2f}°")
    
    wait(50)

light.on(Color.BLUE)
print("\n=== Calibration + Smoothed Gyro Heading Complete ===")
print("Final constants:")
print("angular_velocity_bias =", angular_velocity_bias)
print("heading_correction =", heading_correction)
print("angular_velocity_scale =", angular_velocity_scale)
print(f"\nFinal heading drift: {heading:.2f}° (should be close to 0° if stationary)")