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

# Use a placeholder wheel diameter for now
# (what were trying to calibrate)
drive = DriveBase(motor_left, motor_right, 56, 127)

# -------------------------
# Configuration
# -------------------------
TARGET_DISTANCE_INCHES = 48  # distance pushing robot
TARGET_DISTANCE_MM = TARGET_DISTANCE_INCHES * 25.4  # Convert to millimeters

print("=" * 50)
print("WHEEL DIAMETER CALIBRATION")
print("=" * 50)
print(f"\nTarget distance: {TARGET_DISTANCE_INCHES} inches ({TARGET_DISTANCE_MM:.1f} mm)")
print("\nInstructions:")
print("1. Place robot at starting position")
print("2. Wait 5 seconds for encoders to reset")
print("3. Manually push robot STRAIGHT forward exactly 48 inches")
print("4. Program will wait 5 seconds, then calculate wheel diameter")
print("\nStarting in 5 seconds...")

light.on(Color.ORANGE)
wait(5000)

# Reset encoders to zero
motor_left.reset_angle(0)
motor_right.reset_angle(0)

light.on(Color.GREEN)
print("\n✓ Encoders reset to zero")
print("Push the robot forward 48 inches NOW...")
print("You have 30 seconds to push the robot.")

# Wait for user to push robot
wait(30000)

# Get final encoder readings in deg
left_angle = motor_left.angle()
right_angle = motor_right.angle()

light.on(Color.BLUE)

# Average the two wheels
avg_angle = (left_angle + right_angle) / 2

light.on(Color.BLUE)
print("\n" + "=" * 50)
print("CALIBRATION COMPLETE")
print("=" * 50)
print(f"\nLeft motor:  {left_angle:.1f}°")
print(f"Right motor: {right_angle:.1f}°")
print(f"Average:     {avg_angle:.1f}°")

# Calculate wheel diameter
# Formula: diameter = distance / (rotations * π)
# rotations = angle_degrees / 360
# Using π = 3.14159265359 cuz no math module
PI = 3.14159265359
rotations = avg_angle / 360
calculated_diameter_mm = TARGET_DISTANCE_MM / (rotations * PI)

print(f"\nNumber of wheel rotations: {rotations:.3f}")
print(f"\n{'*' * 50}")
print(f"CALIBRATED WHEEL DIAMETER: {calculated_diameter_mm:.2f} mm")
print(f"{'*' * 50}")

# Convert to inches for reference
calculated_diameter_inches = calculated_diameter_mm / 25.4
print(f"(That's {calculated_diameter_inches:.3f} inches)")

# Show what to update in code
print("\n" + "=" * 50)
print("UPDATE YOUR CODE:")
print("=" * 50)
print("In your DriveBase initialization, use:")
print(f"drive = DriveBase(motor_left, motor_right, {calculated_diameter_mm:.2f}, 127)")
print("\n" + "=" * 50)

# Accuracy check
left_right_diff = abs(left_angle - right_angle)
if left_right_diff > 10:
    light.on(Color.RED)
    print("\n⚠ WARNING: Large difference between left and right wheels!")
    print(f"   Difference: {left_right_diff:.1f}°")
    print("   The robot may not have traveled straight.")
    print("   Consider repeating the calibration.")
else:
    light.on(Color.GREEN)
    print("\n✓ Good calibration - left and right wheels closely matched")

wait(1000)
light.off()