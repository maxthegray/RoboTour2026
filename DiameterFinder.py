from pybricks.hubs import TechnicHub
from pybricks.parameters import Axis, Direction, Port, Stop, Color
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = TechnicHub()
light = hub.light
motor_left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
motor_right = Motor(Port.B, Direction.CLOCKWISE)

# placeholder diameter (what were trying to calibrate)
drive = DriveBase(motor_left, motor_right, 56, 127)

TARGET_DISTANCE_INCHES = 48  # distance pushing robot
TARGET_DISTANCE_MM = TARGET_DISTANCE_INCHES * 25.4

print("Wheel diameter calibration")
print(f"Target distance: {TARGET_DISTANCE_INCHES} in ({TARGET_DISTANCE_MM:.1f} mm)")
print("Push the robot straight forward 48 in once the light turns green")
print("Starting in 5 seconds...")

light.on(Color.ORANGE)
wait(5000)

motor_left.reset_angle(0)
motor_right.reset_angle(0)

light.on(Color.GREEN)
print("Encoders reset, push now (30s)")

wait(30000)

# deg
left_angle = motor_left.angle()
right_angle = motor_right.angle()

light.on(Color.BLUE)

avg_angle = (left_angle + right_angle) / 2

light.on(Color.BLUE)
print(f"Left motor:  {left_angle:.1f}")
print(f"Right motor: {right_angle:.1f}")
print(f"Average:     {avg_angle:.1f}")

# no math module
PI = 3.14159265359
rotations = avg_angle / 360
calculated_diameter_mm = TARGET_DISTANCE_MM / (rotations * PI)

print(f"Rotations: {rotations:.3f}")
print(f"Wheel diameter: {calculated_diameter_mm:.2f} mm")

calculated_diameter_inches = calculated_diameter_mm / 25.4
print(f"({calculated_diameter_inches:.3f} in)")

print(f"drive = DriveBase(motor_left, motor_right, {calculated_diameter_mm:.2f}, 127)")

left_right_diff = abs(left_angle - right_angle)
if left_right_diff > 10:
    light.on(Color.RED)
    print(f"Left/right differ by {left_right_diff:.1f} deg, probably didn't go straight. Redo it")
else:
    light.on(Color.GREEN)

wait(1000)
light.off()
