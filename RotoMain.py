from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
#TO CHANGE

TARGET_TIME = 80     # seconds

PATH = "IDWWSAQQDWSDQDWAAQWWSSSAWEDQAASDDWAQZAWWDDSAEP"  # PATH GOES HERE

# measure TS
WHEEL_DIAMETER = 55.56   # mm - measure wheels
AXLE_TRACK = 128      # mm - distance between wheel centers

HEADING_CORRECTION = 360.5 #MEASURE TS

# motor ports
LEFT_PORT = Port.A
RIGHT_PORT = Port.B

# True = clockwise i think
LEFT_INVERTED = True
RIGHT_INVERTED = False

STRAIGHT_SPEED = 200    # mm/s - driving speed
STRAIGHT_ACCEL = 400    #acceleration
TURN_RATE = 150         # deg/s - turning speed
TURN_ACCEL = 300        #turn acceleration

SLOW_STRAIGHT_SPEED = 80
SLOW_TURN_RATE = 60

USE_GYRO = True


GRID = 500              # mm one grid square
INITIAL_DISTANCE = 310
ARC_RADIUS = 500        # mm arc radius for Q/E
UBANK_RADIUS = 250
DEPOSIT_RADIUS = 80    # mm smaller radius for g
DEPOSIT_ANGLE = 90      # degrees arc angle for deposit
SHIFT_BACK = 26
shift_back2 = 46

# per-move overhead (ms), doesn't scale with speed
COMMAND_OVERHEAD_MS = 22

# measured ms per move at base speed
TIME_ESTIMATES = {
    'I': 2064,   # start offset 310mm
    'W': 2965,   # fwd 1 square
    'S': 3045,   # back 1 square
    'A': 1100,   # turn left
    'D': 1095,   # turn right
    'Q': 4455,   # arc fwd left
    'E': 4405,   # arc fwd right
    'Z': 4500,   # arc back left
    'C': 4400,   # arc back right
    'P': 735,    # final back-up 60mm
    'R': 8000,   # u-turn left
    'Y': 7925,   # u-turn right
    'F': 7910,   # reverse u-turn left
    'H': 7910,   # reverse u-turn right
    'G': 3315,   # deposit right
    'L': 3400,   # deposit left
}



def create_robot():

    hub = TechnicHub()

    left_dir = Direction.COUNTERCLOCKWISE if LEFT_INVERTED else Direction.CLOCKWISE
    right_dir = Direction.COUNTERCLOCKWISE if RIGHT_INVERTED else Direction.CLOCKWISE

    left_motor = Motor(LEFT_PORT, left_dir)
    right_motor = Motor(RIGHT_PORT, right_dir)


    robot = DriveBase(
        left_motor,
        right_motor,
        wheel_diameter=WHEEL_DIAMETER,
        axle_track=AXLE_TRACK
    )

    robot.settings(
        straight_speed=STRAIGHT_SPEED,
        straight_acceleration=STRAIGHT_ACCEL,
        turn_rate=TURN_RATE,
        turn_acceleration=TURN_ACCEL,

    )
    robot.use_gyro
    if USE_GYRO:
        robot.use_gyro(True)
    hub.imu.settings(heading_correction = HEADING_CORRECTION)

    return hub, robot


def deposit_bottle(robot):
    robot.straight(30)
    robot.arc(DEPOSIT_RADIUS, angle=DEPOSIT_ANGLE)
    robot.arc(DEPOSIT_RADIUS, angle=-DEPOSIT_ANGLE)
    robot.straight(-30)

def deposit_bottle_left(robot):
    robot.straight(30)
    robot.arc(-DEPOSIT_RADIUS, angle=DEPOSIT_ANGLE)
    robot.arc(-DEPOSIT_RADIUS, angle=-DEPOSIT_ANGLE)
    robot.straight(-30)


def execute_path(robot, path):

    for cmd in path:

        if cmd in 'Ww':          # Forward one grid square
            robot.straight(GRID)

        elif cmd in 'Ii':        # Initial move forward
            robot.straight(INITIAL_DISTANCE)

        elif cmd in 'Ss':        # Backward one grid square
            robot.straight(-GRID)

        elif cmd in 'Aa':        # Turn left 90°
            robot.turn(-90, Stop.COAST_SMART)

        elif cmd in 'Dd':        # Turn right 90°
            robot.turn(90, Stop.COAST_SMART)

        elif cmd in 'Qq':        # Arc forward-left 90°
            robot.arc(-ARC_RADIUS, angle=90)

        elif cmd in 'Ee':        # Arc forward-right 90°
            robot.arc(ARC_RADIUS, angle=90)

        elif cmd in 'Zz':        # Arc backward-left 90°
            robot.arc(-ARC_RADIUS, angle=-90)

        elif cmd in 'Cc':        # Arc backward-right 90°
            robot.arc(ARC_RADIUS, angle=-90)

        elif cmd in 'Pp':        # Center ping (short reverse)
            robot.straight(-60.0)

        elif cmd in 'Rr':        # U-turn left
            robot.straight(250)
            robot.arc(-UBANK_RADIUS, angle=180)
            robot.straight(250)

        elif cmd in 'Yy':        # U-turn right
            robot.straight(250)
            robot.arc(UBANK_RADIUS, angle=180)
            robot.straight(250)

        elif cmd in 'Ff':        # Reverse U-turn left
            robot.straight(250)
            robot.arc(-UBANK_RADIUS, angle=-180)
            robot.straight(250)

        elif cmd in 'Hh':        # Reverse U-turn right
            robot.straight(250)
            robot.arc(UBANK_RADIUS, angle=-180)
            robot.straight(250)

        elif cmd in 'Ll':        # Deposit bottle left
            deposit_bottle_left(robot)

        elif cmd in 'Gg':        # Deposit bottle right
            deposit_bottle(robot)




def estimate_time(path):
    total = 0

    for cmd in path:
        key = cmd.upper()
        total += TIME_ESTIMATES.get(key, 0)
    return total


def calculate_speed_factor(path, target_time):
    est_ms = estimate_time(path)
    n_commands = len(path)

    total_overhead_s = n_commands * COMMAND_OVERHEAD_MS / 1000.0
    scalable_s = est_ms / 1000.0 - total_overhead_s

    if scalable_s <= 0 or target_time <= total_overhead_s:
        return 1.0

    factor = scalable_s / (target_time - total_overhead_s)
    return factor



timer = StopWatch()

def main():

    hub, robot = create_robot()



    factor = calculate_speed_factor(PATH, TARGET_TIME)
    print(f"Speed factor: {factor:.2f}")

    robot.settings(
    straight_speed=int(STRAIGHT_SPEED * factor),
    straight_acceleration=int(STRAIGHT_ACCEL * factor * factor),
    turn_rate=int(TURN_RATE * factor),
    turn_acceleration=int(TURN_ACCEL * factor * factor)
    )

    timer.reset()

    execute_path(robot, PATH)
    robot.stop()
    print(timer.time())


    hub.light.on(Color.GREEN)

    wait(3000)


main()
