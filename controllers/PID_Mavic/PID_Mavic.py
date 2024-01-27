from controller import Robot


robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())
PI = 3.1415926535897932384626433
# Helping Functions
def set_speed(v1,v2,v3,v4):
    # Get the motors of the robot
    frontLeftMotor = robot.getDevice('front left propeller')
    frontRightMotor = robot.getDevice('front right propeller')
    backLeftMotor = robot.getDevice('rear left propeller')
    backRightMotor = robot.getDevice('rear right propeller')
    # Set the speed of the motors
    frontLeftMotor.setVelocity(v1)
    frontRightMotor.setVelocity(v2)
    backLeftMotor.setVelocity(v3)
    backRightMotor.setVelocity(v4)
    return    

# Robot Setup

# Motors Setup
frontLeftMotor = robot.getDevice('front left propeller')
frontRightMotor = robot.getDevice('front right propeller')
backLeftMotor = robot.getDevice('rear left propeller')
backRightMotor = robot.getDevice('rear right propeller')

frontLeftMotor.setPosition(float('inf'))
frontRightMotor.setPosition(float('inf'))
backLeftMotor.setPosition(float('inf'))
backRightMotor.setPosition(float('inf'))

set_speed(0,0,0,0)

# Setting LED
front_left_led = robot.getDevice("front left led")
front_right_led = robot.getDevice("front right led")
# Setting gps
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)
# Setting imu
imu = robot.getDevice("inertial unit")
imu.enable(TIME_STEP)
# Setting compass
compass = robot.getDevice("compass")
compass.enable(TIME_STEP)
# Setting gyro
gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)

while robot.step(TIME_STEP) != -1:
    pass


