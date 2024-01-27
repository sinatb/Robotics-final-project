from controller import Robot
from PID import PID

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

set_speed(160,160,160,160)

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

yaw_setpoint=-1

pitchPID = PID(1,0.1,5,goal=0.0)
rollPID = PID(1,0.1,5,goal=0.0)
throttlePID = PID(10,0.1,5,goal=1.0)
yawPID = PID(1,0.1,5,goal=float(yaw_setpoint))

targetX, targetY, target_altitude = 0.0, 0.0, 1.0

while (robot.step(TIME_STEP) != -1):

	led_state = int(robot.getTime()) % 2
	front_left_led.set(led_state)
	front_right_led.set(int(not(led_state)))

	roll = imu.getRollPitchYaw()[0] + PI / 2.0
	pitch = imu.getRollPitchYaw()[1]
	yaw = compass.getValues()[0]
	roll_acceleration = gyro.getValues()[0]
	pitch_acceleration = gyro.getValues()[1]
	
	xGPS = gps.getValues()[2]
	yGPS = gps.getValues()[0]
	zGPS = gps.getValues()[1]

	vertical_input = throttlePID(zGPS)
	yaw_input = yawPID(yaw)

	rollPID.goal = targetX
	pitchPID.goal = targetY
	
	roll_input = 10 * roll + roll_acceleration + rollPID(xGPS)
	pitch_input = 10 * pitch - pitch_acceleration + pitchPID(-yGPS)

	front_left_motor_input = 68.5 + vertical_input - roll_input - pitch_input + yaw_input
	front_right_motor_input = 68.5 + vertical_input + roll_input - pitch_input - yaw_input
	rear_left_motor_input = 68.5 + vertical_input - roll_input + pitch_input - yaw_input
	rear_right_motor_input = 68.5 + vertical_input + roll_input + pitch_input + yaw_input

	set_speed(front_left_motor_input, -front_right_motor_input, -rear_left_motor_input, rear_right_motor_input)


