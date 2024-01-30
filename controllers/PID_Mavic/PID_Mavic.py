from controller import Robot
import numpy as np
import cv2
import tensorflow as tf

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic(Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0  # P constant of the vertical PID.
    K_ROLL_P = 50.0  # P constant of the roll PID.
    K_PITCH_P = 30.0  # P constant of the pitch PID.
    K_ROLL_I = 1.0
    K_PITCH_I = 1.0
    K_ROLL_D = 3.0
    K_PITCH_D = 3.0
    INTEGRAL_LIMIT = 0.1

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1

    PATH = ['photo0.jpg','photo1.jpg','photo2.jpg','photo3.jpg','photo4.jpg']

    # Precision between the target position and the robot position in meters
    target_precision = 0.4

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Get and enable devices.
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)
        self.fl_led = self.getDevice("front left led")
        self.fr_led = self.getDevice("front right led")


        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0.7)
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.probs = [-1.0] * 5
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0
        self.roll_integral = 0.0
        self.pitch_integral = 0.0
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        self.is_landing = False

        self.imgs = [None,None,None,None,None]
    def crop_image(self,image, left=44, right=2, top=14, bottom=74):
        height, width, _ = image.shape
        cropped = image[top:height - bottom, left:width - right, :]
        return cropped
    def is_gray(self,r, g, b):
        if r == g == b:
            return True

        tolerance = 20
        if abs(r - g) <= tolerance and abs(g - b) <= tolerance and abs(b - r) <= tolerance:
            return True

        return False
    def add_border(self,image, border_size=10):
        height, width = image.shape[:2]
        bordered_image = cv2.rectangle(
            image,
            (0, 0),
            (width - 1, height - 1),
            (0, 0, 0),
            border_size
        )
        return bordered_image
    def crop(self,path):
        image = cv2.imread(path)
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                r, g, b = image[i][j]
                if b > 80 and not self.is_gray(r, g, b):
                    image[i][j] = 140

        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                r, g, b = image[i][j]
                r = int(r)
                g = int(g)
                b = int(b)
                if (r + g + b) // 3 < 70:
                    image[i][j] = 0
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                r, g, b = image[i][j]
                r = int(r)
                g = int(g)
                b = int(b)
                avg = (r + g + b) // 3
                if avg > 140:
                    image[i][j] = 10
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                r, g, b = image[i][j]
                r = int(r)
                g = int(g)
                b = int(b)
                avg = (r + g + b) // 3
                if avg > 70:
                    image[i][j] = int(min(avg + 100, 200))
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        image = cv2.threshold(gray, 70, 255, cv2.THRESH_OTSU)[1]
        image = cv2.medianBlur(image, 11)
        contours, h = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts = []
        m = -1
        min_x = 100000000
        min_y = 100000000
        max_x = -1
        max_y = -1
        for c in contours:
            area = cv2.contourArea(c)
            m = max(m, area)
            if 2000 < area < 60000:
                for cc in c:
                    x, y = cc.T
                    x = x[0]
                    y = y[0]
                    min_x = min(min_x, x)
                    min_y = min(min_y, y)
                    max_x = max(max_x, x)
                    max_y = max(max_y, y)
                cnts.append(c)
        if abs(min_x - max_x) < 60:
            max_x += 100
            min_x += 10
        original = cv2.imread(path)
        cropped = original[min_y:max_y, min_x:max_x]
        return cropped
    def black_back(self,image, thresh=95):
      gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
      gray[gray < thresh] = 0
      return gray
    def cnn(self, photo, class_pred):
        model = tf.keras.models.load_model(r'D:\works\study\Term-7\Robotics\Robotics-final-project\controllers\PID_Mavic\sag_model.h5')
        predictions = model.predict(photo.reshape(-1, 28,28,1))
        return predictions[0][class_pred]

    def pre_process(self,num):
        self.imgs[num] = cv2.imread(self.PATH[num])
        self.imgs[num] = self.crop_image(self.imgs[num])
        self.imgs[num] = self.black_back(self.add_border(self.crop(self.PATH[num])))
        cv2.imshow("image",self.imgs[num])
        cv2.waitKey()
        self.imgs[num] = cv2.resize(self.imgs[num], (28, 28))

    def set_position(self, pos):
        self.current_pose = pos

    def set_probs_index(self,index):
        rets = [-1,-1,0,1,-1,-1,2,-1,3,4]
        return rets[index]
    def set_waypoint_index(self,index):
        rets = [0,2,4,6,8]
        return rets[index]
    def check_position(self, yaw, target_index, position):

        # retruns bool
        if target_index == 2:
            return (-4.8 < position[0] < -4.4) and (3.9 < position[1] < 4.2) and (3.10 < yaw < 3.14)
        elif target_index == 3:
            return (-2.7 < position[0] < -2.5) and (-2.5 < position[1] < -2.3) and (-3.12<yaw<-3.10)
        elif target_index == 6:
            return (3.2 < position[0] < 3.4) and (-3.7 < position[1] < -3.5) and (-3.13<yaw<-3.11)
        elif target_index == 8:
            return (5.20 < position[0] < 5.40) and (-0.5 < position[1] < -0.2) and (-3.13<yaw<-3.11)
        elif target_index == 9:
            return (2.4 < position[0] < 2.5) and (5.4 < position[1] < 5.5) and (-3.13<yaw<-3.11)

    def move_to_target(self, yaw, waypoints, verbose_movement=False, verbose_target=False):
        if self.target_position[0:2] == [0, 0]:  # Initialization
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        if self.check_position(yaw, self.target_index, self.current_pose[0:2]):
            if self.probs[self.set_probs_index(self.target_index)] == -1 :
                name = "photo" + str(self.set_probs_index(self.target_index)) + ".jpg"
                self.camera.saveImage(name, 100)
                self.pre_process(self.set_probs_index(self.target_index))
                self.probs[self.set_probs_index(self.target_index)] = self.cnn(self.imgs[self.set_probs_index(self.target_index)],4)

        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in
                zip(self.target_position, self.current_pose[0:2])]):
            if not self.is_landing:
                self.target_index += 1
            if self.target_index > len(waypoints) - 1:
                self.target_index = self.set_waypoint_index(np.argmax(self.probs))
                self.is_landing = True
                self.fl_led.set(1)
                self.fr_led.set(1)
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                print("Target reached! New target: ",
                      self.target_position[0:2])

        # This will be in ]-pi;pi]
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1], self.target_position[0] - self.current_pose[0])
        # This is now in ]-2pi;2pi[
        angle_left = self.target_position[2] - self.current_pose[5]
        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if angle_left > np.pi:
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        # non-proportional and decreasing function
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + (
                    (self.target_position[1] - self.current_pose[1]) ** 2))
            print("remaning angle: {:.4f}, remaning distance: {:.4f}".format(
                angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance

    def run(self):
        t1 = self.getTime()
        t_altitude = self.getTime()

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        # Specify the patrol coordinates
        waypoints = [[-4, 4], [-5, 4]
                    ,[-1,-1.6],[-3, -2], 
                     [4,-1.5],[3, -3.5],
                     [7,0],[5, 0],
                     [4,5],[2, 5]]
        # target altitude of the robot in meters
        self.target_altitude = 3

        while self.step(self.time_step) != -1:

            if self.target_altitude < 0.4:
                break
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])
            if self.is_landing and self.getTime() - t_altitude > 3 :
                t_altitude = self.getTime()
                self.target_altitude -= 0.1
                
            if altitude > self.target_altitude - 1:
                # as soon as it reach the target altitude, compute the disturbances to go to the given waypoints.
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(
                        yaw,
                        waypoints)
                    t1 = self.getTime()

            roll_error = clamp(roll, -1, 1)
            pitch_error = clamp(pitch, -1, 1)

            self.pitch_integral += pitch_error
            self.pitch_integral = clamp(self.pitch_integral, -self.INTEGRAL_LIMIT, self.INTEGRAL_LIMIT)
            self.roll_integral += roll_error
            self.roll_integral = clamp(self.roll_integral, -self.INTEGRAL_LIMIT, self.INTEGRAL_LIMIT)

            roll_input = self.K_ROLL_P * roll_error \
                         + self.K_ROLL_I * self.roll_integral \
                         + self.K_ROLL_D * (roll_error - self.prev_roll) \
                         + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * pitch_error \
                          + self.K_PITCH_I * self.pitch_integral \
                          + self.K_PITCH_D * (pitch_error - self.prev_pitch) \
                          + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)

            self.prev_roll = roll_error
            self.prev_pitch = pitch_error


# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5
robot = Mavic()
robot.run()

robot.front_left_motor.setVelocity(0)
robot.front_right_motor.setVelocity(0)
robot.rear_left_motor.setVelocity(0)
robot.rear_right_motor.setVelocity(0)
