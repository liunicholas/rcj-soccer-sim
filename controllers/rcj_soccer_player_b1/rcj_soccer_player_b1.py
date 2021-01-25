team = "BLUE"
# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math
from time import sleep

# You can also import scripts that you put into the folder with controller
import rcj_soccer_robot
import utils

class MyRobot(rcj_soccer_robot.RCJSoccerRobot):
    def run(self):

        BLOCK = False
        SPOTONE = True
        xbOLD = 0
        ybOLD = 0

        spotX = 0.6
        spotY = 0.2

        while self.robot.step(rcj_soccer_robot.TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()

                # Get the position of our robot
                robot_pos = data[self.name]
                orientation = robot_pos['orientation']
                xr = robot_pos['x']
                yr = robot_pos['y']

                # Get the position of the ball
                ball_pos = data['ball']
                xb = ball_pos['x']
                yb = ball_pos['y']

                b2 = data['B2']
                b3 = data['B3']

                if abs(xr-spotX)<=0.05 and abs(yr-spotY)<=0.05:
                    SPOTONE = False

                if abs(xr-spotX)<=0.05 and abs(yr+spotY)<=0.05:
                    SPOTONE = True

                if team != "BLUE":
                    if (xb-xbOLD) < 0 and xb < 0.1:
                        BLOCK = True
                    else:
                        BLOCK = False
                else:
                    if (xb-xbOLD) > 0 and xb > 0.1:
                        BLOCK = True
                    else:
                        BLOCK = False

                if BLOCK:
                    # Get angle between the robot and the ball
                    # and between the robot and the north
                    ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)

                    # Compute the speed for motors
                    direction = utils.get_direction(ball_angle)

                    # If the robot has the ball right in front of it, go forward,
                    # rotate otherwise
                    if direction == 0:
                        left_speed = -10
                        right_speed = -10
                    elif direction == -1:
                        left_speed = direction * 10
                        right_speed = direction * -10
                    else:
                        left_speed = direction * 10
                        right_speed = direction * -10

                    # Set the speed to motors
                    self.left_motor.setVelocity(left_speed)
                    self.right_motor.setVelocity(right_speed)
                elif SPOTONE:
                    robotPointAngle, robot_angle = utils.getPointAngle(orientation, xr, yr, spotX, spotY)

                    direction = utils.get_direction(robotPointAngle)

                    # If the robot has the ball right in front of it, go forward,
                    # rotate otherwise
                    if direction == 0:
                        left_speed = -10
                        right_speed = -10
                    elif direction == -1:
                        left_speed = direction * 10
                        right_speed = direction * -10
                    else:
                        left_speed = direction * 10
                        right_speed = direction * -10

                    # Set the speed to motors
                    self.left_motor.setVelocity(left_speed)
                    self.right_motor.setVelocity(right_speed)

                else:
                    robotPointAngle, robot_angle = utils.getPointAngle(orientation, xr, yr, spotX, spotY*-1)

                    direction = utils.get_direction(robotPointAngle)

                    # If the robot has the ball right in front of it, go forward,
                    # rotate otherwise
                    if direction == 0:
                        left_speed = -10
                        right_speed = -10
                    elif direction == -1:
                        left_speed = direction * 10
                        right_speed = direction * -10
                    else:
                        left_speed = direction * 10
                        right_speed = direction * -10

                    # Set the speed to motors
                    self.left_motor.setVelocity(left_speed)
                    self.right_motor.setVelocity(right_speed)

                xbOLD = xb
                yBOLD = yb


my_robot = MyRobot()
my_robot.run()
