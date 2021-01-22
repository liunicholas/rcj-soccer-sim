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

        INPOSITION = False
        CENTER = False
        TRYSCORE = False

        def getToPosition():
            if abs(xr+0.15)<=0.05 and abs(yr)<=0.05:
                if robot_pos['orientation'] < 1.5:
                    self.left_motor.setVelocity(15/robot_pos['orientation'])
                    self.right_motor.setVelocity(-15/robot_pos['orientation'])
                else:
                    self.left_motor.setVelocity(-15/robot_pos['orientation'])
                    self.right_motor.setVelocity(15/robot_pos['orientation'])

            else:
                robot_angle: float = robot_pos['orientation']

                # Get the angle between the robot and the ball
                angle = math.atan2(
                    0 - robot_pos['y'],
                    0 - robot_pos['x'],
                )

                if angle < 0:
                    angle = 2 * math.pi + angle

                if robot_angle < 0:
                    robot_angle = 2 * math.pi + robot_angle

                robot_center_angle = math.degrees(angle + robot_angle)

                direction = utils.get_direction(robot_center_angle)

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

        def tryToScore():
            self.left_motor.setVelocity(-10)
            self.right_motor.setVelocity(-10)

        def roam():
            self.left_motor.setVelocity(1)
            self.right_motor.setVelocity(1)

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

                # print(robot_pos['orientation'])

                # Get angle between the robot and the ball
                # and between the robot and the north
                ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)

                if orientation == 1.5 and abs(xr+0.15)<=0.05 and abs(yr)<=0.05:
                    INPOSITION = True
                else:
                    INPOSITION = False

                if abs(xb)<=0.05 and abs(yb)<=0.05:
                    CENTER = True
                else:
                    CENTER = False

                if INPOSITION and CENTER:
                    TRYSCORE = True

                print("in position: ", INPOSITION)
                print("center: ", CENTER)
                print("try score: ", TRYSCORE)

                if TRYSCORE:
                    tryToScore()
                elif CENTER:
                    getToPosition()
                elif INPOSITION:
                    roam()
                else:
                    getToPosition()

                # # Compute the speed for motors
                # direction = utils.get_direction(ball_angle)
                #
                # # If the robot has the ball right in front of it, go forward,
                # # rotate otherwise
                # if direction == 0:
                #     left_speed = -10
                #     right_speed = -10
                # elif direction == -1:
                #     left_speed = direction * 10
                #     right_speed = direction * -10
                # else:
                #     left_speed = direction * 10
                #     right_speed = direction * -10
                #
                # # Set the speed to motors
                # self.left_motor.setVelocity(left_speed)
                # self.right_motor.setVelocity(right_speed)

my_robot = MyRobot()
my_robot.run()
