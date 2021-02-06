team = "BLUE"
# rcj_soccer_player controller - ROBOT B3

###### REQUIRED in order to import files from B1 controller
import sys
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
# You can now import scripts that you put into the folder with your
# robot B1 controller
if team == "BLUE":
    from rcj_soccer_player_b1 import rcj_soccer_robot, utils
else:
    from rcj_soccer_player_y1 import rcj_soccer_robot, utils
######

# Feel free to import built-in libraries
import math


class MyRobot(rcj_soccer_robot.RCJSoccerRobot):

    def evanMethod(self):
        moving = True
        shooting = False
        ball_moving = False
        ball_pos_last = [0,0]
        waiting_for_ball = False
        while self.robot.step(rcj_soccer_robot.TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()
                # Get the position of our robot
                robot_pos = data[self.name]
                # Get the position of the ball
                ball_pos = data['ball']

                if not (ball_pos['x'] == ball_pos_last[0]) and not (ball_pos['y']==ball_pos_last[0]):
                    ball_moving = True

                ball_change_x = ball_pos['x'] - ball_pos_last[0]
                ball_change_y = ball_pos['y'] - ball_pos_last[1]

                # print(ball_change_x, ball_change_y)

                robot_angle_2= robot_pos['orientation']

                # Get angle between the robot and the ball
                # and between the robot and the north
                ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)

                # Compute the speed for motors
                direction = utils.get_direction(ball_angle)


                # If the robot has the ball right in front of it, go forward,
                # rotate otherwise


                rx = robot_pos['x']
                ry = robot_pos['y']
                bx = ball_pos['x']
                by = ball_pos['y']

                ball_x_dist_from_goal = abs(bx+0.75)
                ball_y_dist_from_goal = by

                if (rx-0.05<bx):
                    moving = True
                    shooting = False

                # print("byd"+str(ball_y_dist_from_goal))

                # shotx = bx + 0.15/math.sqrt(ball_x_dist_from_goal**2+ball_y_dist_from_goal**2)*ball_x_dist_from_goal + 20*ball_change_x
                shotx = bx + 0.2/math.sqrt(ball_x_dist_from_goal**2+ball_y_dist_from_goal**2)*ball_x_dist_from_goal + 10*ball_change_x

                shoty = by + 0.2/math.sqrt(ball_x_dist_from_goal**2+ball_y_dist_from_goal**2)*ball_y_dist_from_goal + 23*ball_change_y

                if shoty > 0.65:
                    shoty = shoty = by + 0.2/math.sqrt(ball_x_dist_from_goal**2+ball_y_dist_from_goal**2)*ball_y_dist_from_goal+ 23*ball_change_y

                    if shoty > 0.65:
                        shoty = 0.64

                if shoty < -0.65:
                    shoty = -0.64

                if shotx > 0.75:
                    shotx = 0.74

                if shotx < -0.75:
                    shotx = -0.74

                # print(ball_x_dist_from_goal)

                # print(shotx, shoty)


                #if not on wall or not behind robot and compensate for movement, improve shooting mechanism (focusing on center of goal instead of ball)
                xtarget = shotx

                ytarget = shoty

                ball_pos = data['ball']
                xb = ball_pos['x']
                yb = ball_pos['y']

                b1 = data['B1']

                bX = b1['x']
                bY = b1['y']

                bdist = math.sqrt((bX-xb)**2+(bY-yb)**2)

                if bdist < 0.3:
                    xtarget = 0.2
                    ytarget = 0

                # TO KEEP THE ATTACKER ON OFFENSE
                # if xtarget > 0.1:
                #     xtarget = 0.1
                #     ytarget = 0

                def moveTo(x,y):
                    robot_angle_2= robot_pos['orientation']

                    angle = math.atan2(
                        y - robot_pos['y'],
                        x - robot_pos['x'],
                    )
                    if angle < 0:
                        angle = 2 * math.pi + angle
                    if robot_angle_2 < 0:
                        robot_angle_2 = 2 * math.pi + robot_angle_2
                    angle2 = math.degrees(angle + robot_angle_2)
                    angle2 -= 90
                    if angle2 > 360:
                        angle2 -= 360

                    d2 = utils.get_direction(angle2)
                    return d2


                if moving:
                    sp = moveTo(xtarget,ytarget)
                    if abs(xtarget-rx) < 0.02 and abs(ytarget-ry) < 0.02:
                        moving = False
                        # print("DONE")
                        shooting = True
                    direction = sp

                if shooting:
                    angle = math.atan2(
                        -robot_pos['y'],
                        -0.75 - robot_pos['x'],
                    )
                    if angle < 0:
                        angle = 2 * math.pi + angle
                    if robot_angle_2 < 0:
                        robot_angle_2 = 2 * math.pi + robot_angle_2
                    angle2 = math.degrees(angle + robot_angle_2)
                    angle2 -= 90
                    if angle2 > 360:
                        angle2 -= 360

                    direction = utils.get_direction(angle2)

                if direction == 0:
                    left_speed = -10
                    right_speed = -10
                else:
                    right_speed = direction * -10
                    left_speed = direction * 10

                robot_pos = data[self.name]
                orientation = robot_pos['orientation']
                xr = robot_pos['x']
                yr = robot_pos['y']

                # Get the position of the ball
                ball_pos = data['ball']
                xb = ball_pos['x']
                yb = ball_pos['y']

                b2 = data['B2']
                b1 = data['B1']

                # if utils.decideWho(robot_pos,b1,ball_pos) != "you" and xb < -0.2:
                #     left_speed = 0
                #     right_speed = 0

                # Set the speed to motors
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)

                ball_pos_last = [ball_pos['x'],ball_pos['y']]

    def run(self):
        self.evanMethod()

my_robot = MyRobot()
my_robot.run()
