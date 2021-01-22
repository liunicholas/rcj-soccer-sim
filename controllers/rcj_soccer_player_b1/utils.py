def get_direction(ball_angle: float) -> int:
    """Get direction to navigate robot to face the ball

    Args:
        ball_angle (float): Angle between the ball and the robot

    Returns:
        int: 0 = forward, -1 = right, 1 = left
    """
    if ball_angle >= 345 or ball_angle <= 15:
        return 0
    return -1 if ball_angle < 180 else 1

def getPointAngle(orientation, ball_pos: dict, pointX, pointY) -> Tuple[float, float]:
    robot_angle: float = orientation

    # Get the angle between the robot and the ball
    angle = math.atan2(
        pointY - robot_pos['y'],
        pointX - robot_pos['x'],
    )

    if angle < 0:
        angle = 2 * math.pi + angle

    if robot_angle < 0:
        robot_angle = 2 * math.pi + robot_angle

    robotPointAngle = math.degrees(angle + robot_angle)

    # Axis Z is forward
    # TODO: change the robot's orientation so that X axis means forward
    robotPointAngle -= 90
    if robotPointAngle > 360:
        robotPointAngle -= 360

    return robotPointAngle, robot_angle
