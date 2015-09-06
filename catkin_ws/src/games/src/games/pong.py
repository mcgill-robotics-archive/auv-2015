# -*- coding: utf-8 -*-

"""McGill Robotics pong game."""

import rospy
import bitstring
import numpy as np
from serial import Serial
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

__author__ = "Anass Al-Wohoush"


class GameObject(object):

    """Base game object class."""

    def __init__(self, (x, y), (height, width), field, color):
        self.x = float(x)
        self.y = float(y)
        self.height = float(height)
        self.width = float(width)
        self.field = field
        self.color = color

    def draw(self, image):
        for x in range(int(self.x), int(self.x + self.width)):
            for y in range(int(self.y), int(self.y + self.height)):
                if 0 <= x < self.field.width and 0 <= y < self.field.height:
                    image[y, x] = self.color


class Field(object):

    """Game field class."""

    def __init__(self, height, width, players, blinky):
        self.height = height
        self.width = width
        self.paddles = []
        self.balls = None
        self.scores = [0, 0]
        self.bridge = CvBridge()
        self.players = players
        self.pub = rospy.Publisher("~image", Image, queue_size=1)
        self.blinky = BlinkyScreen(port=blinky) if blinky else None
        self.reset()

    def reset(self):
        rospy.logwarn("SCORE: %r", self.scores)
        center = (self.width / 2, self.height / 2)
        half_length = Paddle.HEIGHT / 2
        self.paddles = [
            Paddle((0, center[1] - half_length), self),
            Paddle((self.width - Paddle.WIDTH, center[1] - half_length), self)
        ]
        self.ball = Ball(
            ((self.width - Ball.WIDTH) / 2, (self.height - Ball.HEIGHT) / 2),
            self)
        if self.players >= 1:
            rospy.Subscriber("~player1", Int8, self.paddles[0].callback)
        if self.players == 2:
            rospy.Subscriber("~player2", Int8, self.paddles[1].callback)
        self.update()

    def draw(self):
        image = np.zeros((self.height, self.width, 3), np.uint8)
        for paddle in self.paddles:
            paddle.draw(image)

        self.ball.draw(image)

        try:
            self.pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

        if self.blinky:
            self.blinky.draw(self.paddles, self.ball)

    def update(self):
        self.draw()

        if self.players == 0:
            self.paddles[0].next()
        if self.players < 2:
            self.paddles[1].next()
        self.ball.next()


class Paddle(GameObject):

    """Paddle class."""

    HEIGHT = 3
    WIDTH = 1
    STEP_SIZE = 1.0
    COLOR = (0, 0, 255)

    def __init__(self, (x, y), field):
        self._counter = 0
        super(Paddle, self).__init__(
            (x, y),
            (Paddle.HEIGHT, Paddle.WIDTH),
            field, Paddle.COLOR)

    def callback(self, msg):
        if msg.data > 0:
            self.up()
        elif msg.data < 0:
            self.down()

    def up(self):
        self.y = max(self.y - Paddle.STEP_SIZE, 0.0)
        rospy.logdebug("PADDLE: %f, %f", self.x, self.y)

    def down(self):
        self.y = min(
            self.y + Paddle.STEP_SIZE,
            self.field.height - Paddle.HEIGHT)
        rospy.logdebug("PADDLE: %f, %f", self.x, self.y)

    def hit(self, ball, (dx, dy)):
        # Verify ball is within bounds. If so, determine new ball location.
        if self.y <= ball.y < self.y + self.height:
            if ((ball.moving_left and ball.x + dx <= self.x + self.width) or
                    (ball.moving_right and ball.x + dx >= self.x)):
                # Flip the ball's orientation.
                if ((self.x <= ball.x and ball.moving_left) or
                        (ball.x <= self.x + self.width and ball.moving_right)):
                    ball.orientation = np.pi - ball.orientation
                ball.orientation %= 2 * np.pi

                # Increase the ball's speed.
                ball.speed = min(Ball.SPEED_STEP + ball.speed, Ball.MAX_SPEED)

                # Determine how the orientation should change depending on how
                # far away from the center of the paddle the ball has been hit.
                # Add random noise for effect.
                paddle_center = self.y + self.height / 2
                distance_from_center = ball.y + ball.height / 2 - paddle_center
                dt = distance_from_center / paddle_center * Ball.MAX_ANGLE
                ball.orientation += dt + np.random.random() / 1000
                if ball.moving_right and ball.orientation < np.pi:
                    ball.orientation = min(
                        ball.orientation,
                        Ball.MAX_ANGLE)
                elif ball.moving_right and ball.orientation > np.pi:
                    ball.orientation = max(
                        ball.orientation,
                        -Ball.MAX_ANGLE)
                elif ball.moving_left and ball.orientation < np.pi:
                    ball.orientation = max(
                        ball.orientation,
                        np.pi - Ball.MAX_ANGLE)
                elif ball.moving_left and ball.orientation > np.pi:
                    ball.orientation = min(
                        ball.orientation,
                        np.pi + Ball.MAX_ANGLE)

                # Determine next ball movement.
                ball.x += ball.speed * np.cos(ball.orientation)
                ball.y += ball.speed * np.sin(ball.orientation)

                return True

        return False

    @property
    def is_left_paddle(self):
        return self.x == 0

    @property
    def is_right_paddle(self):
        return not self.is_left_paddle

    def next(self):
        self._counter += 1
        if not self._counter == 5:
            return

        self._counter = 0
        ball = self.field.ball

        if ((self.is_left_paddle and ball.moving_right) or
                (self.is_right_paddle and ball.moving_left)):
            field_center = self.field.height / 2
            paddle_center = self.y + Paddle.HEIGHT / 2
            if paddle_center > field_center:
                self.up()
            elif paddle_center < field_center:
                self.down()
            return

        dy = ball.speed * np.sin(ball.orientation)

        next_ball_center = ball.y + dy + Ball.HEIGHT / 2
        if self.y + 1 < next_ball_center < self.y + Paddle.HEIGHT - 1:
            pass
        elif next_ball_center > self.y + Paddle.HEIGHT / 2:
            self.down()
        elif next_ball_center < self.y + Paddle.HEIGHT / 2:
            self.up()


class Ball(GameObject):

    """Ball class."""

    HEIGHT = 1
    WIDTH = 1
    START_SPEED = 0.15
    SPEED_STEP = 0.015
    MAX_SPEED = 0.95
    COLOR = (255, 255, 255)
    MAX_ANGLE = np.pi / 4

    def __init__(self, (x, y), field):
        self.orientation = np.pi
        self.speed = Ball.START_SPEED
        super(Ball, self).__init__(
            (x, y),
            (Ball.HEIGHT, Ball.WIDTH),
            field, Ball.COLOR)

    @property
    def moving_left(self):
        return np.pi / 2 <= self.orientation <= 3 * np.pi / 2

    @property
    def moving_right(self):
        return not self.moving_left

    def next(self):
        rospy.logdebug("BALL: %f, %f, %f", self.x, self.y, self.orientation)

        # Determine next ball movement.
        dx = self.speed * np.cos(self.orientation)
        dy = self.speed * np.sin(self.orientation)

        # Verify if score. If so, adjust score and reset field.
        if self.x < 0:
            self.x = 0
            self.field.draw()
            rospy.sleep(2)
            self.field.scores[1] += 1
            self.field.reset()
            self.field.ball.orientation = np.pi
            return
        if self.x >= self.field.width:
            self.x = self.field.width - 1
            self.field.draw()
            rospy.sleep(2)
            self.field.scores[0] += 1
            self.field.reset()
            self.field.ball.orientation = 0.0
            return

        # Verify if upper or lower walls have been hit.
        if self.y + dy <= 0 or self.y + dy >= self.field.height:
            # If so, flip the orientation.
            self.orientation = 2 * np.pi - self.orientation
            self.orientation %= 2 * np.pi
            self.x += dx
            self.y += dy
            return

        # Verify if the ball hit the paddle.
        if self.moving_left and self.field.paddles[0].hit(self, (dx, dy)):
            return
        elif self.moving_right and self.field.paddles[1].hit(self, (dx, dy)):
            return

        # Move the ball normally otherwise.
        self.x += dx
        self.y += dy


class BlinkyScreen(object):

    """BlinkyTape screen."""

    def __init__(self, port):
        self.port = port
        self.serial = Serial(port, 115200)

    def draw(self, paddles, ball):
        message = bitstring.pack(
            "0xFF, uint:8, uint:8, uint:8, uint:8",
            int(paddles[0].y), int(paddles[1].y),
            int(ball.x), int(ball.y))
        self.serial.write(message.tobytes())
        self.serial.flush()


def play(height, width, players, blinky):
    if not (0 <= players <= 2):
        rospy.logfatal("Number of players can only be 0, 1 or 2")
        return

    field = Field(height=height, width=width, players=players, blinky=blinky)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        field.update()
        rate.sleep()
