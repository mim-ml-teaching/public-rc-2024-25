"""
Classic cart-pole system implemented by Rich Sutton et al.
Derived from http://incompleteideas.net/sutton/book/code/pole.c
"""
__copyright__ = "Copyright 2020, Microsoft Corp."


import math
import argparse
import random
from typing import Any
import cv2
import numpy as np
import control
import imageio

# Constants
GRAVITY = 9.8  # a classic...
CART_MASS = 0.31  # kg
POLE_MASS = 0.055  # kg
TOTAL_MASS = CART_MASS + POLE_MASS
POLE_HALF_LENGTH = 0.4 / 2  # half the pole's length in m
POLE_MASS_LENGTH = POLE_MASS * POLE_HALF_LENGTH
FORCE_MAG = 1.0
STEP_DURATION = 0.02  # seconds between state updates (20ms)
TRACK_WIDTH = 1.0  # m
FORCE_NOISE = 0.02  # % of FORCE_MAG


# Model parameters
class CartPoleModel:
    def __init__(self, initial_cart_position: float = 0, initial_pole_angle: float = 0):
        # cart position (m)
        self._cart_position = initial_cart_position

        # cart velocity (m/s)
        self._cart_velocity = 0

        # cart angle (rad)
        self._pole_angle = initial_pole_angle

        # pole angular velocity (rad/s)
        self._pole_angular_velocity = 0

        # pole position (m)
        self._pole_center_position = 0

        # pole velocity (m/s)
        self._pole_center_velocity = 0

    def step(self, command: float):
        # We are expecting the input command to be -1 or 1,
        # but we'll support a continuous action space.
        # Add a small amount of random noise to the force so
        # the policy can't succeed by simply applying zero
        # force each time.
        force = FORCE_MAG * command + random.uniform(-FORCE_NOISE, FORCE_NOISE)

        cosTheta = math.cos(self._pole_angle)
        sinTheta = math.sin(self._pole_angle)

        temp = (
            force + POLE_MASS_LENGTH * self._pole_angular_velocity**2 * sinTheta
        ) / TOTAL_MASS
        angularAccel = (GRAVITY * sinTheta - cosTheta * temp) / (
            POLE_HALF_LENGTH * (4.0 / 3.0 - (POLE_MASS * cosTheta**2) / TOTAL_MASS)
        )
        linearAccel = temp - (POLE_MASS_LENGTH * angularAccel * cosTheta) / TOTAL_MASS

        self._cart_position = self._cart_position + STEP_DURATION * self._cart_velocity
        self._cart_velocity = self._cart_velocity + STEP_DURATION * linearAccel

        self._pole_angle = (
            self._pole_angle + STEP_DURATION * self._pole_angular_velocity
        )
        self._pole_angular_velocity = (
            self._pole_angular_velocity + STEP_DURATION * angularAccel
        )

        # Use the pole center, not the cart center, for tracking
        # pole center velocity.
        self._pole_center_position = (
            self._cart_position + math.sin(self._pole_angle) * POLE_HALF_LENGTH
        )
        self._pole_center_velocity = (
            self._cart_velocity
            + math.sin(self._pole_angular_velocity) * POLE_HALF_LENGTH
        )

    def halted(self):
        # If the pole has fallen past 45 degrees, there's no use in continuing.
        return abs(self._pole_angle) >= math.pi / 4

    def state(self):
        return {
            "cart_position": self._cart_position,
            "cart_velocity": self._cart_velocity,
            "pole_angle": self._pole_angle,
            "pole_angular_velocity": self._pole_angular_velocity,
            "pole_center_position": self._pole_center_position,
            "pole_center_velocity": self._pole_center_velocity,
        }


FRAMES = []


def visualize(state):
    global FRAMES
    width = 512
    image = np.zeros((208, width, 3), np.uint8)
    cart_x = int(state["cart_position"] * 50) + width // 2
    angle = state["pole_angle"]  # in radians
    print(cart_x, angle)
    pole_x = int(math.sin(angle) * 100) + cart_x
    cv2.line(image, (cart_x, 150), (pole_x, 50), (255, 255, 255), 3)
    cv2.circle(image, (cart_x, 150), 6, (255, 255, 255), -1)
    cv2.imshow("pole", image)
    FRAMES.append(image)
    cv2.waitKey(1)


def print_state(state, name):
    print(
        name,
        f'{state["cart_position"]:.4f}, {state["cart_velocity"]:.4f}, {state["pole_angle"]:.4f}, {state["pole_angular_velocity"]:.4f}',
    )


def linearize(): # TODO maybe add some parameters to this function
    # TODO: implement this function
    A = np.array( #TODO: fill in the matrix
        [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ]
    )
    B = np.array( #TODO: fill in the matrix
        [
            [0],
            [0],
            [0],
            [0],
        ]
    )
    return A, B


def no_force():
    pole = CartPoleModel()
    state = pole.state()
    for i in range(10000):
        if pole.halted():
            print("halted")
            break
        pole.step(0)
        state = pole.state()
        visualize(state)
        print_state(state, "current")


def controlled():
    pole = CartPoleModel(initial_cart_position=-3)
    state = pole.state()
    A, B = linearize()
    Q = np.diag([0,0,0,0]) #TODO: fill in the matrix (vector)
    R = [0] # TODO: fill in the matrix (one value)
    K = control.lqr(A, B, Q, R)[0]
    print("K", K)
    for i in range(1300):
        force = -(
            K
            @ np.array(
                [
                    state["cart_position"],
                    state["cart_velocity"],
                    state["pole_angle"],
                    state["pole_angular_velocity"],
                ]
            )
        )[0]
        if force > 1:
            force = 1
        if force < -1:
            force = -1
        print("force", force)
        pole.step(force)
        state = pole.state()
        visualize(state)
        print_state(state, "current")
        if pole.halted():
            print("halted")
            break


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--no-force", action="store_true")
    ap.add_argument("--video-name", type=str)
    args = ap.parse_args()

    if args.no_force:
        no_force()
    else:
        controlled()
    if args.video_name:
        writer = imageio.get_writer(args.video_name, fps=20)
        for frame in FRAMES:
            writer.append_data(frame)
        writer.close()
