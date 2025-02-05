import mujoco
import random
import numpy as np
import math
import cv2
from typing import Tuple

from solution import Detector, DetectorPos


def get_image(renderer, data):
    renderer.update_scene(data, camera="camera")
    img = renderer.render()
    return img


def random_point_on_sphere(radius=1.0):
    theta = random.uniform(0, 2 * math.pi)
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    return (x, y)


def task_1():
    """
    In this task you are supposed to read the code of the simulator and XML of the world

    There will be randomly ball or box placed between 0.7 and 1.3 distance
    from the initial position of the car

    You are supposed to read and understand code of simulation and
    fill the methods in the Detector class
    """

    x, y = random_point_on_sphere(random.uniform(0.7, 1.3))
    geom_type = random.choice(["sphere", "box", ""])
    world = open("car_1.xml").read()

    if geom_type:
        world = world.replace(
            "{{fill}}",
            f"""
            <body name="body" pos="{x} {y} 0.1">
                <geom name="body_geom" type="{geom_type}" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
            </body>
            """,
        )
    else:
        world = world.replace("{{fill}}", "")
    model = mujoco.MjModel.from_xml_string(world)
    renderer = mujoco.Renderer(model, height=480, width=640)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    data.actuator("forward").ctrl = 1
    data.actuator("turn").ctrl = 1
    detector = Detector()
    for _ in range(93 * 20):
        for i in range(10):
            mujoco.mj_step(model, data)
        img = get_image(renderer, data)
        detector.detect(img)

    assert detector.result() == geom_type


def task_2():
    """
    In this task you are supposed to read the code of the simulator

    Car is placed at random position between -0.1 and 0.1 in x and y direction

    You are supposed to read and understand code of simulation and
    fill the methods in the DetectorPos class
    """

    x = random.uniform(-0.1, 0.1)
    y = random.uniform(-0.1, 0.1)
    world = open("car_2.xml").read()

    world = world.replace("{{x}}", str(x))
    world = world.replace("{{y}}", str(y))
    model = mujoco.MjModel.from_xml_string(world)
    renderer = mujoco.Renderer(model, height=480, width=640)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    detector = DetectorPos()
    img = get_image(renderer, data)
    detected_x, detected_y = detector.detect(img)
    assert abs(detected_x - x) < 0.02
    assert abs(detected_y - y) < 0.02


if __name__ == "__main__":
    task_1()
    task_2()
