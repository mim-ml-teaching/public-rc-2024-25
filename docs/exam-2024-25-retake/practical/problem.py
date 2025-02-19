import mujoco
import random
import numpy as np
import math
import cv2

from solution import Detector


def get_image(renderer, data):
    renderer.update_scene(data, camera="camera")
    img = renderer.render()
    return img


def random_point_on_circle(radius=1.0):
    # select random point on a circle (non-uniform)
    theta = random.uniform(0, 2 * math.pi)
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    return (x, y)


def task():
    """
    In this task you are supposed to read the code of the simulator and XML of the world

    There will be randomly ball or box placed between 0.7 and 1.3 distance
    from the initial position of the car

    You are supposed to read and understand code of simulation and
    fill the methods in the Detector class
    """
    x, y = random_point_on_circle(random.uniform(0.7, 1.3))
    world = open("car.xml").read()

    world = world.replace(
        "{{fill}}",
        f"""
            <body name="body" pos="{x} {y} 0.1">
                <geom name="body_geom" type="box" material="cube_markers_1" size="0.1 0.1 0.1"/>
            </body>
            """,
    )
    model = mujoco.MjModel.from_xml_string(world)
    renderer = mujoco.Renderer(model, height=480, width=640)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    data.actuator("forward").ctrl = 1
    data.actuator("turn").ctrl = 1
    detector = Detector(model, data)
    for step in range(100):
        for i in range(10):
            mujoco.mj_step(model, data)
        img = get_image(renderer, data)

        file_name = None
        # Uncomment the following line to save the images
        # file_name = f"img_{step}"
        #
        # If you want to draw something on the images,
        # the easiest way is to do it inside the Detector class
        detector.detect(img, file_name=file_name)


if __name__ == "__main__":
    task()
