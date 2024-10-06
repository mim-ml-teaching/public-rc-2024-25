import mujoco
import time
import random
import numpy as np
import mujoco
from mujoco import viewer
import cv2

### TODO: Add your code here ###

model = mujoco.MjModel.from_xml_path("car.xml")
renderer = mujoco.Renderer(model, height=480, width=640)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
viewer = viewer.launch_passive(model, data)


def get_image():
    renderer.update_scene(data, camera="camera1")
    img = renderer.render()
    return img


def check_ball(seed=1337) -> bool:
    random.seed(seed)
    steps = random.randint(0, 500)
    data.actuator("turn 1").ctrl = 1
    for _ in range(steps):
        mujoco.mj_step(model, data)
    data.actuator("turn 1").ctrl = 0
    for _ in range(1000):
        mujoco.mj_step(model, data)
    # TODO: Add your code here


def drive_to_ball_1(seed=1337):
    random.seed(seed)
    steps = random.randint(0, 2500)
    data.actuator("forward 2").ctrl = -1
    for _ in range(steps):
        mujoco.mj_step(model, data)
    data.actuator("forward 2").ctrl = 0
    for _ in range(1000):
        mujoco.mj_step(model, data)
    # TODO Add your code here


def drive_to_ball_2(seed=1337):
    random.seed(seed)
    steps = random.randint(0, 2500)

    data.actuator("turn 2").ctrl = 1
    for _ in range(steps):
        mujoco.mj_step(model, data)
    data.actuator("turn 2").ctrl = 0
    for _ in range(1000):
        mujoco.mj_step(model, data)

    # TODO Add your code here
