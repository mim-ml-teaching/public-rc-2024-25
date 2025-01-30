import mujoco
import time
import random
import numpy as np
import mujoco
from mujoco import viewer
import cv2

model = mujoco.MjModel.from_xml_path("car.xml")
renderer = mujoco.Renderer(model, height=480, width=640)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
viewer = viewer.launch_passive(model, data)


def is_ball(mask):
    sums = np.sum(mask, axis=0)
    sums = sums[sums > 10]
    return len(sums[sums == sums.max()]) / len(sums) < 0.5


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

    best = 0
    for step in range(500):
        data.actuator("turn 1").ctrl = -1
        mujoco.mj_step(model, data)
        # viewer.sync()
        img = get_image()
        img = img[:, 200 : 640 - 200, :3]
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # cv2.imwrite("image.png", img)

        # lower mask (0-10)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

        # join my masks
        mask = mask0 + mask1

        #    print(step, np.sum(mask))
        if np.sum(mask) > best:
            best_result = is_ball(mask)
            best = np.sum(mask)
    return best_result


# print(check_ball())


def drive_to_ball_1(seed=1337):
    random.seed(seed)
    steps = random.randint(0, 2500)
    data.actuator("forward 2").ctrl = -1
    for _ in range(steps):
        mujoco.mj_step(model, data)
    data.actuator("forward 2").ctrl = 0
    for _ in range(1000):
        mujoco.mj_step(model, data)

    data.actuator("turn 1").ctrl = -1
    for i in range(100):
        mujoco.mj_step(model, data)
        viewer.sync()
    data.actuator("turn 1").ctrl = 0
    data.actuator("forward 2").ctrl = 1
    for i in range(10000):
        mujoco.mj_step(model, data)
        viewer.sync()
        if i % 100 == 0:
            print(data.body("car 2").xpos)
            print(data.body("target-box").xpos)
            dist = np.linalg.norm(
                data.body("car 2").xpos - data.body("target-box").xpos
            )
            print(dist)
            img = get_image()
            if img[280:300, 280:300, 0].sum() > 200000:
                data.actuator("forward 2").ctrl = 0
                break
    for i in range(100):
        mujoco.mj_step(model, data)
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()


def drive_to_ball_2(seed=1337):
    random.seed(seed)
    steps = random.randint(0, 2500)

    data.actuator("turn 2").ctrl = 1
    for _ in range(steps):
        mujoco.mj_step(model, data)
        viewer.sync()
    data.actuator("turn 2").ctrl = 0
    for _ in range(1000):
        mujoco.mj_step(model, data)

    data.actuator("turn 1").ctrl = -1
    for i in range(75):
        mujoco.mj_step(model, data)
        viewer.sync()
    data.actuator("turn 1").ctrl = 0
    for i in range(1000):
        mujoco.mj_step(model, data)
    #        viewer.sync()
    img = get_image()
    img = img[280:300, 220:300, :]
    cv2.imwrite("pattern.png", img)
    data.actuator("turn 2").ctrl = 1
    for i in range(1000):
        mujoco.mj_step(model, data)
        viewer.sync()
        # print(data.body("target-ball").xpos)

        cur_img = get_image()
        cur_img = cur_img[280:300, 220:300, :]
        print(i, np.sum(np.abs(cur_img - img)))
        if i > 200 and np.sum(np.abs(cur_img - img)) < 150000:
            data.actuator("turn 2").ctrl = 0
            break
    data.actuator("turn 2").ctrl = -0.1
    for i in range(1000):
        mujoco.mj_step(model, data)
        viewer.sync()

        cur_img = get_image()
        cur_img = cur_img[280:300, 220:300, :]
        print(i, np.sum(np.abs(cur_img - img)))
        if i > 200 and np.sum(np.abs(cur_img - img)) < 130000:
            data.actuator("turn 2").ctrl = 0
            break
    data.actuator("forward 2").ctrl = 1
    for i in range(900):
        mujoco.mj_step(model, data)
        viewer.sync()
    data.actuator("forward 2").ctrl = 0
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()


drive_to_ball_1()
