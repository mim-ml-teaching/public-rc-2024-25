import mujoco
import random
import numpy as np
from mujoco import viewer
import math
import cv2
from typing import Tuple


def get_image(renderer, data):
    renderer.update_scene(data, camera="camera")
    img = renderer.render()
    return img


class Detector:
    def __init__(self) -> None:
        pass

    def detect(self, img) -> None:
        pass

    def result(self) -> str:
        return "none"


class DetectorPos:
    def __init__(self) -> None:
        pass

    def detect(self, img) -> Tuple[float, float]:
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        # print(img.shape)
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define range of red color in HSV (adjust these values if needed)
        lower_red1 = np.array([0, 100, 100])  # Lower bound for red
        upper_red1 = np.array([10, 255, 255])  # Upper bound for red
        lower_red2 = np.array([160, 100, 100])  # Lower bound for red (wrap-around)
        upper_red2 = np.array([180, 255, 255])  # Upper bound for red (wrap-around)

        mask = cv2.inRange(hsv, lower_red1, upper_red1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        red_rectangles = []
        for contour in contours:
            # Approximate the contour to a polygon
            approx = cv2.approxPolyDP(
                contour, 0.02 * cv2.arcLength(contour, True), True
            )

            # Check if the polygon has 4 sides (rectangle) and is reasonably large
            if (
                len(approx) == 4 and cv2.contourArea(contour) > 1000
            ):  # Adjust area threshold as needed
                red_rectangles.append([i[0] for i in approx])

        if len(red_rectangles) < 2:
            return
        pix_x = float(red_rectangles[0][0][1])
        pix_y = float(red_rectangles[0][0][0])
        # return float(red_rectangles[0][0][0]), float(red_rectangles[0][0][1])
        # 332 -> 0
        # 334 -> 0.01
        return (pix_x - 332) / 200, (274 - pix_y) / 200
        points_2d = np.array(red_rectangles[0] + red_rectangles[1], dtype="double")

        size = 0.1
        points_3d = np.array(
            [
                [0.5 - size, 1, 0.12 - size],  # Point 1 (origin, or any logical point)
                [0.5 - size, 1, 0.12 + size],  # Point 2
                [0.5 + size, 1, 0.12 + size],  # Point 3
                [0.5 + size, 1, 0.12 - size],  # Point 4
                [-0.5 - size, 1, 0.12 - size],  # Point 1 (origin, or any logical point)
                [-0.5 - size, 1, 0.12 + size],  # Point 2
                [-0.5 + size, 1, 0.12 + size],  # Point 3
                [-0.5 + size, 1, 0.12 - size],  # Point 4
            ],
            dtype="double",
        )
        cx = 320
        cy = 240
        fx = 200
        fy = 200
        camera_matrix = np.array(
            [
                [fx, 0, cx],  # Focal length (x), principal point (x)
                [0, fy, cy],  # Focal length (y), principal point (y)
                [0, 0, 1],
            ],
            dtype="double",
        )
        dist_coeffs = np.zeros((5, 1))

        print(points_2d)
        _, rotation_vector, translation_vector = cv2.solvePnP(
            points_3d,
            points_2d,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        print(translation_vector)
        return -translation_vector[0][0], translation_vector[1][0]


DEBUG = True


def random_point_on_sphere(radius=1.0):
    theta = random.uniform(0, 2 * math.pi)
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    return (x, y)


def task_1():
    """
    In this task you are supposed to read the code of the simulator

    There will be randomly ball or box placed between 0.7 and 1.3 distance
    from the initial position of the car

    You are supposed to read and understand code of simulation and
    fill the methods in the Detector class
    """

    x, y = random_point_on_sphere(random.uniform(0.7, 1.3))
    geom_type = random.choice(["sphere", "box", ""])
    world = open("car_2.xml").read()

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
    view = viewer.launch_passive(model, data)
    data.actuator("forward").ctrl = 1
    data.actuator("turn").ctrl = 1
    detector = Detector()
    for _ in range(93 * 20):
        for i in range(10):
            mujoco.mj_step(model, data)
        if DEBUG:
            import time

            view.sync()
            time.sleep(0.1)
        img = get_image(renderer, data)
        detector.detect(img)

        if not view.is_running():
            break
    assert detector.result() == geom_type


# task_1()
def task_2(x, y):
    """
    In this task you are supposed to read the code of the simulator

    There will be randomly ball or box placed between 0.7 and 1.3 distance
    from the initial position of the car

    You are supposed to read and understand code of simulation and
    fill the methods in the Detector class
    """

    # x = random.uniform(-0.1, 0.1)
    # y = random.uniform(-0.1, 0.1)
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
    print(x, y, detected_x, detected_y)
    assert abs(detected_x - x) < 0.02
    assert abs(detected_y - y) < 0.02


for x in [-0.1, 0, 0.1]:
    for y in [-0.1, 0, 0.1]:
        task_2(x, y)
