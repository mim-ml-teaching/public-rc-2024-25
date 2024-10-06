import numpy as np2
import cv2

import numpy as np
import pybullet as p
import pybullet_data
import random
from math import pi, sin, cos

p.connect(p.DIRECT)
#p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

def build_world_with_car(pos=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))):
    p.resetSimulation()
    p.setGravity(0, 0, -10)
    p.loadURDF("plane.urdf")
    car = p.loadURDF("racecar/racecar.urdf")
    p.resetBasePositionAndOrientation(car, pos[0], pos[1])
    return car


def simulate_car(car, steeringAngle=0.2, targetVelocity=-2, steps=5000):
    wheels = [2, 3, 5, 7]
    steering = [4, 6]
    maxForce = 10
    for wheel in wheels:
        p.setJointMotorControl2(
            car,
            wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=targetVelocity,
            force=maxForce,
        )
    for steer in steering:
        p.setJointMotorControl2(
            car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle
        )
    for i in range(steps):
        p.stepSimulation()
    return p.getBasePositionAndOrientation(car)

TOTAL_STEPS = 0
def drive(car, forward, direction, steps=250):
    global TOTAL_STEPS
    TOTAL_STEPS += steps
    if forward:
        speed = 2
    else:
        speed = -2
    if direction < 0:
        steeringAngle = -0.45
    elif direction > 0:
        steeringAngle = 0.45
    else:
        steeringAngle = 0
    simulate_car(car, steeringAngle, speed, steps)


def take_a_photo(car, debug=False):
    pos = p.getBasePositionAndOrientation(car)
    orn = p.getQuaternionFromEuler([0, 0, 0])
    other_pos = [[20, 0, 0], orn]
    combined_pos = p.multiplyTransforms(pos[0], pos[1], other_pos[0], other_pos[1])
    pos = list(pos[0])
    pos[2] += 0.22
    _, _, rgb, _, _ = p.getCameraImage(
        640,
        640,
        viewMatrix=p.computeViewMatrix(pos, combined_pos[0], [0, 0, 1]),
        projectionMatrix=p.computeProjectionMatrixFOV(75, 1, 0.1, 10),
    )
    if p.isNumpyEnabled() == False:
        rgb = np.reshape(rgb, [640, 640, -1])
    return rgb


def width_of_ball(photo):
    photo = photo[0:400, :, :]
    #print(1.0 * photo[:, :, 2] < 60)
    #cv2.imshow("photo", 1.0 * (photo[:, :, 2] < 60))
    #cv2.waitKey(1)
    l = np.sum(photo[:, :, 2] < 60, axis=1)
    width = np.max(l)
    return width


def forward_distance(photo):
    width = width_of_ball(photo)
    correct = (
        (99, 2000),
        (85, 2500),
        (65, 3000),
        (47, 5000),
        (42, 6000),
        (39, 6400),
        (27, 9000),
        (25, 9500),
    )
    for i in range(len(correct)):
        if correct[i][0] == width:
            return correct[i][1]
        if correct[i][0] < width:
            d = (width - correct[i][0]) / (correct[i - 1][0] - correct[i][0])
            return int(correct[i][1] - d * (correct[i][1] - correct[i - 1][1]))


def ball_on_photo(photo):
    photo = photo[0:400, :, :]
    l = np.sum(photo[:, :, 2] < 60)
    return l > 100


def direction_of_ball(photo):
    photo = photo[0:400, :, :]
    hsv_image = cv2.cvtColor(photo, cv2.COLOR_RGB2HSV)

    lower1 = np.array([0, 100, 50])
    upper1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv_image, lower1, upper1)
    lower2 = np.array([160, 100, 50])
    upper2 = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv_image, lower2, upper2)

    mask = mask1 + mask2
    l = np.sum(mask > 100, axis=0)
    direction = 2 * np.argmax(l) / photo.shape[1] - 1
    return direction, max(l)


def direction_of_pole(photo):
    photo = photo[0:200, :, :]
    hsv_image = cv2.cvtColor(photo, cv2.COLOR_RGB2HSV)

    lower = np.array([180 / 2, 10, 10])
    upper = np.array([270 / 2, 255, 255])
    mask = cv2.inRange(hsv_image, lower, upper)
    width = np.sum(mask[0, :] / 255)
    first = np.argmax(mask[0, :])
    last = len(mask[0]) - np.argmax(mask[0, ::-1])
    cv2.waitKey(1)
    return (first + last) / 2, width


def prepare_world_for_task1(steps):
    assert steps > 0 and steps < 2000
    car = build_world_with_car()
    pos_1 = [2, 0, 1]
    ball = p.loadURDF("sphere2red.urdf", pos_1, globalScaling=0.3)    
    for i in range(100):
        p.stepSimulation()
    wheels = [2, 3, 5, 7]
    steering = [4, 6]
    maxForce = 10
    for wheel in wheels:
        p.setJointMotorControl2(
            car,
            wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=2,
            force=maxForce,
        )
    for steer in steering:
        p.setJointMotorControl2(
            car, steer, p.POSITION_CONTROL, targetPosition=0
        )
    target = 0.15
    for i in range(steps):
        p.stepSimulation()
        if (i + 100) // 200 == 0:
            for steer in steering:
                p.setJointMotorControl2(
                    car, steer, p.POSITION_CONTROL, targetPosition=target
                )
            target = -target

def gather_data():
    car = build_world_with_car()
    pos_1 = [2, 0, 1]
    ball = p.loadURDF("sphere2red.urdf", pos_1, globalScaling=0.3)    
    for i in range(100):
        p.stepSimulation()
    wheels = [2, 3, 5, 7]
    steering = [4, 6]
    maxForce = 10
    for wheel in wheels:
        p.setJointMotorControl2(
            car,
            wheel,
            p.VELOCITY_CONTROL,
            targetVelocity=2,
            force=maxForce,
        )
    for steer in steering:
        p.setJointMotorControl2(
            car, steer, p.POSITION_CONTROL, targetPosition=0
        )
    result = {}
    target = 0.15
    for i in range(2000):
        p.stepSimulation()
        if (i + 100) // 200 == 0:
            for steer in steering:
                p.setJointMotorControl2(
                    car, steer, p.POSITION_CONTROL, targetPosition=target
                )
            target = -target
        w = width_of_ball(take_a_photo(car))
        print(i, w)
        result[i] = w
    return result

def task_1():
    prepare = gather_data()
    print (prepare)

def task_2(seed=42):
    random.seed(seed)
    car = build_world_with_car()
    cylinder_x = 4 + 2 * random.random()
    #FIXME: remove
    cylinder_x = 4
    s = p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            visualFramePosition=[cylinder_x, 0, 0],
            rgbaColor=[0, 0, 1, 1],
            radius=0.1,
            length=10,
        )
    p.createMultiBody(baseVisualShapeIndex=s)
    angle = random.random() * 2 * np.pi
    # FIXME: remove
    angle = - np.pi / 4 / 3
    for i in range(4):
        angle += pi / 2
        pos = [cylinder_x + sin(angle), cos(angle), 0.3]
        ball = p.loadURDF("sphere2red.urdf", pos, globalScaling=0.3)

    for i in range(100):
        p.stepSimulation()
    proximity_widht = 100
    photo = take_a_photo(car)
    direction, width = direction_of_ball(photo)
    print(direction, width)
#    drive(car, False, 1, steps=700)
    if abs(direction) > 0.12: # drive forward
        print("drive forward")
        while True:
            drive(car, True, 0, steps=100)
            photo = take_a_photo(car)
            direction, width = direction_of_pole(photo)
            print(direction, width)
            if width > proximity_widht:
                break
    else:
        if direction < 0: # on the left
            drive(car, True, -1, steps=2000)
            drive(car, True, 0, steps=2000)
            drive(car, True, 1, steps=2000)
        else: # on the right
            drive(car, True, 1, steps=2000)
            drive(car, True, 0, steps=2000)
            drive(car, True, -1, steps=2000)
        while True:
            photo = take_a_photo(car)
            direction, width = direction_of_pole(photo)
            print(direction, width)
            if width > proximity_widht:
                break
            if direction < 300:
                drive(car, True, 1, steps=100)
            elif direction > 340:
                drive(car, True, -1, steps=100)
            else:
                drive(car, True, 0, steps=100)
        drive(car, True, 0, steps=1000)
#    drive(car, True, 1, steps=5600)
    print("TOTAL STEPS", TOTAL_STEPS)
    drive(car, False, 0, steps=1000)
    while True:
        p.stepSimulation()

def drive_randomly(car, seed):
    random.seed(seed)
    if car is None:
        car = build_world_with_car()
    drive(car, random.random() < 0.5, random.random() * 2 - 1, steps=random.randint(1000, 2000))
    return p.getBasePositionAndOrientation(car)

def task_3():
    poses = []
    for i in range(3):
        print(i)
        poses.append(drive_randomly(None, i))
    pose = poses[0]
    for i in range(1, len(poses)):
        pose = p.multiplyTransforms(pose[0], pose[1], poses[i][0], poses[i][1])
    print("RESULT", pose)

    car = build_world_with_car()
    for i in range(3):
        print(i)
        drive_randomly(car, i)
    print(p.getBasePositionAndOrientation(car))

if __name__ == "__main__":
    #task_1()
    task_2()
    #task_3()