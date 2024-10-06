---
title: Practical 3
---

We are going to use pybullet for this part of the exam.

There are three tasks. The whole practical exam is worth 18 points (5 + 8 + 5).

# Task 1

```python
import numpy as np
import cv2

import pybullet as p
import pybullet_data
import random

#p.connect(p.DIRECT)
p.connect(p.GUI)

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
    
def prepare_photo_for_task1(steps):
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
    return take_a_photo(car)
```

Function `prepare_world_for_task1` is used to prepare the photo for task 1. It returns a photo of a red ball after `steps` steps of simulation. `steps` should be between 1 and 2000.


Your task is to write a function `task1` that returns the width of the red ball in the photo after `steps` steps. You will be given two points if the function works correctly, and three additional points if the function is fast enough eg. works in constant time without performing simmulation.

It's ok if your solution just memoizes the results for some values of `steps` and returns one of them.

```python
def task1(steps):
    # your code here
    width = 0
    return width
```

# Task 2

```python
def drive(car, forward, direction, steps=250):
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

def task_2(seed=42):
    random.seed(seed)
    car = build_world_with_car()
    cylinder_x = 4 + 2 * random.random()
    s = p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            visualFramePosition=[cylinder_x, 0, 0],
            rgbaColor=[0, 0, 1, 1],
            radius=0.1,
            length=10,
        )
    p.createMultiBody(baseVisualShapeIndex=s)
    angle = random.random() * 2 * np.pi
    for i in range(4):
        angle += np.pi / 2
        pos = [cylinder_x + np.sin(angle), np.cos(angle), 0.3]
        ball = p.loadURDF("sphere2red.urdf", pos, globalScaling=0.3)

    for i in range(100):
        p.stepSimulation()

    # your code here
    # use only drive and take_a_photo functions for interaction with simulator
```

Your task is to finish the code above. Your task is to write a function that interacts with simulator using only `take_a_photo` and `drive` functions, and drives the car to the blue cylinder avoiding red balls (not moving any of them).
The car is close enough to the cylinder if the width of the cylinder in the photo taken from the car's camera is at least 100 pixels wide.

You will be given four points if the function works correctly, and four additional points if the function is fast enough eg. uses less than 15000 simulation steps.

# Task 3

```python
def drive_randomly(car, seed):
    random.seed(seed)
    if car is None:
        car = build_world_with_car()
    drive(car, random.random() < 0.5, random.random() * 2 - 1, steps=random.randint(1000, 2000))
    return p.getBasePositionAndOrientation(car)

def task_3(seed_list):
    # seed_list is a list of seeds for drive_randomly function eg. [1,2,1,4]
    poses = []
    for seed in seed_list:
        poses.append(drive_randomly(None, seed))
    
    car = build_world_with_car()
    for seed in seed_list:
        drive_randomly(car, seed)
    pose = p.getBasePositionAndOrientation(car)

    pose_approximated = (0, 0, 0), (0, 0, 0, 1)
    # your code here
    # pose_approximated should be as close as possible to pose
    return pose_approximated
```

Write a function that approximates the final `pose` of the car after driving and returns it. The should not interact with
simulator in any way, it should use `poses` list.

The task is worth 5 points.

