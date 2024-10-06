# Exam Task - car

This task is worth 15 points, submit through the moodle.

# General

Utilize the provided `car.xml` file, which contains two cars, one equipped with an artificial camera, to complete three tasks:

1. Check, using artificial camera, if there is a ball or a box in the center of the world - 3 points.

2. Drive one car to the box in the center, using the camera on the second car (simple version, straight line is enough) - 5 points.

3. Drive one car to the box in the center, using the camera on the second car, in this version you know initial position of the first car, but you don't know its rotation - 7 points.

In all of the tasks **you should not read directly state of the simulator**.

## Submission Instructions

You should fill `stub_ex_23_1.py` file with your code for each task. You can also send some additional files, if you think they are necessary and send a `zip` file.


# Details

## General

Your code should only access information from the simulator through the `get_image()` function. You can also analyze the `car.xml` file to gain further insights into the scene.

```python
def get_image():
    renderer.update_scene(data, camera="camera1")
    img = renderer.render()
    return img
```

Your code should utilize the following functions to control the car and the simulator:

- `data.actuator("...").ctrl`
- `mujoco.mj_step()`

Please refrain from directly accessing the simulator's internal state.

Each task will commence with a random seed. Your code will be evaluated using various seed values to ensure its robustness.

## Task 1 - 3 points

At the begining car 1 is rotated randomly using the following code. 

```python
def check_ball(seed=1337) -> bool:
    random.seed(seed)
    steps = random.randint(0, 500)
    data.actuator("turn 1").ctrl = 1
    for _ in range(steps):
        mujoco.mj_step(model, data)
    data.actuator("turn 1").ctrl = 0
    for _ in range(1000):
        mujoco.mj_step(model, data)
    # your code here
```

You should, in no more than 1000 `mj_step` steps, find out if there is a ball or a box in the center of the world. You can use the `get_image()` function to get the image from the camera. The function should return `True` if there is a ball in the center of the world and `False` otherwise.

You can inspect the `car.xml` file to find out how the ball looks like (it is commented out) and how the box looks like.

## Task 2 - 5 points

Note that in this task, car 2 drives in a straight line. At the beginning, car 2 drives randomly (backwards only) using the following code.

```python
def drive_to_ball_1(seed=1337):
    random.seed(seed)
    steps = random.randint(0, 2500)
    data.actuator("forward 2").ctrl = -1
    for _ in range(steps):
        mujoco.mj_step(model, data)
    data.actuator("forward 2").ctrl = 0
    for _ in range(1000):
        mujoco.mj_step(model, data)
    # your code here
```

Using the camera on car 1, you should drive car 2 to the box in the center of the world. You can use the `get_image()` function to get an image from the camera. At the end of the function, car 2 should be close to the box in the center of the world (0.4 is acceptable), but **it should not collide with the box during the entire ride**.
f
You code to check if car 2 is close to the box is as follows:

```python
np.linalg.norm(
    data.body("car 2").xpos - 
    data.body("target-box").xpos
)
```


## Task 3 - 7 points

This task is almost identical to task 2, with the only difference being that car 2 is initially rotated randomly using the following code.

```python
def drive_to_ball_2(seed=1337):
    random.seed(seed)
    steps = random.randint(0, 2500)

    data.actuator("turn 2").ctrl = 1
    for _ in range(steps):
        mujoco.mj_step(model, data)
    data.actuator("turn 2").ctrl = 0
    for _ in range(1000):
        mujoco.mj_step(model, data)
    # your code here
```

In this task your car can collide with the box.