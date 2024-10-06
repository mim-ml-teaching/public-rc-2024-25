---
title: Practical Exam 2023 / 24 - Problem 2
---

# Updates:
None yet


# Submission format

**You should submit via moodle before the deadline.**

Before submitting, go through the list below and make sure you took care of all of the requirements.
More details can be found in the detailed task description.
**If you do not comply with these regulations you can be penalized up to obtaining zero points for the task**.

1. Submit a zipped file with only the two following files inside:
- `main.py`
- `pid.py`
2. Do not submit files with simulation conditions changed
3. You are required to use PID control, but it's your task to choose a proper design, i.e. the number and type of PID controllers
4. The simulation should not crash at any stage.
If your solution generates too big torques, it is a bad solution.
5. The car should not turn at any stage.
There is no way to control the turn of the car in the model, so if this happens something must have gone wrong.
6. You should modify only the places in the code which have the `TODO` tags.
7. You should not read any values from mujoco simulation.
All you need is provided by the functions and variables in the `CarSimulator` class.


# Requirements

You should use MuJoCo 3.0 or later.

You can create an appropriate virtual environment for this task using the provided `requirements.txt` file.
Simply create a virtual environment and then run:

```python
pip install -r requirements.txt
```


# Problem Description


## Overview

Your goal is to design and implement a PID control to drive a car to a desired location.
You should be already familiar with a general setting of this problem, because we have used a very similar model during one of the lab sessions.
The following information summarises the model:
 - The car is driving on a straight road which is placed on an inclined plane.
 - There are traffic lights within some distance of the car and your goal is to stop the car where the traffic lights are.
 - You are allowed to cross the traffic lights.
The car simply needs to be approximately below the traffic lights by the end of the simulation and have approximately zero velocity.
 - You control the forward torque at the front wheels


## Assets

### The world and the car model

You will use a simplified robot description (MJCF) of the world with a car which is a slightly modified version of the file used in one of the lab sessions.
You can find it in the `car.xml` file.
The only changes are:

 - ability to turn was removed
 - a gyroscope to measure angular velocity at one of the wheels has been added
 - the limits on the forward torque were lowered

### CarSimulator

Except the world and car models, you are given a `car_simulator.py` script.
You should not edit this script as it will not be a part of your submission.

Inside the script you can find the definiton of a `CarSimulator` class.
It contains attributes and methods to run a simulation of a driving car.
This class should look familiar - we have used a very similar one during one of the homeworks.
It allows you to control conditions of the simulation.
If you want, you can change these conditions when designing and testing your solution.
However, make sure to submit the solution which works with the original conditions.

To complete the tasks you have to know three things about the class:

1. how to get the sensor parameters and readings from the instance.
2. how parameters given to the specific instance determine the conditions

#### Sensors

Assuming that an instance of the `CarSimulator` class was stored in a `cs` variable, you can get the readings from two sensors:
 - current position from a GPS sensor with `cs.position`,
 - current angular velocity measured by the gyroscope at the front left wheel with `cs.wheel_angular_vel`.
Note that in contrast to the homework problem, the sensors return only the current value, i.e. they do not store any previous readings.

If you need a time period between two consequtive sensor readings you can call:
 - `cs.gps_period` for the position
 - `cs.wheel_gyroscope_period` for the angular velocity of the wheel

#### CarSimulator instance

Assuming that a `CarSimulator` was instantiated in the following way:

```python
cs = CarSimulator(gps_freq=0.01, rendering_freq=1)
```

we can easily deduce the conditions of the simulation. Namely:

- `gps_freq` determines how often we get the GPS sensor readings during the simulation.
These readings determine our position.
A frequency of `0.01` means that we receive readings every hundredth step.
- `rendering_freq` determines how quickly frames of the simulation are displayed in the viewer.
If you want to display them more slowly (e.g. for debugging purposes), you should decrease this value.

You can change the simulation conditions when working on the problem, but at the end your solution should work in exactly the conditions specified in the task.


## Task 1 - PID Controller Class (5 points)

Your goal here is to complete the implementation of the class `PID` for a general PID controller.
This is delegated to the `pid.py` script and you should change only this file.
Note that **you are required** to define a class where `sensor_reading` argument in the `PID.output_singal` method is **a single value representing only the most recent reading**.
In other words, your goal is to create a PID controller class which is able to deal with exactly the sensors from the task and so you should not change their behaviour.

## Task 2 - Driving the Car (10 points)

Your goal is to design a PID control which allows you to drive a car and stop it at the desired location.
You are required to use PID control, but it's your task to determine:
 - what types of controllers to use (e.g. you are allowed to use only a PD controller)
 - how many PID controllers you want to have

In this task you should change only the `main.py` script.
The designed control should be placed between these lines:

```python
    # TODO: design control for the system
    ...
    # TODO end
```

and then used to control the forward torque on the wheels between these lines:

```python
    # TODO: find value for `forward_torque` using designed control
    forward_torque = 0
    # TODO end
```

**Do not change any other parts of the file.**

You solution will be considered successful if by the end of the simulation:
 - the car is within the distance of `0.02` of the traffic lights
 - the angular velocity at the front left wheel is smaller than `0.1`

Below you can see a video with an example solution:

<video width="512" height="208" controls>
  <source src="pid-car-control-exam-solution-example.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
