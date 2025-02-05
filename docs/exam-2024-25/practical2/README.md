---
title: Robot Control - Practical Exam - 2024 / 25
---

# Updates:
None yet

# Submission format

**You should submit via moodle before the deadline.**

Before submitting, go through the list below and make sure you took care of all of the requirements.
More details can be found in the detailed task description.
**If you do not comply with these regulations you can be penalized up to obtaining zero points for the task**.

1. Submit a zipped file with only the two following files inside:
- `drone_control.py`
- `pid.py`
2. Do not submit files with simulation conditions changed
3. You are required to use PID control, but it's your task to choose a proper design, i.e. the number and type of PID controllers
4. The simulation should not crash at any stage.
5. You should modify only the places in the code which have the `TODO` tags.
7. You should not read any values from mujoco simulation.
All you need is provided by the functions and variables in the `DroneSimulator` class.

# Requirements

MuJoCo 3.2.6 (earlier versions might work, but this has not been verified)

# Problem Description

## Overview

Your goal is to design and implement a PID control to steer a drone.
You should be already familiar with a general setting of this problem, because we have used a very similar model during the semester.
The following information summarises the problem:
 - The drone is controlled with a thrust at 4 propellers
 - You can control the average `thrust` which simply affects the drone movement in the z direction (in the drone coordiante frame)
 - You can also set `roll_thrust`, `pitch_thrust` and `yaw_thrust` to change the orientation of the drone.
 Check [this video](https://youtu.be/pQ24NtnaLl8?si=PQUBFI-UdfGFejr0) to understand how the roll, pitch and yaw angles are defined.
 - Your first goal is to be able to change the yaw angle of a drone
 - Your second goal is to be able to fly the drone to any position while keeping the yaw angle constant

## Grading

Two notes about grading:

First, your solution will be evaluated using `RandomPathPlanner` class provided in the `planners.py` script.
Since the paths are random, it might happen that you sometimes won't reach the desired pose in time
(e.g. if two consecutive targets are far away).
**This is acceptable.**
Important things are:
 - your solution should, in the given time, more or less reach the desired targets
 - the drone should definitely go in the desired direction
 - the drone should not crash at any point of the simulation
 - if the number of points on the path is decreased to 2-3 points with long periods between the target change,
 your solution should reach them almost perfectly.
 You can check if this is the case using the `FixedPathPlanner` which allows you to set your own targets.

Second, you are not required to deal with all possible yaw targets.
When the yaw angle crosses 180 degrees, there is an abrupt change in value
(a jump between 180 and -180 degrees).
This problem requires some additional effort and we don't require that you solve it.
This is why, as one can deduce from the `RandomPathPlanner` class, yaw targets will be always between -45 and 45 degrees.
**Your solution should work only with targets from this range.**

In essence, your overall solution will be graded qualitatively - whether it works reasonably well.
If you have any doubt, feel free to ask questions during the exam.



## Assets

### Skydio X2

We use a simplified robot description (MJCF) of the [Skydio X2](https://www.skydio.com/skydio-x2) drone developed by [Skydio](https://www.skydio.com/).
The model and necessary assets were downloaded from [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie).
Rotor gears where slighlty changed to allow yaw control.

![Image description](x2.png){: width="400" }

### Drone Simulator

Except the assets from Skydio, you are given a `drone_simulator.py` script.
You should not edit this script as it will not be a part of your submission.

Inside the script you can find the definiton of a `DroneSimulator` class.
It contains attributes and methods to run a simulated flight of a drone.

To complete the tasks you have understand:

1. how initialisation parameters work
2. how roll, pitch and yaw thrusts work
3. how to get the sensor readings from an instance.

#### Drone Simulator instance

Assuming that a `DroneSimulation` was instantiated in the following way:

```python
drone_simulator = DroneSimulator(model, data, viewer, rendering_freq = 1)
```

the `model`, `data` and `viewer` arguments refer to MuJoCo functionalities with which you should be familiar with.
They are already prepared for you in the stub files.

`rendering_freq` allows you to display the simulation faster.
Remember that this does not affect the simulation results!
Hence it allows you to run debugging tests more quickly or slow down the simulation to analyse a specific part.

#### Roll, Pitch and Yaw Thrust

Note: You do not have to understand the physics in detail to complete the task.
Description below is just here to help you.
Feel free to ask questions about these concepts during the exam if something is not clear.

By changing thrust on specific rotors we can change the orientation of the drone.
Check [this video](https://youtu.be/pQ24NtnaLl8?si=PQUBFI-UdfGFejr0) to understand how the roll, pitch and yaw angles are defined.

Roll and pitch are quite easy to understand.
For example, by increasing the thrust at the rotors in the back of the drone we can pitch the drone.
If at the same time we will decrease the thrust at the front,
the effect will be even bigger and we will keep the average thrust unchanged at the same time.
The roll thrust works the same.

Yaw angle is a little bit more complicated -
by changing the thrust on diagonal rotors we can affect the average torque on the drone
(think of a rotational force).
This allows as to rotate the drone around z axis in the drone coordinate frame.

Here's the part of the code which implements these thrust inputs:


```python
def sim_step(self, thrust, roll_thrust, pitch_thrust, yaw_thrust, steps=1, view=True):
        self.data.actuator("thrust1").ctrl = thrust + roll_thrust - pitch_thrust - yaw_thrust
        self.data.actuator("thrust2").ctrl = thrust - roll_thrust - pitch_thrust + yaw_thrust
        self.data.actuator("thrust3").ctrl = thrust - roll_thrust + pitch_thrust - yaw_thrust
        self.data.actuator("thrust4").ctrl = thrust + roll_thrust + pitch_thrust + yaw_thrust
```


#### Sensors

Assuming that an instance of the `DroneSimulation` class was stored in a `drone_simulator` variable,
you can get:
 - the last two registered `[x, y, z]` position of the drone with `drone_simulator.position_sensor()`
 - the last two registered `[roll, pitch, yaw]` orientation with `drone_simulator.orientation_sensor()`

The calls two these methods are already present in the stub file `drone_control.py`.

Both sensors provide you with a new reading in each simulation timestep.

#### Summary

This is all you should need to implement the control for the drone.
If you want to understand the `DroneSimulation` class better, you are always encouraged to go through the class definition.

## Task 1 - PID Class (3 points)

Your goal here is to complete the implementation of the class `PID` for a general PID controller.
This is delegated to the `pid.py` script and you should change only this file.
**You are required** to define a class where arguments have specified data types.
Note that you should define a class which allows you to limit the range of output values.
The `output_limits` parameter should allow the user to specify what is the minimum and maximum returned value.
For example the code below:

```python
pid_controller = PID(
        gain_prop = 10, gain_int = 0, gain_der = 50,
        sensor_period = model.opt.timestep, output_limits=(-20, 50)
    )
```

should create a PID controller which outputs only values bigger than -20 and smaller than 50.


## Task 2 - Yaw Control (6 points)

Now we move to the `drone_control.py` file.
You are not allowed to change any other files than `drone_control.py` and `pid.py`.

As discussed above, the drone changes its orientation with roll, pitch and yaw thrusts.
In this task your goal is to change the yaw angle of the drone.
However, naivly changing only this angle will not suffice, because the drone will quickly loose balance:

<video width="512" height="208" controls>
  <source src="yaw-failure.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

This is because we are not making sure that the roll and pitch angles are not changed.
Hence, you have to tune 3 PID controlles - one for each angle.

Hint:
 - Start working with a roll or pitch controller and make sure that the drone can quite easily change the angle of interest
 - The same gains should work for both roll and pitch controller
 - When you have roll and pitch controller ready, tune the yaw controller
 - If necessary, refine all gains at the end

Here's a video with an example working solution:

<video width="512" height="208" controls>
  <source src="yaw-solution.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Task 3 - Position Control (8 points)

The final task is steer the drone to any position.
You should use the controllers defined in the previous task and the following:
 - changing the roll and pitch angles allows us to move in the x and y direction.
 - changing the average `thrust` allows us to change the z position of the drone.

Here's an example solution:

<video width="512" height="208" controls>
  <source src="drone-complete-solution.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

Note that we assume that the yaw angle is kept fixed around zero when changing the position of the drone.
In other words, you can assume that changing the roll angle moves the drone right and left
and changing the pitch angle moves the drone to the front or back.
The yaw angle should be kept constant with a PID controller which you have already defined in the previous task.

# References and Licenses

The [Skydio X2](https://www.skydio.com/skydio-x2) model is released under an [Apache-2.0 License](LICENSE). No changes were made to the original model.
