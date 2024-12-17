---
title: Lab 12
---

# Goals:

- learn about PID controller
- implement PID controller in MuJoCo to control a car

# Requirements

MuJoCo 3.0 or later

# What are PID controllers?

Before starting the tasks you should watch at least the first video in [this](https://youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y&si=LTdF48nP-bvG1kfO) playlist.
We will be solving a very similar problem to the one discussed there.
Namely, we will be driving a car on an inclined plane with the goal to stop at a given point.
This is very similar to the problem of hovering a drone at a given altitude.
We encourage you to watch the entire playlist.

# Lab session

As already mentioned, the goal of this lab session is to create a PID controller which will allow us to drive a car on an inclined plane.
We will make it stop at traffic lights and keep it there.

## Assets

### Car and World models

In this lab we use a simplified robot description (MJCF) of a four wheel car.
It is a modified version of [this](https://github.com/google-deepmind/mujoco/blob/2.3.7/model/car/car.xml) three wheel car model provided by google-deepmind.
You can find the car model in the `car.xml`.

Except that there is a model of a scene with a road and traffic lights.
You can find it in `traffic_lights_world.xml`.

### PID controller

Apart from the assets, you are provided with a `traffic_lights.py` script that contains a partially implemented PID controller. This is where you will focus your efforts. In the following paragraphs, you will:

1. Execute the `traffic_lights.py` script and choose an appropriate proportional gain for the PID controller to get the car moving in the desired direction.
2. Add a missing formula in the implementation of the derivative part of the PID controller to enhance the stability of the car's movement.
3. Implement the integral part of the PID controller to allow the car to reach the traffic lights and remain stationary there.

You will only be working on the `car_control` function. The `sim_step` function simply runs the simulation, so you do not need to change it to complete the tasks.

Let's start!

## Implementing PID Controller

### Proportional part

The proportional part of the controller has already been implemented.
- However, you can run the script and see that the car starts moving in the incorrect direction.
- Why is this the case?
- Try to find an answer and change the value of the `gain_prop` argument in the function call at the end of the script to start moving the car in the correct direction:

```python
# Change the values here to change the respective gains or the desired altitude
if __name__ == '__main__':
    car_control(gain_prop = -1, gain_int = 0, gain_der = 0)
```

After setting a proper `gain_prop`, the car is able to overcome the force of gravity (which is what was causing it to go in the wrong direction!) and drive towards the traffic lights.
This is because the resultant force of the car's motor is programmed to be directly proportional to the `error_prop`, i.e., the signed distance of the car to the traffic lights.

```python
# TODO: implement PID controller
error_prop = data.body("car").xpos[1] - data.body("traffic_light_gate").xpos[1]

forward_torque = gain_prop * error_prop
```

However, we can clearly see that the car is going back and forth around some point.
This is because, as explained in the video for the drone example, with proportional gain, we only consider the distance from the target.
We don't track how fast we are going when reaching the goal.
Hence we cross the target, go far off track, at some point turn around and the cycle repeats.
You can see that the oscillations are getting smaller and smaller, but this behaviour is definitely not satisfactory.

### Derivative part

We can improve how the car behaves by adding the derivative part to our controller.
To do this you should consider how the error is changing, i.e. use a derivative.

In this case, the derivative error is proportional to the car's velocity.
However, we should not directly retrieve it from MuJoCo (e.g., using `data.body("x2").cvel[5]`).
In real-world scenarios, we only have access to the information provided by the sensors.
The sensor we have here measures the distance from the traffic lights.
Therefore, we should only utilize this information.

Since we are dealing with discrete measurements, we have to somehowe approximate the derivative.
So you have to add variables which will store current and previous error
(or current and previous position, because the position of the traffic lights is constant and will disappear during differentation).
This will be the difference `f(t + delta_t) - f(t)` where `delta_t` is a timestep between two measurements.
The question is now how to approximate `delta_t`.
Since we are assuming that the sensor is providing as information at each step of the simulation, we can simply use `model.opt.timestep`.
Note that this is the same timestep MuJoCo uses to approximate physics in the simulation.

In the end your code should look more or less like this:

```python
for _ in range(4000):
    # TODO: implement PID controller
    error_prop = data.body("car").xpos[1] - data.body("traffic_light_gate").xpos[1]
    error_der = some_error_derivative_approximation

    forward_torque = gain_prop * error_prop + something_added_here
    # Your code ends here
```

Make sure to find an appropriate `gain_der` for your implementation.
You should do this by setting a value for the `gain_der` parameter in the function call at the end of the script.

Note that it is possible to ignore the `delta_t` aprroximation and incorporate it inside the `gain_der`.
However if one wants to be able to interpret the derivative error term (as we can do here by realising that it's the car's velocity) and use it with some intuitive units, the timestep has to be taken into account.

Now, if your implementation is correct and if you have chosen a correct gain, your car should quickly stay steady at some point.
Sadly, we can clearly see that... this is not at the traffic lights!
We still have to work on that.

### Integral part

The problem of not reaching the traffic lights was discussed in the drone example in the video.
The problem is that the car stabilizes at a point where the resultant force is zero.
For this to happen the motor has to generate a force which whill cancel out gravity.
With only proportional and derivative term it is impossible to obtain such a force for a car standing at the traffic lights, because the force generated by a PD controlled motor in this case is zero.
Hence we have to solve it by adding the integral part to the PID controller.

Now you should do everything on your own, i.e.:

 - figure out what to use as the error for the integral part.
 - add appropriate code to the `car_control` function to implement the integral part of the controller
 - choose appropriate `gain_int` which is already passed as an argument to `car_control`.
 Set it in the function call at the end of the script.

### P, PI and so on controllers

As mentioned in the video, if some part is missing in the PID controller you describe it with the letters of the parts which are present.
For example, you might find it interesting to see what happens if we decide to use PI control. 
With the whole PID controller implemented this is as easy as setting the derivative gain to zero. 

Note that if you need you can easily change the length of the simulation:

```python
# Increase the number of iterations for longer simulation
for _ in range(4000):
```

# Summary

Congratulations, that's all!
You have successfully implemented your first PID controller.
In the homework, you will work on a similar PID controller for a drone – the exact challenge covered in the videos.
We will delve into more advanced aspects of utilizing PID controllers in a drone setting.
