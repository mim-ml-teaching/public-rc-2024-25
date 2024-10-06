---
title: Exam 2023 Practical 2
---

# Practical Task 2 - Fixing the Railway

This task is worth 15 points, submit through the moodle.

In this task your goal is to fill in a gap on a railway within a given time.
You have a robot with two prismatic joints, one revolute joint and an end effector in a form of a flat plate.
The two prismatic joints allow you to change the position in 2D.
The revolute joint allows you to change orientation (yaw) of the plate.

Both the existing parts of the railway and the missing block are modelled using cuboids.
The missing part is slightly narrower than the gap, but just enough to slip it in.
Your goal is to place it in such a way that the railway is complete.

You know that:
- the missing block dimensions are always (20 cm x 5 cm x 5 cm)
- the width of the gap is always 21 cm
- the missing block is not attached to the plate
- at the beginning of each simulation:
    - the end effector is always at the origin of the world coordinate frame
    - the missing block is always placed on the end effector with the center of mass at `x = 0` and `y = 0`
    - the relative yaw of the missing block to the flat plate is chosen randomly
    - the railway is always straight and parallel to the Y axis.
    - the `x` coordinate where the railway is placed is chosen randomly with `|x| > 11 cm` (i.e. at least 11 cm from the world coordinate frame origin)

Below are the `urdf` files describing the robot, the railway and the missing block:

- [robot.urdf](files/robot.urdf)
- [filler.urdf](files/filler.urdf)
- [bridge.urdf](files/bridge.urdf)

You are also given:

- a template for your solution: [task_2.py](files/task_2.py)
- a set of fixed test cases to check your solution [test-cases.json](files/test-cases.json)

You can run the template before you start working on the problem to understand how the simulated world looks like and how the robot behaves.

Requirements and additional information:

- `MAX_DISPLACEMENT_ERROR = 5e-3` is the maximum amount by which the missing block can be misplaced in both `x` and `y` direction relative to the rest of the railway.
- `MAX_ROTATION_ERROR = 0.01` is the maximum amount by which the missing block can be rotated relative to the rest of the railway.
- The missing block has to be static in the final state, i.e. both the linear and angular velocity should be zero.
The variables `LINEAR_VELOCITY_THRESHOLD = 0.01` and `ANGULAR_VELOCITY_THRESHOLD = 0.1` specify the thresholds below which the linear and angular velocities are considered to be eual to zero, respectively.
- The `y` coordinate of the gap is always equal to `GAP_Y = 0.31`.
- `TIMECAP = 10` is the maximum number of seconds in which the final state has to be reached.
Note that one simulation step is `1/240` seconds (default in `pybullet`), so actually you have `10*240` simulation steps to reach the desired state.
For your convenience, the time left to complete the task is displayed when you run simulations in GUI.
- For your convenience, the block will be displayed in different colours during the simulation to let you know what is its current state:
    - if there is time left and the missing block is:
        - not correctly placed: purple (like the rest of the railway)
        - correctly placed but not static: yellow
        - correctly placed and static: green (test case accepted)
    - if there is no time left: red (it means you didn't manage to solve the test case in the given timecap)
- The missing block **does not have to be** at the center of the end effector after reaching the final state.
It is ok if the missing block slides a little bit during the movement, however we suggest keeping these displacements in a reasonable range.
- The missing block cannot be rolled over to another side at the final state.
While it is fine for it to roll during the movement, we highly recommend avoiding movements which cause such behaviour - it will be very difficult for you to place it back on a proper side.
- Hitting the railway with the missing block and disturbing the original position of the railway is considered as failure.
You should move the robot in such a way that you avoid collisions with the existing railway
- Your solution:
    - should be only a modified version of the `task_2.py` file - don't submit any other files
    - should modify only part of the file between `# EDIT ONLY BELOW THIS LINE` and `# EDIT ONLY ABOVE THIS LINE` comment
    - should not modify other parts of the `task_2.py` file nor any other files
    - will be graded based on:
        - the success rate in a test consisting of 10 provided fixed test cases + 10 random test cases.
        The success rate is reported after all simulations are run (100% points)
        - not doing anything hacky, like teleporting the block to the target pose (up to -100% points)

Hints:

- Since you are not uploading the `test-cases.json` file you can add your own test cases to this file or rearrange / remove the existing ones when working on your solution.
Remember, however, that your final solution will be graded based on the list of test cases we provide + 10 random cases.
- Make sure to check out `pybullet`
[documentation](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3)
, in particular:
    - the `getBasePositionAndOrientation`, which allows you to get the pose of the filler
    - the `setJointMotorControl2` and its `maxVelocity` and `force` parameters when working on your solution


## Running the code

Tested on the MIMUW lab machines:

```bash
git clone https://github.com/mim-uw/rc-2022-23 &&
cd rc-2022-23/docs/practical-2/files/ &&
python3 -m venv venv &&
. venv/bin/activate &&
pip install numpy pybullet &&
./task_2.py
```
