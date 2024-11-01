# Robot Control lab 2

In this lab session, we will focus on 3D transformations, the pinhole camera model, and the fundamentals of stereo vision. We'll start with introductory exercises on these topics. Then, you'll apply this knowledge to capture a video of a moving car in MuJoCo.

[Here](notes-lab-2.pdf) you can find notes with solutions and hints for the exercises below.

## 3D Transformations

Do it using pen and paper.

### Basic transformations

1. Create a matrix M1 that rotates 3D space around the x-axis by 90 degrees.
2. Create a matrix M2 that rotates 3D space around the y-axis by 90 degrees.
3. Create a matrix M3 that translates 3D space by 1 unit along the x axis.

### Composition of transformations

{:start="4"}
4. Consider two transformations:

- First, translate 3D space by 1 unit along the x-axis and then rotate it around the y-axis by 90 degrees.
- Second, rotate 3D space around the y-axis by 90 degrees and then translate it by 1 unit along the x-axis.

Do you expect the matrices representing the above transformations to be the same?

{:start="5"}
5. Consider the results of two multiplications: `M4 = M2 * M3` and `M5 = M3 * M2`. Do you expect to get the same results? Check computationally if your predictions were correct.
6. Consider two multiplications: `M4 * M5` and `M5 * M4`. Do you expect to get the same results? Check computationally if your predictions were correct.
7. Create a matrix `M6` that rotates a 3D space around the `(1, 1, 0)` vector by any given number of degrees.

*Hint:* Sometimes it is easier to describe a transformation using different coordinates (e.g., choosing a different basis). If you have a new coordinate system, then describing the transformation in the original coordinates requires only finding the transformation between your two coordinate systems and composing the appropriate transformations in the correct order.

*Note:* Make sure the directions of the basis vectors satisfy the [right hand rule](https://en.wikipedia.org/wiki/Right-hand_rule).

### Different representations of transformations

Familirize yourself with the Rodrigues Formula: <https://mathworld.wolfram.com/RodriguesRotationFormula.html>.

{:start="8"}
8. Use the Rodrigues Formula to create a matrix `M6` from the previous exercise. Compare the results and make sure they are the same.
9. Use the Rodrigues Formula to create a matrix `M7` that rotates 3D space around the (1, 1, 1) vector by 90 degrees.

If you want to know more about Rodrigues Formula and how it is derived you can check out these videos: <https://www.youtube.com/watch?v=UaK2q22mMEg> and <https://www.youtube.com/watch?v=q-ESzg03mQc>.

{:start="10"}
10. We have the following rotation matrix:

```python
R = [[0.966496, -0.214612, 0.14081],
     [0.241415, 0.946393, -0.214612],
     [-0.0872034, 0.241415, 0.966496]]
```

What is the axis and the angle of rotation R? *Hint:* consider a vector parallel to the axis of rotation. How the matrix R should act on this vector?

## Camera Callibration and Stereo

### Intrinsic camera parameters

{:start="11"}
11. Determine the intrinsic camera matrix of a pinhole cameras which has focal length of 1 and optical center at `(300.5, 300.5)`.
12. Find 2D coordinates of the projection of the 3D point `(10, 10, 5)` onto an image captured by the camera from the previous exercise.

### Basic triangulation

{:start="13"}
13. Imagine that you have two cameras like in the previous exercises. The optical axes of both cameras are parallel. The axis of the second camera is displaced by `(x=1, y=0, z=0)` relative to the first camera. Estimate the distance from the first camera to a point whose coordinates are `(303, 303)` on an image captured by the first camera and `(298, 303)` on the image captured by the second camera.

## MuJoCo Simulation

We are going to:

- build a car with wheels and a radar
- learn how to teleport the car to any position
- write a python program that creates a video of the car driving in a circle

### Coordinate frames

Let's start with the following XML file describing the world:

```xml
<?xml version="1.0" ?>
<mujoco>
    <worldbody>
         <body name="floor" pos="0 0 -0.1">
            <geom size="2.0 2.0 0.02" rgba="0.2 0.2 0.2 1" type="box"/>
        </body>
        <body name="x_arrow" pos="0.5 0 0">
            <geom size="0.5 0.01 0.01" rgba="1 0 0 0.5" type="box"/>
        </body>
        <body name="y_arrow" pos="0 0.5 0">
            <geom size="0.01 0.5 0.01" rgba="0 1 0 0.5" type="box"/>
        </body>
      
    </worldbody>
</mujoco>
```

You can view the world in MuJoCo simulator. You can also install mujoco `pip install mujoco`, and run:

```python
python -m mujoco.viewer
```

Whichever method you choose, you should see something like this:

![MuJoCo world with x and y arrows](lab_2_1.png)

We represent unit vectors along the x and y axes using narrow boxes and call them "arrows".

{:start="14"}
14. Now add a third arrow for the unit vector along the z-axis.

Note, that the arrows we have created are indeed unit vectors.
This is because MuJoCo uses half-sizes for the bodies of a box type as stated in the documentation:

```txt
Three size parameters are required, corresponding to the half-sizes of the box along the X, Y and Z axes of the geom’s frame.
```

Therefore in our case the boxes have length equal to 1 and width and depth equal to 0.02.

You can find full documentation of different body types here: <https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom>.

### Car body and wheels

We provide you with the following, **imperfect** car body XML description:

```xml
<body name="car" pos="0 0 0.1" axisangle="0 0 1 0">
    <geom size="0.2 0.1 0.02" rgba="1 1 1 0.9" type="box"/>
</body>
<body name="wheel_1" pos="0.1 0.1 0.1" axisangle="1 0 0 90">
    <geom size="0.07 0.01" rgba="1 1 1 0.9" type="cylinder"/>
</body>
<body name="wheel_2" pos="-0.1 0.1 0.1" axisangle="1 0 0 0">
    <geom size="0.07 0.01" rgba="1 1 1 0.9" type="cylinder"/>
</body>
```

Add it to your XML file and check how the result looks like.
As you can see there are two deffects:

- one of the wheels has an incorrect pose,
- the car has only two wheels and we need four.

#### Correct pose of the wheels

What is wrong with the `wheel_2`? It's incorrectly rotated.
The rotation is defined by the `axisangle` attribute with four numbers:

- the first three numbers define the axis of rotation
- the last number determines the angle of rotation.

The axis is defined in the local coordinate frame of the body.
The angle is measured in degrees.
You can read more about it here: <https://mujoco.readthedocs.io/en/stable/modeling.html#corientation>

{:start="15"}
15. Rotate the wheel to position it correctly.

Your result should look like this:

![Correctly positioned wheel](lab_2_2.png)

#### Adding missing wheels

Another problem we have is that there are only two wheels and we need four.

{:start="16"}
16. Add the other two wheels.

Experiment with different positions, rotations and sizes of the wheels.

### Radar

{:start="17"}
17. The last step is to add a radar to the car.

You can do it by adding the following description to your XML file:

```xml
<body name="radar_1" pos="0 -0.1 0.2" axisangle="1 0 0 30">
    <geom size="0.01 0.01 0.1" rgba="1 1 1 1" type="box"/>
</body>
<body name="radar_2" pos="0 -0.15 0.29" axisangle="1 0 0 30">
    <geom size="0.03 0.01" rgba="1 0 0 1" type="cylinder"/>
</body>
```

As you can see, the radar is a box with a red cylinder on top.
The angle between the radar and the direction of the z axis is constant and equals 30 degrees.
Initially this is obtained by rotating the radar by 30 degrees about the x axis.
Check the snippet above to make sure that this is the case.
The final result should look like this:

![Final result of the radar](lab_2_3.png)

### Video of a driving car

Take a look at the following python code:

```python
import mujoco
import matplotlib.pylab as plt


for i in range(10):
    xml = f"""
<mujoco>
  <visual>
     <global offwidth="1280" offheight="1280"/>
  </visual>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
    <geom name="green_sphere" pos="{.2 + i / 10} .2 .2" size=".1" rgba="0 1 0 1"/>
  </worldbody>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, 1280, 1280)

    mujoco.mj_forward(model, data)
    renderer.update_scene(data)
    plt.imsave(f"frame_{i:03d}.png", renderer.render())
    renderer.close()
```

After a short inspection you can probably realize that it produces consecutive frames of a video.
You can run the script and then use the `ffmpeg` command to produce the video:

```bash
ffmpeg -framerate 30 -pattern_type glob -i 'frame*.png' -c:v libx264 -pix_fmt yuv420p output.mp4
```

Our goal is to produce a video of the car we have just created.
The car will be driving on a circle.
Simultanously we will also rotate the radar, so that it points all the time to the center of the circle.
Let's divide the task into two parts.

#### Generating a driving car

{:start="18"}
18. First write a python program which will generate a world scene with the car in specified positions and orientations.

We will worry about rendering frames in a minute.

The program should take four parameters:

- the x coordinate of the position of the car
- the y coordinate of the position of the car
- the orientation of the car described by a clockwise rotation about the z axis
- the orientation of the radar described by a clockwise rotation about the z axis.

Remember that the angle between the radar and the z axis is constant (30 degrees).
Therefore this parameter determines only the direction of the XY projection of the radar.

The output of the program is an XML file for MuJoCo simulator.
You can use the file you have created in the previous part as a template.
The file describes a world with the car and its radar in a given pose.
The XML file should be saved as `car.xml`.

#### Rendering frames

{:start="19"}
19. Write a python program which renders video frames of a moving car.

The car should start at coordinates (-1,-1) and point in the (1, 0) direction.
Then it should drive on a circle with radius 1 and a center at (-1, 0).
Looking from the top, the car moves in the counterclockwise direction and finishes at the point (0, 0).

Remember to rotate the radar during car's movement, so that it always points to the center of the circle.
You should:

- inititally rotate the radar 30 degrees about the x axis,
- keep the angle between the radar and the z direction constant, i.e. rotate only XY projection of the radar.

If the scene doesn't fit in the camera frame, you can change the world size in the XML file.

The output of the program might look like this:

![Car moving in a circle](car_moving.gif)
