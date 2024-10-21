# Retake Exam Task 

Use MuJoCo 3.1.x

Utilize the provided `car.xml` file, which contains one car.

Send the solution (`car.xml`, `README.md` and `prog.py`) as a zip file to the slack.

Small tasks (steps are meant to be done in order):

1. Add two red boxes to the world (modify `car.xml` file), one in front of the car (one unit away) and one in the back (one unit away). The boxes should be 0.1 units in size.

2. Using `turn` and `forward` actuators, turn the car 90 degrees to the right and move it forward 2 units (some inaccuracy is allowed).

3. Using `turn` and `forward` actuators, turn the car randomly from 0 to 360 degrees.

4. Attach the following camera to the car:

```xml
<camera name="camera1" pos="-0.1 0 0.05" fovy="90" euler="180 -80 90"/>
```

5. Using the camera, take a picture from the car and save it to a file called `car.png`.

6. Using the camera and actuators move the car to the position between the two boxes. 

In any of the tasks you should not read the position from the simulation. You should use the camera images in step 6 to determine the rotation of the car.