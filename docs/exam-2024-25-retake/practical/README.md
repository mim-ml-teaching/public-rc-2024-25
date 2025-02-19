# Robot Control - Practical Exam - 2024 / 25 Object Detection and Pose Estimation


## Updates:
None yet


## Submission format

**You should submit via moodle before the deadline.**

 Before submitting, go through the list below and make sure you took care of all of the requirements.
 More details can be found in the detailed task description.
 **If you do not comply with these regulations you can be penalized up to obtaining zero points for the task**.

 1. Submit only the following files:
    - `solution.py`
    - `requirements.txt` (if you decide to import additional packages)
 2. Do not submit any other files.
 3. Change only the sections of the file between the `#TODO:` and `#END TODO` comments.
 You can add imports at the top of the file.
 However the task can be easily solved without any additional imports.
 4. You can assume that MuJoCo, opencv, numpy, and other standard libraries are installed on the grading server.


## Problem description
This assignment focuses on developing computer vision algorithms for object detection and pose estimation in a simulated robot environment.
You will be working with the MuJoCo physics engine and OpenCV to process images from a simulated camera mounted on a car.
The goal is to create a `Detector` class which will detect a cube with aRuCo markers.
The simulation code is provided in the file `problem.py`.
You are only required to submit the `solution.py` file containing your implementation of the `Detector` class.


As already mentioned `Detector` class will be responsible for estimating the position of the cube.
The world will contains a single cube, randomly placed within a certain distance from the world's origin.
The cube's position is determined by the `random_point_on_circle` function.
The function takes circle radius as input and then selects a random angle.
The x, y coordinates of the point corresponding to this angle are calculated.
The cube is placed at these coordinates.
You should analyse the code in `problem.py` and XML file in `car.xml` to understand the cube placement and size.
The radius of the circle can vary between `0.7` and `1.3`.
Your task is to analyze the images provided by the simulator and accurately estimate the position of the cube.

**Specific Requirements:**

1. Implement the `detect` method within the Detector class.
This method receives the image `(img)` captured from the simulated camera as input.
This image may contain a visual representation of the cube.
2. The method has to:
   - deteremine whether the cube is visible on the image
   - estimate the position of the cube **in the world coordinate frame**
3. You are provided with a `test_detection` method which asserts how good your estimation was.
Your implementation should almost always pass this test.
It is acceptable if sometimes the test fails due to weak marker detection.
This should happen very rarely.
Nevertheless, the cube will be usually detected multiple times during the simulation.
Hence we accept a failure case to happen, but at most once every 10 runs of the `problem.py` script.
