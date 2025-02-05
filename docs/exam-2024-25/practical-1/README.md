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
    - `requirements.txt`
 2. Do not submit any other files.
 3. You can assume that MuJoCo, opencv, numpy, and other standard libraries are installed on the grading server.

## General description
This assignment focuses on developing computer vision algorithms for object detection and pose estimation in a simulated robot environment. You will be working with the MuJoCo physics engine and OpenCV to process images from a simulated camera mounted on a car.  The goal is to create two Python classes: `Detector` and `DetectorPos`, each designed for a specific task.  The simulation code is provided in the file `problem.py`.  You are only required to submit the `solution.py` file containing your implementations of the `Detector` and `DetectorPos` classes.

## Task 1: Object Classification (Detector Class)

In this task, your `Detector` class will be responsible for identifying the type of object present in the simulated world. The world will contain either a sphere, a box, or nothing.  The object, if present, will be randomly placed within a certain distance from the car's starting position.  Your task is to analyze the images provided by the simulator and correctly classify the object.

**Specific Requirements:**

1. Implement the `detect(img)` method within the `Detector` class. This method will receive an image `(img)` as input, captured from the simulated camera.
2. Process the input image using OpenCV or other suitable image processing techniques to determine if an object is present and, if so, its type (sphere or box).
3. Implement the `result()` method within the Detector class. This method should return a string representing the detected object type: "sphere", "box", or "" (empty string) if no object is detected.
4. You are provided with the `car_1.xml` file, which defines the world's structure, and the simulation code in `problem.py`. Carefully examine these files to understand how the simulation works and how objects are placed in the world. Your solution should be robust to variations in object placement and type.


## Task 2: Pose Estimation (DetectorPos Class)

In this task, your DetectorPos class will be responsible for estimating the car's position within the world. The car's initial position will be slightly randomized. You will receive an image from the simulated camera and must process it to accurately determine the car's x and y coordinates.

**Specific Requirements:** 

1. Implement the `detect(img)` method within the DetectorPos class. This method will receive an image `(img)` as input.
2. Process the image using OpenCV or other suitable image processing techniques to determine the car's `x`and `y` position in the world.
3. The `detect(img)` method should return a tuple `(detected_x, detected_y)` representing the estimated `x` and `y` coordinates of the car.
4. You are provided with the `car_2.xml` file, which defines the world's structure, and the simulation code in `problem.py`. Examine these files to understand how the car's position is randomized. Your solution should be accurate within a small tolerance (0.02) of the true car position.

## General Instructions:

1. You are provided with starter code in solution.py. Implement your solutions within the provided class structures in this file.
2. You can use any standard Python libraries, including OpenCV, NumPy, and others as needed.
3. Your solutions will be tested automatically using modified version of the problem.py script. Ensure that your `detect()` and `result()` (for Task 1) methods function correctly and return the expected data types.
4. You are only required to submit the solution.py file. Do not submit `problem.py` or the XML files.
5. Understanding the provided XML files and the simulation code in `problem.py` is crucial for success in this assignment.
