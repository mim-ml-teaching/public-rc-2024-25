---
title: Exam 2023 Practical 1
---

# Practical Task 1

This task is worth 15 points, submit through the moodle.
## Changes

```
dictionaryId = '6x6_250'
```

## Guidelines

1. Each subtask is assigned a weight of 25% in terms of points.
2. All answers must be submitted in a single python script file (.py)
3. Your name should be added at the start of the file.
4. If you use any libraries that are not part of the standard Python libraries, you must include a comment explaining this at the start of the file.

## Problem

You are given two photos

![](frame-002.png)
![](frame-253.png)

Both those photos are made by the same camera with the same camera matrix.
You can read the camera matrix from the file `calibration.pckl` with the following code:

```python
import pickle
f = open("calibration.pckl", "rb")
cameraMatrix, distCoeffs, _, _ = pickle.load(f)
```

Both: `cameraMatrix` and `distCoeffs` are numpy arrays in format accepted by OpenCV.

# Subtasks

1. There is an arucoboard in the first image. Detect it and draw markers using `drawDetectedMarkers` function, squareLength is equal 0.026 while markerLength is equal 0.015. Dictionary id is '6x6_250'
2. Using drawFrameAxes function or similar, draw the coordinate system of every detected marker.
3. Find the pose of the marker one (1) on the first image.
4. Pose of the marker one (1) on the second image is equal to:
```
trn = [[0.01428826]
       [0.02174878]
       [0.37597986]]
rot = [[ 1.576368  ]
       [-1.03584672]
       [ 0.89579336]]
```
Draw the missing marker (blue color) on the second image using given pose.
