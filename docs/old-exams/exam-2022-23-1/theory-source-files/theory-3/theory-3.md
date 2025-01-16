---
title: Exam 2023 Theory-3
usemathjax: true
date:   2023-02-02 12:00:00 +0200
---

<script type="text/javascript" id="MathJax-script" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js">
</script>

\\( \newcommand\mycolv[1]{\begin{bmatrix}#1\end{bmatrix}} \\)

# Theory-3 Transformations

In this task we have a robot that is equipped with a 3D camera,
and the camera's frame is represented by the coordinate system \\( C \\).
The global coordinate system is represented by the coordinate system \\( W \\).
The position and orientation of the camera
(transformation from \\( W \\) to \\( C \\))
is given by a homogeneous transformation matrix \\( H^W_C \\).

## Step 1

Suppose that a point \\( P \\) has coordinates \\( P^C \\) in the camera frame \\( C \\).
Write the expression for the point \\( P^W \\), that is the same point \\( P \\) expressed in the world frame.
Use homogeneous coordinates.

## Step 2

In this step you cannot use the matrix inverse operator \\( M^{-1} \\) directly.
Use your knowledge about the inverse of the homogeneous transformation matrix.

The point \\( P \\) is the same one as in the previous step. Now there is also a point \\( Q \\).
The difference between the two points is given by the vector \\(\delta^W\\)
in the global frame \\( W \\), meaning \\( P^W + \delta^W = Q^W \\) (all in the global frame \\( W \\) ).
Write an expression for the coordinates of the second point \\( Q \\) in the camera frame \\( C \\), that is \\( Q^C \\).

Express the result using \\( P^C \\), \\(\delta^W \\) and component blocks of \\( H^W_C \\).

## Step 3

Use the result from step 1 and step 2 to find the coordinates of point \\( Q \\) in the camera frame,
assuming the following values:

$$
H^W_C = \begin{bmatrix} \cos(45^\circ) & -\sin(45^\circ) & 0 & 1 \cr
\sin(45^\circ) & \cos(45^\circ) & 0 & 2 \cr
0 & 0 & 1 & 3 \cr
0 & 0 & 0 & 1 \end{bmatrix}
$$


$$
P^C = \mycolv{ p^C_x \cr p^C_y \cr p^C_z} = \mycolv{3 \cr 2 \cr 1}
$$


$$
\delta^W = \mycolv{d_x \cr d_y \cr d_z} = \mycolv{2 \cr 0 \cr -1}
$$
