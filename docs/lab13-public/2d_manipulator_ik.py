import mujoco
from mujoco import viewer
import numpy as np
from matplotlib import pyplot as plt
import matplotlib
import math

matplotlib.use("Agg")

import time

xml_path = "lab13-public/manipulator.xml"  # xml file
model = mujoco.MjModel.from_xml_path(xml_path)  # MuJoCo model
renderer = mujoco.Renderer(model, height=480, width=640)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
viewer_window = mujoco.viewer.launch_passive(model, data)


N = 100
theta1 = np.pi / 3
theta2 = -np.pi / 2

# initialize (by setting joint angles, teleporting the robot, and stepping the simulation)
data.qpos[0] = theta1
data.qpos[1] = theta2
mujoco.mj_forward(model, data)


position_Q = data.site_xpos[0]  # position of the end-effector

r = 0.5
center = np.array([position_Q[0] - r, position_Q[1]])

phi = np.linspace(0, 2 * np.pi, N)
x_ref = center[0] + r * np.cos(phi)
y_ref = center[1] + r * np.sin(phi)

x_all = []
y_all = []

i = 0
time_local = 0
dt = 0.001
step_size = 0.001


def go_to(expected_theta_1: float, expected_theta_2: float):
    """
    Go to the given joint angles, using P control.
    """
    step = 0
    speed = 0.1
    while True:
        step += 1
        if step > 100000:
            raise Exception("stop")
        current_theta_1 = data.qpos[0]
        current_theta_2 = data.qpos[1]
        data.ctrl[0] = speed * (expected_theta_1 - current_theta_1)
        data.ctrl[1] = speed * (expected_theta_2 - current_theta_2)
        if (
            abs(current_theta_1 - expected_theta_1) < 0.01
            and abs(current_theta_2 - expected_theta_2) < 0.01
        ):
            break
        mujoco.mj_step(model, data)
        viewer_window.sync()
        renderer.update_scene(data)


for i in range(N):
    # Compute Jacobian J

    position_Q = data.site_xpos[0]

    with viewer_window.lock():
        rgba = np.array([1, 0, 0, 1])
        viewer_window.user_scn.ngeom += 1
        mujoco.mjv_initGeom(
            viewer_window.user_scn.geoms[viewer_window.user_scn.ngeom - 1],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([0.01, 0.01, 0.01]),
            pos=position_Q,  # np.array([0, 0, 0.1]),
            mat=np.eye(3).flatten(),
            rgba=rgba.astype(np.float32),
        )
        viewer_window.sync()

    jacp = np.zeros((3, 2))  # 3 is for x,y,z and 2 is for theta1 and theta2
    mujoco.mj_jac(model, data, jacp, None, position_Q, 2)

    # print(jacp)
    J = jacp[[0, 1], :]
    # print(J)

    # Compute inverse Jacobian Jinv
    Jinv = np.linalg.inv(J)
    # print(Jinv)

    # Compute dX
    # dX = X_ref - X
    dX = # TODO
    # print(dX)

    # Compute dq = Jinv*dX
    dq = # TODO 
    # print(dq)

    x_all.append(position_Q[0])
    y_all.append(position_Q[1])

    # update theta1 and theta2
    theta1 # TODO
    theta2 # TODO 
    
    go_to(theta1, theta2)

    mujoco.mj_step(model, data)
    renderer.update_scene(data)
    viewer_window.sync()


plt.figure(1)
plt.plot(x_all, y_all, "bx")
plt.plot(x_ref, y_ref, "r-.")
plt.ylabel("y")
plt.xlabel("x")
plt.gca().set_aspect("equal")
plt.savefig("wynik.png")  # Save the plot to a file
