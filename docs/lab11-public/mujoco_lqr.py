"""
This script demonstrates the use of MuJoCo for simulating and controlling a cart-pole system using Linear Quadratic Regulator (LQR).

Functions:
- sim_reset: Resets the simulation environment.
- sim_step: Performs a simulation step with the given control input.

Global Variables:
- VIEW: Boolean flag to enable or disable the viewer.
- queue: List to store simulation data.
- renderer: Renderer object for visualizing the simulation.
"""

import time
import mujoco
from mujoco import viewer
import numpy as np
import control
import imageio

VIEW = False
queue = []
renderer = None


def sim_reset():
    global model, data, viewer_window, renderer
    if "viewer_window" in globals():
        viewer_window.close()
    model = mujoco.MjModel.from_xml_path("mujoco_cartpole.xml")
    renderer = mujoco.Renderer(model, height=480, width=640)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if VIEW:
        viewer_window = viewer.launch_passive(model, data)


def sim_step(forward):
    global renderer, queue
    data.actuator("slide").ctrl = forward
    step_size = 0.01
    step_start = time.time()
    mujoco.mj_step(model, data)

    if renderer is not None:
        renderer.update_scene(data)
        frame = renderer.render()
        queue.append(frame)
    if VIEW:
        viewer_window.sync()

        time_until_next_step = step_size - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def linearize():
    # fill in the matrix A and B
    return A, B


def one_run():
    for _ in range(50):
        sim_step(0.03)

    # TODO: maybe you want to perform some precomputation here
    A, B = linearize()  # TODO: maybe you need to add some parameters
    Q = np.diag([0, 0, 0, 0])  # TODO: fill in the matrix (vector)
    R = [0]  # TODO: fill in the matrix (one value)
    K = control.lqr(A, B, Q, R)[0]
    print("K", K)
    for _ in range(400):
        angle = data.joint("hinge").qpos[0]
        print("angle", angle)
        angle_vel = data.joint("hinge").qvel[0]
        xpos = data.body("cart").xpos[0]
        xvel = data.body("cart").cvel[3]
        state = np.array([xpos, xvel, angle, angle_vel])
        sim_step(-(K @ state)[0])


def save_video(queue, filename, fps):
    writer = imageio.get_writer(filename, fps=fps)
    for frame in queue:
        writer.append_data(frame)
    writer.close()


if __name__ == "__main__":
    sim_reset()
    one_run()
    # print(model.body_mass)
    # save_video(queue, "lab11-public/mujoco-controlled.mp4", 20)
