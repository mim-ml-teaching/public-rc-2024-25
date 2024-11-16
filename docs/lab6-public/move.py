import mujoco
import numpy as np
from mujoco import viewer
import time

# TODO: better file path
model = mujoco.MjModel.from_xml_path("lab6-public/robot.xml")
data = mujoco.MjData(model)

viewer = viewer.launch_passive(model, data)

step = 0
while True:
    step_start = time.time()
    step += 1

    if step % 1000 == 0:
        data.actuator("ac_x").ctrl = np.random.uniform(-1.1, 1.1)
        data.actuator("ac_y").ctrl = np.random.uniform(-1.1, 1.1)
        data.actuator("ac_z").ctrl = np.random.uniform(-1.1, 1.1)
        data.actuator("ac_r").ctrl = np.random.uniform(-3, 3)
        print(data.joint("box1-joint").qpos)
    mujoco.mj_step(model, data)
    viewer.sync()
    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)
