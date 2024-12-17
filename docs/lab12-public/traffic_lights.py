import time
import math
import numpy as np
import mujoco
from mujoco import viewer

model = mujoco.MjModel.from_xml_path("traffic_lights_world.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
viewer = viewer.launch_passive(model, data)
viewer.cam.distance = 6.
viewer.cam.lookat = [0, 2, 0]


def sim_step(forward, turn, steps=1, view=False):
    data.actuator("forward").ctrl = forward
    data.actuator("turn").ctrl = turn
    for _ in range(steps):
        step_start = time.time()
        mujoco.mj_step(model, data)
        if view:
            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

def car_control(gain_prop, gain_int, gain_der):
    # TODO: if needed, add additional variables here

    # Increase the number of iterations for longer simulation
    for _ in range(4000):
        # TODO: implement PID controller
        error_prop = data.body("car").xpos[1] - data.body("traffic_light_gate").xpos[1]

        forward_torque = gain_prop * error_prop
        # Your code ends here

        sim_step(forward_torque, 0, steps=1, view=True)

    viewer.close()

if __name__ == '__main__':
    car_control(gain_prop = -1, gain_int = 0, gain_der = 0)
