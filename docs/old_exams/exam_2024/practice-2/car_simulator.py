import mujoco
from mujoco import viewer
import numpy as np
import time


class CarSimulator:
    def __init__(self, view=True, gps_freq=0.01, rendering_freq=1):
        self.model = mujoco.MjModel.from_xml_path("traffic_lights_world.xml")
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.view = view
        self.viewer = viewer.launch_passive(self.model, self.data)
        self.viewer.cam.distance = 6.
        self.viewer.cam.lookat = [0, 2, 0]
        self.rendering_freq = rendering_freq

        self.gps_freq = gps_freq

        self.wheel_gyroscope_period = self.model.opt.timestep
        self.gps_period = 1/self.gps_freq * self.model.opt.timestep

        self.position = self.data.body("car").xpos[1]
        self.wheel_angular_vel = self.data.sensor("wheel_angular_vel").data[2]
        self.total_steps = 0

    def sim_step(self, forward):
        self.total_steps += 1
        self.data.actuator("forward").ctrl = forward

        step_start = time.time()
        mujoco.mj_step(self.model, self.data)

        if self.total_steps % int(1/self.gps_freq) == 0:
            self.position = self.data.body("car").xpos[1]

        self.wheel_angular_vel = self.data.sensor("wheel_angular_vel").data[2]

        if self.view:
            self.viewer.sync()
            time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep((1/self.rendering_freq)*time_until_next_step)
