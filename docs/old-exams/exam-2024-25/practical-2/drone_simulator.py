import mujoco
import numpy as np
import time


from scipy.spatial.transform import Rotation as R


class DroneSimulator:
    def __init__(self, model, data, viewer, steps=1, view=True, rendering_freq = 1):
        self.model = model
        self.data = data
        self.viewer = viewer
        self.steps = steps
        self.view = view
        self.rendering_freq = rendering_freq
        self.position_measurements = [self.data.body("x2").xpos.copy(), self.data.body("x2").xpos.copy()]
        self.orientation_measurements = [self.xquat_to_euler(self.data.body("x2").xquat) for _ in [1, 2]]


    def sim_step(self, thrust, roll_thrust, pitch_thrust, yaw_thrust, steps=1, view=True):
        self.data.actuator("thrust1").ctrl = thrust + roll_thrust - pitch_thrust - yaw_thrust
        self.data.actuator("thrust2").ctrl = thrust - roll_thrust - pitch_thrust + yaw_thrust
        self.data.actuator("thrust3").ctrl = thrust - roll_thrust + pitch_thrust - yaw_thrust
        self.data.actuator("thrust4").ctrl = thrust + roll_thrust + pitch_thrust + yaw_thrust
        for _ in range(steps):
            step_start = time.time()
            mujoco.mj_step(self.model, self.data)
            if view:
                self.viewer.sync()
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep((1/self.rendering_freq)*time_until_next_step)
        return 0


    def position_sensor(self):
        self.position_measurements.pop()
        self.position_measurements.insert(0, self.data.body("x2").xpos.copy())
        return self.position_measurements


    def xquat_to_euler(self, xquat):
        return R.from_quat([xquat[1], xquat[2], xquat[3], xquat[0]]).as_euler('xyz', degrees=True)

    def orientation_sensor(self):
        self.orientation_measurements.pop()
        self.orientation_measurements.insert(0, self.xquat_to_euler(self.data.body("x2").xquat))
        return self.orientation_measurements
