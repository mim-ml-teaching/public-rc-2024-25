import mujoco
import numpy as np
import time

class DroneSimulator:
    def __init__(
        self, model, data, viewer, desired_altitude, steps=1, view=True,
        altitude_sensor_freq = 1, wind_change_prob = 0, wind_strength = 0,
        rendering_freq = 1
        ):
        self.model = model
        self.data = data
        self.viewer = viewer
        self.steps = steps
        self.view = view
        self.desired_altitude = desired_altitude
        self.altitude_history = [0, 0, 0, 0, 0]
        self.measured_altitudes = [self.data.body("x2").xpos[2], self.data.body("x2").xpos[2]]
        self.altitude_sensor_freq = altitude_sensor_freq
        self.altitude_sensor_period = (1/altitude_sensor_freq)*model.opt.timestep
        self.__readings_counter__ = 0
        self.wind_change_prob = wind_change_prob
        self.wind_strength = wind_strength
        self.rendering_freq = rendering_freq

    def sim_step(self, thrust, steps=1, view=True):
        if np.random.uniform() < self.wind_change_prob:
            self.wind_strength = np.random.uniform(-1, 1)
        self.data.actuator("thrust1").ctrl = thrust + self.wind_strength
        self.data.actuator("thrust2").ctrl = thrust + self.wind_strength
        self.data.actuator("thrust3").ctrl = thrust + self.wind_strength
        self.data.actuator("thrust4").ctrl = thrust + self.wind_strength
        for _ in range(steps):
            step_start = time.time()
            mujoco.mj_step(self.model, self.data)
            self.update_altitude_tracker()
            self.report_altitudes()
            self.altitude_sensor()
            if view:
                self.viewer.sync()
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep((1/self.rendering_freq)*time_until_next_step)

        return 0

    def update_altitude_tracker(self):
        self.altitude_history[2] = self.altitude_history[1]
        self.altitude_history[1] = self.altitude_history[0]
        self.altitude_history[0] = self.data.body("x2").xpos[2]
        if self.altitude_history[0] < self.altitude_history[1] and not self.altitude_history[1] < self.altitude_history[2]:
            self.altitude_history[4] = self.altitude_history[1]
        if self.altitude_history[0] > self.altitude_history[1] and not self.altitude_history[1] > self.altitude_history[2]:
            self.altitude_history[3] = self.altitude_history[1]

    def report_altitudes(self):
        print(f"Desired altitude: {self.desired_altitude}")
        print(f"Current altitude: {self.altitude_history[0]}")
        print(f"Most recent max: {self.altitude_history[4]}")
        print(f"Most recent min: {self.altitude_history[3]}")

    def altitude_sensor(self):
        self.__readings_counter__ += 1
        if int((self.__readings_counter__+1)*self.altitude_sensor_freq) > int(self.__readings_counter__*self.altitude_sensor_freq):
            self.measured_altitudes[1] = self.measured_altitudes[0]
            self.measured_altitudes[0] = self.data.body("x2").xpos[2]
        return self.measured_altitudes