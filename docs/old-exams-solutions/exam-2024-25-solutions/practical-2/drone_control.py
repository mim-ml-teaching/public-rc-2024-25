import mujoco
from mujoco import viewer
import numpy as np
import pandas as pd


model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
viewer = viewer.launch_passive(model, data)
viewer.cam.distance = 4.
viewer.cam.lookat = np.array([0, 0, 1])
viewer.cam.elevation = -30.


from drone_simulator import DroneSimulator
from pid import PID
from planners import FixedPathPlanner, RandomPathPlanner


SIM_TIME = 4500
RANDOM_PLANNER = 1


if RANDOM_PLANNER:
    planner = RandomPathPlanner(
        num_pos_targets=5, num_yaw_targets=2, sim_time=SIM_TIME, target_change_period=500
        )
else:
    # You can choose a specific path for testing purposes below
    # Make sure to revert these changes before submitting the files
    fixed_pos_targets = [
        [0, 0, 1], [0, 0, 1], [0, 0, 1], [1, -1, 0.2],
        [-0.3, 0.7, 0.4], [-1, -1, .5], [0.8, 0.4, 1.7], [1, 1, 1.5]
        ] # [x, y, z] positions
    fixed_yaw_targets = [30, -45, 0, 0, 0, 0, 0, 0] # degrees
    fixed_time_stamps = [500, 1000, 1500, 2000, 2500, 3000, 3500, 4000] # when the next target is set

    planner = FixedPathPlanner(
        pos_targets=fixed_pos_targets, yaw_targets=fixed_yaw_targets,
        time_stamps=fixed_time_stamps
        )


if __name__ == '__main__':
    # If you want the simulation to be displayed more slowly, decrease rendering_freq
    # Note that this DOES NOT change the timestep used to approximate the physics of the simulation!
    drone_simulator = DroneSimulator(model, data, viewer, rendering_freq = 1)

    # TODO: Design PID control
    pid_roll = PID(
        gain_prop = 0.05, gain_int = 0.001, gain_der = 0.01,
        sensor_period = model.opt.timestep, output_limits=(-1, 1)
    )

    pid_pitch = PID(
        gain_prop = 0.05, gain_int = 0.001, gain_der = 0.01,
        sensor_period = model.opt.timestep, output_limits=(-1, 1)
    )

    pid_yaw = PID(
        gain_prop = 0.1, gain_int = 0.001, gain_der = 0.04,
        sensor_period = model.opt.timestep, output_limits=(-1, 1)
    )

    pid_position = PID(
        gain_prop = -3, gain_int = 0, gain_der = -5,
        sensor_period = model.opt.timestep, output_limits=(-10, 10)
    )

    pid_thrust = PID(
        gain_prop = 0.3, gain_int = 0.001, gain_der = 0.5,
        sensor_period = model.opt.timestep, output_limits=(-1, 1)
    )
    # END OF TODO

    for i in range(SIM_TIME):
        current_pos, previous_pos = drone_simulator.position_sensor()
        current_orien, previous_orien = drone_simulator.orientation_sensor()

        pos_target, yaw_target = planner.get_targets(i)

        # TODO: use PID controllers to steer the drone
        desired_thrust = 3.2496 - pid_thrust.output_signal(pos_target[2], [current_pos[2], previous_pos[2]])

        desired_roll = -pid_position.output_signal(pos_target[1], [current_pos[1], previous_pos[1]])
        desired_pitch = pid_position.output_signal(pos_target[0], [current_pos[0], previous_pos[0]])
        desired_yaw = yaw_target

        roll_thrust = pid_roll.output_signal(desired_roll, [current_orien[0], previous_orien[0]])
        pitch_thrust = pid_pitch.output_signal(desired_pitch, [current_orien[1], previous_orien[1]])
        yaw_thrust = -pid_yaw.output_signal(desired_yaw, [current_orien[2], previous_orien[2]])
        # END OF TODO

        data = np.array([pos_target + [desired_roll, desired_pitch, desired_yaw], np.concat([current_pos, current_orien])]).T

        row_names = ["x", "y", "z", "roll", "pitch", "yaw"]
        headers = ["desired", "current"]

        print(pd.DataFrame(data, index=row_names, columns=headers))

        drone_simulator.sim_step(
            desired_thrust, roll_thrust=roll_thrust,
            pitch_thrust=pitch_thrust, yaw_thrust=yaw_thrust
            )
