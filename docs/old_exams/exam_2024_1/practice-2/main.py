import mujoco
from mujoco import viewer
from car_simulator import CarSimulator
from pid import PID


def simulate(cs, desired_pos):
    # TODO: design control for the system
    ...
    # TODO end

    for _ in range(3_000):
        # TODO: find value for `forward_torque` using designed control
        forward_torque = 0
        # TODO end

        print('-' * 20)
        print(f'Gyro: {cs.wheel_angular_vel}')
        print(f'GPS: {cs.position}')
        print(f'Forward_torque: {forward_torque}')
        print()

        cs.sim_step(forward_torque)

    cs.viewer.close()


if __name__ == '__main__':
    # If you want the simulation to be displayed more slowly, decrease rendering_freq
    # Note that this DOES NOT change the timestep used to approximate the physics of the simulation!
    cs = CarSimulator(gps_freq=0.01, rendering_freq=1)
    desired_pos = cs.data.body("traffic_light_gate").xpos[1]

    simulate(cs=cs, desired_pos=desired_pos)
