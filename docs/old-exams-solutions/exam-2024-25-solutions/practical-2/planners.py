from abc import ABC
import numpy as np


class PathPlanner(ABC):
    def __init__(self):
        pass


    def get_targets(self, current_time):
        if current_time > self.time_stamps[0] and len(self.pos_targets) > 1:
            self.pos_targets.pop(0)
            self.yaw_targets.pop(0)
            self.time_stamps.pop(0)
        
        return [self.pos_targets[0], self.yaw_targets[0]]


class FixedPathPlanner(PathPlanner):
    def __init__(self, pos_targets, yaw_targets, time_stamps):
        self.pos_targets = pos_targets
        self.yaw_targets = yaw_targets
        self.time_stamps = time_stamps
        super().__init__()  


class RandomPathPlanner(PathPlanner):
    def __init__(self, num_yaw_targets, num_pos_targets, sim_time=4500, target_change_period=500):
        # to test the yaw controller:
        fixed_pos_targets = [[0, 0, 1] for _ in range(num_yaw_targets+1)]
        random_yaw_targets = [np.degrees(np.random.uniform(-np.pi/4, np.pi/4)) for _ in range(num_yaw_targets)]

        # to test the position controller
        fixed_yaw_targets = [0 for _ in range(num_pos_targets+1)]
        random_pos_targets = [self._generate_random_target() for _ in range(num_pos_targets)]

        self.pos_targets = fixed_pos_targets + random_pos_targets
        self.yaw_targets = random_yaw_targets + fixed_yaw_targets
        self.time_stamps = list(range(target_change_period, sim_time + target_change_period, target_change_period))
        super().__init__()


    def _generate_random_target(self):
        x = np.random.uniform(-1, 1)
        y = np.random.uniform(-1, 1)
        z = np.random.uniform(0.1, 2)
        return [x, y, z]
