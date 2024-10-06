import pybullet as p
import pybullet_data
import time
import random
from scipy.spatial import distance

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

def build_world():
  global robot, obstacle, cube
  p.resetSimulation()
  p.setGravity(0, 0, -10)
  p.loadURDF("plane.urdf", [0, 0, -0.1], useFixedBase=True)
  robot = p.loadURDF("robot.urdf")
  obstacle = p.loadURDF("cube.urdf", [random.uniform(-1, 1), 0.4, 0.1], globalScaling = 0.4)
  cube = p.loadURDF("cube.urdf", [random.uniform(-0.6, 0.6), -0.3, 0.1], globalScaling = 0.2)

def cube_pos():
  return p.getBasePositionAndOrientation(cube)[0]

def obstacle_pos():
  return p.getBasePositionAndOrientation(obstacle)[0]

def simulate(x, y, z, pos, targetVelocity, steps=5000):
  p.setJointMotorControl2(robot, 0, p.POSITION_CONTROL, z, maxVelocity=targetVelocity)
  p.setJointMotorControl2(robot, 1, p.POSITION_CONTROL, y, maxVelocity=targetVelocity)
  p.setJointMotorControl2(robot, 2, p.POSITION_CONTROL, x, maxVelocity=targetVelocity)
  p.setJointMotorControl2(robot, 3, p.POSITION_CONTROL, pos, targetVelocity=targetVelocity)
  for i in range(steps):
     p.stepSimulation()


# START
# YOUR CODE GOES HERE ...

def move_block():
  pos = cube_pos()
  obs = obstacle_pos()
  # this example achieves 0.37 success rate
  simulate(1, 0, 0, 0, 0.1)
  simulate(1, pos[1], 0, 0, 0.1)
  simulate(0.1, pos[1], 0, 0, 0.1)
  simulate(0.1, -1, 0, 0, 0.1)
  simulate(0, -1, 0, 0, 0.1)
  simulate(0, 0.7, 0, 0, 0.1)

# DON'T CHANGE ANYTHING BELOW
# END

NUMBER_OF_TESTS = 100

def test_exam():
  success = 0
  for seed in range(NUMBER_OF_TESTS):
    random.seed(seed)
    print("test", seed)
    build_world()
    obs_pos = obstacle_pos()
    move_block()
    pos = cube_pos()
    if distance.euclidean(pos, [0, 0.8, 0]) < 0.1 and distance.euclidean(obs_pos, obstacle_pos()) < 0.1:
      success += 1
  print("Success rate", success / NUMBER_OF_TESTS)
  return (success / NUMBER_OF_TESTS)

test_exam()
