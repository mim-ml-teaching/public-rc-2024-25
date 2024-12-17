# LQR control

<script type="text/javascript"
src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.3/MathJax.js?config=TeX-AMS-MML_HTMLorMML">
</script>
In this class we will implement LQR control for a cartpole system. We will use the [control](https://python-control.readthedocs.io/) library to design the controller and simulate the system.

The lab consists of three parts:

1. We will use the `control` library for a simple linear system. Just to get a feel for the library.
2. We will use the `control` library for a cartpole system. We will use the linearized dynamics of the cartpole system. The system will be simulated using  <https://github.com/microsoft/cartpole-py/blob/main/cartpole.py> which is a Python implementation of the cartpole system from the Microsoft.
3. We will use the `control` library for a cartpole system simmulated using MuJoCo.

In the second step, you need to linearize the cartpole system at a fixed point using the basic simulation source code you have. Then, employ this linearization in the third step.

## Part 1: Simple linear system

The code for this part is in the `timy_lqr.py` file in this repo. The code is also shown below, for your convenience.

We have a simple linear System

```python
import control as ctrl
import numpy as np

# Define system matrices
A = np.array([[1, 2], [3, 4]])
B = np.array([[5], [6]])

class System:
    def __init__(self):
        self.x = np.array([[1], [2]])  # Initial state

    def measure_state(self):
        return self.x

    def apply_control(self, u):
        dt = 0.01
        self.x = self.x + np.dot(A, self.x) * dt + np.dot(B, u) * dt

    def print_state(self):
        print(f"{self.x[0].item():.2f}, {self.x[1].item():.2f}")
```

The system is simulated using the following code:

```python
system = System()

for _ in range(10):
    x = system.measure_state()
    # Compute control signal
    signal = random.uniform(-1, 1)

    system.apply_control(signal)
    system.print_state()
```

If you run the code, you should see the system moves randomly. We want to design a controller that stabilizes the system to the origin `[0,0]`. We will use the `control` library to design the controller. First, let's prepare the matrices for LQR controller and compute the gains. This is an operation we have to do only once, so we make this computations outside the loop.

```python
# Define cost matrices
Q = np.array([[1, 0], [0, 1]])
R = np.array([[1]])

# Compute LQR controller gain
K, S, E = ctrl.lqr(A, B, Q, R)

# K is your controller gain
print("Controller gain K:", K)
```

Now, we can use the controller gain `K` to stabilize the system. We will use the following code to simulate the system - note that we have increased the number of iterations in the `for` loop to have a longer observation.

```python
for _ in range(1000):
    # Measure or estimate the current state
    x = system.measure_state()

    # Calculate the control input
    u = -np.dot(K, x)

    # Apply the control input to the system
    system.apply_control(u)

    # Wait for the system to react before next iteration
    # (This could be a time delay in a real-time system)
    time.sleep(0.01)

    # Print the current state
    system.print_state()
```

Try to run the code. You should see that the system is stabilized to the origin.

Check what happens if you change the cost matrices `Q` and `R`. What happens if you increase the cost of the control signal `R`? What happens if you increase the cost of the state `Q`? Maybe make a plot of the state trajectory for different values of `Q` and `R`.

## Part 2: Cartpole system - using specific simulation code

In this part, we will use the cartpole system from Microsoft. The code is available at <https://github.com/microsoft/cartpole-py/blob/main/cartpole.py>, but we have included it here for convenience (with some minor modifications).

You should download the `simple-cartpole.py` file from this repo and run it.

The crucial part of the code is the `step` function that simulates the cartpole system. The function takes a command as an input and returns the state of the system. You can see it below:

```python

# Constants
GRAVITY = 9.8  # a classic...
CART_MASS = 0.31  # kg
POLE_MASS = 0.055  # kg
TOTAL_MASS = CART_MASS + POLE_MASS
POLE_HALF_LENGTH = 0.4 / 2  # half the pole's length in m
POLE_MASS_LENGTH = POLE_MASS * POLE_HALF_LENGTH
FORCE_MAG = 1.0
STEP_DURATION = 0.02  # seconds between state updates (20ms)
TRACK_WIDTH = 1.0  # m
FORCE_NOISE = 0.02  # % of FORCE_MAG


# Model parameters
class CartPoleModel:
    def step(self, command: float):
        # We are expecting the input command to be -1 or 1,
        # but we'll support a continuous action space.
        # Add a small amount of random noise to the force so
        # the policy can't succeed by simply applying zero
        # force each time.
        force = FORCE_MAG * command + random.uniform(-FORCE_NOISE, FORCE_NOISE)

        cosTheta = math.cos(self._pole_angle)
        sinTheta = math.sin(self._pole_angle)

        temp = (
            force + POLE_MASS_LENGTH * self._pole_angular_velocity**2 * sinTheta
        ) / TOTAL_MASS
        angularAccel = (GRAVITY * sinTheta - cosTheta * temp) / (
            POLE_HALF_LENGTH * (4.0 / 3.0 - (POLE_MASS * cosTheta**2) / TOTAL_MASS)
        )
        linearAccel = temp - (POLE_MASS_LENGTH * angularAccel * cosTheta) / TOTAL_MASS

        self._cart_position = self._cart_position + STEP_DURATION * self._cart_velocity
        self._cart_velocity = self._cart_velocity + STEP_DURATION * linearAccel

        self._pole_angle = (
            self._pole_angle + STEP_DURATION * self._pole_angular_velocity
        )
        self._pole_angular_velocity = (
            self._pole_angular_velocity + STEP_DURATION * angularAccel
        )

        # Use the pole center, not the cart center, for tracking
        # pole center velocity.
        self._pole_center_position = (
            self._cart_position + math.sin(self._pole_angle) * POLE_HALF_LENGTH
        )
        self._pole_center_velocity = (
            self._cart_velocity
            + math.sin(self._pole_angular_velocity) * POLE_HALF_LENGTH
        )
```

When you run the code, you should see the cartpole system in action. Since there is no controller, the system will fall down (the pole angle will be greater than 45 degrees) as seen on a video below.

<video width="512" height="208" controls>
  <source src="simple-free.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

Check what happens if you change the initial cart position and pole angle. What happens if you remove the noise from the force? What happens if you change the mass of the cart or the pole? What happens if you change the length of the pole? What happens if you change the gravity constant (ok, it's a joke)?

Your task is to design an LQR controller that stabilizes the cartpole system to the origin.The linearization should be done at a fixed point.
You should write a function that returns the matrices `A` and `B` of the linearized system.  You should know how to linearize the system from the previous classes.

(hint) You may find following equations usefull:

$$\ddot{\theta} = \frac{(M+m)g\sin\theta - \cos\theta \left[ F + ml\dot{\theta}^2 \sin\theta \right]}{\left( \frac{4}{3} \right)(M+m) - ml\cos^2\theta}$$

$$\ddot{x} = \frac{ \left\{ F + ml \left[ \dot{\theta}^2 \sin\theta - \ddot{\theta}\cos\theta \right] \right\} }{M+m}$$

```python
def linearize(): # you can add arguments if you want
    #TODO
    
    A = np.array(
    #TODO
    )
    B = np.array(
    #TODO
    )
    return A, B


def controlled():
    pole = CartPoleModel(initial_cart_position=-3)
    state = pole.state()
    A, B = linearize() #maybe add arguments
    Q = # TODO
    R = # TODO
    K = control.lqr(A, B, Q, R)[0]
    print("K", K)
    for i in range(10000):
        if pole.halted():
            print("halted")
            break
        force = -(
            K
            @ np.array(
                [
                    state["cart_position"],
                    state["cart_velocity"],
                    state["pole_angle"],
                    state["pole_angular_velocity"],
                ]
            )
        )[0]
        if force > 1:
            force = 1
        if force < -1:
            force = -1
        print("force", force)
        pole.step(force)
        state = pole.state()
        visualize(state)
        print_state(state, "current")

controlled()
```

After filling in the missing parts (marked by comments and TODOs), you should see the cartpole system stabilized to the origin as seen on a video below.

<video width="512" height="208" controls>
  <source src="simple-controlled.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

Please note, that system starts in -3 position, so it is not stabilized in the center. Also note, that we limit the force to be between -1 and 1.

## Part 3: Cartpole system - using MuJoCo

In this part, we will use the cartpole system from MuJoCo. Cartpole XML is a modified version of the cartpole.xml from MuJoCo and is available in this repo with `mujoco_cartpole.xml` name. If you want to find mass of the cart and pole, you can do it by using `print(model.body_mass)` after loading the model.

Using the stub code from `mujoco_lqr.py`. You should be able to stabilize the cartpole system using MuJoCo. You don't need to change linearization code from the previous part. You should only change the simulation code.

You should see the cartpole system stabilized to the origin as seen on a video below. Note, that
at the beginning the system is not stabilized in the center because it takes 50 steps with force 0.03 as
in provided code.

<video width="500" height="400" controls>
  <source src="mujoco-controlled.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
