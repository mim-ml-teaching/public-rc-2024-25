---
title: Lab 9
usemathjax: true
---
<script type="text/javascript" id="MathJax-script" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js">
</script>

# Kalman filter

In this class, you will implement a basic [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) along with a mouse-tracking example similar to what you've seen on [lecture 6](https://docs.google.com/presentation/d/e/2PACX-1vSz1_lcR62_W_70RWyGCPF-Fo27qdCqLja3BiG2DSFc6O-GOUDKi9tNskNLJgnpWl0bnpCcE5ucM9I4/pub?start=false&loop=false&delayms=3000#slide=id.g1ec40388a3a_0_17).
The model is overly simplified, as we are going to assume constant velocity (a more elaborate constant acceleration model would yield better results), and we assume no control input.

Kalman filter is used to approximate true state of the world given noisy observations (e.g. sensor noise). 

Some notation: at timestep \\(k\\):
- \\(z_k\\) denotes the observation
- \\(t_k\\) denotes the true state of the world
- \\(x_k\\) denotes the estimate of the state of the world
- \\(P_k\\) denotes the  covariance of the estimate of the state of the world
- \\(A_k\\) is a matrix that denotes the state transition model (note that since we are dealing with discrete model, this matrix might depent on time elapsed from previous timestep)
- \\(v_k\\) and \\(w_k\\) denote random (gaussian and independent) noise

Kalman filter assumes that world state transitions in a following manner:

$$ t_{k} = A_k t_{k-1} + v_k  $$


Kalman filter has 3 matrix parameters:

- measurement matrix \\(H\\) which denotes how states are mapped to the observation. To be precise 
  $$ z_k = H t_k + w_k $$
- process covariance matrix \\(Q\\) which is the covariance of the random noise \\(v_k\\) (\\(v_k\\) comes from distribution \\(N(0, Q)\\))
- measurement covariance matrix \\(R\\) which is the covariance of the random noise \\(w_k\\) (\\(w_k\\) comes from distribution \\(N(0, R)\\)) 

Kalman filter has two phases:

- **predict** when it predicts the new state and updates covariance matrix. Here we rely solely on the linear model of the state transitions.

  $$ x'_k = A_k x_{k-1} $$

  $$ P'_k = A_k P_k A_k^T + Q $$
  

- **update** when it updates the predictions based on observations
  
  $$ y = z_k - Hx'_k $$

  $$ S = HP'_kH^T + R$$

  $$ K = P'_kH^TS^{-1} $$

  $$ x_k = x'_k + Ky$$

  $$ P_k = (I - KH)P'_k  $$

If we don't have an observation from the current timestep, then \\(x_k = x'_k\\) and \\(P_k = P'_k\\).

## Example calculation

In this example, we will observe one variable - position on the x-axis - and try to predict it assuming constant velocity.
Our state will be of the form [position, velocity]<sup>T</sup>.
Since we can only observe position, out measurement matrix is
```
H = [1, 0]
``` 
Our model assumes that the velocity is constant.
We assume that the process covariance matrix of the model is
```
Q = [[1, 0]
     [0, 1]]
```
And that measurement covariance matrix is
```
R = [10000]
```
(This means that a variance in our measurement is very high - we are often off by more than a 100 units).
We also assume our current state and it's covariance looks like that:
```
x = [231.97740566  56.30126413]^T
P = [[3288.01907438  1115.25715518]
     [1115.25715518  693.39189069]]
```

Firstly, we make a prediction. If the time elapsed from last step is `0.002541`, then the A matrix looks as follows:
```
A = [[1.       0.002541]
     [0.       1.      ]]
```

Which gives us
```
x' = A @ x = [232.12046717  56.30126413]^T

P' = A @ P @ A.T + Q = [[3294.69128825  1117.01906397]
                        [1117.01906397  694.39189069]]

```
Next, let's assume we measured our position `z = 313`.
We correct the state to include the observation

```
y = z - H @ x' = [80.87953283]

S = H @ P' @ H.T + R = [13294.69128825]

K = P' @ H.T @ inv(S) = [0.24782007, 0.08401993] # In this case S is actually a scalar, so no need to invert a matrix

x = x' + K @ y = [252.16403843  63.09675701]

P = (I - K @ H) @ P' = [[2478.20067185  840.19932449]
                        [840.19932449  600.54002439]]


```


**Note**: that measurement matrix (H) and that state transition matrix (A) are unit independent, i.e., they
are the same regardless whether we measure our position in meters, kilometers or pixels. 
On the other hand entries in both covariance matrices depend on the units.

## Task 1 - filter implementation

You are given kalman.py file with a partialy implemented Kalman filter.
Implement the measurement matrix
```python
        # Measurement Matrix
        ## TODO ##
        # Set the measurement matrix H
        self.H = ...
```
Then implement predicting the next state
```python
    def predict(self, dt):
        ### TODO ###
        # State Transition Matrix
        A = ...
        x = ...
        P = ...
        ###
```
\\(A\\) should be a state transition matrix with the assumption that velocity is constant.

Finally, implement update of the state and covarience matrix after measurement.
```python
    def update(self, measurement):
        # Update the state with the new measurement
        ### TODO ###
        ...
        x = ...
        P = ...
        pass
        ### ###
```

## Task 2 - mouse tracking

In a file kalman.py there is a partly implemented mouse tracker. 

Complete implementation of `mouse_callback` so that it makes prediction and calls update method of kalman filter. 
```python
    def mouse_callback(self, event, x, y, flags, param):
        # Check if the event is a left button click
        if event == cv2.EVENT_LBUTTONDOWN:
            current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print(f"Time: {current_time}, Position: ({x}, {y})")
            new_time = datetime.datetime.now()
            ### TODO ###
            # Predict the next state

            ###
            self.cur_time = new_time

            cv2.circle(self.img, (x, y), 2, (0, 0, 255), -1)  # Red color, filled circle

            ### TODO ###
            # Update the state with the new measurement

            ###
            print(f"Updated State: {kf.x}")
```

Then, complete the `run` method so that it predicts the state and draws a circle where the mouse click should be

```python
  def run(self):
        # Main loop to display the window
        while True:
            # Display an empty image (or any image you want to display)

            new_time = datetime.datetime.now()

            ### TODO ###
            # Predict the next state

            self.cur_time = new_time

            ### TODO ###
            # Use the predicted state to draw a circle on the image
            x = ...
            y = ...
            cv2.circle(
                self.img, (int(x), int(y)), 2, (255, 0, 0), -1
            )  # Red color, filled circle
            cv2.imshow(self.window_name, self.img)
```

End result should look something like this:

![mouse tracker](mouse_filter.png "Mouse Tracker")
