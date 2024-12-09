import numpy as np
import cv2
import datetime

process_var = 1  # Process noise variance
measurement_var = 1e4  # Measurement noise variance


class KalmanFilter:
    def __init__(self, process_var, measurement_var):
        # dt: time interval
        # process_var: process variance, represents uncertainty in the model
        # measurement_var: measurement variance, represents measurement noise

        # Measurement Matrix
        ## TODO ##
        # Set the measurement matrix H
        self.H = ...

        # Process Covariance Matrix
        self.Q = np.eye(4) * process_var

        # Measurement Covariance Matrix
        self.R = np.eye(2) * measurement_var

        # Initial State Covariance Matrix
        self.P = np.eye(4)

        # Initial State
        self.x = np.zeros((4, 1))

    def predict(self, dt):
        ### TODO ###
        # State Transition Matrix
        A = ...
        x = ...
        P = ...
        ###

    def update(self, measurement):
        # Update the state with the new measurement
        ### TODO ###
        ...
        x = ...
        P = ...
        pass
        ### ###


kf = KalmanFilter(process_var, measurement_var)


class ClickReader:
    def __init__(self, window_name="Click Window"):
        self.window_name = window_name
        self.cur_time = datetime.datetime.now()
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.img = 255 * np.ones((500, 500, 3), np.uint8)

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

            # Exit on pressing the 'ESC' key
            if cv2.waitKey(1) & 0xFF == 27:
                break

        cv2.destroyAllWindows()


click_reader = ClickReader()
click_reader.run()
