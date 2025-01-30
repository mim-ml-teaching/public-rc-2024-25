class PID:
    def __init__(self, gain_prop, gain_int, gain_der, sensor_period):
        self.gain_prop = gain_prop
        self.gain_der = gain_der
        self.gain_int = gain_int
        self.sensor_period = sensor_period

        # TODO: initialize additional attributes
        self.error_int = 0
        self.previous_error = 0

    def output_signal(self, commanded_variable, sensor_reading):
        # TODO: compute output signal of the controller
        error_prop = sensor_reading - commanded_variable
        error_der = (error_prop - self.previous_error) / self.sensor_period
        self.error_int += error_prop*self.sensor_period
        self.previous_error = error_prop

        return self.gain_prop*error_prop + self.gain_der*error_der + self.gain_int*self.error_int
