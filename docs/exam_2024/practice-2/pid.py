class PID:
    def __init__(self, gain_prop, gain_int, gain_der, sensor_period):
        self.gain_prop = gain_prop
        self.gain_der = gain_der
        self.gain_int = gain_int
        self.sensor_period = sensor_period
        # TODO: initialize additional attributes
        ...

    def output_signal(self, commanded_variable, sensor_reading):
        # TODO: compute output signal of the controller
        ...
