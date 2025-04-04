class PID:
    def __init__(
            self, gain_prop: int, gain_int: int, gain_der: int, sensor_period: float,
            output_limits: tuple[float, float]
            ):
        self.gain_prop = gain_prop
        self.gain_der = gain_der
        self.gain_int = gain_int
        self.sensor_period = sensor_period
        # TODO: define additional attributes you might need
        self.error_int = 0
        self.output_min = output_limits[0]
        self.output_max = output_limits[1]
        # END OF TODO


    # TODO: implement function which computes the output signal
    # The controller should output only in the range of output_limits
    def output_signal(self, commanded_variable: float, sensor_readings: list[float]) -> float:
        error_prop = sensor_readings[0] - commanded_variable
        error_der = (sensor_readings[0] - sensor_readings[1])/self.sensor_period
        self.error_int += error_prop*self.sensor_period

        output = self.gain_prop*error_prop + self.gain_der*error_der + self.gain_int*self.error_int

        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min

        return output
    # END OF TODO
