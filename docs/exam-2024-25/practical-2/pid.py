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
        # END OF TODO


    # TODO: implement function which computes the output signal
    # The controller should output only in the range of output_limits
    def output_signal(self, commanded_variable: float, sensor_readings: list[float]) -> float:
        return 0
    # END OF TODO
