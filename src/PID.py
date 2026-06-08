class PID:
    """
    Discrete-time PID controller.

    output = Kp * error + Ki * integral(error) + Kd * derivative(error)
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float = 0.0,
        output_limits: tuple[float | None, float | None] = (None, None),
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.setpoint = setpoint

        self.integral = 0.0
        self.prev_error = 0.0

        self.min_output, self.max_output = output_limits

    def reset(self) -> None:
        """
        Reset controller state.
        """
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, measurement: float, dt: float) -> float:
        """
        Compute the next control output.

        Parameters:
        - measurement : float
            - Current process value.
        dt : float
            - Time step in seconds.

        Returns:
        - float: Controller output.
        """
        if dt <= 0:
            raise ValueError("dt must be positive")

        error = self.setpoint - measurement

        self.integral += error * dt

        derivative = (error - self.prev_error) / dt

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        if self.min_output is not None:
            output = max(output, self.min_output)

        if self.max_output is not None:
            output = min(output, self.max_output)

        self.prev_error = error

        return output