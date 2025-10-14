class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits

        self.integral = 0
        self.prev_error = 0

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt

        # Ableitung
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        # PID-Summe
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Begrenzung
        low, high = self.output_limits
        if low is not None:
            output = max(low, output)
        if high is not None:
            output = min(high, output)

        # Anti-Windup
        if (low is not None and output == low) or (high is not None and output == high):
            self.integral -= error * dt  # kein weiteres Aufintegrieren

        self.prev_error = error
        return output
