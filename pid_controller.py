class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.error = 0
        self.integral = 0
        self.derivative = 0
        self.last_error = 0

    def update(self, measured_value, dt):
        self.error = self.setpoint - measured_value
        self.integral += self.error * dt
        self.derivative = (self.error - self.last_error) / dt
        output = self.kp * self.error + self.ki * self.integral + self.kd * self.derivative
        self.last_error = self.error
        return output
