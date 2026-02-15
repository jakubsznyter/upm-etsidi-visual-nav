import math

class RobotOdometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, v, omega, dt):
        """Actualiza la posición y orientación del robot."""
        delta_s = v * dt
        delta_theta = omega * dt
        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalizar