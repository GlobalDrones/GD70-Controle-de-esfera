import math
import time
from pid_controller import PIDController

class SphereSimulation:
    def __init__(self, radius, mass, initial_position=0, initial_velocity=0):
        self.radius = radius
        self.mass = mass
        self.position = initial_position
        self.velocity = initial_velocity
        self.acceleration = 0

    def update(self, force, dt):
        self.acceleration = force / self.mass
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt

def simulate_sphere():
    sphere = SphereSimulation(radius=0.1, mass=1.0)
    pid = PIDController(kp=1.0, ki=0.1, kd=0.01, setpoint=1.0)
    dt = 0.01
    duration = 10.0
    current_time = 0.0

    while current_time < duration:
        force = pid.update(sphere.position, dt)
        sphere.update(force, dt)
        print(f"Time: {current_time:.2f}s, Position: {sphere.position:.2f}, Velocity: {sphere.velocity:.2f}")
        time.sleep(dt)
        current_time += dt

if __name__ == "__main__":
    simulate_sphere()
