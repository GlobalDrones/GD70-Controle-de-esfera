import math
import time
from pid_controller import PIDController

class SphereSimulation:
    def __init__(self, radius, mass, rope_length, rope_torsional_stiffness, initial_position=(0, 0, 0), initial_velocity=(0, 0, 0), initial_heading=0):
        self.radius = radius
        self.mass = mass
        self.rope_length = rope_length
        self.rope_torsional_stiffness = rope_torsional_stiffness
        self.position = initial_position  # (x, y, z)
        self.velocity = initial_velocity  # (vx, vy, vz)
        self.acceleration = (0, 0, 0)  # (ax, ay, az)
        self.heading = initial_heading
        self.heading_velocity = 0
        self.heading_acceleration = 0

    def update(self, force_x, force_y, torque_heading, dt):
        # Update position and velocity in X and Y
        self.acceleration = (force_x / self.mass, force_y / self.mass, 0)
        self.velocity = (self.velocity[0] + self.acceleration[0] * dt,
                         self.velocity[1] + self.acceleration[1] * dt,
                         self.velocity[2] + self.acceleration[2] * dt)
        self.position = (self.position[0] + self.velocity[0] * dt,
                         self.position[1] + self.velocity[1] * dt,
                         self.position[2] + self.velocity[2] * dt)

        # Update heading
        self.heading_acceleration = torque_heading / self.rope_torsional_stiffness
        self.heading_velocity += self.heading_acceleration * dt
        self.heading += self.heading_velocity * dt

def simulate_sphere():
    sphere = SphereSimulation(radius=0.1, mass=1.0, rope_length=1.0, rope_torsional_stiffness=10.0)
    pid_x = PIDController(kp=1.0, ki=0.1, kd=0.01, setpoint=0.0)
    pid_y = PIDController(kp=1.0, ki=0.1, kd=0.01, setpoint=0.0)
    pid_heading = PIDController(kp=1.0, ki=0.1, kd=0.01, setpoint=0.0)
    dt = 0.01
    duration = 10.0
    current_time = 0.0

    while current_time < duration:
        force_x = pid_x.update(sphere.position[0], dt)
        force_y = pid_y.update(sphere.position[1], dt)
        torque_heading = pid_heading.update(sphere.heading, dt)
        sphere.update(force_x, force_y, torque_heading, dt)
        print(f"Time: {current_time:.2f}s, Position: {sphere.position}, Velocity: {sphere.velocity}, Heading: {sphere.heading:.2f}")
        time.sleep(dt)
        current_time += dt

if __name__ == "__main__":
    simulate_sphere()
