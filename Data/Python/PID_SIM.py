import numpy as np
import matplotlib.pyplot as plt

# PID controller class
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0

    def update(self, current_value, dt):
        # Error between the setpoint and the current value
        error = self.setpoint - current_value

        # Integral is the sum of errors over time
        self.integral += error * dt

        # Derivative is the rate of change of error
        derivative = (error - self.previous_error) / dt

        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Update the previous error
        self.previous_error = error

        return output


# Simulating an unstable system (for example, a mass-spring-damper system)
def unstable_system(output, current_value):
    # Simple second-order unstable system model
    acceleration = output - 0.1 * current_value
    return acceleration


# Simulate PID control of an unstable system
def simulate_pid(kp, ki, kd, duration=20, dt=0.01):
    pid = PIDController(kp, ki, kd, setpoint=0)
    
    # Time array
    t = np.arange(0, duration, dt)
    
    # Initialize variables
    x = np.zeros_like(t)  # Position
    v = np.zeros_like(t)  # Velocity

    for i in range(1, len(t)):
        # PID controller output
        control_output = pid.update(x[i-1], dt)

        # Unstable system dynamics
        acceleration = unstable_system(control_output, x[i-1])

        # Simple integration to get velocity and position
        v[i] = v[i-1] + acceleration * dt
        x[i] = x[i-1] + v[i] * dt

    # Plot results
    plt.figure(figsize=(10, 6))
    plt.plot(t, x, label="Position (System Response)")
    plt.axhline(0, color='black', linestyle='--', label='Setpoint')
    plt.title(f"PID Controller Simulation (Kp={kp}, Ki={ki}, Kd={kd})")
    plt.xlabel("Time (s)")
    plt.ylabel("Position")
    plt.legend()
    plt.grid(True)
    plt.show()


# Example usage
if __name__ == "__main__":
    # You can change the Kp, Ki, Kd values to see how they affect the system
    simulate_pid(kp=2.0, ki=0.1, kd=1.0)
