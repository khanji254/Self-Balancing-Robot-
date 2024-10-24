import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# PID controller class
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

# Simulating an unstable system
def unstable_system(output, current_value):
    acceleration = output - 0.1 * current_value
    return acceleration

# Simulate and update plot
def simulate_pid(kp, ki, kd, duration=20, dt=0.01):
    pid = PIDController(kp, ki, kd, setpoint=0)
    t = np.arange(0, duration, dt)
    x = np.zeros_like(t)
    v = np.zeros_like(t)
    
    for i in range(1, len(t)):
        control_output = pid.update(x[i-1], dt)
        acceleration = unstable_system(control_output, x[i-1])
        v[i] = v[i-1] + acceleration * dt
        x[i] = x[i-1] + v[i] * dt

    line.set_ydata(x)
    fig.canvas.draw_idle()

# Setup the plot
duration = 20
dt = 0.01
t = np.arange(0, duration, dt)

fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.25)
x = np.zeros_like(t)
line, = ax.plot(t, x, label="Position (System Response)")
ax.axhline(0, color='black', linestyle='--', label='Setpoint')
ax.set_xlabel("Time (s)")
ax.set_ylabel("Position")
ax.set_title("PID Controller Simulation")
plt.grid(True)

# Initial PID constants
kp0, ki0, kd0 = 2.0, 0.1, 1.0

# Create sliders
axcolor = 'lightgoldenrodyellow'
ax_kp = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)
ax_ki = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
ax_kd = plt.axes([0.25, 0.05, 0.65, 0.03], facecolor=axcolor)

slider_kp = Slider(ax_kp, 'Kp', 0.1, 10.0, valinit=kp0)
slider_ki = Slider(ax_ki, 'Ki', 0.0, 2.0, valinit=ki0)
slider_kd = Slider(ax_kd, 'Kd', 0.0, 5.0, valinit=kd0)

# Update the simulation when slider value is changed
def update(val):
    kp = slider_kp.val
    ki = slider_ki.val
    kd = slider_kd.val
    simulate_pid(kp, ki, kd, duration, dt)

slider_kp.on_changed(update)
slider_ki.on_changed(update)
slider_kd.on_changed(update)

# Initial plot update
simulate_pid(kp0, ki0, kd0)

plt.show()
