import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Define the robot dynamics as a system of nonlinear ODEs with closed-loop control
def kinematics(state, t, thetad):
    x, y, theta, v = state

    # Compute the desired angular velocity
    w_desired = -k * (theta - thetad)

    # kinematics
    dxdt = v * np.cos(theta)
    dydt = v * np.sin(theta)
    dthetadt = w_desired
    dvdt = 0  

    return [dxdt, dydt, dthetadt, dvdt]

# Control gain
k = float(input("Enter control gain (k): "))

# Initial conditions
x_initial = float(input("Enter initial X position for Agent : "))
y_initial = float(input("Enter initial Y position for Agent : "))
theta_initial = float(input("Enter initial orientation (theta) for Agent  (in radians): "))
v_initial = float(input("Enter initial linear velocity for Agent : "))

# Desired orientation 
thetad = float(input("Enter desired orientation (thetad) for Agent (in radians): "))

# Time points for simulation
t = np.linspace(0, 10, 100)

# Solve the ODEs for the robot with closed-loop control
solution = odeint(kinematics,[x_initial, y_initial, theta_initial, v_initial], t, args=(thetad,))

# Extract the robot's position, orientation, and control input over time
x_trajectory = solution[:, 0]
y_trajectory = solution[:, 1]
theta_trajectory = solution[:, 2]
v_trajectory = solution[:, 3]

# Plot the robot's trajectory and control input
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(x_trajectory, y_trajectory, label='Robot moving Path')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Robot Trajectory in closed loop system')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(t, theta_trajectory, label='Orientation')
plt.plot(t, v_trajectory, label='Linear Velocity')
plt.xlabel('Time')
plt.ylabel('Robot State')
plt.title('Robot Orientation and Linear Velocity')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()



