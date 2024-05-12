import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def agent_a_kinematics(state, t, thetad_a):
    x_a, y_a, theta_a, v_a = state

    # Compute the desired angular velocity
    w_desired_a = -k_a * (theta_a - thetad_a)
    # kinematics
    dxadt = v_a * np.cos(theta_a)
    dyadt = v_a * np.sin(theta_a)
    dthetadt_a = w_desired_a
    dvdt_a = 0  
    return [dxadt, dyadt, dthetadt_a, dvdt_a]
def agent_b_kinematics(state, t, thetad_b):
    x_b, y_b, theta_b, v_b = state

    # Compute the desired angular velocity
    w_desired_b = -k_b * (theta_b - thetad_b)
    # kinematics
    dxbdt = v_b * np.cos(theta_b)
    dybdt = v_b * np.sin(theta_b)
    dthetadt_b = w_desired_b
    dvdt_b = 0  # Assume constant linear velocity (no acceleration)
    return [dxbdt, dybdt, dthetadt_b, dvdt_b]

# Control gain for Agent A and Agent B
k_a = float(input("Enter control gain for agent A (k_a): "))
k_b = float(input("Enter control gain for agent B (k_b): "))
# Initial conditions for Agent A 
x_initial_a = float(input("Enter initial X position for Agent A : "))
y_initial_a = float(input("Enter initial Y position for Agent A : "))
theta_initial_a = float(input("Enter initial orientation (theta_a) for Agent A  (in radians): "))
v_initial_a = float(input("Enter initial linear velocity for Agent A : "))
# Initial conditions for Agent B
x_initial_b = float(input("Enter initial X position for Agent B : "))
y_initial_b = float(input("Enter initial Y position for Agent B : "))
theta_initial_b = float(input("Enter initial orientation (theta_b) for Agent B  (in radians): "))
v_initial_b = float(input("Enter initial linear velocity for Agent B : "))

# Desired orientations 
thetad_a = float(input("Enter desired orientation (thetad_a) for Agent A (in radians): "))
thetad_b = float(input("Enter desired orientation (thetad_b) for Agent B (in radians): "))
t = np.linspace(0, 10, 100)
# Solve the ODEs for Agent A and Agent B with closed-loop control
solution_a = odeint(agent_a_kinematics, [x_initial_a, y_initial_a, theta_initial_a, v_initial_a], t, args=(thetad_a,))
solution_b = odeint(agent_b_kinematics, [x_initial_b, y_initial_b, theta_initial_b, v_initial_b], t, args=(thetad_b,))

# Extract the positions and orientations of Agent A and Agent B over time
x_trajectory_a = solution_a[:, 0]
y_trajectory_a = solution_a[:, 1]
theta_trajectory_a = solution_a[:, 2]
v_trajectory_a = solution_a[:, 3]
x_trajectory_b = solution_b[:, 0]
y_trajectory_b = solution_b[:, 1]
theta_trajectory_b = solution_b[:, 2]
v_trajectory_b = solution_b[:, 3]

# Plot the trajectories of Agent A and Agent B
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(x_trajectory_a, y_trajectory_a, label='Agent A')
plt.plot(x_trajectory_b, y_trajectory_b, label='Agent B')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Agent A and Agent B Trajectories')
plt.grid(True)
plt.legend()
plt.subplot(2, 1, 2)
plt.plot(t, theta_trajectory_a, label='Agent A Orientation')
plt.plot(t, theta_trajectory_b, label='Agent B Orientation')
plt.plot(t, v_trajectory_a, label='Linear Velocity_a')
plt.plot(t, v_trajectory_b, label='Linear Velocity_b')
plt.xlabel('Time')
plt.ylabel('Agent states')
plt.title('Agent Orientations and linear velocities')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
