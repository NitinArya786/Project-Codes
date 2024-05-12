import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

k_a = float(input("Enter control gain for Agent A (k_a): "))
k_b = float(input("Enter control gain for Agent B (k_b): "))
desired_distance = float(input("Enter the desired distance between agents: "))
k_distance = float(input("Enter the control gain for maintaining distance (k_distance): "))
x_initial_a = float(input("Enter initial X position for Agent A: "))
y_initial_a = float(input("Enter initial Y position for Agent A: "))
theta_initial_a = float(input("Enter initial orientation (theta_a) for Agent A (in radians): "))
v_initial_a = float(input("Enter initial linear velocity for Agent A: "))
x_initial_b = float(input("Enter initial X position for Agent B: "))
y_initial_b = float(input("Enter initial Y position for Agent B: "))
theta_initial_b = float(input("Enter initial orientation (theta_b) for Agent B (in radians): "))
v_initial_b = float(input("Enter initial linear velocity for Agent B: "))
thetad_a = float(input("Enter desired orientation (thetad_a) for Agent A (in radians): "))
thetad_b = float(input("Enter desired orientation (thetad_b) for Agent B (in radians): "))

def agent_a_kinematics(state, t, x_b, y_b):
    x_a, y_a, theta_a = state

    w_desired_a = -k_a * (theta_a - np.arctan2(y_b - y_a, x_b - x_a))
    # Calculate the desired linear velocity based on maintaining distance
    distance_to_b = (np.sqrt((x_a - x_b) ** 2 + (y_a - y_b) ** 2))+(.1)
    error_distance = distance_to_b - desired_distance
    v_desired_a = k_distance * error_distance

    dxadt = v_desired_a * np.cos(theta_a)
    dyadt = v_desired_a * np.sin(theta_a)
    dthetadt_a = w_desired_a
    return [dxadt, dyadt, dthetadt_a]
def agent_b_kinematics(state, t, x_a, y_a):
    x_b, y_b, theta_b = state

    w_desired_b = -k_b * (theta_b - np.arctan2(y_a - y_b, x_a - x_b))
    # Calculate the desired linear velocity based on maintaining distance
    distance_to_a = (np.sqrt((x_b - x_a) ** 2 + (y_b - y_a) ** 2))+(.1)
    error_distance = distance_to_a - desired_distance
    v_desired_b = k_distance * error_distance

    dxbdt = v_desired_b * np.cos(theta_b)
    dybdt = v_desired_b * np.sin(theta_b)
    dthetadt_b = w_desired_b
    return [dxbdt, dybdt, dthetadt_b]

t = np.linspace(0, 20, 2000)
# Solve the ODEs for Agent A and Agent B with closed-loop control
solution_a = odeint(agent_a_kinematics, [x_initial_a, y_initial_a, theta_initial_a], t, args=(x_initial_b, y_initial_b))
solution_b = odeint(agent_b_kinematics, [x_initial_b, y_initial_b, theta_initial_b], t, args=(x_initial_a, y_initial_a))

# Extract the positions and orientations of Agent A and Agent B over time
x_trajectory_a = solution_a[:, 0]
y_trajectory_a = solution_a[:, 1]
theta_trajectory_a = solution_a[:, 2]
x_trajectory_b = solution_b[:, 0]
y_trajectory_b = solution_b[:, 1]
theta_trajectory_b = solution_b[:, 2]

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
plt.xlabel('Time')
plt.ylabel('Agent orientations')
plt.title('Agent Orientations')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

