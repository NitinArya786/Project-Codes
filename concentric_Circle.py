import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import random
import math

class Agent:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def move(self, new_x, new_y):
        self.x = new_x
        self.y = new_y
def calculate_distance(agent1, agent2):
    return math.sqrt((agent1.x - agent2.x)**2 + (agent1.y - agent2.y)**2)

p=0
q=0
r=7
s=7

agent1 = Agent(p, q)
agent2 = Agent(r, s)
point1_positions = [(p, q)]
point2_positions = [(r, s)]
while True:
    distance = calculate_distance(agent1, agent2)
    if distance < 5:
        # Calculate the direction to move both agents away from each other.
        direction_x = (agent2.x - agent1.x) / distance
        direction_y = (agent2.y - agent1.y) / distance
        # Move both agents away from each other while maintaining their relative position.
        new_x1 = agent1.x - 0.5 * direction_x
        new_y1 = agent1.y - 0.5 * direction_y
        new_x2 = agent2.x + 0.5 * direction_x
        new_y2 = agent2.y + 0.5 * direction_y
        
        agent1.move(new_x1, new_y1)
        agent2.move(new_x2, new_y2)
        point1_positions.append((new_x1, new_y1))
        point2_positions.append((new_x2, new_y2))
        break
    if distance > 5:
        point1 = (p, q)
        point2 = (r, s)
# Define the maximum distance between the points
        max_distance = 5
# Function to calculate the distance between two points
        def distance(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
# Main loop to bring the points closer
        while distance(point1, point2) > max_distance:
    # Calculate the direction from point1 to point2
            dx = point2[0] - point1[0]
            dy = point2[1] - point1[1]
 # Normalize the direction to move one unit
            norm = math.sqrt(dx**2 + dy**2)
            dx /= norm
            dy /= norm
# Move the points closer by one unit
            point1 = (point1[0] + dx, point1[1] + dy)
            point2 = (point2[0] - dx, point2[1] - dy)
            point1_positions.append(point1)
            point2_positions.append(point2)
        break
    if distance == 5:
          point1 = (p, q)
          point2 = (r, s)
          break
print("Final positions:")
print("Point1:", point1_positions[-1])
print("Point2:", point2_positions[-1])
b,h = point1_positions[-1]
v,g = point2_positions[-1]
X1, Y1 = zip(*point1_positions)
X2, Y2 = zip(*point2_positions)
# Robot kinematics with control
def circle_ode(y, t, r, v):
    x, y = y
    theta_dot = v / r  # Constant angular velocity
    dxdt = -r * theta_dot * np.sin(theta_dot * t)
    dydt = r * theta_dot * np.cos(theta_dot * t)
    return [dxdt, dydt]
# Constants
desired_distance = 5.0  # Desired fixed distance between agents
velocity1 = 1
velocity2 = 1

# Fixed initial positions of Agent 1 and Agent 2
x1, y1 = b, h  # Initial positions of Agent 1
x2, y2 = v, g   # Initial positions of Agent 2
n = random.randint(1, 4)
# Define points A and B
A = np.array([x1, y1])  # Replace with the coordinates of point A
B = np.array([x2, y2])  # Replace with the coordinates of point B
# Define the desired distance "d"
r = desired_distance + n
# Calculate vector AB
AB = B - A
# Normalize AB to get a unit vector uAB
magnitude = np.linalg.norm(AB)
uAB = AB / magnitude
# Calculate point C at distance "d" from point A along the line AB
# Store the coordinates of point C in variables p and q
p, q = tuple(A + r * uAB)
# You can now use "p" and "q" as the x and y coordinates of point C
print("Point C (p, q):", (p, q))
# Calculate the center of the concentric circles
(center_x, center_y) = (p,q)
# Calculate the radii for the concentric circles
radius1 = r
radius2 = n
# Time points for simulation
t = np.linspace(0, 200, 2000)
initial_conditions1 = [center_x + radius1, center_y]
initial_conditions2 = [center_x + radius2, center_y]
# Solve the ODE for robot motion with closed-loop control
# Solve the ODEs for both circles
solution1 = odeint(circle_ode, initial_conditions1, t, args=(radius1, velocity1))
solution2 = odeint(circle_ode, initial_conditions2, t, args=(radius2, velocity2))
# Extract x and y coordinates from the solutions
x1, y1 = solution1.T
x2, y2 = solution2.T
# Plot the trajectory of total path
plt.plot(X1, Y1, label='Agent 1')
plt.plot(X2, Y2, label='Agent 2')
plt.title('Trajectory of Agents')
plt.plot(x1, y1, label=f"Circle 1 (R={radius1}, V={velocity1})")
plt.plot(x2, y2, label=f"Circle 2 (R={radius2}, V={velocity2})")
plt.gca().set_aspect('equal', adjustable='box')
plt.title("Two Circles with the Same Center")
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.grid()
plt.show()


