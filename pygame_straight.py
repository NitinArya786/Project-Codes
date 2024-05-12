import pygame
import sys
import math
import numpy as np
from scipy.integrate import solve_ivp

# Initialize Pygame
pygame.init()

# Screen settings
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Leader-Follower Straight Line Path")

# Colors
white = (255, 255, 255)
red = (255, 0, 0)

# Leader parameters
leader_speed = 1.0  # Leader's speed for straight-line motion

# Follower parameters
follower_follow_distance = 50
follower_speed = 1.0

# System of ODEs for the leader and follower
def ode_system(t, Y):
    leader_x, leader_y, follower_x, follower_y = Y

    # ODEs for the leader's straight-line motion
    dx_L_dt = leader_speed
    dy_L_dt = 0  # Leader only moves horizontally

    # ODEs for the follower to maintain a fixed distance
    D_x = leader_x - follower_x
    D_y = leader_y - follower_y
    D_mag = math.sqrt(D_x**2 + D_y**2)
    
    if D_mag == 0:
        N_x, N_y = 0, 0
    else:
        N_x = D_x / D_mag
        N_y = D_y / D_mag

    dx_F_dt = follower_speed * (leader_x - follower_x - follower_follow_distance * N_x)
    dy_F_dt = follower_speed * (leader_y - follower_y - follower_follow_distance * N_y)

    return [dx_L_dt, dy_L_dt, dx_F_dt, dy_F_dt]

# Initial conditions
initial_conditions = [100, 300, 100, 300]  # Starting positions for leader and follower

# Time span for integration
t_span = (0, 20)

# Main loop
running = True
t = t_span[0]  # Initialize time
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Integrate the ODE system with multiple intermediate time points
    time_points = np.linspace(t, t + 1, 100)  # Adjusted the time step
    sol = solve_ivp(ode_system, (t, t + 1), initial_conditions)

    leader_x, leader_y, follower_x, follower_y = sol.y[:, -1]  # Get the final positions

    screen.fill(white)  # Clear the screen
    # Draw leader and follower
    pygame.draw.circle(screen, red, (int(leader_x), int(leader_y)), 5)
    pygame.draw.circle(screen, red, (int(follower_x), int(follower_y)), 5)
    # Update the display
    pygame.display.update()
    # Control the speed of animation
    pygame.time.delay(20)  # Delay in milliseconds
    # Update initial conditions and time for the next iteration
    initial_conditions = [leader_x, leader_y, follower_x, follower_y]
    t += 1
# Quit Pygame
pygame.quit()
sys.exit()
