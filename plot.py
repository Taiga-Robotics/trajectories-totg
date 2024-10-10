#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv
import numpy as np

def read_csv(filename):
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        return list(reader)

def calculate_derivative(data, time):
    return np.gradient(data, time)

# Read trajectory data from CSV
trajectory_data = read_csv('trajectory_results.csv')
header = trajectory_data[0]
trajectory_data = [[float(x) for x in row] for row in trajectory_data[1:]]

# Separate data into columns
time = np.array([row[header.index('Time')] for row in trajectory_data])
pos_x = np.array([row[header.index('Position_X')] for row in trajectory_data])
pos_y = np.array([row[header.index('Position_Y')] for row in trajectory_data])
pos_z = np.array([row[header.index('Position_Z')] for row in trajectory_data])
vel_x = np.array([row[header.index('Velocity_X')] for row in trajectory_data])
vel_y = np.array([row[header.index('Velocity_Y')] for row in trajectory_data])
vel_z = np.array([row[header.index('Velocity_Z')] for row in trajectory_data])

# Calculate acceleration
acc_x = calculate_derivative(vel_x, time)
acc_y = calculate_derivative(vel_y, time)
acc_z = calculate_derivative(vel_z, time)

# Calculate jerk
jerk_x = calculate_derivative(acc_x, time)
jerk_y = calculate_derivative(acc_y, time)
jerk_z = calculate_derivative(acc_z, time)

# Create the plot
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(12, 20), sharex=True)

# Position plot
ax1.plot(time, pos_x, label='X')
ax1.plot(time, pos_y, label='Y')
ax1.plot(time, pos_z, label='Z')
ax1.set_ylabel('Position')
ax1.legend()
ax1.grid(True)
ax1.set_title('Trajectory: Position, Velocity, Acceleration, and Jerk over Time')

# Velocity plot
ax2.plot(time, vel_x, label='VX')
ax2.plot(time, vel_y, label='VY')
ax2.plot(time, vel_z, label='VZ')
ax2.set_ylabel('Velocity')
ax2.legend()
ax2.grid(True)

# Acceleration plot
ax3.plot(time, acc_x, label='AX')
ax3.plot(time, acc_y, label='AY')
ax3.plot(time, acc_z, label='AZ')
ax3.set_ylabel('Acceleration')
ax3.legend()
ax3.grid(True)

# Jerk plot
ax4.plot(time, jerk_x, label='JX')
ax4.plot(time, jerk_y, label='JY')
ax4.plot(time, jerk_z, label='JZ')
ax4.set_xlabel('Time')
ax4.set_ylabel('Jerk')
ax4.legend()
ax4.grid(True)

plt.tight_layout()
plt.savefig('trajectory_plot.png')
plt.show()

print("Plot saved to trajectory_plot.png")
