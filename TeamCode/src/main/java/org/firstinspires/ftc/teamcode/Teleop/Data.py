import numpy as np
from sklearn.linear_model import LinearRegression
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# Data
distance = [33.45, 50.91, 69.97, 84.85, 103.23, 113.84, 145]
rpm = [1200, 1275, 1375, 1425, 1525, 1650, 1750]
servo = [0, 0.3, 0.41, 0.47, 0.46, 0.6, 0.65]

# Create figure and axis
fig, ax1 = plt.subplots(figsize=(10,6))

# Plot RPM on left y-axis
color_rpm = 'tab:blue'
ax1.set_xlabel('Distance')
ax1.set_ylabel('RPM', color=color_rpm)
ax1.plot(distance, rpm, marker='o', color=color_rpm, label='RPM')
ax1.tick_params(axis='y', labelcolor=color_rpm)

# Plot ServoPos on right y-axis
ax2 = ax1.twinx()
color_servo = 'tab:red'
ax2.set_ylabel('Servo Position', color=color_servo)
ax2.plot(distance, servo, marker='s', color=color_servo, label='ServoPos')
ax2.tick_params(axis='y', labelcolor=color_servo)

# Title and grid
plt.title('RPM and Servo Position vs Distance')
ax1.grid(True)

# Show plot
fig.tight_layout()
plt.show()
