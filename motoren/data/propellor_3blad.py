import numpy as np
import matplotlib.pyplot as plt

# Data voor propellor 3 bladed in het positieve scenario
voltage_1_pos = np.array([1, 2, 3, 4, 5, 6])
meting_1_pos = np.array([15.88, 56.71, 113.41, 174.65, 247.24, 278.99])

# Data voor propellor 3 bladed in het positieve scenario
voltage_2_pos = np.array([1, 2, 3, 4, 5, 6])
meting_2_pos = np.array([13.61, 43.09, 86.19, 131.56, 183.73, 213.21])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(voltage_2_pos, meting_2_pos, color='blue', label='Motor 2 (+)')
plt.plot(voltage_1_pos, meting_1_pos, color='red', label='Motor 1 (+)')


plt.xlabel('Voltage (V)')
plt.ylabel('Kracht (N)')
plt.title('Motor 1, 3 bladed propellor')
plt.legend()

plt.grid(True)
