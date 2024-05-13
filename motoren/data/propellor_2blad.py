import numpy as np
import matplotlib.pyplot as plt

# Data voor propellor 2 bladed in het positieve scenario
voltage_1_pos = np.array([1, 2, 3, 4, 5, 6])
meting_1_pos = np.array([13.61, 40.83, 77.12, 124.75, 174.65, 233.63])

# Data voor propellor 2 bladed in het positieve scenario
voltage_2_pos = np.array([1, 2, 3, 4, 5, 6])
meting_2_pos = np.array([6.8, 40.83, 81.66, 136.09, 190.53, 249.50])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(voltage_2_pos, meting_2_pos, color='blue',marker = 'o', label='Motor 2 (+)')
plt.plot(voltage_1_pos, meting_1_pos, color='red',marker = 'o', label='Motor 1 (+)')


plt.xlabel('Voltage (V)')
plt.ylabel('Kracht (N)')
plt.title('Motor 1, 2 bladed propellor')
plt.legend()

plt.grid(True)
