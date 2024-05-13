import numpy as np
import matplotlib.pyplot as plt

# Data voor propellor 6 bladed in het positieve scenario
voltage_1_pos = np.array([1, 2, 3, 4, 5, 6])
meting_1_pos = np.array([15.88, 65.78, 90.73, 154.24, 224.55, 229.09])

# Data voor propellor 6 bladed in het positieve scenario
voltage_2_pos = np.array([1, 2, 3, 4, 5, 6])
meting_2_pos = np.array([15.88, 52.17, 104.34, 156.51, 204.14, 201.87])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(voltage_2_pos, meting_2_pos, color='blue',marker = 'o', label='Motor 2 (+)')
plt.plot(voltage_1_pos, meting_1_pos, color='red',marker = 'o', label='Motor 1 (+)')


plt.xlabel('Voltage (V)')
plt.ylabel('Kracht (N)')
plt.title('Motor 1, 6 bladed propellor')
plt.legend()

plt.grid(True)
