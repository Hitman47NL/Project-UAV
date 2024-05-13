import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 1 in het positieve scenario
voltage_1_pos = np.array([1, 2, 3, 4, 5, 6])
meting_1_pos = np.array([6.8, 20.41, 36.30, 54.44, 86.20, 117.95])

# Data voor motor 2 in het positieve scenario
voltage_2_pos = np.array([1, 2, 3, 4, 5, 6])
meting_2_pos = np.array([6.8, 18.16, 36.30, 56.70, 95.26, 127.02])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(voltage_2_pos, meting_2_pos, color='blue',marker = 'o', label='Motor 2 (+)')
plt.plot(voltage_1_pos, meting_1_pos, color='red',marker = 'o', label='Motor 1 (+)')


plt.xlabel('Voltage (V)')
plt.ylabel('Kracht (N)')
plt.title('Motor 1 vs Motor 2 (+)')
plt.legend()

plt.grid(True)
