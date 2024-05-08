import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 2 in het positieve scenario
voltage_2_pos = np.array([1, 2, 3])
meting_2_pos = np.array([24.95, 72.58, 115.68])

# Data voor motor 2 in het negatieve scenario
voltage_2_neg = np.array([1, 2, 3])
meting_2_neg = np.array([29.49, 92.99, 142.89])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(voltage_2_pos, meting_2_pos, color='blue', label='Motor 2 (+)')
plt.plot(voltage_2_neg, meting_2_neg, color='red', label='Motor 2 (-)')

plt.xlabel('Voltage (V)')
plt.ylabel('Meting (g)')
plt.title('Metingen van kleine motor')
plt.legend()

plt.grid(True)

